#include "BMI088.h"

BMI088::BMI088(TwoWire &wire, uint8_t accAddress, uint8_t gyroAddress)
    : _wire(wire), _acc{this, accAddress}, _gyro{this, gyroAddress} {
    _dev.intf = BMI08_I2C_INTF;
    _dev.variant = BMI088_VARIANT;
    _dev.intf_ptr_accel = &_acc;
    _dev.intf_ptr_gyro = &_gyro;
    _dev.read = busRead;
    _dev.write = busWrite;
    _dev.delay_us = delayUs;
    // 16-byte even chunks divide the Bosch configuration stream exactly;
    // one register byte + payload also fits a 32-byte Wire TX buffer.
    _dev.read_write_len = 16;
}

bool BMI088::check(int8_t result, const char *stage) {
    _error = result;
    _stage = stage;
    return result == BMI08_OK;
}

int8_t BMI088::busFailure(const char *phase, uint8_t address, uint8_t reg,
                         uint8_t status, uint32_t requested, uint32_t completed) {
    ++_i2cErrors;
    _busFailure = {phase, address, reg, status, requested, completed};
    return -1;
}

void BMI088::printLastBusError(Stream &port) const {
    port.print("Last I2C failure: phase="); port.print(_busFailure.phase);
    port.print(" addr=0x"); port.print(_busFailure.address, HEX);
    port.print(" reg=0x"); port.print(_busFailure.reg, HEX);
    port.print(" WireStatus="); port.print(_busFailure.status);
    port.print(" requested="); port.print(_busFailure.requested);
    port.print(" completed="); port.print(_busFailure.completed);
    port.print(" total="); port.println(_i2cErrors);
    port.println("WireStatus is raw endTransmission result; 255=local buffer/argument error.");
    port.println("For RX short reads, status=0 refers only to register-address transmission.");
}

BMI08_INTF_RET_TYPE BMI088::busRead(uint8_t reg, uint8_t *data, uint32_t len, void *context) {
    auto *c = static_cast<BusContext *>(context);
    auto &self = *c->owner;
    auto &wire = self._wire;
    if (!data || !len || len > 32)
        return self.busFailure("READ_ARGUMENT", c->address, reg, 255, len, 0);
    wire.beginTransmission(c->address);
    if (wire.write(reg) != 1) {
        wire.endTransmission();
        return self.busFailure("READ_REGISTER_BUFFER", c->address, reg, 255, 1, 0);
    }
    const uint8_t status = wire.endTransmission(false);
    if (status != 0)
        return self.busFailure("READ_REGISTER_TX", c->address, reg, status, len, 0);
    const size_t count = wire.requestFrom(c->address, static_cast<size_t>(len), true);
    if (count != len || wire.available() < static_cast<int>(len)) {
        while (wire.available()) wire.read();
        return self.busFailure("READ_SHORT", c->address, reg, 0, len, count);
    }
    for (uint32_t i = 0; i < len; ++i) data[i] = static_cast<uint8_t>(wire.read());
    return BMI08_INTF_RET_SUCCESS;
}

BMI08_INTF_RET_TYPE BMI088::busWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *context) {
    auto *c = static_cast<BusContext *>(context);
    auto &self = *c->owner;
    auto &wire = self._wire;
    if (!data || !len || len > 31)
        return self.busFailure("WRITE_ARGUMENT", c->address, reg, 255, len, 0);
    wire.beginTransmission(c->address);
    const size_t r = wire.write(reg);
    const size_t n = wire.write(data, static_cast<size_t>(len));
    const uint8_t status = wire.endTransmission(true);
    if (r != 1 || n != len)
        return self.busFailure("WRITE_BUFFER", c->address, reg, 255, len, n);
    if (status != 0)
        return self.busFailure("WRITE_TX", c->address, reg, status, len, n);
    return BMI08_INTF_RET_SUCCESS;
}

void BMI088::delayUs(uint32_t us, void *) {
    if (us >= 1000) delay(us / 1000);
    if (us % 1000) delayMicroseconds(us % 1000);
}

bool BMI088::isConnection() {
    return check(bmi08xa_init(&_dev), "accel ID") &&
           check(bmi08g_init(&_dev), "gyro ID");
}

bool BMI088::beginSync(SyncRate rate, Stream *startupLog, bool resetGyro) {
    _ready = false;
    if (rate != SyncRate::Hz400 && rate != SyncRate::Hz1000)
        return check(BMI08_E_INVALID_INPUT, "unsupported I2C sync rate");
    if (!isConnection()) return false;
    if (startupLog) startupLog->println("IDs OK: ACC=0x1E GYR=0x0F");
    uint8_t originalConf, originalCtrl;
    // Probe Accel before the optional gyro reset to locate any loss of access.
    if (!readRegister(false, BMI08_REG_ACCEL_PWR_CONF, originalConf))
        return check(_error, "accel baseline PWR_CONF (before gyro step)");
    if (!readRegister(false, BMI08_REG_ACCEL_PWR_CTRL, originalCtrl))
        return check(_error, "accel baseline PWR_CTRL (before gyro step)");
    if (startupLog) {
        startupLog->print("ACC baseline: PWR_CONF=0x"); startupLog->print(originalConf, HEX);
        startupLog->print(" PWR_CTRL=0x"); startupLog->println(originalCtrl, HEX);
    }
    if (resetGyro) {
        if (startupLog) startupLog->println("GYR reset: ENABLED (diagnostic A/B)");
        const int8_t result = bmi08g_soft_reset(&_dev);
        // On error the Bosch API does not wait; still allow reset to settle.
        if (result != BMI08_OK) delay(30);
        if (!check(result, "gyro reset")) return false;
        if (!readRegister(false, BMI08_REG_ACCEL_PWR_CONF, originalConf))
            return check(_error, "accel PWR_CONF immediately after gyro reset");
        if (startupLog) startupLog->println("ACC access after GYR reset: OK");
    } else if (startupLog) {
        startupLog->println("GYR reset: SKIPPED (Crazyflie-style startup)");
    }
    // No reset does NOT mean no configuration. Later stages explicitly set
    // gyro normal mode, range, ODR/BW, and INT3/INT4 routing, then read back.
    // BMI088 datasheet rev1.9 section 4.8.1: active-mode soft reset can
    // release SDA during ACK. Prepare APS + disabled sensor BEFORE reset.
    // Use documented full-byte values (0x7C=0x03, 0x7D=0x00), not 0x7C=1.
    // Bosch API waits 5ms between writes and 5ms after the power transition.
    _dev.accel_cfg.power = BMI08_ACCEL_PM_SUSPEND;
    if (!check(bmi08a_set_power_mode(&_dev), "accel pre-reset suspend")) return false;
    uint8_t pwrConf, pwrCtrl;
    if (!readRegister(false, BMI08_REG_ACCEL_PWR_CONF, pwrConf) ||
        !readRegister(false, BMI08_REG_ACCEL_PWR_CTRL, pwrCtrl)) return false;
    if (startupLog) {
        startupLog->print("ACC pre-reset: PWR_CONF=0x"); startupLog->print(pwrConf, HEX);
        startupLog->print(" PWR_CTRL=0x"); startupLog->println(pwrCtrl, HEX);
    }
    if (pwrConf != 0x03 || pwrCtrl != 0x00)
        return check(BMI08_E_INVALID_CONFIG, "accel pre-reset power readback");
    const int8_t resetResult = bmi08a_soft_reset(&_dev);
    // Wait even if the transaction failed; do not access a possibly resetting die.
    delay(5);
    if (!check(resetResult, "accel reset after suspend")) return false;
    if (startupLog) startupLog->println("ACC reset ACK OK");
    if (!isConnection()) return false;
    if (!readRegister(false, BMI08_REG_ACCEL_PWR_CONF, pwrConf) ||
        !readRegister(false, BMI08_REG_ACCEL_PWR_CTRL, pwrCtrl)) return false;
    if (pwrConf != 0x03 || pwrCtrl != 0x00)
        return check(BMI08_E_INVALID_CONFIG, "accel post-reset power readback");
    if (startupLog) startupLog->println("Post-reset IDs OK; loading sync config...");
    // Includes configuration stream upload, startup delay and INTERNAL_STATUS check.
    const uint32_t errorsBeforeUpload = _i2cErrors;
    if (!check(bmi08a_load_config_file(&_dev), "load Bosch sync config")) return false;
    // Upstream upload loop may overwrite an earlier chunk error with a later OK.
    if (_i2cErrors != errorsBeforeUpload)
        return check(BMI08_E_COM_FAIL, "sync config chunk transfer");
    _dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    if (!check(bmi08a_set_power_mode(&_dev), "accel active")) return false;
    _dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    if (!check(bmi08g_set_power_mode(&_dev), "gyro normal")) return false;
    _dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    _dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    bmi08_data_sync_cfg sync{};
    sync.mode = rate == SyncRate::Hz400 ? BMI08_ACCEL_DATA_SYNC_MODE_400HZ
                                      : BMI08_ACCEL_DATA_SYNC_MODE_1000HZ;
    if (!check(bmi08xa_configure_data_synchronization(sync, &_dev), "sync mode")) return false;

    bmi08_int_cfg irq{};
    // Configure INT2 as input FIRST. INT2 and INT3 are physically connected.
    auto &input = irq.accel_int_config_1;
    input.int_channel = BMI08_INT_CHANNEL_2;
    input.int_type = BMI08_ACCEL_SYNC_INPUT;
    input.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    input.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    input.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
    // INT1 is synchronized DRDY, not the ordinary accelerometer DRDY.
    auto &output = irq.accel_int_config_2;
    output.int_channel = BMI08_INT_CHANNEL_1;
    output.int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY;
    output.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    output.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    output.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
    auto &g3 = irq.gyro_int_config_1;
    g3.int_channel = BMI08_INT_CHANNEL_3;
    g3.int_type = BMI08_GYRO_INT_DATA_RDY;
    g3.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    g3.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    g3.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
    auto &g4 = irq.gyro_int_config_2;
    g4 = g3;
    g4.int_channel = BMI08_INT_CHANNEL_4;
    g4.int_pin_cfg.enable_int_pin = BMI08_DISABLE;
    if (!check(bmi08a_set_data_sync_int_config(&irq, &_dev), "interrupt routing")) return false;

    // Validate the critical directions, mappings and ranges before reporting ready.
    struct Check { bool gyro; uint8_t reg, mask, expected; };
    const Check checks[] = {
        {false, 0x54, 0x1F, 0x13}, // INT2 edge input, output disabled
        {false, 0x53, 0x1E, 0x0A}, // INT1 active-high push-pull output
        {false, 0x56, 0x01, 0x01}, // sync DRDY -> INT1
        {false, 0x58, 0x44, 0x00}, // ordinary accel DRDY unmapped
        {false, 0x41, 0x03, 0x01}, // +/-6g
        {true,  0x0F, 0x07, 0x00}, // +/-2000dps
        {true,  0x15, 0x80, 0x80}, // gyro DRDY enabled
        {true,  0x16, 0x03, 0x01}, // INT3 active-high push-pull
        {true,  0x18, 0x81, 0x01}  // gyro DRDY -> INT3 only
    };
    for (const auto &c : checks) {
        uint8_t value;
        if (!readRegister(c.gyro, c.reg, value)) return false;
        if ((value & c.mask) != c.expected)
            return check(BMI08_E_INVALID_CONFIG, "interrupt/range readback");
    }
    _ready = true;
    return check(BMI08_OK, "ready");
}

bool BMI088::readSynchronized(Sample &out) {
    if (!_ready) return check(BMI08_E_INVALID_CONFIG, "not initialized");
    bmi08_sensor_data a{}, g{};
    // Bosch reads synced accel XY at 0x1E, Z at 0x27, then gyro at 0x02.
    // Reading normal accel registers 0x12..0x17 would bypass synchronization.
    if (!check(bmi08a_get_synchronized_data(&a, &g, &_dev), "read synchronized")) return false;
    constexpr float accScale = 6.0f * 9.80665f / 32768.0f;
    constexpr float gyrScale = 2000.0f / 32768.0f;
    Sample sample{a.x * accScale, a.y * accScale, a.z * accScale,
                  g.x * gyrScale, g.y * gyrScale, g.z * gyrScale};
    out = sample;
    return true;
}

bool BMI088::readTemperature(float &celsius) {
    int32_t milliC;
    if (!check(bmi08a_get_sensor_temperature(&_dev, &milliC), "temperature")) return false;
    celsius = milliC / 1000.0f;
    return true;
}

bool BMI088::readRegister(bool gyro, uint8_t reg, uint8_t &value) {
    uint8_t v;
    if (busRead(reg, &v, 1, gyro ? static_cast<void *>(&_gyro) : static_cast<void *>(&_acc)) != 0)
        return check(BMI08_E_COM_FAIL, "register read");
    value = v;
    return check(BMI08_OK, "register read");
}

void BMI088::printDiagnostics(Stream &port) {
    const uint8_t accRegs[] = {0x00, 0x02, 0x2A, 0x40, 0x41, 0x53, 0x54, 0x56, 0x57, 0x58, 0x7C, 0x7D};
    const uint8_t gyrRegs[] = {0x00, 0x0F, 0x10, 0x11, 0x15, 0x16, 0x18};
    for (unsigned die = 0; die < 2; ++die) {
        const uint8_t *regs = die ? gyrRegs : accRegs;
        const unsigned count = die ? sizeof(gyrRegs) : sizeof(accRegs);
        for (unsigned i = 0; i < count; ++i) {
            uint8_t value;
            port.print(die ? "GYR 0x" : "ACC 0x"); port.print(regs[i], HEX);
            if (readRegister(die, regs[i], value)) { port.print(" = 0x"); port.println(value, HEX); }
            else port.println(" = READ_FAIL");
        }
    }
}
