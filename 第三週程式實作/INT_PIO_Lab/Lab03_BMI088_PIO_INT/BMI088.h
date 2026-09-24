/* CURIO BMI088 I2C + hardware data synchronization.
 * RP2354A / Arduino-Pico. 製作：火箭鳥創客倉庫
 * INT3 -> INT2 (input); INT1 -> GPIO22; INT3 -> GPIO23.
 * Bosch SensorAPI and configuration stream are included in src/bosch.
 * New API: use beginSync() + readSynchronized(); not a drop-in flight driver.
 */
#ifndef CURIO_BMI088_H
#define CURIO_BMI088_H
#include <Arduino.h>
#include <Wire.h>
#include "src/bosch/bmi08x.h"

class BMI088 {
public:
    enum class SyncRate : uint16_t { Hz400 = 400, Hz1000 = 1000 };
    struct Sample {
        float ax, ay, az; // m/s^2, gravity INCLUDED, uncalibrated sensor axes
        float gx, gy, gz; // deg/s, uncalibrated sensor axes
    };
    BMI088(TwoWire &wire, uint8_t accAddress = 0x18, uint8_t gyroAddress = 0x69);
    BMI088(const BMI088 &) = delete;
    BMI088 &operator=(const BMI088 &) = delete;

    // Wire must already be configured and started. Call only from loop/core0.
    bool isConnection();
    // Crazyflie-style default: configure gyro without issuing its soft reset.
    // resetGyro=true is an explicit diagnostic A/B option, not a fallback.
    bool beginSync(SyncRate rate = SyncRate::Hz400, Stream *startupLog = nullptr,
                   bool resetGyro = false);
    // Call ONLY after an INT1 synchronized-data-ready event, never in an ISR.
    // On failure, out is unchanged; caller MUST discard it as a fresh sample.
    bool readSynchronized(Sample &out);
    bool readTemperature(float &celsius);
    bool readRegister(bool gyro, uint8_t reg, uint8_t &value);
    void printDiagnostics(Stream &port);
    void printLastBusError(Stream &port) const;
    int8_t lastError() const { return _error; }
    const char *lastStage() const { return _stage; }
    uint32_t i2cErrors() const { return _i2cErrors; }
    bool ready() const { return _ready; }

private:
    struct BusContext { BMI088 *owner; uint8_t address; };
    TwoWire &_wire;
    BusContext _acc, _gyro;
    bmi08_dev _dev{};
    int8_t _error = 0;
    const char *_stage = "not initialized";
    uint32_t _i2cErrors = 0;
    bool _ready = false;
    struct BusFailure {
        const char *phase = "none";
        uint8_t address = 0, reg = 0, status = 0;
        uint32_t requested = 0, completed = 0;
    } _busFailure;
    int8_t busFailure(const char *phase, uint8_t address, uint8_t reg,
                     uint8_t status, uint32_t requested, uint32_t completed);
    bool check(int8_t result, const char *stage);
    static BMI08_INTF_RET_TYPE busRead(uint8_t reg, uint8_t *data, uint32_t len, void *context);
    static BMI08_INTF_RET_TYPE busWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *context);
    static void delayUs(uint32_t us, void *context);
};
#endif
