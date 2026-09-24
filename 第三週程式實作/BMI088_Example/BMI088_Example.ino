/* CURIO / RP2354A / BMI088 I2C + interrupt + Bosch data synchronization
 * 製作：火箭鳥創客倉庫
 * Arduino IDE: use Earle Philhower Arduino-Pico, RP2350 target.
 * Wiring: INT1 -> GPIO22; INT3 -> GPIO23 AND INT2.
 * INT2 is INPUT. Never set GPIO22/23 or sensor INT2 as a driven output.
 * All Wire/driver calls run on core0; ISR only counts and timestamps edges.
 */
#include "BMI088.h"
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <pico/platform.h>
#include <math.h>
#include <stdio.h>

constexpr uint8_t I2C0_SDA = 20;  // preserved from supplied sketch
constexpr uint8_t I2C0_SCL = 25;  // valid I2C0 SCL; need not be adjacent to SDA
constexpr uint8_t PIN_INT_ACC = 22;
constexpr uint8_t PIN_INT_GYR = 23;
constexpr uint8_t ACC_ADDRESS = 0x18; // supplied sketch; SDO1 high uses 0x19
constexpr uint8_t GYR_ADDRESS = 0x69;
// Keep bus clock/timeout unchanged for the first A/B test. If false works,
// set true and cold-power-cycle once to isolate the gyro reset transaction.
constexpr bool RESET_GYRO_ON_STARTUP = false;
constexpr BMI088::SyncRate SYNC_RATE = BMI088::SyncRate::Hz1000;
constexpr uint32_t SYNC_HZ = static_cast<uint16_t>(SYNC_RATE);
constexpr uint32_t PERIOD_US = 1000000UL / SYNC_HZ;
constexpr uint32_t STALE_US = 5 * PERIOD_US;

BMI088 bmi088(Wire, ACC_ADDRESS, GYR_ADDRESS);
volatile uint32_t accIrqs = 0, gyrIrqs = 0;
volatile uint32_t accEdgeUs = 0;

// SRAM callback bodies. Arduino core GPIO dispatch overhead still exists.
void __not_in_flash_func(onAccReady)() {
    accEdgeUs = time_us_32();
    ++accIrqs;
}
void __not_in_flash_func(onGyroReady)() { ++gyrIrqs; }

struct IrqSnapshot { uint32_t acc, gyr, edgeUs; };
// Explicit prototype also avoids Arduino auto-prototype/custom-type ordering.
IrqSnapshot snapshotIrqs();
IrqSnapshot snapshotIrqs() {
    const uint32_t irqState = save_and_disable_interrupts();
    IrqSnapshot s{accIrqs, gyrIrqs, accEdgeUs};
    restore_interrupts(irqState);
    return s;
}

BMI088::Sample latest{NAN, NAN, NAN, NAN, NAN, NAN};
uint32_t consumed = 0, accepted = 0, skipped = 0, late = 0;
uint32_t crossed = 0, readFails = 0, maxReadUs = 0, latestEdgeUs = 0;
uint32_t lastReportMs = 0, prevAcc = 0, prevGyr = 0, prevAccepted = 0;
float temperatureC = NAN;
bool haveSample = false;

void setup() {
    Serial.begin(115200);
    const uint32_t waitStart = millis();
    while (!Serial && millis() - waitStart < 3000) delay(10);
    Serial.println("\nCURIO BMI088 I2C IRQ + DATA SYNC v1.2 (RP2354A, GYRO RESET ISOLATION)");
    Serial.println("INT1->GPIO22; INT3->GPIO23 + INT2(INPUT)");
    // On the RP2354A this is hardware I2C0, not software bit-banging.
    if (!Wire.setSDA(I2C0_SDA) || !Wire.setSCL(I2C0_SCL)) {
        Serial.println("ERROR: invalid I2C0 pin mapping for selected board.");
        while (true) delay(1000);
    }
    Wire.begin();
    Wire.setClock(400000); // BMI088 I2C specified maximum, do not use 1 MHz
    Wire.setTimeout(3);   // Arduino-Pico Wire timeout, milliseconds
    pinMode(PIN_INT_ACC, INPUT);
    pinMode(PIN_INT_GYR, INPUT);

    Serial.print("I2C pre-init: SDA="); Serial.print(digitalRead(I2C0_SDA));
    Serial.print(" SCL="); Serial.println(digitalRead(I2C0_SCL));
    if (!bmi088.beginSync(SYNC_RATE, &Serial, RESET_GYRO_ON_STARTUP)) {
        const int sdaAtFailure = digitalRead(I2C0_SDA);
        const int sclAtFailure = digitalRead(I2C0_SCL);
        Serial.print("INIT FAILED at "); Serial.print(bmi088.lastStage());
        Serial.print(" error="); Serial.println(bmi088.lastError());
        bmi088.printLastBusError(Serial);
        Serial.print("I2C after failure: SDA="); Serial.print(sdaAtFailure);
        Serial.print(" SCL="); Serial.println(sclAtFailure);
        Serial.println("Arduino-Pico 5.6.0: WireStatus=5 means TIMEOUT (not a decoded NACK).");
        Serial.println("Initialization stopped. Copy the complete log above; reset to retry.");
        while (true) delay(1000);
    }
    bmi088.printDiagnostics(Serial);
    Serial.print("Sync output Hz="); Serial.println(SYNC_HZ);
    Serial.println("A/G/OK=Hz; skip/late/cross/fail/bus=cumulative; us=max read time; fresh=1 means recent accepted sample.");
    Serial.println("a=m/s^2 (gravity included), g=deg/s, T=C. No calibration, no motors.");

    // Attach after slow startup diagnostics, then discard the warm-up interval.
    attachInterrupt(digitalPinToInterrupt(PIN_INT_ACC), onAccReady, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_INT_GYR), onGyroReady, RISING);
    delay(100);
    const IrqSnapshot first = snapshotIrqs();
    consumed = prevAcc = first.acc;
    prevGyr = first.gyr;
    lastReportMs = millis();
}

void loop() {
    const IrqSnapshot before = snapshotIrqs();
    const uint32_t pending = before.acc - consumed; // unsigned wrap-safe
    if (pending) {
        consumed = before.acc;
        // Registers contain only latest data; pending events are NOT a FIFO.
        if (pending > 1) skipped += pending - 1;
        const uint32_t readStart = time_us_32();
        if (readStart - before.edgeUs > PERIOD_US / 4) {
            ++late; // delayed service: wait for the next synchronized edge
        } else {
            BMI088::Sample sample;
            const bool ok = bmi088.readSynchronized(sample);
            const uint32_t elapsed = time_us_32() - readStart;
            if (elapsed > maxReadUs) maxReadUs = elapsed;
            const IrqSnapshot after = snapshotIrqs();
            if (!ok) {
                ++readFails;
            } else if (after.acc != before.acc || after.gyr != before.gyr ||
                       time_us_32() - before.edgeUs >= PERIOD_US) {
                ++crossed; // an update may have split the three I2C reads
            } else {
                latest = sample;
                latestEdgeUs = before.edgeUs;
                haveSample = true;
                ++accepted;
                // Put calibration / estimator work HERE, only on an accepted sample.
                // No Wire or sensor access from setup1()/loop1().
            }
        }
    }

    // Telemetry is decimated. Never delay(50) in the acquisition path.
    const uint32_t nowMs = millis();
    if (nowMs - lastReportMs >= 1000) {
        const uint32_t intervalMs = nowMs - lastReportMs;
        const IrqSnapshot stats = snapshotIrqs();
        const float aHz = (stats.acc - prevAcc) * 1000.0f / intervalMs;
        const float gHz = (stats.gyr - prevGyr) * 1000.0f / intervalMs;
        const float okHz = (accepted - prevAccepted) * 1000.0f / intervalMs;
        prevAcc = stats.acc; prevGyr = stats.gyr; prevAccepted = accepted;
        lastReportMs = nowMs;
        // Slow auxiliary read; failure never publishes old temperature as new.
        if (!bmi088.readTemperature(temperatureC)) temperatureC = NAN;
        const bool fresh = haveSample && time_us_32() - latestEdgeUs <= STALE_US;
        char line[256];
        const int n = snprintf(line, sizeof(line),
            "A=%.0f G=%.0f OK=%.0f skip=%lu late=%lu cross=%lu fail=%lu bus=%lu us=%lu fresh=%u a=%.2f,%.2f,%.2f g=%.2f,%.2f,%.2f T=%.2f\n",
            aHz, gHz, okHz, (unsigned long)skipped, (unsigned long)late,
            (unsigned long)crossed, (unsigned long)readFails,
            (unsigned long)bmi088.i2cErrors(), (unsigned long)maxReadUs, (unsigned)fresh,
            fresh ? latest.ax : NAN, fresh ? latest.ay : NAN, fresh ? latest.az : NAN,
            fresh ? latest.gx : NAN, fresh ? latest.gy : NAN, fresh ? latest.gz : NAN, temperatureC);
        // Drop a report if USB is disconnected/full; do not wait for a host.
        if (n > 0 && n < (int)sizeof(line) && Serial && Serial.availableForWrite() >= n)
            Serial.write(reinterpret_cast<const uint8_t *>(line), (size_t)n);
    }
}
