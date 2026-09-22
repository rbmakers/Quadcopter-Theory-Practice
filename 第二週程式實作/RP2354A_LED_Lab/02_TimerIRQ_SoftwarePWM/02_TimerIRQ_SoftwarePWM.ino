#include <Arduino.h>

// GPIO numbers, not package-pin or board-header numbers.
// Active HIGH: GPIO -> resistor -> LED anode; LED cathode -> GND.
constexpr uint8_t LED_PINS[3] = {7, 8, 9}; // Blue, Red, Yellow
constexpr uint16_t OFFSETS[3] = {0, 256, 128};
constexpr uint16_t FRAME_COUNT = 384;
constexpr uint32_t FRAME_MS = 10; // One colour cycle = 3.84 s

// Triangle envelope + approximate gamma 2.0 correction.
// maximum=64 (software PWM) or 255 (PIO / hardware PWM).
uint16_t brightness(uint16_t frame, uint16_t offset, uint16_t maximum) {
  uint16_t p = (frame + offset) % FRAME_COUNT;
  uint16_t a = (p < 192) ? (192 - p) : (p - 192);
  return (uint32_t(a) * a * maximum + 18432u) / 36864u;
}

#include "pico/time.h"
#include "hardware/gpio.h"
constexpr uint32_t LED_MASK = (1u << 7) | (1u << 8) | (1u << 9);
constexpr uint8_t PWM_STEPS = 64;
constexpr int64_t SLOT_US = 50;
constexpr uint16_t TICKS_PER_FRAME = 200; // 200 * 50us = 10ms
uint8_t dutyTable[FRAME_COUNT][3]; // 1152 bytes; written before timer starts
repeating_timer_t ledTimer; // Must remain alive for the timer's entire lifetime
bool timerReady = false;

bool ledTimerCallback(repeating_timer_t *timer) {
  (void)timer;
  static uint8_t slot = 0;
  static uint16_t frame = 0;
  static uint16_t envelopeTicks = 0;
  static uint8_t activeDuty[3] = {0, 0, 0};
  if (slot == 0) { // Latch all duties only at a PWM-period boundary
    for (uint8_t i = 0; i < 3; ++i) activeDuty[i] = dutyTable[frame][i];
  }
  uint32_t bits = 0;
  for (uint8_t i = 0; i < 3; ++i)
    if (slot < activeDuty[i]) bits |= (1u << LED_PINS[i]);
  gpio_put_masked(LED_MASK, bits);
  slot = (slot + 1) % PWM_STEPS;
  if (++envelopeTicks == TICKS_PER_FRAME) {
    envelopeTicks = 0;
    frame = (frame + 1) % FRAME_COUNT;
  }
  return true; // false would stop this repeating timer
}

void setup() {
  Serial.begin(115200); // Never wait for the serial monitor
  for (uint8_t pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (uint16_t f = 0; f < FRAME_COUNT; ++f)
    for (uint8_t i = 0; i < 3; ++i)
      dutyTable[f][i] = brightness(f, OFFSETS[i], PWM_STEPS);
  // Negative period requests start-to-start scheduling (not end-to-start).
  timerReady = add_repeating_timer_us(-SLOT_US, ledTimerCallback,
                                      nullptr, &ledTimer);
}

void loop() {
  static uint32_t lastMessageMs = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastMessageMs) >= 1000) {
    lastMessageMs = now;
    if (Serial) Serial.println(timerReady ? "Timer LED running" :
                                           "ERROR: timer allocation failed");
  }
  // Even delay(2000) here would leave this IRQ-driven animation running.
  // Do not put Serial, delay(), allocation, or blocking I/O in the callback.
}
