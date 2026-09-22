#include <Arduino.h>

constexpr uint8_t LED_PINS[3] = {7, 8, 9}; // Blue, Red, Yellow; active HIGH
// Bit 0=Blue, bit 1=Red, bit 2=Yellow. These are channel bits, NOT GPIO bits.
constexpr uint8_t PATTERN[] = {
  0b001, 0b011, 0b010, 0b110, 0b100, 0b101, 0b111, 0b000
};
constexpr uint32_t STEP_MS = 180;
constexpr uint8_t PATTERN_COUNT = sizeof(PATTERN) / sizeof(PATTERN[0]);
uint8_t phase = 0;
uint32_t lastStepMs = 0;

void showPattern(uint8_t bits) {
  for (uint8_t i = 0; i < 3; ++i)
    digitalWrite(LED_PINS[i], (bits & (1u << i)) ? HIGH : LOW);
}

void setup() {
  for (uint8_t pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  showPattern(PATTERN[phase]);
  lastStepMs = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t steps = (uint32_t)(now - lastStepMs) / STEP_MS;
  if (steps != 0) {
    lastStepMs += steps * STEP_MS;
    phase = (phase + steps % PATTERN_COUNT) % PATTERN_COUNT;
    showPattern(PATTERN[phase]);
  }
  // Other nonblocking work goes here: buttons, serial commands, sensors.
  // No delay(), no busy wait. This version intentionally uses ON/OFF only.
}
