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

constexpr uint16_t PWM_MAX = 255;
constexpr uint32_t PWM_HZ = 1000;

void setup() {
  analogWriteFreq(PWM_HZ);
  analogWriteRange(PWM_MAX); // Core maps 0..255 to duty 0..100%
  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    analogWrite(LED_PINS[i], brightness(0, OFFSETS[i], PWM_MAX));
  }
}

void loop() {
  static uint16_t frame = 0;
  static uint32_t lastFrameMs = millis();
  uint32_t now = millis();
  uint32_t steps = (uint32_t)(now - lastFrameMs) / FRAME_MS;
  if (steps == 0) return;
  lastFrameMs += steps * FRAME_MS;
  frame = (frame + steps % FRAME_COUNT) % FRAME_COUNT;
  for (uint8_t i = 0; i < 3; ++i)
    analogWrite(LED_PINS[i], brightness(frame, OFFSETS[i], PWM_MAX));
  // Hardware keeps producing PWM between updates. Do not digitalWrite()
  // these pins while PWM should remain active: that changes the pin mux.
}
