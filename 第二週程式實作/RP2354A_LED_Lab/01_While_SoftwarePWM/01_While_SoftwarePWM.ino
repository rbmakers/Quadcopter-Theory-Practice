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

#include "hardware/gpio.h"
constexpr uint32_t LED_MASK = (1u << 7) | (1u << 8) | (1u << 9);
constexpr uint8_t PWM_STEPS = 64;
constexpr uint32_t SLOT_US = 50; // Nominal PWM: 1/(64*50us)=312.5 Hz

void setup() {
  for (uint8_t pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void loop() {
  static uint16_t frame = 0;
  static uint32_t lastFrameMs = millis();
  uint32_t now = millis();
  uint32_t steps = (uint32_t)(now - lastFrameMs) / FRAME_MS;
  if (steps != 0) {
    lastFrameMs += steps * FRAME_MS;
    frame = (frame + steps % FRAME_COUNT) % FRAME_COUNT;
  }
  uint8_t duty[3];
  for (uint8_t i = 0; i < 3; ++i)
    duty[i] = brightness(frame, OFFSETS[i], PWM_STEPS);

  uint8_t slot = 0;
  while (slot < PWM_STEPS) { // Generate one complete PWM period in software
    uint32_t beginUs = micros();
    uint32_t bits = 0;
    for (uint8_t i = 0; i < 3; ++i)
      if (slot < duty[i]) bits |= (1u << LED_PINS[i]);
    gpio_put_masked(LED_MASK, bits); // Change only GPIO7/8/9
    while ((uint32_t)(micros() - beginUs) < SLOT_US) {
      // Busy wait: CPU cannot execute other foreground work here.
      // Interrupts remain enabled, so this timing can have jitter.
    }
    ++slot;
  }
  // Arduino calls loop() again. A long task here distorts the PWM waveform.
}
