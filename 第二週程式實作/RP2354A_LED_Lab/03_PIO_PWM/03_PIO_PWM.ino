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

#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/clocks.h"
// Seven-instruction PWM engine adapted from Raspberry Pi pico-examples.
// Copyright (c) 2020 Raspberry Pi (Trading) Ltd. BSD-3-Clause.
// See THIRD_PARTY_NOTICES.txt in this teaching package.
constexpr uint32_t PWM_TOP = 255;
constexpr uint32_t PWM_HZ = 1000;
constexpr uint32_t CYCLES_PER_PERIOD = 3 + 3 * (PWM_TOP + 1); // 771
PIO ledPio = pio0;
int sms[3] = {-1, -1, -1};
uint programOffset = 0;
bool pioReady = false;
uint16_t instructions[7];
pio_program_t ledProgram = {};

bool startPIO() {
  // .side_set 1 opt: 1 data bit + 1 optional-enable bit => count=2.
  instructions[0] = pio_encode_pull(false, false) | pio_encode_sideset_opt(1, 0);
  instructions[1] = pio_encode_mov(pio_x, pio_osr);
  instructions[2] = pio_encode_mov(pio_y, pio_isr);
  instructions[3] = pio_encode_jmp_x_ne_y(5);
  instructions[4] = pio_encode_jmp(6) | pio_encode_sideset_opt(1, 1);
  instructions[5] = pio_encode_nop();
  instructions[6] = pio_encode_jmp_y_dec(3);
  ledProgram.instructions = instructions;
  ledProgram.length = 7;
  ledProgram.origin = -1; // Relocatable; pio_add_program adjusts JMP addresses
  if (!pio_can_add_program(ledPio, &ledProgram)) return false;
  for (uint8_t i = 0; i < 3; ++i) {
    sms[i] = pio_claim_unused_sm(ledPio, false);
    if (sms[i] < 0) {
      for (uint8_t j = 0; j < i; ++j) pio_sm_unclaim(ledPio, sms[j]);
      return false;
    }
  }
  programOffset = pio_add_program(ledPio, &ledProgram);
  uint32_t enableMask = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    uint sm = (uint)sms[i];
    uint pin = LED_PINS[i];
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, programOffset, programOffset + 6);
    sm_config_set_sideset(&c, 2, true, false);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) /
                             (PWM_HZ * CYCLES_PER_PERIOD));
    pio_sm_init(ledPio, sm, programOffset, &c); // SM remains disabled
    pio_sm_set_pins_with_mask(ledPio, sm, 0, 1u << pin);
    pio_sm_set_consecutive_pindirs(ledPio, sm, pin, 1, true);
    pio_gpio_init(ledPio, pin); // GPIO output mux now belongs to PIO
    // ISR (Input Shift Register, NOT Interrupt Service Routine) stores TOP.
    pio_sm_put_blocking(ledPio, sm, PWM_TOP);
    pio_sm_exec(ledPio, sm, pio_encode_pull(false, true));
    pio_sm_exec(ledPio, sm, pio_encode_mov(pio_isr, pio_osr));
    // X=-1 cannot match Y=255..0, giving exact OFF on an empty FIFO.
    pio_sm_exec(ledPio, sm, pio_encode_mov_not(pio_x, pio_null));
    uint32_t d = brightness(0, OFFSETS[i], PWM_TOP);
    pio_sm_put_blocking(ledPio, sm, d == 0 ? 0xffffffffu : d);
    enableMask |= 1u << sm;
  }
  pio_enable_sm_mask_in_sync(ledPio, enableMask);
  return true;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  pioReady = startPIO();
}

void loop() {
  if (!pioReady) {
    if (Serial) Serial.println("ERROR: PIO0 needs 3 free SMs and 7 instructions");
    delay(1000);
    return;
  }
  static uint16_t frame = 0;
  static uint32_t lastFrameMs = millis();
  uint32_t now = millis();
  uint32_t steps = (uint32_t)(now - lastFrameMs) / FRAME_MS;
  if (steps == 0) return;
  lastFrameMs += steps * FRAME_MS;
  frame = (frame + steps % FRAME_COUNT) % FRAME_COUNT;
  for (uint8_t i = 0; i < 3; ++i) {
    uint32_t d = brightness(frame, OFFSETS[i], PWM_TOP);
    // 100 writes/s/channel; FIFO is drained about 1000 times/s/channel.
    pio_sm_put_blocking(ledPio, (uint)sms[i], d == 0 ? 0xffffffffu : d);
  }
  // CPU supplies the slow envelope; PIO autonomously repeats the last PWM.
  // No new FIFO data => pull noblock copies X into OSR (no stall).
}
