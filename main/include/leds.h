#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t leds_init(void);
void leds_set(bool idle_led, bool capture_led); // legacy generic setter (idle, capture)
void leds_set_idle(bool on);
void leds_set_capture(bool on);
void leds_blink_idle(uint8_t times, uint32_t delay_ms);
void leds_blink_capture(uint8_t times, uint32_t delay_ms);
void leds_blink_both(uint8_t times, uint32_t delay_ms);
void leds_blink_alternate(uint8_t times, uint32_t delay_ms);

// Configure button pin (PCA9536 IO1) as input and read helpers
esp_err_t leds_configure_button_input(void);
esp_err_t leds_read_inputs(uint8_t *out_inputs);
bool leds_button_is_pressed(void);

#ifdef __cplusplus
}
#endif
