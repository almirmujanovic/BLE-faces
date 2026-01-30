#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*button_cb_t)(void);

// start polling button; click_cb invoked on short press, hold_cb invoked on long press
// polling_ms: poll interval (e.g. 50)
// click_max_ms: max duration considered a click (e.g. 500)
// hold_min_ms: duration to consider a hold (e.g. 2000)
esp_err_t button_start(button_cb_t click_cb, button_cb_t hold_cb,
                       uint32_t polling_ms, uint32_t click_max_ms, uint32_t hold_min_ms);

// stop polling
void button_stop(void);

#ifdef __cplusplus
}
#endif