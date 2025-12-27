#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register HID service (count+add only). Safe to call before ble_gatts_start().
esp_err_t ble_hid_init(void);

// Resolve handles after ble_gatts_start() (e.g., from your on_sync()).
esp_err_t ble_hid_get_handles(void);

// Set/clear connection handle (call on connect/disconnect).
void      ble_hid_set_conn(uint16_t conn_hdl);

// Mirror GAP SUBSCRIBE event (NimBLE gives VALUE handle here).
void      ble_hid_check_cccd_subscribe(uint16_t attr_handle, uint16_t cur_notify);

// Optional: let callers know when it's safe to send (connected + CCCD enabled).
bool      ble_hid_is_ready(void);

// Bitmap sender (lower 7 bits valid per the report map). Enqueues safely.
esp_err_t ble_hid_tap_consumer_bits(uint16_t bits);

// Convenience one-bit taps (enqueue to worker; safe to call from any task).
esp_err_t ble_hid_cc_tap_vol_up(void);
esp_err_t ble_hid_cc_tap_vol_down(void);
esp_err_t ble_hid_cc_tap_mute(void);
esp_err_t ble_hid_cc_tap_play_pause(void);
esp_err_t ble_hid_cc_tap_next(void);
esp_err_t ble_hid_cc_tap_prev(void);
esp_err_t ble_hid_cc_tap_stop(void);

#ifdef __cplusplus
}
#endif
