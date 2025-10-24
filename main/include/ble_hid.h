#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===== Public API =====
esp_err_t ble_hid_init(void);                   // register HID service (count+add only)
esp_err_t ble_hid_get_handles(void);            // resolve handles after ble_gatts_start()
void      ble_hid_set_conn(uint16_t conn_hdl);  // set/clear connection handle
void      ble_hid_check_cccd_subscribe(uint16_t attr_handle, uint16_t cur_notify);


/** Tap (press+release) standard Consumer Control keys */
esp_err_t ble_hid_cc_tap_vol_up(void);
esp_err_t ble_hid_cc_tap_vol_down(void);
esp_err_t ble_hid_cc_tap_mute(void);
esp_err_t ble_hid_cc_tap_play_pause(void);
esp_err_t ble_hid_cc_tap_next(void);
esp_err_t ble_hid_cc_tap_prev(void);
esp_err_t ble_hid_cc_tap_stop(void);
esp_err_t ble_hid_tap_consumer_bits(uint8_t bits);


// Demo task (optional, for dev)
static void demo_task(void *arg);
void ble_hid_start_demo_task(void);
#ifdef __cplusplus
}
#endif
