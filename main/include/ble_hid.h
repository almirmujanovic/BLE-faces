#ifndef BLE_HID_H
#define BLE_HID_H

#include <stdint.h>
#include "esp_err.h"

// Consumer Control HID usage bits (bitfield positions in 16-bit report)
#define HID_CONSUMER_MUTE           (1 << 0)  // Bit 0
#define HID_CONSUMER_VOL_UP         (1 << 2)  // Bit 2
#define HID_CONSUMER_VOL_DOWN       (1 << 3)  // Bit 3
#define HID_CONSUMER_PLAY_PAUSE     (1 << 4)  // Bit 4
#define HID_CONSUMER_NEXT           (1 << 5)  // Bit 5
#define HID_CONSUMER_PREV           (1 << 6)  // Bit 6
#define HID_CONSUMER_STOP           (1 << 7)  // Bit 7

/**
 * @brief Initialize BLE HID service (adds to existing GATT server)
 */
esp_err_t ble_hid_init(void);

/**
 * @brief Set connection handle when device connects
 */
void ble_hid_set_conn(uint16_t conn_handle);

/**
 * @brief Get HID characteristic handles after GATT start
 */
esp_err_t ble_hid_get_handles(void);

/**
 * @brief Send consumer control HID report (press + release)
 * @param bits Bitfield of HID_CONSUMER_* flags
 */
esp_err_t ble_hid_tap_consumer_bits(uint16_t bits);

/**
 * @brief Check if client subscribed to HID notifications
 */
void ble_hid_check_cccd_subscribe(uint16_t attr_handle, uint16_t notify_enable);

#endif // BLE_HID_H