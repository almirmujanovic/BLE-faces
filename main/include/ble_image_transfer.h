#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Service + characteristics (NUS-like layout for familiarity)
// Fixed: Use proper array notation for UUID128 definitions
#define BLE_IMG_SVC_UUID128   0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e,0x00,0x00
#define BLE_IMG_RX_UUID128    0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e,0x00,0x00
#define BLE_IMG_INFO_UUID128  0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e,0x00,0x00
#define BLE_IMG_DATA_UUID128  0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x04,0x00,0x40,0x6e,0x00,0x00

typedef enum {
    BLE_IMG_FMT_RGB565 = 0,
    BLE_IMG_FMT_JPEG   = 1,
} ble_img_format_t;

#pragma pack(push,1)
typedef struct {
    uint8_t  version;      // =1
    uint8_t  format;       // ble_img_format_t
    uint16_t width;        // crop width
    uint16_t height;       // crop height
    uint16_t x;            // ROI origin (optional)
    uint16_t y;
    uint32_t total_len;    // # of bytes that will follow on DATA
    uint32_t frame_id;     // increments per crop
    uint32_t crc32;        // optional (0 if unused)
} ble_img_info_t;
#pragma pack(pop)

// ---- Lifecycle / plumbing (call from your GAP events) ----
void ble_img_xfer_init(void);                       // registers the GATT service
void ble_img_xfer_on_connect(uint16_t conn);        // set MTU/DLE/PHY requests
void ble_img_xfer_on_disconnect(void);
void ble_img_xfer_on_subscribe(uint16_t attr_handle, bool notify_enabled);
void ble_img_xfer_on_notify_tx(void);               // resume sending when TX frees

// ---- Send one frame (metadata + chunked payload) ----
bool ble_img_xfer_send_frame(const ble_img_info_t *info,
                             const uint8_t *data, uint32_t len);

// Convenience for RGB565 crops
bool ble_img_xfer_send_rgb565(uint32_t frame_id,
                              uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              const uint8_t *rgb565, uint32_t len);

#ifdef __cplusplus
}
#endif