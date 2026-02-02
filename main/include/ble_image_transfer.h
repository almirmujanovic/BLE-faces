#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Service + characteristics (NUS-like layout for familiarity)

#define BLE_IMG_RX_UUID128    0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e,0x00,0x00
#define BLE_IMG_SVC_UUID128   0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x00,0x00,0x40,0x6e,0x00,0x00
#define BLE_IMG_INFO_UUID128  0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e,0x00,0x00
#define BLE_IMG_DATA_UUID128  0x3e,0xca,0x4d,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x04,0x00,0x40,0x6e,0x00,0x00


#define EVENT_TYPE_FACE_DETECTION   0x00
#define EVENT_TYPE_PHOTO_TAKEN      0x10
#define EVENT_TYPE_VIDEO_FRAME      0x20
#define IMG_FORMAT_RGB565           0x01

typedef enum {
    BLE_IMG_FMT_RGB565 = 0,
    BLE_IMG_FMT_JPEG   = 1,
} ble_img_format_t;

#pragma pack(push,1)
typedef struct {
    uint8_t  version;      // =1 1
    uint8_t  format;       // ble_img_format_t1
    uint16_t width;        // crop width2
    uint16_t height;       // crop height2
    uint16_t x;            // ROI origin (optional)2
    uint16_t y;//2
    uint32_t total_len;    // # of bytes that will follow on DATA4
    uint32_t frame_id;     // increments per crop4
    uint32_t crc32;        // optional (0 if unused)
} ble_img_info_t;
#pragma pack(pop)

// Lifecycle / plumbing 
void ble_img_xfer_init(void);                       // registers the GATT service
void ble_img_xfer_on_connect(uint16_t conn);        // set MTU/DLE/PHY requests
void ble_img_xfer_on_disconnect(void);
void ble_img_xfer_on_subscribe(uint16_t attr_handle, bool notify_enabled);
void ble_img_xfer_on_notify_tx(void);               // resume sending when TX frees
void ble_img_xfer_resolve_handles(void);
// Send one frame (metadata + chunked payload).
// Ownership: on success, the transfer takes ownership of `data` and will free it.
//            `data` must be heap-allocated (PSRAM ok) and must not be freed by caller.
//            On failure, the caller retains ownership.
bool ble_img_xfer_send_frame(const ble_img_info_t *info,
                             const uint8_t *data, uint32_t len);

// Convenience for RGB565 crops (same ownership rules as ble_img_xfer_send_frame)
bool ble_img_xfer_send_rgb565(uint32_t frame_id,
                              uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              const uint8_t *rgb565, uint32_t len);

#ifdef __cplusplus
}
#endif
