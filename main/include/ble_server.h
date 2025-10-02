#pragma once

#include <stdint.h>
#include "host/ble_hs.h"  // Add this for os_mbuf


#ifdef __cplusplus
extern "C" {
#endif

// Starts a FreeRTOS task that brings up the NimBLE host + advertises.
// 'device_name' becomes the GAP complete name shown in nRF Connect.
void ble_server_start(const char *device_name);
void ble_send_face_notification(uint32_t frame_id, uint8_t face_count, 
                                uint16_t x, uint16_t y, uint16_t width, uint16_t height, float confidence);
// In ble_server.h - add this declaration:
void ble_send_face_notification_and_crop(uint32_t frame_id, uint8_t face_count,
                                         uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                         float confidence,
                                         const uint8_t *rgb565, uint32_t rgb_len);

#ifdef __cplusplus
}
#endif