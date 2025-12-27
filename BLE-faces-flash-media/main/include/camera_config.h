#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include "esp_camera.h"

// SAME PINOUT AS YOUR WORKING EXAMPLE
#define CAM_PWDN_GPIO  9
#define CAM_RESET_GPIO 20
#define CAM_XCLK_GPIO  4

#define CAM_SIOD_GPIO  2
#define CAM_SIOC_GPIO  3

#define CAM_VSYNC_GPIO 7
#define CAM_HREF_GPIO  8
#define CAM_PCLK_GPIO  21

#define CAM_D0_GPIO    19
#define CAM_D1_GPIO    10
#define CAM_D2_GPIO    18
#define CAM_D3_GPIO    17
#define CAM_D4_GPIO    14
#define CAM_D5_GPIO    13
#define CAM_D6_GPIO    12
#define CAM_D7_GPIO    11

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t init_camera(void);

camera_fb_t *capture_frame(void);
void release_frame(camera_fb_t *fb);
void camera_discard_initial_frames(int count);

#ifdef __cplusplus
}
#endif
#endif
