#ifndef MEDIA_H
#define MEDIA_H

#include "esp_err.h"
#include "esp_camera.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


esp_err_t media_storage_init(void);
esp_err_t media_get_storage_info(size_t *total_bytes, size_t *used_bytes, size_t *free_bytes);
esp_err_t media_list_files(void);
esp_err_t media_delete_oldest(void);
static void generate_burst_prefix(char *buf, size_t len);

esp_err_t media_save_photo(camera_fb_t *fb);

// camera busy helpers (for face_detection.cpp)
bool media_is_camera_busy(void);
void media_set_camera_busy(bool busy);

// burst “video” capture: 10s @ 20fps (or whatever you pass)
esp_err_t media_capture_jpeg_burst(uint32_t duration_ms,
                                   uint32_t target_fps,
                                   size_t   max_frames);

#ifdef __cplusplus
}
#endif

#endif // MEDIA_H