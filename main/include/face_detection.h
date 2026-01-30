#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include "esp_camera.h"

#ifdef __cplusplus
#include <list>
#include "dl_detect_define.hpp"
#endif

typedef struct {
    int x;
    int y;
    int width;
    int height;
    float confidence;
} face_box_t;

typedef struct {
    face_box_t* boxes;
    int count;
} face_detection_result_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t init_face_detection(void);
void face_detection_deinit(void);
face_detection_result_t* detect_faces(camera_fb_t* fb);
void free_detection_result(face_detection_result_t* result);
void face_detection_task(void *pvParameter);

// NEW: Image processing functions (called from separate task)
void queue_image_processing_request(uint32_t frame_id, uint16_t x, uint16_t y, 
                                   uint16_t w, uint16_t h, float confidence,
                                   const uint8_t* image_data, size_t image_len,
                                   uint16_t img_width, uint16_t img_height);

void process_face_crop_and_send(uint32_t frame_id, uint16_t x, uint16_t y, 
                               uint16_t w, uint16_t h, float confidence,
                               const uint8_t* image_data, size_t image_len,
                               uint16_t img_width, uint16_t img_height);

bool crop_rgb565_region_safe(const uint8_t* src, uint16_t src_w, uint16_t src_h,
                            uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            uint8_t* dst);

#ifdef __cplusplus
}

// C++ only function
std::list<dl::detect::result_t> detect_faces(const uint8_t *image_buf, int height, int width, pixformat_t format);
#endif

#endif // FACE_DETECTION_H
