#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include "esp_camera.h"

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
face_detection_result_t* detect_faces(camera_fb_t* fb);
void free_detection_result(face_detection_result_t* result);
void face_detection_task(void *pvParameter);

#ifdef __cplusplus
}
#endif

#endif // FACE_DETECTION_H