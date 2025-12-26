#ifndef HAND_GESTURE_ML_H
#define HAND_GESTURE_ML_H

#include "esp_err.h"
#include "esp_camera.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t hand_gesture_ml_init(void);
void hand_gesture_ml_deinit(void);
bool hand_gesture_ml_is_ready(void);
void hand_gesture_ml_process_frame(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif

#endif // HAND_GESTURE_ML_H
