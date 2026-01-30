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

// Preload ML model to avoid lazy-loading conflicts with HTTP server
esp_err_t hand_gesture_ml_preload_model(void);

// Unload ML models to free RAM for WiFi (call before WiFi starts)
esp_err_t hand_gesture_ml_unload_models(void);
// Request unload from the gesture task context and wait for completion
esp_err_t hand_gesture_ml_request_unload_and_wait(uint32_t timeout_ms);

// Reload ML models after WiFi stops (call after WiFi stops)
esp_err_t hand_gesture_ml_reload_models(void);

#ifdef __cplusplus
}
#endif

#endif // HAND_GESTURE_ML_H
