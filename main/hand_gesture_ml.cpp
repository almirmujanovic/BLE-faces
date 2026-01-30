#include <vector>
#include <string.h>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "dl_image_define.hpp"
#include "hand_detect.hpp"
#include "hand_gesture_recognition.hpp"
#include "include/hand_gesture_ml.h"
#include "include/media.h"
#include "include/system_state.h"
#include "include/leds.h"
#include "include/wifi_control.h"
#include "include/ble_hid.h"

static const char *TAG = "hand_gesture_recognition";

static HandDetect *s_hand_detect = nullptr;
static HandGestureRecognizer *s_gesture_recognizer = nullptr;
static uint32_t s_frame_count = 0;
static SemaphoreHandle_t s_ml_lock = nullptr;

typedef enum {
    HAND_ACTION_NONE = 0,
    HAND_ACTION_PHOTO,
    HAND_ACTION_VIDEO,
    HAND_ACTION_HID_STOP_PAUSE,
    HAND_ACTION_HID_NEXT,
    HAND_ACTION_HID_PREV,
    HAND_ACTION_HID_VOL_UP,
    HAND_ACTION_HID_VOL_DOWN,
    HAND_ACTION_EMERGENCY,
} hand_action_t;

static const char *kGesturePhoto = "ok";
static const char *kGestureVideo = "one";
static const char *kGestureEmergency = "five";
static const char *kGestureStopPause = "two";
static const char *kGestureNext = "three";
static const char *kGesturePrev = "four";
static const char *kGestureVolUp = "like";
static const char *kGestureVolDown = "dislike";
static const char *kGestureNone = "no_gesture";
static const char *kHandNone = "no_hand";
static const float kGestureScoreThreshold = 0.40f;  // Lowered to 0.40 for easier recognition
static const uint32_t kGestureStableFrames = 2;     // Keep at 2 for balance
static const int64_t kPhotoCooldownUs = 2 * 1000000;
static const int64_t kVideoCooldownUs = 12 * 1000000;
static const int64_t kHidCooldownUs = 500 * 1000;
static const int64_t kEmergencyCooldownUs = 5 * 1000000;

static const char *s_stable_label = nullptr;
static uint32_t s_stable_count = 0;
static int64_t s_last_photo_us = 0;
static int64_t s_last_video_us = 0;
static int64_t s_last_hid_us = 0;
static int64_t s_last_emergency_us = 0;
static QueueHandle_t s_capture_queue = nullptr;
static TaskHandle_t s_capture_task = nullptr;
static uint32_t s_stable_hand_count = 0;  // Track hand stability
static bool s_hand_present = false;
static bool s_wifi_blocked_logged = false;
static volatile bool s_unload_requested = false;
static volatile bool s_unload_done = false;

typedef struct {
    hand_action_t action;
} capture_request_t;

static void capture_task(void *pv)
{
    (void)pv;
    capture_request_t req;
    for (;;) {
        if (xQueueReceive(s_capture_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (system_is_idle()) {
            ESP_LOGW(TAG, "Capture ignored (IDLE)");
            continue;
        }
        if (req.action == HAND_ACTION_PHOTO) {
            media_set_camera_busy(true);
            camera_fb_t *cap = esp_camera_fb_get();
            if (!cap) {
                media_set_camera_busy(false);
                ESP_LOGW(TAG, "Photo capture failed: no frame");
                continue;
            }
            esp_err_t ret = media_save_photo(cap);
            esp_camera_fb_return(cap);
            media_set_camera_busy(false);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Photo capture failed: %s", esp_err_to_name(ret));
            }
        } else if (req.action == HAND_ACTION_VIDEO) {
            media_set_camera_busy(true);
            esp_err_t ret = media_capture_jpeg_burst(10000, 10, 120);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Video capture failed: %s", esp_err_to_name(ret));
            }
            media_set_camera_busy(false);
        }
    }
}

static bool is_label(const char *label, const char *target)
{
    return label && target && (strcmp(label, target) == 0);
}

static bool is_no_gesture_label(const char *label)
{
    return is_label(label, kGestureNone) || is_label(label, kHandNone);
}

esp_err_t hand_gesture_ml_init(void)
{
    if (!s_ml_lock) {
        s_ml_lock = xSemaphoreCreateMutex();
        if (!s_ml_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    xSemaphoreTake(s_ml_lock, portMAX_DELAY);
    if (s_hand_detect || s_gesture_recognizer) {
        xSemaphoreGive(s_ml_lock);
        return ESP_OK;
    }

    s_hand_detect = new HandDetect();
    s_hand_detect->set_score_thr(0.02f);  // Very permissive: 0.02 (seeing scores of 0.047)
    s_hand_detect->set_nms_thr(0.45f);    // Keep moderate

    s_gesture_recognizer = new HandGestureRecognizer(HandGestureCls::MOBILENETV2_0_5_S8_V1);
    s_frame_count = 0;

    // Advanced camera optimization for hand detection
    ESP_LOGI(TAG, "Optimizing camera for hand detection...");
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Exposure and gain
        s->set_ae_level(s, 0);        // Normal exposure
        s->set_aec2(s, 1);            // Enable AEC DSP for better exposure
        s->set_gain_ctrl(s, 1);       // Enable auto gain
        s->set_gainceiling(s, (gainceiling_t)4);  // Moderate gain ceiling
        
        // Color and white balance
        s->set_wb_mode(s, 0);         // Auto white balance
        s->set_whitebal(s, 1);        // Enable white balance
        s->set_awb_gain(s, 1);        // Auto white balance gain
        s->set_saturation(s, 0);      // Normal saturation
        
        // Image quality
        s->set_contrast(s, 0);        // Normal contrast
        s->set_sharpness(s, 0);       // Normal sharpness
        s->set_denoise(s, 0);         // Disable denoise (can blur hands)
        s->set_special_effect(s, 0);  // No special effects
        
        // Pixel correction
        s->set_bpc(s, 0);             // Disable black pixel cancel
        s->set_wpc(s, 1);             // Enable white pixel cancel
        
        ESP_LOGI(TAG, "✓ Advanced camera tuning applied");
    } else {
        ESP_LOGW(TAG, "Could not access camera sensor for optimization");
    }

    if (!s_capture_queue) {
        s_capture_queue = xQueueCreate(2, sizeof(capture_request_t));
        if (!s_capture_queue) {
            ESP_LOGE(TAG, "Failed to create capture queue");
        }
    }
    if (s_capture_queue && !s_capture_task) {
        if (xTaskCreate(capture_task, "capture_task", 4096, NULL, 4, &s_capture_task) != pdPASS) {
            s_capture_task = nullptr;
            ESP_LOGE(TAG, "Failed to create capture task");
        }
    }

    esp_err_t ret = (s_hand_detect && s_gesture_recognizer) ? ESP_OK : ESP_FAIL;
    if (ret == ESP_OK) {
        // Preload models now to avoid lazy-loading crashes during unload
        esp_err_t preload = hand_gesture_ml_preload_model();
        if (preload != ESP_OK) {
            ESP_LOGW(TAG, "Model preload failed: %s", esp_err_to_name(preload));
        }
    }
    xSemaphoreGive(s_ml_lock);
    return ret;
}

void hand_gesture_ml_deinit(void)
{
    if (s_ml_lock) {
        xSemaphoreTake(s_ml_lock, portMAX_DELAY);
    }

    delete s_gesture_recognizer;
    s_gesture_recognizer = nullptr;
    delete s_hand_detect;
    s_hand_detect = nullptr;

    if (s_ml_lock) {
        xSemaphoreGive(s_ml_lock);
    }
}

// Unload ML models to free RAM (call before WiFi starts)
esp_err_t hand_gesture_ml_unload_models(void)
{
    if (s_ml_lock) {
        xSemaphoreTake(s_ml_lock, portMAX_DELAY);
    }

    ESP_LOGI(TAG, "=== Unloading ML models to free RAM for WiFi ===");
    ESP_LOGI(TAG, "Memory BEFORE unload - Internal: %u, largest: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    if (s_gesture_recognizer) {
        delete s_gesture_recognizer;
        s_gesture_recognizer = nullptr;
        ESP_LOGI(TAG, "✓ Gesture classifier unloaded");
    }
    
    if (s_hand_detect) {
        delete s_hand_detect;
        s_hand_detect = nullptr;
        ESP_LOGI(TAG, "✓ Hand detector unloaded");
    }

    ESP_LOGI(TAG, "Memory AFTER unload - Internal: %u, largest: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "=== ML models unloaded - RAM freed for WiFi ===");

    if (s_ml_lock) {
        xSemaphoreGive(s_ml_lock);
    }

    return ESP_OK;
}

esp_err_t hand_gesture_ml_request_unload_and_wait(uint32_t timeout_ms)
{
    s_unload_done = false;
    s_unload_requested = true;

    const uint32_t wait_step_ms = 50;
    uint32_t waited_ms = 0;
    while (!s_unload_done && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(wait_step_ms));
        waited_ms += wait_step_ms;
    }

    return s_unload_done ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Reload ML models (call after WiFi stops)
esp_err_t hand_gesture_ml_reload_models(void)
{
    ESP_LOGI(TAG, "=== Reloading ML models after WiFi stopped ===");
    ESP_LOGI(TAG, "Memory BEFORE reload - Internal: %u, largest: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    if (s_ml_lock) {
        xSemaphoreTake(s_ml_lock, portMAX_DELAY);
    }

    // Recreate models if not already created
    if (!s_hand_detect) {
        s_hand_detect = new HandDetect();
        s_hand_detect->set_score_thr(0.02f);
        s_hand_detect->set_nms_thr(0.45f);
        ESP_LOGI(TAG, "✓ Hand detector recreated");
    }
    
    if (!s_gesture_recognizer) {
        s_gesture_recognizer = new HandGestureRecognizer(HandGestureCls::MOBILENETV2_0_5_S8_V1);
        ESP_LOGI(TAG, "✓ Gesture classifier recreated");
    }

    if (s_ml_lock) {
        xSemaphoreGive(s_ml_lock);
    }

    // Force preload both models so they're ready to use
    esp_err_t ret = hand_gesture_ml_preload_model();
    
    ESP_LOGI(TAG, "Memory AFTER reload - Internal: %u, largest: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "=== ML models reloaded and ready ===");

    return ret;
}

bool hand_gesture_ml_is_ready(void)
{
    return (s_hand_detect != nullptr) && (s_gesture_recognizer != nullptr);
}

void hand_gesture_ml_process_frame(camera_fb_t *fb)
{
    if (!fb) {
        return;
    }

    if (s_unload_requested) {
        system_mark_task_paused(SYSTEM_TASK_GESTURE, true);
        hand_gesture_ml_unload_models();
        s_unload_requested = false;
        s_unload_done = true;
        return;
    }

    // Allow WiFi start to pause gesture processing safely before model unload
    if (system_is_pause_for_wifi()) {
        system_mark_task_paused(SYSTEM_TASK_GESTURE, true);
        return;
    }
    system_mark_task_paused(SYSTEM_TASK_GESTURE, false);

    // Disable hand gestures when WiFi is active (prevents memory conflicts/crashes)
    // Gestures automatically resume when WiFi is turned off via button
    if (wifi_control_is_running()) {
        if (!s_wifi_blocked_logged) {
            ESP_LOGW(TAG, "Hand gestures paused while WiFi is active");
            s_wifi_blocked_logged = true;
        }
        s_stable_hand_count = 0;
        s_stable_label = nullptr;
        s_stable_count = 0;
        s_hand_present = false;
        return;
    }
    if (s_wifi_blocked_logged) {
        ESP_LOGI(TAG, "Hand gestures resumed");
        s_wifi_blocked_logged = false;
    }

    if (s_ml_lock) {
        if (xSemaphoreTake(s_ml_lock, pdMS_TO_TICKS(2000)) != pdTRUE) {
            return;
        }
    }

    if (!hand_gesture_ml_is_ready()) {
        if (s_ml_lock) {
            xSemaphoreGive(s_ml_lock);
        }
        return;
    }

    s_frame_count++;

    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGE(TAG, "Unexpected pixel format: %d", fb->format);
        if (s_ml_lock) {
            xSemaphoreGive(s_ml_lock);
        }
        return;
    }

    if (fb->len < (fb->width * fb->height * 2)) {
        ESP_LOGW(TAG,
                 "Short frame: len=%u expected>=%u",
                 static_cast<unsigned>(fb->len),
                 static_cast<unsigned>(fb->width * fb->height * 2));
    }

    dl::image::img_t img = {
        .data = fb->buf,
        .width = static_cast<uint16_t>(fb->width),
        .height = static_cast<uint16_t>(fb->height),
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565,
    };
    auto &detections = s_hand_detect->run(img);

    if (detections.empty()) {
        s_stable_hand_count = 0;  // Reset stability counter
        s_hand_present = false;
        if (s_ml_lock) {
            xSemaphoreGive(s_ml_lock);
        }
        return;
    }

    if (!s_hand_present) {
        s_hand_present = true;
        ESP_LOGI(TAG, "Hand detected (%u)", static_cast<unsigned>(detections.size()));
        leds_blink_idle(1, 80);
    }

    // Check hand stability before gesture recognition (reduced to 1 frame for faster response)
    s_stable_hand_count++;
    if (s_stable_hand_count < 1) {
        // Hand just appeared, wait for stability (now just 1 frame)
        if (s_ml_lock) {
            xSemaphoreGive(s_ml_lock);
        }
        return;
    }

    std::vector<dl::cls::result_t> results =
        s_gesture_recognizer->recognize(img, detections);

    hand_action_t action = HAND_ACTION_NONE;
    const dl::cls::result_t *best = nullptr;
    for (const auto &res : results) {
        if (!best || res.score > best->score) {
            best = &res;
        }
    }

    if (best && best->cat_name) {
        if (best->score < kGestureScoreThreshold || is_no_gesture_label(best->cat_name)) {
            s_stable_label = nullptr;
            s_stable_count = 0;
        } else {
            if (s_stable_label && strcmp(best->cat_name, s_stable_label) == 0) {
                s_stable_count++;
            } else {
                s_stable_label = best->cat_name;
                s_stable_count = 1;
            }

            if (s_stable_count >= kGestureStableFrames) {
                int64_t now_us = esp_timer_get_time();
                int five_count = 0;
                for (const auto &res : results) {
                    if (res.score >= kGestureScoreThreshold &&
                        is_label(res.cat_name, kGestureEmergency)) {
                        five_count++;
                    }
                }

                if (is_label(best->cat_name, kGestureEmergency) && five_count >= 2) {
                    if ((now_us - s_last_emergency_us) >= kEmergencyCooldownUs) {
                        s_last_emergency_us = now_us;
                        action = HAND_ACTION_EMERGENCY;
                        s_stable_count = 0;
                    }
                } else if (is_label(best->cat_name, kGesturePhoto)) {
                    if ((now_us - s_last_photo_us) >= kPhotoCooldownUs) {
                        s_last_photo_us = now_us;
                        action = HAND_ACTION_PHOTO;
                        s_stable_count = 0;
                    }
                } else if (is_label(best->cat_name, kGestureVideo)) {
                    if ((now_us - s_last_video_us) >= kVideoCooldownUs) {
                        s_last_video_us = now_us;
                        action = HAND_ACTION_VIDEO;
                        s_stable_count = 0;
                    }
                } else if ((now_us - s_last_hid_us) >= kHidCooldownUs) {
                    if (is_label(best->cat_name, kGestureStopPause)) {
                        s_last_hid_us = now_us;
                        action = HAND_ACTION_HID_STOP_PAUSE;
                        s_stable_count = 0;
                    } else if (is_label(best->cat_name, kGestureNext)) {
                        s_last_hid_us = now_us;
                        action = HAND_ACTION_HID_NEXT;
                        s_stable_count = 0;
                    } else if (is_label(best->cat_name, kGesturePrev)) {
                        s_last_hid_us = now_us;
                        action = HAND_ACTION_HID_PREV;
                        s_stable_count = 0;
                    } else if (is_label(best->cat_name, kGestureVolUp)) {
                        s_last_hid_us = now_us;
                        action = HAND_ACTION_HID_VOL_UP;
                        s_stable_count = 0;
                    } else if (is_label(best->cat_name, kGestureVolDown)) {
                        s_last_hid_us = now_us;
                        action = HAND_ACTION_HID_VOL_DOWN;
                        s_stable_count = 0;
                    }
                }
            }
        }
    } else {
        s_stable_label = nullptr;
        s_stable_count = 0;
    }

    if (s_ml_lock) {
        xSemaphoreGive(s_ml_lock);
    }
    if (action == HAND_ACTION_NONE) {
        return;
    }

    leds_blink_both(1, 80);

    if (action == HAND_ACTION_EMERGENCY) {
        ESP_LOGW(TAG, "Emergency gesture detected (two-hand five) - placeholder");
        return;
    }

    if (action == HAND_ACTION_HID_STOP_PAUSE ||
        action == HAND_ACTION_HID_NEXT ||
        action == HAND_ACTION_HID_PREV ||
        action == HAND_ACTION_HID_VOL_UP ||
        action == HAND_ACTION_HID_VOL_DOWN) {
        if (!ble_hid_is_ready()) {
            ESP_LOGW(TAG, "HID not ready; gesture ignored");
            return;
        }
        esp_err_t rc = ESP_OK;
        if (action == HAND_ACTION_HID_STOP_PAUSE) {
            rc = ble_hid_cc_tap_play_pause();
            ESP_LOGI(TAG, "Gesture: two -> play/pause");
        } else if (action == HAND_ACTION_HID_NEXT) {
            rc = ble_hid_cc_tap_next();
            ESP_LOGI(TAG, "Gesture: three -> next");
        } else if (action == HAND_ACTION_HID_PREV) {
            rc = ble_hid_cc_tap_prev();
            ESP_LOGI(TAG, "Gesture: four -> previous");
        } else if (action == HAND_ACTION_HID_VOL_UP) {
            rc = ble_hid_cc_tap_vol_up();
            ESP_LOGI(TAG, "Gesture: like -> volume up");
        } else if (action == HAND_ACTION_HID_VOL_DOWN) {
            rc = ble_hid_cc_tap_vol_down();
            ESP_LOGI(TAG, "Gesture: dislike -> volume down");
        }
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "HID action failed: %s", esp_err_to_name(rc));
        }
        return;
    }
    if (!s_capture_queue) {
        ESP_LOGW(TAG, "Capture queue unavailable");
        return;
    }
    if (system_is_idle()) {
        ESP_LOGW(TAG, "Capture ignored (IDLE)");
        return;
    }
    if (media_is_camera_busy()) {
        ESP_LOGW(TAG, "Capture ignored (camera busy)");
        return;
    }

    capture_request_t req = {
        .action = action,
    };
    if (xQueueSend(s_capture_queue, &req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Capture request dropped (queue full)");
        return;
    }
    if (action == HAND_ACTION_PHOTO) {
        ESP_LOGI(TAG, "Gesture: ok -> photo");
    } else if (action == HAND_ACTION_VIDEO) {
        ESP_LOGI(TAG, "Gesture: one -> video");
    }
}

// Preload BOTH ML models at boot to avoid lazy-loading during runtime
// This prevents memory conflicts with WiFi/HTTP server
esp_err_t hand_gesture_ml_preload_model(void)
{
    if (!s_hand_detect || !s_gesture_recognizer) {
        ESP_LOGE(TAG, "Hand gesture ML not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create a small dummy image (96x96 RGB565)
    const int width = 96;
    const int height = 96;
    const size_t img_size = width * height * 2; // RGB565 = 2 bytes per pixel
    
    uint8_t *dummy_buf = (uint8_t *)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!dummy_buf) {
        ESP_LOGE(TAG, "Failed to allocate dummy buffer for model preload");
        return ESP_ERR_NO_MEM;
    }

    // Fill with neutral gray pixels (RGB565: 0x8410 = gray)
    uint16_t *pixels = (uint16_t *)dummy_buf;
    for (int i = 0; i < width * height; i++) {
        pixels[i] = 0x8410;  // Gray color
    }

    dl::image::img_t img = {
        .data = dummy_buf,
        .width = static_cast<uint16_t>(width),
        .height = static_cast<uint16_t>(height),
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565,
    };

    // Preload HandDetect model
    std::list<dl::detect::result_t> detections = s_hand_detect->run(img);
    (void)detections;

    // Create fake detection covering center of image
    dl::detect::result_t fake_det{};
    fake_det.box = {width / 4, height / 4, 3 * width / 4, 3 * height / 4};
    fake_det.score = 0.99f;  // High confidence
    fake_det.category = 0;
    
    std::list<dl::detect::result_t> fake_detections;
    fake_detections.push_back(fake_det);
    
    // Force the gesture classifier to load by calling recognize()
    std::vector<dl::cls::result_t> results = s_gesture_recognizer->recognize(img, fake_detections);
    (void)results;

    heap_caps_free(dummy_buf);
    
    ESP_LOGI(TAG, "Hand gesture ML preload OK");
    return ESP_OK;
}
