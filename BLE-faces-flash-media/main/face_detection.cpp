#include "include/face_detection.h"
#include "human_face_detect.hpp"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "camera_config.h"

static const char *TAG = "FACE_DETECT";
static HumanFaceDetect *face_detector = nullptr;

//  detection functions 
extern "C" {

esp_err_t init_face_detection(void)
{
    face_detector = new HumanFaceDetect();
    
    if (!face_detector) {
        ESP_LOGE(TAG, "Failed to create face detector");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Face detection initialized");
    return ESP_OK;
}

face_detection_result_t* detect_faces(camera_fb_t* fb)
{
    if (!face_detector || !fb) {
        return nullptr;
    }

    dl::image::img_t img;
    img.data = fb->buf;
    img.width = (uint16_t)fb->width;
    img.height = (uint16_t)fb->height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

    // official API: run() method
    auto &detect_results = face_detector->run(img);
    
    // Convert to result format
    face_detection_result_t* result = (face_detection_result_t*)malloc(sizeof(face_detection_result_t));
    if (!result) {
        return nullptr;
    }
    
    result->count = detect_results.size();
    
    if (result->count > 0) {
        result->boxes = (face_box_t*)malloc(sizeof(face_box_t) * result->count);
        if (!result->boxes) {
            free(result);
            return nullptr;
        }
        
        int i = 0;
        for (const auto &res : detect_results) {
            ESP_LOGI(TAG, "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                     res.score, res.box[0], res.box[1], res.box[2], res.box[3]);
            
            result->boxes[i].x = res.box[0];
            result->boxes[i].y = res.box[1]; 
            result->boxes[i].width = res.box[2] - res.box[0];
            result->boxes[i].height = res.box[3] - res.box[1];
            result->boxes[i].confidence = res.score;
            i++;
        }
    } else {
        result->boxes = nullptr;
    }
    
    return result;
}

void free_detection_result(face_detection_result_t* result)
{
    if (!result) return;
    if(result->boxes) free(result->boxes);
    free(result);
}

// Ultra-lightweight face detection task (NO BLE calls)
void face_detection_task(void *pvParameter)
{
    uint32_t frame_id = 0;
    uint32_t capture_failures = 0;
    uint32_t faces_detected_total = 0;
    
    ESP_LOGI(TAG, "Face detection task started - waiting for camera stabilization...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    for (;;) {
        camera_fb_t *fb = capture_frame();
        if (!fb) { 
            capture_failures++;
            if (capture_failures % 10 == 1) {
                ESP_LOGE(TAG, "Camera capture failures: %lu", capture_failures); 
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue; 
        }

        // Quick validation
        if (fb->len == 0 || !fb->buf) {
            release_frame(fb);
            frame_id++;
            continue;
        }

        // LIGHTWEIGHT: Only run face detection (no BLE, no notifications)
        auto *result = detect_faces(fb);
        
        if (result && result->count > 0) {
            faces_detected_total += result->count;
            ESP_LOGI(TAG, "Frame %lu: Detected %d face(s)", frame_id, result->count);
            
            // Process only first face (keep it simple)
            const int rx = result->boxes[0].x;
            const int ry = result->boxes[0].y;
            const int rw = result->boxes[0].width;
            const int rh = result->boxes[0].height;
            const float conf = result->boxes[0].confidence;

            ESP_LOGI(TAG, "  Face 0: x=%d, y=%d, w=%d, h=%d, conf=%.3f", 
                     rx, ry, rw, rh, conf);

            // REMOVED: NO BLE calls here - just queue for processing
            // REMOVED: ble_send_face_notification(...);

            // ONLY queue image processing (no BLE stack usage)
            queue_image_processing_request(frame_id, rx, ry, rw, rh, conf,
                                         fb->buf, fb->len, fb->width, fb->height);
        }

        // Stats every 20 frames
        if (frame_id % 20 == 0 && frame_id > 0) {
            ESP_LOGI(TAG, "Stats: Frame %lu, Failures: %lu, Total faces: %lu", 
                     frame_id, capture_failures, faces_detected_total);
        }

        release_frame(fb);
        if (result) free_detection_result(result);
        
        frame_id++;
        vTaskDelay(pdMS_TO_TICKS(300)); // Slower to reduce load while BLE is active
    }
}

} // extern "C"