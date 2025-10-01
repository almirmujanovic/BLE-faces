#include "include/face_detection.h"
#include "human_face_detect.hpp"
#include "esp_log.h"
#include <stdlib.h>
#include "camera_config.h"
#include "ble_server.h"

static const char *TAG = "FACE_DETECT";
static HumanFaceDetect *face_detector = nullptr;

extern "C" {

esp_err_t init_face_detection(void)
{
    // Create face detector (no parameters needed like in official example)
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

    // Convert camera frame to dl::image format - fix the initialization order
    dl::image::img_t img;
    img.data = fb->buf;
    img.width = (uint16_t)fb->width;
    img.height = (uint16_t)fb->height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

    // Use the official API: run() method
    auto &detect_results = face_detector->run(img);
    
    // Convert to your result format
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
            // Log detection like in official example
            ESP_LOGI(TAG, "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                     res.score, res.box[0], res.box[1], res.box[2], res.box[3]);
            
            // Convert to your format
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

// Main face detection task
// Main face detection task
void face_detection_task(void *pvParameter)
{
    uint32_t frame_id = 0;
    uint32_t capture_failures = 0;
    uint32_t faces_detected_total = 0;
    
    ESP_LOGI(TAG, "Face detection task started - waiting for camera stabilization...");
    vTaskDelay(pdMS_TO_TICKS(3000)); // Give BLE and camera time to initialize
    
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

        // Run detection
        auto *result = detect_faces(fb);
        
                // In your face detection task, update the face detection part:
        if (result && result->count > 0) {
            faces_detected_total += result->count;
            ESP_LOGI(TAG, "Frame %lu: Detected %d face(s)", frame_id, result->count);
            
            // Send BLE notification for each face detected
            for (int i = 0; i < result->count; i++) {
                ESP_LOGI(TAG, "  Face %d: x=%d, y=%d, w=%d, h=%d, conf=%.3f", 
                        i, result->boxes[i].x, result->boxes[i].y, 
                        result->boxes[i].width, result->boxes[i].height, 
                        result->boxes[i].confidence);
                
                ESP_LOGI(TAG, "*** CALLING ble_send_face_notification for face %d ***", i);
                
                // Send BLE notification for this face
                ble_send_face_notification(
                    frame_id,
                    result->count,
                    result->boxes[i].x,
                    result->boxes[i].y,
                    result->boxes[i].width,
                    result->boxes[i].height,
                    result->boxes[i].confidence
                );
                
                ESP_LOGI(TAG, "*** ble_send_face_notification call completed ***");
                
                // Small delay between notifications if multiple faces
                if (i < result->count - 1) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }

        // Stats every 20 frames
        if (frame_id % 20 == 0 && frame_id > 0) {
            ESP_LOGI(TAG, "Stats: Frame %lu, Failures: %lu, Total faces: %lu", 
                     frame_id, capture_failures, faces_detected_total);
        }

        release_frame(fb);
        if (result) free_detection_result(result);
        
        frame_id++;
        vTaskDelay(pdMS_TO_TICKS(500)); // 2 FPS for good responsiveness
    }
}

} // extern "C"