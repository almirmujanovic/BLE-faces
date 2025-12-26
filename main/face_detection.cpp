#include "include/face_detection.h"
#include "include/hand_gesture_ml.h"
#include "human_face_detect.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include <stdlib.h>
#include <string.h>
#include "camera_config.h"
#include "include/media.h"

static const char *TAG = "FACE_DETECT";
static HumanFaceDetect *face_detector = nullptr;

static void log_yuv422_stats(const camera_fb_t *fb, uint32_t frame_id)
{
    if (!fb || !fb->buf || fb->len == 0 || fb->width == 0 || fb->height == 0) {
        return;
    }

    const size_t expected_len = (size_t)fb->width * fb->height * 2;
    if (fb->len != expected_len) {
        ESP_LOGW(TAG, "YUV422 len mismatch: got=%u expected=%u",
                 (unsigned)fb->len, (unsigned)expected_len);
    }

    const size_t total_pairs = (size_t)fb->width * fb->height / 2;
    const size_t sample_pairs = (total_pairs > 2000) ? 2000 : total_pairs;
    const uint8_t *p = fb->buf;

    uint32_t sum[4] = {0, 0, 0, 0};
    uint8_t minv[4] = {255, 255, 255, 255};
    uint8_t maxv[4] = {0, 0, 0, 0};

    for (size_t i = 0; i < sample_pairs; ++i) {
        for (int b = 0; b < 4; ++b) {
            uint8_t v = p[b];
            sum[b] += v;
            if (v < minv[b]) minv[b] = v;
            if (v > maxv[b]) maxv[b] = v;
        }
        p += 4;
    }

    float avg0 = (float)sum[0] / (float)sample_pairs;
    float avg1 = (float)sum[1] / (float)sample_pairs;
    float avg2 = (float)sum[2] / (float)sample_pairs;
    float avg3 = (float)sum[3] / (float)sample_pairs;

    ESP_LOGI(TAG, "YUV422 stats f%lu pairs=%u b0[%u..%u] avg=%.1f b1[%u..%u] avg=%.1f b2[%u..%u] avg=%.1f b3[%u..%u] avg=%.1f",
             (unsigned long)frame_id, (unsigned)sample_pairs,
             minv[0], maxv[0], avg0,
             minv[1], maxv[1], avg1,
             minv[2], maxv[2], avg2,
             minv[3], maxv[3], avg3);
}

static bool yuv422_to_rgb565_via_rgb888(const uint8_t *yuv, size_t yuv_len,
                                        uint16_t width, uint16_t height, uint8_t *rgb565)
{
    if (!yuv || !rgb565 || width == 0 || height == 0 || yuv_len == 0) {
        return false;
    }

    const size_t total_pixels = (size_t)width * height;
    const size_t rgb888_len = total_pixels * 3;
    uint8_t *rgb888 = (uint8_t *)heap_caps_malloc(rgb888_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb888) {
        return false;
    }

    if (!fmt2rgb888(yuv, yuv_len, PIXFORMAT_YUV422, rgb888)) {
        heap_caps_free(rgb888);
        return false;
    }

    // fmt2rgb888 writes BGR byte order for YUV/RGB565 inputs.
    for (size_t i = 0, j = 0; i < total_pixels; ++i, j += 3) {
        uint8_t b = rgb888[j];
        uint8_t g = rgb888[j + 1];
        uint8_t r = rgb888[j + 2];
        uint16_t p = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        rgb565[i * 2] = (uint8_t)(p >> 8);
        rgb565[i * 2 + 1] = (uint8_t)(p & 0xFF);
    }

    heap_caps_free(rgb888);
    return true;
}

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
    const uint32_t hand_gesture_every_n = 3;
    
    ESP_LOGI(TAG, "Face detection task started - waiting for camera stabilization...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    for (;;) {
        if (media_is_camera_busy()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
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

        uint8_t *rgb_buf = nullptr;
        camera_fb_t rgb_fb = {};
        bool needs_free = false;

        if (fb->format == PIXFORMAT_YUV422) {
            if ((frame_id % 50) == 0) {
                log_yuv422_stats(fb, frame_id);
            }
            const size_t rgb_len = (size_t)fb->width * fb->height * 2;
            rgb_buf = (uint8_t *)heap_caps_malloc(rgb_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!rgb_buf) {
                ESP_LOGE(TAG, "Failed to allocate RGB buffer (%u bytes)", (unsigned)rgb_len);
                release_frame(fb);
                frame_id++;
                continue;
            }
            if (!yuv422_to_rgb565_via_rgb888(fb->buf, fb->len, fb->width, fb->height, rgb_buf)) {
                ESP_LOGE(TAG, "YUV->RGB conversion failed");
                heap_caps_free(rgb_buf);
                release_frame(fb);
                frame_id++;
                continue;
            }

            rgb_fb.buf = rgb_buf;
            rgb_fb.len = rgb_len;
            rgb_fb.width = fb->width;
            rgb_fb.height = fb->height;
            rgb_fb.format = PIXFORMAT_RGB565;
            needs_free = true;
        }

        camera_fb_t *ml_fb = needs_free ? &rgb_fb : fb;

        // Run hand gesture model periodically using the same frame
        if ((frame_id % hand_gesture_every_n) == 0) {
            hand_gesture_ml_process_frame(ml_fb);
        }

        // LIGHTWEIGHT: Only run face detection (no BLE, no notifications)
        auto *result = detect_faces(ml_fb);
        
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
                                         ml_fb->buf, ml_fb->len, ml_fb->width, ml_fb->height);
        }

        // Stats every 20 frames
        if (frame_id % 20 == 0 && frame_id > 0) {
            ESP_LOGI(TAG, "Stats: Frame %lu, Failures: %lu, Total faces: %lu", 
                     frame_id, capture_failures, faces_detected_total);
        }

        release_frame(fb);
        if (rgb_buf) {
            heap_caps_free(rgb_buf);
        }
        if (result) free_detection_result(result);
        
        frame_id++;
        vTaskDelay(pdMS_TO_TICKS(300)); // Slower to reduce load while BLE is active
    }
}

} // extern "C"
