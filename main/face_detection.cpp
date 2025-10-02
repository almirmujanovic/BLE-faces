#include "include/face_detection.h"
#include "human_face_detect.hpp"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>               // NEW: for memcpy
#include "camera_config.h"
#include "ble_server.h"
#include "ble_image_transfer.h"   // NEW: only needed if you want to call image API directly

static const char *TAG = "FACE_DETECT";
static HumanFaceDetect *face_detector = nullptr;

// NEW: Forward declaration in case the helper isn't declared in ble_server.h.
//      This function is defined in ble_server.c in our previous step.
//      If you already added it to ble_server.h, remove this extern.
extern "C" void ble_send_face_notification_and_crop(uint32_t frame_id, uint8_t face_count,
                                                    uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                                    float confidence,
                                                    const uint8_t *rgb565, uint32_t rgb_len);

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

/* ========================= NEW: RGB565 crop helper =========================
 * Crops a rectangular ROI from the camera frame buffer (RGB565).
 * - Returns a heap-allocated buffer containing the crop (caller frees).
 * - Clamps ROI to frame bounds and enforces even X/width so RGB565 alignment
 *   is preserved (2 bytes per pixel).
 * - Out params return the clamped crop origin/size as actually used.
 */
static uint8_t* crop_rgb565_from_fb(const camera_fb_t* fb,
                                    int x, int y, int w, int h,
                                    int &out_x, int &out_y,
                                    int &out_w, int &out_h,
                                    uint32_t &out_len_bytes)
{
    if (!fb || !fb->buf || w <= 0 || h <= 0) {
        return nullptr;
    }

    const int frame_w = (int)fb->width;
    const int frame_h = (int)fb->height;

    // Clamp ROI to frame bounds
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + w > frame_w) w = frame_w - x;
    if (y + h > frame_h) h = frame_h - y;

    // If ROI is empty after clamping, bail
    if (w <= 0 || h <= 0) return nullptr;

    // Ensure even X and width for RGB565 (2 bytes per pixel)
    if (x & 1) { x--; w++; if (x < 0) x = 0; if (x + w > frame_w) w = frame_w - x; }
    if (w & 1) { w--; if (w <= 0) return nullptr; }

    // Calculate buffer sizes
    const int bytes_per_pixel = 2; // RGB565
    const int src_stride_bytes = frame_w * bytes_per_pixel;
    const int row_bytes = w * bytes_per_pixel;
    out_len_bytes = (uint32_t)(row_bytes * h);

    // Allocate crop buffer
    uint8_t *crop = (uint8_t*)malloc(out_len_bytes);
    if (!crop) {
        ESP_LOGE(TAG, "Failed to allocate crop buffer (%d x %d)", w, h);
        return nullptr;
    }

    // Copy row by row
    const uint8_t *src_base = fb->buf + (y * src_stride_bytes) + (x * bytes_per_pixel);
    uint8_t *dst = crop;

    for (int row = 0; row < h; ++row) {
        const uint8_t *src = src_base + row * src_stride_bytes;
        memcpy(dst, src, row_bytes);
        dst += row_bytes;
    }

    // Return the actual ROI used
    out_x = x; out_y = y; out_w = w; out_h = h;
    return crop;
}
/* ======================= end RGB565 crop helper =========================== */

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
        
        if (result && result->count > 0) {
            faces_detected_total += result->count;
            ESP_LOGI(TAG, "Frame %lu: Detected %d face(s)", frame_id, result->count);
            
            // For each face: (1) log, (2) build RGB565 crop, (3) send BLE notify + image stream
            for (int i = 0; i < result->count; i++) {
                const int rx = result->boxes[i].x;
                const int ry = result->boxes[i].y;
                const int rw = result->boxes[i].width;
                const int rh = result->boxes[i].height;

                ESP_LOGI(TAG, "  Face %d: x=%d, y=%d, w=%d, h=%d, conf=%.3f", 
                        i, rx, ry, rw, rh, result->boxes[i].confidence);

                // --- Build a tight RGB565 crop from the full frame ---
                int use_x=0, use_y=0, use_w=0, use_h=0;
                uint32_t crop_len = 0;
                uint8_t *crop_buf = crop_rgb565_from_fb(fb, rx, ry, rw, rh,
                                                        use_x, use_y, use_w, use_h, crop_len);

                if (!crop_buf) {
                    // If crop failed, still send the face notification without image.
                    ESP_LOGW(TAG, "Crop failed for face %d; sending notification only", i);

                    // Keep the original notification call to preserve your behavior.
                    ble_send_face_notification(
                        frame_id,
                        result->count,
                        (uint16_t)rx, (uint16_t)ry,
                        (uint16_t)rw, (uint16_t)rh,
                        result->boxes[i].confidence
                    );

                } else {
                    // --- Send BLE face notification + then the image crop (chunked) ---
                    // The helper ensures the order: FaceNotification -> ImageInfo -> DATA chunks.
                    ble_send_face_notification_and_crop(
                        frame_id,
                        result->count,
                        (uint16_t)use_x, (uint16_t)use_y,
                        (uint16_t)use_w, (uint16_t)use_h,
                        result->boxes[i].confidence,
                        crop_buf,
                        crop_len
                    );

                    // Free the temporary crop buffer
                    free(crop_buf);
                }
                
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
