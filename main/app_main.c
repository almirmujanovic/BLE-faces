#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "include/camera_config.h"
#include "include/face_detection.h"
#include "include/ble_server.h"
#include "include/ble_image_transfer.h"
#include "include/gesture.h"

static const char *TAG = "BLE_FACES";

// Image processing request structure
typedef struct {
    uint32_t frame_id;
    uint16_t x, y, w, h;
    float confidence;
    uint8_t *image_data;     // Will be allocated in PSRAM
    size_t image_len;
    uint16_t img_width, img_height;
} image_process_request_t;

static QueueHandle_t image_process_queue = NULL;

// Memory monitoring function
void log_memory_usage(const char* context)
{
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    
    ESP_LOGI("MEMORY", "%s - Internal: %zu/%zu KB, PSRAM: %zu/%zu KB", 
             context,
             internal_free/1024, internal_total/1024,
             psram_free/1024, psram_total/1024);
}

// RGB565 crop helper (moved from face_detection.cpp to handle heavy processing)
static uint8_t* crop_rgb565_from_buffer(const uint8_t* src_buf, int src_w, int src_h,
                                        int x, int y, int w, int h,
                                        int *out_x, int *out_y,
                                        int *out_w, int *out_h,
                                        uint32_t *out_len_bytes)
{
    if (!src_buf || w <= 0 || h <= 0) {
        return NULL;
    }

    // Clamp ROI to frame bounds
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + w > src_w) w = src_w - x;
    if (y + h > src_h) h = src_h - y;

    if (w <= 0 || h <= 0) return NULL;

    // Ensure even X and width for RGB565 (2 bytes per pixel alignment)
    if (x & 1) { 
        x--; 
        w++; 
        if (x < 0) x = 0; 
        if (x + w > src_w) w = src_w - x; 
    }
    if (w & 1) { 
        w--; 
        if (w <= 0) return NULL; 
    }

    // Calculate buffer sizes
    const int bytes_per_pixel = 2; // RGB565
    const int src_stride_bytes = src_w * bytes_per_pixel;
    const int row_bytes = w * bytes_per_pixel;
    *out_len_bytes = (uint32_t)(row_bytes * h);

    // Allocate crop buffer in PSRAM
    uint8_t *crop = (uint8_t*)heap_caps_malloc(*out_len_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!crop) {
        ESP_LOGE("IMG_PROC", "Failed to allocate PSRAM crop buffer (%d x %d = %lu bytes)", 
                 w, h, *out_len_bytes);
        return NULL;
    }

    // Copy row by row
    const uint8_t *src_base = src_buf + (y * src_stride_bytes) + (x * bytes_per_pixel);
    uint8_t *dst = crop;

    for (int row = 0; row < h; ++row) {
        const uint8_t *src = src_base + row * src_stride_bytes;
        memcpy(dst, src, row_bytes);
        dst += row_bytes;
    }

    *out_x = x; 
    *out_y = y; 
    *out_w = w; 
    *out_h = h;
    
    ESP_LOGD("IMG_PROC", "Cropped %dx%d region at (%d,%d) = %lu bytes", 
             w, h, x, y, *out_len_bytes);
    
    return crop;
}

// Heavy image processing (runs in separate task with PSRAM)
void process_face_crop_and_send(uint32_t frame_id, uint16_t x, uint16_t y, 
                               uint16_t w, uint16_t h, float confidence,
                               const uint8_t* image_data, size_t image_len,
                               uint16_t img_width, uint16_t img_height)
{
    ESP_LOGI("IMG_PROC", "Processing crop: %dx%d at (%d,%d) from %dx%d image", 
             w, h, x, y, img_width, img_height);
    
    // Send the notification (lightweight)
    ble_send_face_notification(frame_id, 1, x, y, w, h, confidence);
    
    // Do heavy image processing here (safe with large stack)
    int use_x, use_y, use_w, use_h;
    uint32_t crop_len = 0;
    
    uint8_t *crop_buf = crop_rgb565_from_buffer(
        image_data, img_width, img_height,
        x, y, w, h,
        &use_x, &use_y, &use_w, &use_h, &crop_len);
    
    if (crop_buf && crop_len > 0) {
        ESP_LOGI("IMG_PROC", "Sending %dx%d crop via BLE (%lu bytes)", 
                 use_w, use_h, crop_len);
        
        // Send the crop via BLE using the image transfer API
        bool sent = ble_img_xfer_send_rgb565(frame_id,
                                            use_x, use_y, use_w, use_h,
                                            crop_buf, crop_len);
        
        if (sent) {
            ESP_LOGI("IMG_PROC", "Successfully queued crop for BLE transfer");
        } else {
            ESP_LOGW("IMG_PROC", "Failed to queue crop for BLE transfer");
        }
        
        heap_caps_free(crop_buf); // Free PSRAM crop buffer
    } else {
        ESP_LOGE("IMG_PROC", "Failed to create crop from %dx%d at (%d,%d)", 
                 w, h, x, y);
    }
}

// Also update the queue function to handle BLE better:
void queue_image_processing_request(uint32_t frame_id, uint16_t x, uint16_t y, 
                                   uint16_t w, uint16_t h, float confidence,
                                   const uint8_t* image_data, size_t image_len,
                                   uint16_t img_width, uint16_t img_height)
{
    if (!image_process_queue) {
        ESP_LOGW("IMG_PROC", "Image processing queue not initialized");
        return;
    }
    
    // Check if queue is getting full (prevent backup)
    UBaseType_t queue_spaces = uxQueueSpacesAvailable(image_process_queue);
    if (queue_spaces == 0) {
        ESP_LOGW("IMG_PROC", "Queue full, dropping frame %lu processing request", frame_id);
        return;
    }
    
    // Use PSRAM for the image data copy
    uint8_t *data_copy = (uint8_t*)heap_caps_malloc(image_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!data_copy) {
        ESP_LOGE("IMG_PROC", "Failed to allocate PSRAM for image data copy (%zu bytes)", image_len);
        return;
    }
    
    // Copy the image data to PSRAM (since camera buffer will be returned)
    memcpy(data_copy, image_data, image_len);
    
    image_process_request_t request = {
        .frame_id = frame_id,
        .x = x, .y = y, .w = w, .h = h,
        .confidence = confidence,
        .image_data = data_copy,  
        .image_len = image_len,
        .img_width = img_width,
        .img_height = img_height
    };
    
    // Send to processing task (non-blocking)
    if (xQueueSend(image_process_queue, &request, 0) != pdTRUE) {
        ESP_LOGW("IMG_PROC", "Failed to queue frame %lu processing request", frame_id);
        heap_caps_free(data_copy);  // Clean up if queue is full
    } else {
        ESP_LOGD("IMG_PROC", "Queued frame %lu for processing", frame_id);
    }
}

// Image processing task 
void image_processing_task(void *pvParameters)
{
    ESP_LOGI("IMG_PROC", "Image processing task started");
    log_memory_usage("IMG_PROC_START");
    
    image_process_request_t request;
    
    while (1) {
        // Wait for image processing requests
        if (xQueueReceive(image_process_queue, &request, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD("IMG_PROC", "Received processing request for frame %lu", request.frame_id);
            log_memory_usage("BEFORE_PROCESSING");
            
            // Do heavy image processing here
            process_face_crop_and_send(request.frame_id, request.x, request.y, 
                                      request.w, request.h, request.confidence,
                                      request.image_data, request.image_len,
                                      request.img_width, request.img_height);
            
            // Free the PSRAM copy
            heap_caps_free(request.image_data);
            log_memory_usage("AFTER_PROCESSING");
        }
    }
}


// Application main function
void app_main(void)
{
    ESP_LOGI(TAG, "=== Starting BLE Face Detection Application ===");
    log_memory_usage("APP_START");

    // Initialize NVS for BLE
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize camera
    ESP_LOGI(TAG, "Initializing camera...");
    ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: 0x%x", ret);
        return;
    }
    log_memory_usage("AFTER_CAMERA");

    // Test camera capture before proceeding
    ESP_LOGI(TAG, "Testing camera capture...");
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (test_fb) {
        ESP_LOGI(TAG, "Camera test OK: %dx%d, format=%d, len=%zu", 
                 test_fb->width, test_fb->height, test_fb->format, test_fb->len);
        esp_camera_fb_return(test_fb);
    } else {
        ESP_LOGE(TAG, "Camera test failed - cannot capture frames");
        return;
    }

    ESP_LOGI(TAG, "Starting BLE server...");
    ble_server_start("ESP-Face-Detector");
    
    // Wait for BLE server to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize face detection
    ESP_LOGI(TAG, "Initializing face detection...");
    ret = init_face_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Face detection initialization failed: 0x%x", ret);
        return;
    }
    log_memory_usage("AFTER_FACE_DETECT");

    // Create image processing queue
    ESP_LOGI(TAG, "Creating image processing queue...");
    image_process_queue = xQueueCreate(3, sizeof(image_process_request_t));
    if (!image_process_queue) {
        ESP_LOGE(TAG, "Failed to create image processing queue");
        return;
    }

    // Create face detection task (KEEP original small stack)
    ESP_LOGI(TAG, "Creating face detection task...");
    BaseType_t ok = xTaskCreate(face_detection_task, "face_detection_task", 
                               8192, NULL, 6, NULL); // 8KB - same as before
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create face_detection_task");
        return;
    }
    
    log_memory_usage("AFTER_FACE_TASK");

    // Create separate image processing task with large stack for PSRAM operations
    ESP_LOGI(TAG, "Creating image processing task...");
    ok = xTaskCreate(image_processing_task, "image_processing_task", 
                    65536, NULL, 4, NULL); // 64KB for heavy processing
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create image_processing_task");
        return;
    }
    
    log_memory_usage("AFTER_IMG_TASK");

    // Start BLE server
    // Initialize gesture sensor AFTER BLE is ready
    ESP_LOGI(TAG, "Initializing gesture sensor...");
    ret = paj7620_i2c_init();
    if (ret == ESP_OK) {
        ret = paj7620_init();
        if (ret == ESP_OK) {
            paj7620_set_high_rate(true); // Fast detection mode
            xTaskCreatePinnedToCore(paj7620_task, "paj", 4096, NULL, 2 /*low*/, NULL, 1 /*APP core*/);            ESP_LOGI(TAG, "Gesture sensor initialized successfully");
        } else {
            ESP_LOGW(TAG, "Gesture sensor init failed, continuing without gestures");
        }
    } else {
        ESP_LOGW(TAG, "Gesture I2C init failed, continuing without gestures");
    }
    
    log_memory_usage("APP_COMPLETE");
    ESP_LOGI(TAG, "=== Application started successfully ===");
    ESP_LOGI(TAG, "Face detection: 8KB stack task");
    ESP_LOGI(TAG, "Image processing: 64KB stack task with PSRAM buffers");
    ESP_LOGI(TAG, "Ready to detect faces and stream crops via BLE!");
}