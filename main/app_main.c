#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
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
#include "include/hand_gesture_ml.h"
#include "include/media.h"
#include "include/leds.h"
#include "include/power.h"
#include "include/system_state.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "spi_flash_mmap.h"
#include "leds.h"
#include "button.h"
#include "wifi_control.h"
static const char *TAG = "BLE_FACES";

static TaskHandle_t g_face_task_handle = NULL;
static TaskHandle_t g_img_task_handle  = NULL;
static bool g_idle_transition = false;
static bool g_wifi_was_running = false;
// toggles idle: pause camera/ML/gesture and restore on resume
static void system_toggle_idle(void)
{
    if (g_idle_transition) {
        ESP_LOGW(TAG, "IDLE transition already in progress");
        return;
    }
    g_idle_transition = true;

    if (!system_is_idle()) {
        ESP_LOGI(TAG, "Entering IDLE: pausing camera, ML, and gesture sensor...");
        system_set_idle(true);

        g_wifi_was_running = wifi_control_is_running();
        if (g_wifi_was_running) {
            wifi_control_stop();
            ESP_LOGI(TAG, "WiFi/server stopped as part of IDLE");
        }

        uint32_t pause_mask = 0;
        if (g_face_task_handle) {
            pause_mask |= SYSTEM_TASK_FACE;
        }

        if (pause_mask) {
            const uint32_t wait_step_ms = 50;
            uint32_t waited_ms = 0;
            while (!system_are_tasks_paused(pause_mask) && waited_ms < 5000) {
                vTaskDelay(pdMS_TO_TICKS(wait_step_ms));
                waited_ms += wait_step_ms;
            }
            if (!system_are_tasks_paused(pause_mask)) {
                ESP_LOGE(TAG, "Timeout waiting for tasks to pause (mask=0x%02X)", (unsigned)pause_mask);
                system_set_idle(false);
                if (g_wifi_was_running) {
                    esp_err_t werr = wifi_control_start();
                    if (werr != ESP_OK) {
                        ESP_LOGW(TAG, "WiFi/HTTP restart failed after IDLE abort: %s", esp_err_to_name(werr));
                    }
                    g_wifi_was_running = false;
                }
                g_idle_transition = false;
                return;
            }
        }

        uint32_t busy_wait_ms = 0;
        while (media_is_camera_busy() && busy_wait_ms < 5000) {
            vTaskDelay(pdMS_TO_TICKS(100));
            busy_wait_ms += 100;
        }
        if (media_is_camera_busy()) {
            ESP_LOGW(TAG, "Camera still busy; proceeding with idle transition");
        }


        ESP_LOGI(TAG, "Keeping camera driver initialized to preserve shared I2C");

        leds_blink_idle(3, 150);
        leds_set_idle(true);

        ESP_LOGI(TAG, "System entered IDLE (camera/ML/gesture paused)");
    } else {
        ESP_LOGI(TAG, "Exiting IDLE: resuming camera, ML...");

        leds_configure_button_input();

        camera_discard_initial_frames(3);
        vTaskDelay(pdMS_TO_TICKS(50));

        system_set_idle(false);

        if (g_wifi_was_running) {
            esp_err_t werr = wifi_control_start();
            if (werr != ESP_OK) {
                ESP_LOGW(TAG, "WiFi/HTTP restart failed after IDLE: %s", esp_err_to_name(werr));
            }
            g_wifi_was_running = false;
        }

        leds_blink_idle(2, 150);
        leds_set_idle(false);
        ESP_LOGI(TAG, "System resumed from IDLE");
    }

    g_idle_transition = false;
}

// toggles wifi/server on button hold
static void system_toggle_wifi(void)
{
    if (system_is_idle()) {
        ESP_LOGW(TAG, "Ignoring WiFi toggle while IDLE");
        return;
    }
    if (wifi_control_is_running()) {
        wifi_control_stop();
        leds_blink_alternate(2, 80);
        ESP_LOGI(TAG, "WiFi + HTTP server stopped by button hold");
    } else {
        esp_err_t r = wifi_control_start();
        if (r == ESP_OK) {
            leds_blink_both(2, 80);
            if (wifi_control_get_server_handle()) {
                ESP_LOGI(TAG, "WiFi + HTTP server started by button hold");
            } else {
                ESP_LOGW(TAG, "WiFi started, but HTTP server failed to start");
            }
        } else {
            ESP_LOGW(TAG, "Failed to start WiFi via button hold: %s", esp_err_to_name(r));
        }
    }
}

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
        bool sent = false;

        sent = ble_img_xfer_send_rgb565(frame_id,
                                        use_x, use_y, use_w, use_h,
                                        crop_buf, crop_len);

        
        if (sent) {
            ESP_LOGI("IMG_PROC", "Successfully queued crop for BLE transfer");
        } else {
            ESP_LOGW("IMG_PROC", "Failed to queue crop for BLE transfer");
            heap_caps_free(crop_buf); // Free PSRAM crop buffer on failure
        }
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



void app_main(void)
{
    ESP_LOGI(TAG, "=== Starting BLE Face Detection Application ===");
    log_memory_usage("APP_START");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP stack EARLY - before any SPIFFS operations
    // This prevents flash writes from corrupting the network stack
    ESP_LOGI(TAG, "Initializing TCP/IP stack early...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "TCP/IP stack initialized");

    // Start BLE early to avoid SPI flash cache contention during SPIFFS I/O.
    ESP_LOGI(TAG, "Starting BLE server (early)...");
    ble_server_start("ESP-Face-Detector");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "BLE server started");


    // Initialize media storage (BEFORE gesture sensor)
    ESP_LOGI(TAG, "Initializing media storage...");
    ret = media_storage_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Media storage init failed: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Photo/video capture will not work");
    } else {
        ESP_LOGI(TAG, "Media storage ready");
        
        // Uncomment to wipe all media files on boot.
        // media_delete_all();

        // List existing files
        media_list_files();
        
        // Get storage info
        size_t total, used, free;
        if (media_get_storage_info(&total, &used, &free) == ESP_OK) {
            ESP_LOGI(TAG, "Storage: %zu KB total, %zu KB used, %zu KB free", 
                     total/1024, used/1024, free/1024);
        }
    }
    
    log_memory_usage("AFTER_MEDIA_STORAGE");

    ESP_LOGI(TAG, "Allowing flash cache to stabilize...");
    spi_flash_mmap_dump();
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Initializing shared I2C bus...");
    ret = power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power/I2C init failed: %s", esp_err_to_name(ret));
        return;
    }


    ESP_LOGI(TAG, "Initializing camera...");
    ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: 0x%x", ret);
        return;
    }

    // Let the sensor/JPEG pipeline settle before first capture.
    camera_discard_initial_frames(3);


    /*
    esp_err_t res = media_capture_jpeg_burst(10000, 20, 200);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Captured burst video (VGA) for %d ms at %d FPS", 10000, 20);
        } else {
            ESP_LOGE(TAG, "Burst video capture failed: %s", esp_err_to_name(res));
        }
        return;
*/

    
    // WiFi AP + HTTP server now starts only on user toggle
    // TEMP: start WiFi/HTTP at boot for crash testing of toggle-off path
    /*
    {
        esp_err_t werr = wifi_control_start();
        if (werr != ESP_OK) {
            ESP_LOGW(TAG, "TEMP WiFi/HTTP start at boot failed: %s", esp_err_to_name(werr));
        } else {
            ESP_LOGI(TAG, "TEMP WiFi/HTTP started at boot (for toggle-off crash test)");
        }
    }
    */
    // Initialize face detection
    ESP_LOGI(TAG, "Initializing face detection...");
    ret = init_face_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Face detection initialization failed: 0x%x", ret);
        return;
    }
    log_memory_usage("AFTER_FACE_DETECT");
    // Initialize ML-based hand gesture recognition (camera-based)
    // Note: Hand gestures are automatically disabled when WiFi is active to prevent memory conflicts
    ESP_LOGI(TAG, "Initializing hand gesture ML...");
    ret = hand_gesture_ml_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Hand gesture ML init failed: 0x%x", ret);
    } else {
        ESP_LOGI(TAG, "✓ Hand gesture ML initialized (auto-disabled when WiFi active)");
    }

    // Create image processing queue
    ESP_LOGI(TAG, "Creating image processing queue...");
    image_process_queue = xQueueCreate(3, sizeof(image_process_request_t));
    if (!image_process_queue) {
        ESP_LOGE(TAG, "Failed to create image processing queue");
        return;
    }

    // Create face detection task (pinned to Core 0)
    // NOTE: Using internal RAM for stack (not PSRAM) to avoid cache freeze issues
    // when models are lazy-loaded from flash memory during runtime.
    ESP_LOGI(TAG, "Creating face detection task on Core 0...");
    BaseType_t ok = xTaskCreatePinnedToCore(face_detection_task, "face_detection_task",
                               8192, NULL, 6, &g_face_task_handle,
                               0);  // Pin to Core 0 for dedicated face+gesture processing
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create face_detection_task");
        return;
    }
    log_memory_usage("AFTER_FACE_TASK");

    // Create image processing task (pinned to Core 1)
    // NOTE: Using internal RAM for stack (not PSRAM) to avoid cache freeze issues
    // when flash operations occur (e.g., model loading, partition mapping).
    ESP_LOGI(TAG, "Creating image processing task on Core 1...");
    ok = xTaskCreatePinnedToCore(image_processing_task, "image_processing_task",
                    16384, NULL, 4, &g_img_task_handle,
                    1);  // Pin to Core 1 for BLE/network operations
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create image_processing_task");
        return;
    }
    log_memory_usage("AFTER_IMG_TASK");
    // Read BQ25155 device ID (if present) on the shared bus.
    uint8_t id;
    power_bq25155_read_id(&id);  // will just warn if not present

    // Read and display BQ25155 configuration (verify 4.2V setting)
    power_bq25155_read_config();

    // Log battery status and percentage
    float battery_voltage = 0.0f;
    float battery_percent = 0.0f;
    esp_err_t batt_ret = power_bq25155_read_battery(&battery_voltage, &battery_percent);
    if (batt_ret == ESP_OK) {
        ESP_LOGI(TAG, "=== BATTERY STATUS ===");
        ESP_LOGI(TAG, "Battery Voltage: %.3f V", battery_voltage);
        ESP_LOGI(TAG, "Battery Percentage: %.1f%%", battery_percent);
        
        // Read charging current
        float charge_current = 0.0f;
        esp_err_t curr_ret = power_bq25155_read_charge_current(&charge_current);
        if (curr_ret == ESP_OK) {
            ESP_LOGI(TAG, "Charging Current: %.1f mA", charge_current);
        }
        
        // Read charging status
        uint8_t stat0 = 0, stat1 = 0;
        power_bq25155_read_status(&stat0, &stat1);
        
        ESP_LOGI(TAG, "======================");
    } else {
        ESP_LOGW(TAG, "Failed to read battery status: %s", esp_err_to_name(batt_ret));
    }

    // Start periodic battery monitoring (log every 30 seconds)
    power_start_battery_monitor(30000);

    // Initialize LEDs on PCA9536
    if (leds_init() == ESP_OK) {
        leds_blink_both(2, 100);  // quick sanity blink
    }
        button_start(system_toggle_idle, system_toggle_wifi, 50, 500, 2000);        
    
    log_memory_usage("APP_COMPLETE");
    ESP_LOGI(TAG, "=== Application started successfully ===");
    ESP_LOGI(TAG, "Photo: Hand ML 'ok'");
    ESP_LOGI(TAG, "Video: Hand ML 'one'");
    ESP_LOGI(TAG, "Ready!");
}
