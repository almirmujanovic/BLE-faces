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
#include "include/hand_gesture_ml.h"
#include "include/media.h"
#include "include/leds.h"
#include "include/power.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include <lwip/inet.h>
#include "leds.h"
#include "button.h"
#include "wifi_control.h"
static const char *TAG = "BLE_FACES";

static bool g_system_idle = false;
static TaskHandle_t g_face_task_handle = NULL;
static TaskHandle_t g_img_task_handle  = NULL;
static TaskHandle_t g_paj_task_handle  = NULL;
// toggles idle: fully power down camera and suspend tasks; on resume reinit & resume
static void system_toggle_idle(void)
{
    if (!g_system_idle) {
        // suspend camera-related tasks (keep button task running)
        if (g_face_task_handle) vTaskSuspend(g_face_task_handle);
        if (g_img_task_handle)  vTaskSuspend(g_img_task_handle);
        if (g_paj_task_handle)  vTaskSuspend(g_paj_task_handle);

        ESP_LOGI(TAG, "Entering IDLE: shutting down camera (driver only)...");

        // Deinit camera driver only — do NOT toggle CAM_PWDN_GPIO to avoid disabling shared I2C devices
        esp_err_t err = esp_camera_deinit();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_camera_deinit failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Camera driver deinitialized");
        }

        // Blink idle LED 3 times as confirmation
        leds_blink_idle(3, 150);

        // Stop WiFi/server if running
        if (wifi_control_is_running()) {
            wifi_control_stop();
            ESP_LOGI(TAG, "WiFi/server stopped as part of IDLE");
        }

        ESP_LOGI(TAG, "System entered IDLE (camera driver deinit, tasks suspended)");
        g_system_idle = true;
    } else {
        ESP_LOGI(TAG, "Exiting IDLE: reinitializing camera driver...");

        // Reinitialize camera driver (do not touch global power rails)
        esp_err_t err = init_camera();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera re-init failed: %s. Remaining in IDLE.", esp_err_to_name(err));
            return;
        }

        // Reconfigure PCA9536 button input (in case it was affected)
        leds_configure_button_input();

        // Allow AWB/AEC to settle
        camera_discard_initial_frames(5);
        vTaskDelay(pdMS_TO_TICKS(50));

        // resume tasks
        if (g_face_task_handle) vTaskResume(g_face_task_handle);
        if (g_img_task_handle)  vTaskResume(g_img_task_handle);
        if (g_paj_task_handle)  vTaskResume(g_paj_task_handle);

        ESP_LOGI(TAG, "System resumed from IDLE (camera driver reinitialized)");
        g_system_idle = false;
    }
}

// toggles wifi/server on button hold
static void system_toggle_wifi(void)
{
    if (wifi_control_is_running()) {
        wifi_control_stop();
        ESP_LOGI(TAG, "WiFi + HTTP server stopped by button hold");
    } else {
        esp_err_t r = wifi_control_start();
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "WiFi + HTTP server started by button hold");
        } else {
            ESP_LOGW(TAG, "Failed to start WiFi via button hold: %s", esp_err_to_name(r));
        }
    }
}

extern httpd_handle_t start_http_server(void);
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
#if 0
    ble_send_face_notification(frame_id, 1, x, y, w, h, confidence);
#endif
    
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
#if 0
        sent = ble_img_xfer_send_rgb565(frame_id,
                                        use_x, use_y, use_w, use_h,
                                        crop_buf, crop_len);
#endif
        
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



static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create AP interface with custom IP
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    
    // Stop DHCP server temporarily to set static IP
    esp_netif_dhcps_stop(ap_netif);
    
    // Set custom IP: 192.168.1.100 (gateway/router IP)
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 1, 100);       // AP IP
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 100);       // Gateway (same)
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);  // Subnet mask
    
    esp_netif_set_ip_info(ap_netif, &ip_info);
    
    // Restart DHCP server (will assign 192.168.1.101, 192.168.1.102, etc. to clients)
    esp_netif_dhcps_start(ap_netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32-Media",
            .ssid_len = strlen("ESP32-Media"),
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .channel = 1
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI("WIFI", "✅ WiFi AP started");
    ESP_LOGI("WIFI", "   SSID: ESP32-Media");
    ESP_LOGI("WIFI", "   Password: 12345678");
    ESP_LOGI("WIFI", "   IP Address: http://192.168.1.100");
    ESP_LOGI("WIFI", "   Browse media at:");
    ESP_LOGI("WIFI", "     • http://192.168.1.100/images");
    ESP_LOGI("WIFI", "     • http://192.168.1.100/videos");
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


    // Initialize media storage (BEFORE gesture sensor)
    ESP_LOGI(TAG, "Initializing media storage...");
    ret = media_storage_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Media storage init failed: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Photo/video capture will not work");
    } else {
        ESP_LOGI(TAG, "✅ Media storage ready");
        
        // Uncomment to wipe all media files on boot.
        media_delete_all();

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


    ESP_LOGI(TAG, "Initializing camera...");
    ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: 0x%x", ret);
        return;
    }

    // Let the sensor/JPEG pipeline settle before first capture.
    camera_discard_initial_frames(3);

    // Take one picture and save it via media.c
    ESP_LOGI(TAG, "Capturing test image...");
    camera_fb_t *test_fb = esp_camera_fb_get();
    if (!test_fb) {
        ESP_LOGE(TAG, "Test capture failed: no frame buffer");
    } else {
        ret = media_save_photo(test_fb);
        esp_camera_fb_return(test_fb);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Test capture failed: 0x%x", ret);
        } else {
            ESP_LOGI(TAG, "Test capture OK. Check latest photo_* file on SPIFFS.");
        }
    }

    // Initialize WiFi AP
   // ESP_LOGI(TAG, "Starting WiFi Access Point...");
  //  wifi_init();
    
    // Start HTTP server for media access
   // ESP_LOGI(TAG, "Starting HTTP file server...");
    //httpd_handle_t server = start_http_server();
    //if (server) {
     //   ESP_LOGI(TAG, "✅ HTTP server running at http://192.168.1.100/");
       // ESP_LOGI(TAG, "   Connect to WiFi: SSID='ESP32-Media', Password='12345678'");
   // } else {
     //   ESP_LOGW(TAG, "Failed to start HTTP server");
    //}

    /*
    esp_err_t res = media_capture_jpeg_burst(10000, 20, 200);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "🎥 Captured burst video (VGA) for %d ms at %d FPS", 10000, 20);
        } else {
            ESP_LOGE(TAG, "❌ Burst video capture failed: %s", esp_err_to_name(res));
        }
        return;
*/

    // Initialize BLE (disabled for now)
#if 0
    ESP_LOGI(TAG, "Starting BLE server...");
    ble_server_start("ESP-Face-Detector");
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

    // Start WiFi AP + HTTP server on boot
    ESP_LOGI(TAG, "Starting WiFi AP + HTTP server...");
    ret = wifi_control_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi/HTTP start failed: %s", esp_err_to_name(ret));
    }
    
    // Initialize face detection
    ESP_LOGI(TAG, "Initializing face detection...");
    ret = init_face_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Face detection initialization failed: 0x%x", ret);
        return;
    }
    log_memory_usage("AFTER_FACE_DETECT");
/*
    // Initialize ML-based hand gesture recognition (camera-based)
    ESP_LOGI(TAG, "Initializing hand gesture ML...");
    ret = hand_gesture_ml_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Hand gesture ML init failed: 0x%x", ret);
    }
        */

    // Create image processing queue
    ESP_LOGI(TAG, "Creating image processing queue...");
    image_process_queue = xQueueCreate(3, sizeof(image_process_request_t));
    if (!image_process_queue) {
        ESP_LOGE(TAG, "Failed to create image processing queue");
        return;
    }

    // Create face detection task
    ESP_LOGI(TAG, "Creating face detection task...");
    BaseType_t ok = xTaskCreate(face_detection_task, "face_detection_task", 
                               8192, NULL, 6, &g_face_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create face_detection_task");
        return;
    }
    log_memory_usage("AFTER_FACE_TASK");

    // Create image processing task
    ESP_LOGI(TAG, "Creating image processing task...");
    ok = xTaskCreate(image_processing_task, "image_processing_task", 
                    16384, NULL, 4, &g_img_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create image_processing_task");
        return;
    }
    log_memory_usage("AFTER_IMG_TASK");
    // Init I2C bus for power/LEDs
    if (power_init() == ESP_OK) {
        uint8_t id;
        power_bq25155_read_id(&id);  // will just warn if not present
    }

    // Initialize LEDs on PCA9536
    if (leds_init() == ESP_OK) {
        leds_blink_both(2, 100);  // quick sanity blink
    }
        // button_start(system_toggle_idle, system_toggle_wifi, 50, 500, 2000);

    // Initialize gesture sensor LAST
   
    ESP_LOGI(TAG, "Initializing gesture sensor...");
    ret = paj7620_i2c_init();
    if (ret == ESP_OK) {
        ret = paj7620_init();
        if (ret == ESP_OK) {
            paj7620_set_high_rate(true);
            xTaskCreatePinnedToCore(paj7620_task, "paj7620_task", 
                                   4096, NULL, 2, &g_paj_task_handle, 1);
            ESP_LOGI(TAG, "✅ Gesture sensor initialized");
            ESP_LOGI(TAG, "   CLOCKWISE gesture   = Take photo");
            ESP_LOGI(TAG, "   ANTICLOCKWISE gesture = Record 10s video");
            ESP_LOGI(TAG, "   UP/DOWN/LEFT/RIGHT = BLE HID (when connected)");
        } else {
            ESP_LOGW(TAG, "Gesture sensor init failed");
        }
    } else {
        ESP_LOGW(TAG, "Gesture I2C init failed");
    }
        
    
    log_memory_usage("APP_COMPLETE");
    ESP_LOGI(TAG, "=== Application started successfully ===");
    ESP_LOGI(TAG, "📸 Photo: CLOCKWISE gesture");
    ESP_LOGI(TAG, "🎥 Video: ANTICLOCKWISE gesture");
    ESP_LOGI(TAG, "Ready!");
}
