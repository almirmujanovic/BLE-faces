#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "include/camera_config.h"
#include "include/gesture.h"

static const char *TAG = "BLE_FACES";

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

// Application main function
void app_main(void)
{
    ESP_LOGI(TAG, "=== Starting MINIMAL EI Test (Camera + Gesture Only) ===");
    log_memory_usage("APP_START");

    // Initialize NVS (needed for camera calibration data)
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

    // Test camera
    ESP_LOGI(TAG, "Testing camera capture...");
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (test_fb) {
        ESP_LOGI(TAG, "Camera test OK: %dx%d", test_fb->width, test_fb->height);
        esp_camera_fb_return(test_fb);
    } else {
        ESP_LOGE(TAG, "Camera test failed");
        return;
    }

    // Wait for system to stabilize
    ESP_LOGI(TAG, "Waiting for system to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    log_memory_usage("BEFORE_GESTURE_TASK");

    // Create ONLY the EI gesture detection task
    ESP_LOGI(TAG, "Creating Edge Impulse gesture detection task...");
    BaseType_t ok = xTaskCreatePinnedToCore(
        gesture_ei_task, 
        "gesture_ei", 
        16384,      // Reduced to 16KB stack
        NULL, 
        2,          // Priority
        NULL, 
        1           // Core 1 (APP CPU)
    );
    
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gesture_ei_task");
        log_memory_usage("TASK_CREATE_FAILED");
        return;
    }
    
    log_memory_usage("APP_COMPLETE");
    ESP_LOGI(TAG, "=== Minimal application started successfully ===");
    ESP_LOGI(TAG, "Only Camera + EI Gesture Detection running");
    ESP_LOGI(TAG, "Gesture detection: 16KB stack on Core 1");
}