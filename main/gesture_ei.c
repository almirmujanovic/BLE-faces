#include "include/gesture.h"
#include "include/ei_infer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "GESTURE_EI";

// Gesture→HID mapping
static void handle_gesture(const char *gesture, float confidence)
{
    if (!gesture) return;
    ESP_LOGI(TAG, "Gesture detected: %s (%.3f)", gesture, confidence);
}

// Gesture detection task
void gesture_ei_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Edge Impulse gesture detection task started");
    
    // Wait for camera and system to stabilize
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Check heap integrity
    ESP_LOGI(TAG, "Checking PSRAM heap integrity...");
    if (!heap_caps_check_integrity(MALLOC_CAP_SPIRAM, true)) {
        ESP_LOGE(TAG, "❌ PSRAM heap is CORRUPTED!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "✓ PSRAM heap integrity OK");
    
    // Log memory state
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "PSRAM: %zu KB free / %zu KB total", psram_free / 1024, psram_total / 1024);
    ESP_LOGI(TAG, "PSRAM largest contiguous block: %zu KB", psram_largest / 1024);
    ESP_LOGI(TAG, "Model needs: 3164 KB for tensor arena");
    
    if (psram_largest < 3200000) {
        ESP_LOGE(TAG, "❌ Not enough contiguous PSRAM! Need 3.2MB, have %zu KB", 
                 psram_largest / 1024);
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize Edge Impulse buffers
    ESP_LOGI(TAG, "Initializing Edge Impulse buffers...");
    if (!ei_infer_init()) {
        ESP_LOGE(TAG, "Failed to initialize Edge Impulse!");
        vTaskDelete(NULL);
        return;
    }
    
    // Check heap after buffer allocation
    if (!heap_caps_check_integrity(MALLOC_CAP_SPIRAM, true)) {
        ESP_LOGE(TAG, "❌ PSRAM heap corrupted after ei_infer_init!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "✓ PSRAM heap still OK after ei_infer_init");
    
    // CRITICAL: DE-INITIALIZE camera completely before TFLite allocation
    ESP_LOGI(TAG, "⚠️  Stopping camera completely for TFLite arena allocation...");
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Camera deinit failed: 0x%x", err);
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // Wait for all DMA to stop
    
    // Check heap after stopping camera
    if (!heap_caps_check_integrity(MALLOC_CAP_SPIRAM, true)) {
        ESP_LOGE(TAG, "❌ PSRAM heap corrupted after stopping camera!");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Running FIRST inference to trigger TFLite arena allocation...");
    
    // This will fail (no camera), but will allocate the TFLite arena
    float confidence = 0.0f;
    const char *gesture = ei_infer_gesture(&confidence);
    
    // Expected to fail - that's OK, we just needed to allocate the arena
    ESP_LOGI(TAG, "Arena allocation attempt complete (expected to fail without camera)");
    
    // Re-initialize camera
    ESP_LOGI(TAG, "Restarting camera...");
    extern esp_err_t init_camera(void);
    err = init_camera();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to restart camera!");
        vTaskDelete(NULL);
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));  // Let camera stabilize
    
    // Check heap after camera restart
    if (!heap_caps_check_integrity(MALLOC_CAP_SPIRAM, true)) {
        ESP_LOGE(TAG, "❌ PSRAM heap corrupted after camera restart!");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "✅ TFLite arena allocated, camera restarted");
    ESP_LOGI(TAG, "✓ PSRAM heap integrity still OK");
    ESP_LOGI(TAG, "Starting continuous gesture inference loop...");

    uint32_t frame_count = 0;
    uint32_t detect_count = 0;

    for (;;) {
        confidence = 0.0f;
        gesture = ei_infer_gesture(&confidence);

        if (gesture && confidence > 0.5f) {
            detect_count++;
            handle_gesture(gesture, confidence);
        }

        frame_count++;
        if (frame_count % 50 == 0) {
            ESP_LOGI(TAG, "Stats: Frames=%lu, Detections=%lu", frame_count, detect_count);
            
            // Periodic heap check
            if (!heap_caps_check_integrity(MALLOC_CAP_SPIRAM, false)) {
                ESP_LOGE(TAG, "❌ PSRAM heap corruption detected during runtime!");
                break;
            }
        }

        // Run at ~3 FPS
        vTaskDelay(pdMS_TO_TICKS(333));
    }
    
    ESP_LOGE(TAG, "Task exiting due to errors");
    vTaskDelete(NULL);
}