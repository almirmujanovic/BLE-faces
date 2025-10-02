#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "include/camera_config.h"
#include "include/face_detection.h"
#include "include/ble_server.h"

static const char *TAG = "BLE_FACES";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting BLE Face Detection Application");

    // Camera
    esp_err_t ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return;
    }

    // Face detector
    ret = init_face_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Face detection initialization failed");
        return;
    }

    // Detection task
    BaseType_t ok = xTaskCreate(face_detection_task, "face_detection_task", 8192, NULL, 6, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create face_detection_task");
        return;
    }

    ble_server_start("ESP-NimBLE-Server");
    ESP_LOGI("APP", "Started BLE server task.");

    //validate BLE server starts
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Application started successfully");}