#include "include/button.h"
#include "include/leds.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

static const char *TAG = "BUTTON";

static TaskHandle_t s_btn_task = NULL;
static button_cb_t s_click_cb = NULL;
static button_cb_t s_hold_cb  = NULL;
static uint32_t s_poll_ms = 50;
static uint32_t s_click_max_ms = 500;
static uint32_t s_hold_min_ms = 2000;
static bool s_running = false;

static void button_task(void *pv)
{
    bool last_state = false;
    TickType_t press_start = 0;
    bool hold_reported = false;

    while (s_running) {
        bool cur = leds_button_is_pressed(); // read PCA9536 input

        if (cur && !last_state) {
            // pressed now
            press_start = xTaskGetTickCount();
            hold_reported = false;
        } else if (!cur && last_state) {
            // released now
            TickType_t now = xTaskGetTickCount();
            uint32_t dur_ms = pdTICKS_TO_MS(now - press_start);
            if (dur_ms <= s_click_max_ms) {
                if (s_click_cb) s_click_cb();
            } else {
                // if hold already reported, ignore; otherwise call hold on release
                if (!hold_reported && s_hold_cb) s_hold_cb();
            }
            press_start = 0;
            hold_reported = false;
        } else if (cur && !hold_reported && press_start != 0) {
            TickType_t now = xTaskGetTickCount();
            uint32_t dur_ms = pdTICKS_TO_MS(now - press_start);
            if (dur_ms >= s_hold_min_ms) {
                // report hold once while pressed
                if (s_hold_cb) s_hold_cb();
                hold_reported = true;
            }
        }

        last_state = cur;
        vTaskDelay(pdMS_TO_TICKS(s_poll_ms));
    }
}

esp_err_t button_start(button_cb_t click_cb, button_cb_t hold_cb,
                       uint32_t polling_ms, uint32_t click_max_ms, uint32_t hold_min_ms)
{
    if (s_running) return ESP_OK;
    s_click_cb = click_cb;
    s_hold_cb  = hold_cb;
    s_poll_ms = (polling_ms == 0) ? 50 : polling_ms;
    s_click_max_ms = (click_max_ms == 0) ? 500 : click_max_ms;
    s_hold_min_ms  = (hold_min_ms == 0) ? 2000 : hold_min_ms;

    // ensure PCA9536 input is configured
    esp_err_t err = leds_configure_button_input();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Button input unavailable: %s", esp_err_to_name(err));
        return err;
    }

    s_running = true;
    // increased stack from 2048 to 4096 to avoid stack overflow
    if (xTaskCreate(button_task, "button_task", 4096, NULL, 5, &s_btn_task) != pdPASS) {
        s_running = false;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Button poll started");
    return ESP_OK;
}

void button_stop(void)
{
    s_running = false;
    // task will self-delete
}
