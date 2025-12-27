#include "include/wifi_control.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

extern httpd_handle_t start_http_server(void); // your existing server creator

static const char *TAG = "WIFI_CTRL";
static bool s_running = false;
static httpd_handle_t s_server = NULL;
static esp_netif_t *s_ap_netif = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static bool s_event_handler_registered = false;
#define WIFI_AP_STARTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base != WIFI_EVENT || !s_wifi_event_group) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_START) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
    }
}

esp_err_t wifi_control_start(void)
{
    if (s_running) return ESP_OK;

    esp_err_t err;
    ESP_LOGI(TAG, "Starting WiFi AP + HTTP server...");

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
        if (!s_wifi_event_group) {
            ESP_LOGE(TAG, "Failed to create WiFi event group");
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_event_handler_registered) {
        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(err));
            return err;
        }
        s_event_handler_registered = true;
    }

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) {
            ESP_LOGE(TAG, "Failed to create default WiFi AP netif");
            return ESP_FAIL;
        }
    } else {
        esp_netif_set_default_netif(s_ap_netif);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32-Media",
            .ssid_len = 0,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .channel = 1
        },
    };

    xEventGroupClearBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_AP_STARTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(3000));
    if ((bits & WIFI_AP_STARTED_BIT) == 0) {
        ESP_LOGW(TAG, "WiFi AP start timeout; continuing anyway");
    }

    // start HTTP server
    s_server = start_http_server();
    if (!s_server) {
        ESP_LOGW(TAG, "HTTP server failed to start");
        // do not fail WiFi; return OK but server NULL
    } else {
        ESP_LOGI(TAG, "HTTP server started");
    }

    s_running = true;
    return ESP_OK;
}

esp_err_t wifi_control_stop(void)
{
    if (!s_running) return ESP_OK;

    ESP_LOGI(TAG, "Stopping HTTP server + WiFi...");
    if (s_server) {
        esp_err_t r = httpd_stop(s_server);
        if (r != ESP_OK) ESP_LOGW(TAG, "httpd_stop failed: %s", esp_err_to_name(r));
        s_server = NULL;
    }

    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop failed: %s", esp_err_to_name(err));
    }
    err = esp_wifi_deinit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_deinit failed: %s", esp_err_to_name(err));
    }

    s_running = false;
    return ESP_OK;
}

bool wifi_control_is_running(void) { return s_running; }
httpd_handle_t wifi_control_get_server_handle(void) { return s_server; }
