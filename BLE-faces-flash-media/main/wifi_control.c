#include "include/wifi_control.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include <string.h>

extern httpd_handle_t start_http_server(void); // your existing server creator

static const char *TAG = "WIFI_CTRL";
static bool s_running = false;
static httpd_handle_t s_server = NULL;

esp_err_t wifi_control_start(void)
{
    if (s_running) return ESP_OK;

    esp_err_t err;
    ESP_LOGI(TAG, "Starting WiFi AP + HTTP server...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

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