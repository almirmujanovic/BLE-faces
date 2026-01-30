#include "include/wifi_control.h"
#include "include/hand_gesture_ml.h"
#include "include/system_state.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <lwip/inet.h>
#include <string.h>

extern httpd_handle_t start_http_server(void); // your existing server creator

static const char *TAG = "WIFI_CTRL";
static bool s_running = false;
static httpd_handle_t s_server = NULL;
static esp_netif_t *s_ap_netif = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static bool s_event_handler_registered = false;
static bool s_wifi_initialized = false;  // Track if esp_wifi_init() was called
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
    
    // Request gesture task pause and unload models on the gesture-task core
    system_set_pause_for_wifi(true);
    esp_err_t unload_err = hand_gesture_ml_request_unload_and_wait(5000);
    if (unload_err != ESP_OK) {
        ESP_LOGW(TAG, "Gesture unload did not complete in time; aborting WiFi start");
        system_set_pause_for_wifi(false);
        return unload_err;
    }

    
    // Log memory BEFORE WiFi start
    ESP_LOGI(TAG, "📶 BEFORE WiFi - Internal free: %u, largest block: %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    
    ESP_LOGI(TAG, "Starting WiFi AP + HTTP server...");

    // Note: esp_netif_init() and esp_event_loop_create_default() 
    // are called once in app_main() - don't call them here

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

    // Always create fresh netif (we destroy it on stop to free RAM)
    if (!s_ap_netif) {
        ESP_LOGI(TAG, "Creating AP netif...");
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) {
            ESP_LOGE(TAG, "Failed to create default WiFi AP netif");
            return ESP_FAIL;
        }
    }

    // Configure static AP IP before starting WiFi/DHCP
    esp_netif_dhcps_stop(s_ap_netif);
    esp_netif_ip_info_t ap_ip_info;
    IP4_ADDR(&ap_ip_info.ip, 192, 168, 1, 100);
    IP4_ADDR(&ap_ip_info.gw, 192, 168, 1, 100);
    IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(s_ap_netif, &ap_ip_info);

    // Initialize WiFi driver (we deinit on stop to free RAM)
    if (!s_wifi_initialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
            return err;
        }
        s_wifi_initialized = true;
        ESP_LOGI(TAG, "WiFi driver initialized");
    }

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

    xEventGroupClearBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_AP_STARTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(3000));
    if ((bits & WIFI_AP_STARTED_BIT) == 0) {
        ESP_LOGW(TAG, "WiFi AP start timeout; continuing anyway");
    }

    // Start DHCP server (192.168.1.101+)
    esp_err_t dhcp_err = esp_netif_dhcps_start(s_ap_netif);
    if (dhcp_err == ESP_OK || dhcp_err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGI(TAG, "DHCP server ready");
    } else {
        ESP_LOGW(TAG, "DHCP restart issue: %s", esp_err_to_name(dhcp_err));
    }

    // Verify AP IP
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(s_ap_netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "AP IP: " IPSTR, IP2STR(&ip_info.ip));
    }

    // start HTTP server
    s_server = start_http_server();
    if (!s_server) {
        ESP_LOGW(TAG, "HTTP server failed to start");
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

    // Stop DHCP server
    if (s_ap_netif) {
        esp_netif_dhcps_stop(s_ap_netif);
    }

    // Stop WiFi
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop failed: %s", esp_err_to_name(err));
    }

    s_running = false;
    
    // Reboot to restore ML models with clean (unfragmented) memory
    ESP_LOGI(TAG, "🔄 Rebooting to restore hand gesture recognition...");
    vTaskDelay(pdMS_TO_TICKS(500));  // Let log flush
    esp_restart();
    
    // Never reached
    return ESP_OK;
}

bool wifi_control_is_running(void) { return s_running; }
httpd_handle_t wifi_control_get_server_handle(void) { return s_server; }
