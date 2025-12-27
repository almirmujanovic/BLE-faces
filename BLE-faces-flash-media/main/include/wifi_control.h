#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start AP + HTTP server. If server already running returns ESP_OK.
// http_server_start will call your existing start_http_server() internally.
esp_err_t wifi_control_start(void);

// Stop HTTP server + WiFi. Safe to call if not running.
esp_err_t wifi_control_stop(void);

// Query whether AP+HTTP server running
bool wifi_control_is_running(void);

// Access the httpd handle (NULL if not running)
httpd_handle_t wifi_control_get_server_handle(void);

#ifdef __cplusplus
}
#endif