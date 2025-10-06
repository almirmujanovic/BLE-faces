#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ble_image_transfer.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "os/os_mbuf.h"

static const char *TAG = "IMG_XFER";

// Connection state
static uint16_t g_conn = BLE_HS_CONN_HANDLE_NONE;
static bool g_info_ntf = false, g_data_ntf = false;

// Attribute handles
static uint16_t h_tx_info, h_tx_data;

// Transfer state (no direct PSRAM access from BLE callbacks)
static TaskHandle_t pump_task_handle = NULL;
static QueueHandle_t pump_queue = NULL;

// Transfer state structure
typedef struct {
    uint8_t *data_copy;      // Internal RAM copy for BLE operations
    uint32_t total_len;
    uint32_t sent;
    bool active;
} transfer_state_t;

static transfer_state_t g_transfer = {0};

// Pump task message
typedef struct {
    enum { PUMP_START, PUMP_CONTINUE, PUMP_CLEANUP } cmd;
    uint8_t *data;
    uint32_t len;
} pump_msg_t;

// UUID declarations 
static const ble_uuid128_t img_svc_uuid = 
    BLE_UUID128_INIT(BLE_IMG_SVC_UUID128);
static const ble_uuid128_t img_rx_uuid = 
    BLE_UUID128_INIT(BLE_IMG_RX_UUID128);
static const ble_uuid128_t img_info_uuid = 
    BLE_UUID128_INIT(BLE_IMG_INFO_UUID128);
static const ble_uuid128_t img_data_uuid = 
    BLE_UUID128_INIT(BLE_IMG_DATA_UUID128);

// Forward declarations
static int img_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);
static int img_info_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
static int img_data_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);

// Service definition
static const struct ble_gatt_svc_def img_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &img_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &img_rx_uuid.u,
                .access_cb = img_rx_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &img_info_uuid.u,
                .access_cb = img_info_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &h_tx_info,
            },
            {
                .uuid = &img_data_uuid.u,
                .access_cb = img_data_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &h_tx_data,
            },
            {
                0,
            },
        },
    },
    {
        0,
    },
};

// Access callbacks 
static int img_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "RX write: %.*s", ctxt->om->om_len, ctxt->om->om_data);
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int img_info_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return BLE_ATT_ERR_UNLIKELY;
}

static int img_data_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return BLE_ATT_ERR_UNLIKELY;
}

//Transfer cleanup
static void cleanup_transfer(void)
{
    if (g_transfer.data_copy) {
        ESP_LOGD(TAG, "Freeing transfer buffer");
        free(g_transfer.data_copy);  // Internal RAM buffer
        g_transfer.data_copy = NULL;
    }
    g_transfer.total_len = 0;
    g_transfer.sent = 0;
    g_transfer.active = false;
}

// Pump task (runs in separate task context, not BLE callback)
static void pump_task(void *pvParameters)
{
    pump_msg_t msg;
    
    ESP_LOGI(TAG, "Pump task started");
    
    while (1) {
        if (xQueueReceive(pump_queue, &msg, portMAX_DELAY) == pdTRUE) {
            switch (msg.cmd) {
                case PUMP_START:
                    ESP_LOGD(TAG, "Starting pump with %lu bytes", msg.len);
                    // Data is already copied to internal RAM, just update state
                    g_transfer.data_copy = msg.data;
                    g_transfer.total_len = msg.len;
                    g_transfer.sent = 0;
                    g_transfer.active = true;
                    // Fall through to start pumping
                    
                case PUMP_CONTINUE:
                    if (!g_transfer.active || g_conn == BLE_HS_CONN_HANDLE_NONE || !g_data_ntf) {
                        break;
                    }
                    
                    // Calculate chunk size
                    int mtu = ble_att_mtu(g_conn);
                    int chunk_size = (mtu > 23) ? (mtu - 3) : 20;
                    
                    uint32_t remaining = g_transfer.total_len - g_transfer.sent;
                    if (remaining == 0) {
                        ESP_LOGI(TAG, "Transfer complete: %lu bytes", g_transfer.sent);
                        cleanup_transfer();
                        break;
                    }
                    
                    uint32_t to_send = (remaining > chunk_size) ? chunk_size : remaining;
                    
                    // Create mbuf from internal RAM data (safe)
                    struct os_mbuf *om = ble_hs_mbuf_from_flat(g_transfer.data_copy + g_transfer.sent, to_send);
                    if (!om) {
                        ESP_LOGE(TAG, "Failed to allocate mbuf");
                        cleanup_transfer();
                        break;
                    }
                    
                    int rc = ble_gattc_notify_custom(g_conn, h_tx_data, om);
                    if (rc == 0) {
                        g_transfer.sent += to_send;
                        ESP_LOGD(TAG, "Sent chunk: %lu/%lu bytes", g_transfer.sent, g_transfer.total_len);
                    } else {
                        ESP_LOGE(TAG, "Notify failed: %d", rc);
                        os_mbuf_free_chain(om);
                        cleanup_transfer();
                    }
                    break;
                    
                case PUMP_CLEANUP:
                    ESP_LOGD(TAG, "Cleanup requested");
                    cleanup_transfer();
                    break;
            }
        }
    }
}

// Public API
void ble_img_xfer_init(void)
{
    ESP_LOGI(TAG, "Initializing image transfer service...");
    
    // Create pump queue and task
    pump_queue = xQueueCreate(10, sizeof(pump_msg_t));
    if (!pump_queue) {
        ESP_LOGE(TAG, "Failed to create pump queue");
        return;
    }
    
    BaseType_t ret = xTaskCreate(pump_task, "ble_pump", 8192, NULL, 3, &pump_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create pump task");
        return;
    }
    
    int rc = ble_gatts_count_cfg(img_svc);
    ESP_LOGI(TAG, "count_cfg rc=%d", rc);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }
    
    rc = ble_gatts_add_svcs(img_svc);
    ESP_LOGI(TAG, "add_svcs rc=%d", rc);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "Image Transfer service registered successfully");
    ESP_LOGI(TAG, "INFO handle: %d, DATA handle: %d", h_tx_info, h_tx_data);
}

void ble_img_xfer_on_connect(uint16_t conn)
{
    ESP_LOGI(TAG, "Client connected: %d", conn);
    g_conn = conn;
    ble_att_set_preferred_mtu(247);
}

void ble_img_xfer_on_disconnect(void)
{
    ESP_LOGI(TAG, "Client disconnected");
    g_conn = BLE_HS_CONN_HANDLE_NONE;
    g_info_ntf = g_data_ntf = false;
    
    // Request cleanup via task
    if (pump_queue) {
        pump_msg_t msg = { .cmd = PUMP_CLEANUP };
        xQueueSend(pump_queue, &msg, 0);
    }
}

void ble_img_xfer_on_subscribe(uint16_t attr_handle, bool notify_enabled)
{
    ESP_LOGI(TAG, "Subscribe: handle=%d, enabled=%d", attr_handle, notify_enabled);
    if (attr_handle == h_tx_info) {
        g_info_ntf = notify_enabled;
        ESP_LOGI(TAG, "INFO notifications %s", notify_enabled ? "enabled" : "disabled");
    }
    if (attr_handle == h_tx_data) {
        g_data_ntf = notify_enabled;
        ESP_LOGI(TAG, "DATA notifications %s", notify_enabled ? "enabled" : "disabled");
    }
}

//  Called from BLE callback, just signals pump task
void ble_img_xfer_on_notify_tx(void)
{
    if (pump_queue && g_transfer.active) {
        pump_msg_t msg = { .cmd = PUMP_CONTINUE };
        // Non-blocking send from BLE callback context
        xQueueSend(pump_queue, &msg, 0);
    }
}

// Frame sending with internal RAM copy
bool ble_img_xfer_send_frame(const ble_img_info_t *info,
                             const uint8_t *data, uint32_t len)
{
    if (g_conn == BLE_HS_CONN_HANDLE_NONE || !g_info_ntf || !g_data_ntf) {
        ESP_LOGW(TAG, "Client not ready for transfer");
        return false;
    }
    
    // If already transferring, reject new request
    if (g_transfer.active) {
        ESP_LOGW(TAG, "Transfer already in progress, dropping frame");
        return false;
    }
    
    ESP_LOGI(TAG, "Sending frame: %dx%d, %lu bytes", info->width, info->height, len);
    
    // Copy data to internal RAM for safe BLE access
    uint8_t *internal_copy = (uint8_t*)malloc(len);
    if (!internal_copy) {
        ESP_LOGE(TAG, "Failed to allocate internal RAM buffer (%lu bytes)", len);
        return false;
    }
    
    // Copy from PSRAM to internal RAM
    memcpy(internal_copy, data, len);
    ESP_LOGD(TAG, "Copied %lu bytes from PSRAM to internal RAM", len);
    
    // Send INFO notification
    struct os_mbuf *om = ble_hs_mbuf_from_flat(info, sizeof(ble_img_info_t));
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate INFO mbuf");
        free(internal_copy);
        return false;
    }
    
    int rc = ble_gattc_notify_custom(g_conn, h_tx_info, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send INFO: %d", rc);
        os_mbuf_free_chain(om);
        free(internal_copy);
        return false;
    }
    
    // Start pumping via task
    if (pump_queue) {
        pump_msg_t msg = { 
            .cmd = PUMP_START, 
            .data = internal_copy, 
            .len = len 
        };
        if (xQueueSend(pump_queue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to queue pump start");
            free(internal_copy);
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Pump queue not initialized");
        free(internal_copy);
        return false;
    }
    
    return true;
}

bool ble_img_xfer_send_rgb565(uint32_t frame_id,
                              uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              const uint8_t *rgb565, uint32_t len)
{
    ble_img_info_t info = {
        .version = 1, 
        .format = BLE_IMG_FMT_RGB565,
        .width = w, 
        .height = h, 
        .x = x, 
        .y = y,
        .total_len = len, 
        .frame_id = frame_id, 
        .crc32 = 0
    };
    return ble_img_xfer_send_frame(&info, rgb565, len);
}