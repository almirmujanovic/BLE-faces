#include <string.h>
#include "esp_log.h"
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

// Transfer state
static const uint8_t *cur_ptr = NULL;
static uint32_t cur_len = 0, sent = 0;

// FIXED: Proper UUID declarations
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

// FIXED: Proper service definition with correct structure
static const struct ble_gatt_svc_def img_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &img_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // RX characteristic (client writes commands)
                .uuid = &img_rx_uuid.u,
                .access_cb = img_rx_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // INFO characteristic (notify image metadata)
                .uuid = &img_info_uuid.u,
                .access_cb = img_info_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &h_tx_info,
            },
            {
                // DATA characteristic (notify image data chunks)
                .uuid = &img_data_uuid.u,
                .access_cb = img_data_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &h_tx_data,
            },
            {
                0, // Sentinel to end characteristics array
            },
        },
    },
    {
        0, // Sentinel to end services array
    },
};

// Access callback implementations
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
    // This is a notify-only characteristic
    return BLE_ATT_ERR_UNLIKELY;
}

static int img_data_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // This is a notify-only characteristic
    return BLE_ATT_ERR_UNLIKELY;
}

static void pump_chunks(void);

// Public API
void ble_img_xfer_init(void)
{
    ESP_LOGI(TAG, "Initializing image transfer service...");
    
    int rc = ble_gatts_count_cfg(img_svc);
    ESP_LOGI(TAG, "count_cfg rc=%d", rc);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return; // Don't assert, just return
    }
    
    rc = ble_gatts_add_svcs(img_svc);
    ESP_LOGI(TAG, "add_svcs rc=%d", rc);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return; // Don't assert, just return
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
    cur_ptr = NULL; 
    cur_len = sent = 0;
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

void ble_img_xfer_on_notify_tx(void)
{
    pump_chunks();
}

static void pump_chunks(void)
{
    if (g_conn == BLE_HS_CONN_HANDLE_NONE || !g_data_ntf || !cur_ptr || sent >= cur_len) {
        return;
    }

    int mtu = ble_att_mtu(g_conn);
    int chunk_size = (mtu > 23) ? (mtu - 3) : 20;
    
    uint32_t remaining = cur_len - sent;
    if (remaining == 0) return;
    
    uint32_t to_send = (remaining > chunk_size) ? chunk_size : remaining;
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(cur_ptr + sent, to_send);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate mbuf");
        return;
    }
    
    int rc = ble_gattc_notify_custom(g_conn, h_tx_data, om);
    if (rc == 0) {
        sent += to_send;
        if (sent >= cur_len) {
            ESP_LOGI(TAG, "Transfer complete: %lu bytes", sent);
            cur_ptr = NULL;
            cur_len = sent = 0;
        }
    } else {
        ESP_LOGE(TAG, "Notify failed: %d", rc);
        os_mbuf_free_chain(om);
    }
}

bool ble_img_xfer_send_frame(const ble_img_info_t *info,
                             const uint8_t *data, uint32_t len)
{
    if (g_conn == BLE_HS_CONN_HANDLE_NONE || !g_info_ntf || !g_data_ntf) {
        ESP_LOGW(TAG, "Client not ready for transfer");
        return false;
    }
    
    ESP_LOGI(TAG, "Sending frame: %dx%d, %lu bytes", info->width, info->height, len);
    
    // Send INFO notification
    struct os_mbuf *om = ble_hs_mbuf_from_flat(info, sizeof(*info));
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate INFO mbuf");
        return false;
    }
    
    int rc = ble_gattc_notify_custom(g_conn, h_tx_info, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send INFO: %d", rc);
        os_mbuf_free_chain(om);
        return false;
    }
    
    // Setup for DATA notifications
    cur_ptr = data;
    cur_len = len;
    sent = 0;
    
    pump_chunks(); // Start sending
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