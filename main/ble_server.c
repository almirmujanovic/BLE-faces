#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
#include "host/ble_store.h"
#include "ble_server.h"

static const char *TAG = "BLE_TASK";

// Connection tracking
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool g_notify_enabled = false;
static uint8_t g_own_addr_type;

// GATT handles
static uint16_t g_chr_val_handle;
static uint16_t g_face_notify_handle;

// Face detection notification data structure
typedef struct {
    uint32_t frame_id;
    uint8_t face_count;
    uint16_t face_x;
    uint16_t face_y;
    uint16_t face_width;
    uint16_t face_height;
    float confidence;
} __attribute__((packed)) face_notification_t;

// First characteristic - READ
static const char *g_hello = "ESP32-S3 Face Detection Server";

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, g_hello, strlen(g_hello)) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

// Face notification characteristic
static int face_notify_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0; // Empty read
    }
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

// Client Characteristic Configuration Descriptor
static int face_notify_desc_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        uint16_t notify_state = g_notify_enabled ? 0x0001 : 0x0000;
        return os_mbuf_append(ctxt->om, &notify_state, sizeof(notify_state)) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } 
    
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        uint8_t data[4] = {0};
        int len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > 4) len = 4;
        
        os_mbuf_copydata(ctxt->om, 0, len, data);
        
        // Check for notification enable (0x0001) or disable (0x0000)
        bool new_notify_state = false;
        if (len >= 2) {
            uint16_t cccd_value = data[0] | (data[1] << 8);
            new_notify_state = (cccd_value & 0x0001) != 0;
        } else if (len == 1) {
            new_notify_state = (data[0] != 0);
        }
        
        if (new_notify_state != g_notify_enabled) {
            g_notify_enabled = new_notify_state;
            ESP_LOGI(TAG, "Notifications %s by client", g_notify_enabled ? "enabled" : "disabled");
        }
        
        return 0;
    }
    
    return BLE_ATT_ERR_UNLIKELY;
}

// UUIDs
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x00,0xfa,0x10);
static const ble_uuid128_t CHR_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x01,0xfa,0x10);
static const ble_uuid128_t FACE_NOTIFY_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x02,0xfa,0x10);

// GATT characteristics
static const struct ble_gatt_chr_def g_chrs[] = {
    {
        .uuid = &CHR_UUID.u,
        .access_cb = chr_access_cb,
        .val_handle = &g_chr_val_handle,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &FACE_NOTIFY_UUID.u,
        .access_cb = face_notify_access_cb,
        .val_handle = &g_face_notify_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .descriptors = (struct ble_gatt_dsc_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16),
                .access_cb = face_notify_desc_cb,
                .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
            },
            { 0 }
        }
    },
    { 0 }
};

static const struct ble_gatt_svc_def g_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = g_chrs,
    },
    { 0 }
};

// Send face detection notification to connected client
void ble_send_face_notification(uint32_t frame_id, uint8_t face_count, 
                                uint16_t x, uint16_t y, uint16_t width, uint16_t height, float confidence)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return; // No connection
    }

    // Auto-enable notifications to work around Android permission restrictions
    if (!g_notify_enabled) {
        g_notify_enabled = true;
    }

    face_notification_t notification = {
        .frame_id = frame_id,
        .face_count = face_count,
        .face_x = x,
        .face_y = y,
        .face_width = width,
        .face_height = height,
        .confidence = confidence
    };

    ESP_LOGI(TAG, "Sending notification: Frame %lu, Face at (%d,%d) %dx%d, confidence %.2f", 
             frame_id, x, y, width, height, confidence);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&notification, sizeof(notification));
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate notification buffer");
        return;
    }

    int rc = ble_gatts_notify_custom(g_conn_handle, g_face_notify_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send notification: error %d", rc);
    } else {
        ESP_LOGD(TAG, "Notification sent successfully");
    }
}

// GAP event callback
static int gap_event_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            g_conn_handle = ev->connect.conn_handle;
            ESP_LOGI(TAG, "Client connected (handle %u)", ev->connect.conn_handle);
            
            // Auto-enable notifications for immediate functionality
            g_notify_enabled = true;
            
        } else {
            ESP_LOGW(TAG, "Connection failed (status %d), restarting advertising", ev->connect.status);
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            struct ble_gap_adv_params advp = {0};
            advp.conn_mode = BLE_GAP_CONN_MODE_UND;
            advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
            ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Client disconnected (reason %d), restarting advertising", ev->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_notify_enabled = false;
        {
            struct ble_gap_adv_params advp = {0};
            advp.conn_mode = BLE_GAP_CONN_MODE_UND;
            advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
            ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
        }
        break;
        
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated to %u bytes", ev->mtu.value);
        break;
        
    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)"ESP-Face-Detector";
    fields.name_len = strlen("ESP-Face-Detector");
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE advertising started");
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type: %d", rc);
        return;
    }
    start_advertising();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_task(void *param) {
    const char *device_name = (const char *)param;

    ESP_ERROR_CHECK(nvs_flash_init());
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (device_name && strlen(device_name)) {
        ble_svc_gap_device_name_set(device_name);
    }

    int rc = ble_gatts_count_cfg(g_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(g_svcs);
    assert(rc == 0);

    ble_att_set_preferred_mtu(247);

    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);
    vTaskDelete(NULL);
}

void ble_server_start(const char *device_name)
{
    xTaskCreatePinnedToCore(ble_task, "ble_task", 4096, (void*)device_name, 5, NULL, tskNO_AFFINITY);
}