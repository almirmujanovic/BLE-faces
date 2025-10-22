#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "store/config/ble_store_config.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"

#ifndef BLE_GAP_APPEARANCE_HID_GENERIC
#define BLE_GAP_APPEARANCE_HID_GENERIC 0x03C0
#endif
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_store.h"

#include "ble_server.h"
#include "ble_image_transfer.h"   
#include "ble_hid.h"
#include "ble_gatts_utils.h"
extern void ble_store_config_init(void);

static const char *TAG = "BLE_TASK";

static void start_advertising(void);
static int gap_event_cb(struct ble_gap_event *ev, void *arg);
static void on_reset(int reason);
static void on_sync(void);
static void host_task(void *param);
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

// wipe bonds


// UUIDs
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x00,0xfa,0x10);
static const ble_uuid128_t CHR_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x01,0xfa,0x10);
static const ble_uuid128_t FACE_NOTIFY_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x02,0xfa,0x10);

// GATT characteristics
// --- In your BLE server file (not function-local) ---
static const struct ble_gatt_dsc_def face_notify_descs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16),
        .access_cb = face_notify_desc_cb,
        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
    },
    { 0 }
};

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
        .descriptors = face_notify_descs,
    },
    { 0 }
};

// Define the GATT service table
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

/**
 * NEW: Helper can be called from detection.
 * It keeps existing notification flow intact and then kicks off
 * the chunked image transfer via the separate Image Transfer service.
 */
void ble_send_face_notification_and_crop(uint32_t frame_id, uint8_t face_count,
                                         uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                         float confidence,
                                         const uint8_t *rgb565, uint32_t rgb_len)
{
    // 1) existing metadata notification
    ble_send_face_notification(frame_id, face_count, x, y, width, height, confidence);

    // 2) image transfer (INFO notify -> chunked DATA notifies)
    if (rgb565 && rgb_len > 0) {
        (void)ble_img_xfer_send_rgb565(frame_id, x, y, width, height, rgb565, rgb_len);
    }
}

// GAP event callback
static int gap_event_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            g_conn_handle = ev->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, handle=%u", g_conn_handle);

            // Set connection handles for both services
            ble_hid_set_conn(g_conn_handle);
            ble_img_xfer_on_connect(g_conn_handle);

            // Initiate security for iOS pairing
            ESP_LOGI(TAG, "Initiating security to force pairing...");
            if (ble_gap_security_initiate(g_conn_handle) != 0) {
                ESP_LOGW(TAG, "Security initiate failed");
            }
        } else {
            ESP_LOGE(TAG, "Connection failed; status=%d", ev->connect.status);
            start_advertising();
        }
        break;
        
     case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", ev->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_img_xfer_on_disconnect();
        start_advertising();
        break;
        
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated to %u bytes", ev->mtu.value);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "SUBSCRIBE: handle=%u cur=%d prev=%d reason=%d",
                 ev->subscribe.attr_handle,
                 ev->subscribe.cur_notify,
                 ev->subscribe.prev_notify,
                 ev->subscribe.reason);
        
            // HID
        ble_hid_check_cccd_subscribe(ev->subscribe.attr_handle, ev->subscribe.cur_notify);

        // Image Xfer (INFO/DATA)
        ble_img_xfer_on_subscribe(ev->subscribe.attr_handle, ev->subscribe.cur_notify);

        // Face notify CCCD (value handle + 1 is CCCD)
        if (ev->subscribe.attr_handle == (uint16_t)(g_face_notify_handle + 1)) {
            g_notify_enabled = (ev->subscribe.cur_notify != 0);
            ESP_LOGI(TAG, "FACE notify %s", g_notify_enabled ? "ENABLED" : "DISABLED");
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (ev->enc_change.status == 0) {
            ESP_LOGI(TAG, "🔐 Successfully paired and encrypted!");
        } else {
            ESP_LOGW(TAG, "Encryption change failed: %d", ev->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:  // resume pumping image chunks when TX frees
        ble_img_xfer_on_notify_tx();                               
        break;
        
    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields adv;   // primary advertising data
    struct ble_hs_adv_fields rsp;   // scan response data
    int rc;

    // --- Primary ADV packet: flags + tx power + appearance + HID service UUID ---
    memset(&adv, 0, sizeof(adv));
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.tx_pwr_lvl_is_present = 1;
    adv.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // GAP Appearance: HID Generic (you can swap to KEYBOARD/MOUSE if you prefer)
    adv.appearance = BLE_GAP_APPEARANCE_HID_GENERIC;   // 0x03C0
    adv.appearance_is_present = 1;

    // Include HID service UUID (0x1812) in the adv (lets iOS classify you as HID)
    static const ble_uuid16_t hid_uuid16 = BLE_UUID16_INIT(0x1812);
    adv.uuids16 = (ble_uuid16_t *)&hid_uuid16;
    adv.num_uuids16 = 1;
    adv.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set ADV fields: %d", rc);
        return;
    }

    // --- Scan Response: put the (long) device name here so it won't get truncated ---
    memset(&rsp, 0, sizeof(rsp));
    const char *name = "ESP-Face-Detector";
    rsp.name = (uint8_t *)name;
    rsp.name_len = strlen(name);
    rsp.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response fields: %d", rc);
        return;
    }

    // --- Params and start ---
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // connectable undirected
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // general discoverable
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE advertising started (HID appearance + 0x1812 in ADV, name in scan response)");
}


static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason: %d", reason);
}

static void on_sync(void)
{
    // Start all registered GATT services once, from on_sync
    int rc = ble_gatts_start();
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "All GATT services started");

    // Now that services are started, resolve handles
    ble_img_xfer_resolve_handles();      // INFO/DATA handles
    ESP_ERROR_CHECK(ble_hid_get_handles()); // HID value/CCCD handles

    // Set preferred MTU (optional, here is fine)
    ble_att_set_preferred_mtu(247);



    uint8_t own_addr_type;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "infer addr type rc=%d", rc); return; }
    g_own_addr_type = own_addr_type;           // keep this global
    // Optional but nice: log the address we’re using.
    // Get and log our BLE address
    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_copy_addr rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Using addr type=%u %02X:%02X:%02X:%02X:%02X:%02X",
            g_own_addr_type,
            addr_val[5], addr_val[4], addr_val[3],
            addr_val[2], addr_val[1], addr_val[0]);

    start_advertising();
}


// Good ble_start_all(): register everything, set callbacks. Do NOT start or resolve yet.
static void ble_start_all(const char *device_name)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    nimble_port_init();
    ble_store_config_init();

    // Configure NimBLE host callbacks BEFORE the host starts
    ble_hs_cfg.store_status_cb = NULL;
    ble_hs_cfg.reset_cb        = on_reset;
    ble_hs_cfg.sync_cb         = on_sync;

    // Core GAP/GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (device_name && *device_name) {
        ble_svc_gap_device_name_set(device_name);
    }

    // 1) Register HID (count + add only)
    ESP_ERROR_CHECK(ble_hid_init());

    // 2) Register your FACE service (count + add only)
    int rc = ble_gatts_count_cfg(g_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(g_svcs);
    assert(rc == 0);

    // (optional) sanity_check_tables(...) + dump_table(...)

    // 3) Register IMAGE XFER (count + add only; do not resolve yet)
    ble_img_xfer_init();

    // Do NOT call ble_gatts_start() here.
    // Do NOT resolve handles here.
    // Do NOT set MTU here.

    // Start the NimBLE host task; on_sync() will run shortly.
    nimble_port_freertos_init(host_task);
}


static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/*
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

    vTaskDelay(1000);
    // register the image transfer service (separate GATT service)
    ble_img_xfer_init();  

    ble_att_set_preferred_mtu(247);

    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);
    vTaskDelete(NULL);
}
*/

void ble_task(void *param){
    const char *device_name = (const char*) param;
    ble_start_all(device_name);
    //nimble_port_freertos_init(host_task);
    vTaskDelete(NULL);
}
void ble_server_start(const char *device_name)
{
    xTaskCreatePinnedToCore(ble_task, "ble_task", 8192, (void*)device_name, 5, NULL, tskNO_AFFINITY);
}