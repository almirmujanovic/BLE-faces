#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs.h"
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

// If we reset while connected, clear bond data on next boot.
#define BLE_CONN_MAGIC 0x434F4E4E

RTC_NOINIT_ATTR static uint32_t s_ble_conn_magic;
RTC_NOINIT_ATTR static uint32_t s_ble_conn_active;

// Persistent connection flag for power-cycle detection.
#define BLE_STATE_NS  "ble_state"
#define BLE_STATE_KEY "conn_active"

static void ble_clear_bonds_on_reset(void);
static bool ble_get_persisted_conn_active(bool *found);
static void ble_set_persisted_conn_active(bool active);
static bool ble_namespace_has_entries(const char *ns);

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

// wipe bonds


// UUIDs
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x00,0xfa,0x10);
static const ble_uuid128_t CHR_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x01,0xfa,0x10);
static const ble_uuid128_t FACE_NOTIFY_UUID =
    BLE_UUID128_INIT(0xf0,0xde,0xac,0xbe,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0xaa,0x00,0x02,0xfa,0x10);

// GATT characteristics
// UUID for CCCD descriptor (must be static, not compound literal)
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
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || !g_notify_enabled) {
        return; // No connection
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
            g_notify_enabled = false;
            s_ble_conn_active = 1;
            ble_set_persisted_conn_active(true);
            ESP_LOGI(TAG, "Connected, handle=%u", g_conn_handle);
            ESP_LOGI(TAG, "CONNECT status=%d handle=%u", ev->connect.status, ev->connect.conn_handle);
            // Set connection handles for both services
            ble_hid_set_conn(g_conn_handle);
            ble_img_xfer_on_connect(g_conn_handle);
           // ble_hid_start_demo_task();

            // Initiate security for iOS pairing
            ESP_LOGI(TAG, "Initiating security to force pairing...");
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(g_conn_handle, &desc);
            if (rc == 0 && !desc.sec_state.encrypted) {
                rc = ble_gap_security_initiate(g_conn_handle);
                ESP_LOGI(TAG, "Security initiate rc=%d", rc);
                if (rc != 0) {
                    ESP_LOGW(TAG, "Security initiate failed");
                }
            }
        } else {
            ESP_LOGE(TAG, "Connection failed; status=%d", ev->connect.status);
            //ble_app_set_random_addr();
            start_advertising();
        }
        break;
        
     case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", ev->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_notify_enabled = false;
        s_ble_conn_active = 0;
        ble_set_persisted_conn_active(false);
        ble_hid_set_conn(BLE_HS_CONN_HANDLE_NONE);
        ble_img_xfer_on_disconnect();
        ESP_LOGW(TAG, "DISCONNECT reason=%d", ev->disconnect.reason);
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
            ESP_LOGI(TAG, "SUBSCRIBE attr=0x%04x cur=%d", ev->subscribe.attr_handle, ev->subscribe.cur_notify);
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
            ESP_LOGI(TAG, "Successfully paired and encrypted");
        } else {
            ESP_LOGW(TAG, "Encryption change failed: %d", ev->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(ev->repeat_pairing.conn_handle, &desc);
        if (rc == 0) {
            ESP_LOGW(TAG, "Repeat pairing; deleting old bond");
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

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

    // --- Prepare UUID arrays that outlive this function ---
    static const ble_uuid16_t s_uuid16_hid = BLE_UUID16_INIT(0x1812);
    // Your image service UUID (matches ble_image_transfer.h BLE_IMG_SVC_UUID128)
    static const ble_uuid128_t s_uuid128_img = BLE_UUID128_INIT(BLE_IMG_SVC_UUID128);

    // --- Try to fit both UUIDs into the primary ADV first ---
    memset(&adv, 0, sizeof(adv));
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    // TX power is nice but optional; leave out to save bytes
    // adv.tx_pwr_lvl_is_present = 1;
    // adv.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // HID appearance (Keyboard or Generic HID both OK)
    adv.appearance_is_present = 1;
    adv.appearance = 0x03C1; // HID Keyboard (better iOS compatibility)

    // Include BOTH service UUID lists in primary ADV (best for iOS filtering)
    adv.uuids16 = (ble_uuid16_t *)&s_uuid16_hid;
    adv.num_uuids16 = 1;
    adv.uuids16_is_complete = 1;

    adv.uuids128 = (ble_uuid128_t *)&s_uuid128_img;
    adv.num_uuids128 = 1;
    adv.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv);

    bool moved_img_uuid_to_rsp = false;
    if (rc == BLE_HS_EMSGSIZE) {
        // Too big. Keep 0x1812 in ADV; move 128-bit image UUID to scan response.
        ESP_LOGW(TAG, "ADV payload full; moving 128-bit Image UUID to scan response");
        memset(&adv, 0, sizeof(adv));
        adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
        adv.appearance_is_present = 1;
        adv.appearance = 0x03C1;
        adv.uuids16 = (ble_uuid16_t *)&s_uuid16_hid;
        adv.num_uuids16 = 1;
        adv.uuids16_is_complete = 1;

        rc = ble_gap_adv_set_fields(&adv);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to set ADV fields (fallback): %d", rc);
            return;
        }
        moved_img_uuid_to_rsp = true;
    } else if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set ADV fields: %d", rc);
        return;
    }

    // --- Scan Response: name (maybe shortened) and 128-bit UUID if moved ---
    memset(&rsp, 0, sizeof(rsp));

    // Put device name in scan response (safer vs truncation).
    const char *full_name = "ESP";
    rsp.name = (uint8_t *)full_name;
    rsp.name_len = strlen(full_name);
    rsp.name_is_complete = 1;

    if (moved_img_uuid_to_rsp) {
        rsp.uuids128 = (ble_uuid128_t *)&s_uuid128_img;
        rsp.num_uuids128 = 1;
        rsp.uuids128_is_complete = 1;

        rc = ble_gap_adv_rsp_set_fields(&rsp);
        if (rc == BLE_HS_EMSGSIZE) {
            // Still too big: shorten the name (incomplete name flag)
            ESP_LOGW(TAG, "Scan response full; shortening name");
            static char short_name[12]; // stays in .bss
            snprintf(short_name, sizeof(short_name), "ESP-Face");
            rsp.name = (uint8_t *)short_name;
            rsp.name_len = strlen(short_name);
            rsp.name_is_complete = 0; // indicate truncated name

            rc = ble_gap_adv_rsp_set_fields(&rsp);
        }
    } else {
        // 128-bit UUID already in primary ADV; scan response only carries name
        rc = ble_gap_adv_rsp_set_fields(&rsp);
        if (rc == BLE_HS_EMSGSIZE) {
            ESP_LOGW(TAG, "Scan response name too long; shortening");
            static char short_name[12];
            snprintf(short_name, sizeof(short_name), "ESP-Face");
            rsp.name = (uint8_t *)short_name;
            rsp.name_len = strlen(short_name);
            rsp.name_is_complete = 0;
            rc = ble_gap_adv_rsp_set_fields(&rsp);
        }
    }

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response fields: %d", rc);
        return;
    }

    // --- Params and start ---
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // connectable undirected
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // general discoverable
    adv_params.itvl_min  = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max  = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started:");
    ESP_LOGI(TAG, "  ADV: flags+appearance+%s%s",
             "0x1812", moved_img_uuid_to_rsp ? "" : " + IMG-UUID128");
    ESP_LOGI(TAG, "  RSP: %s%s",
             "name",
             moved_img_uuid_to_rsp ? " + IMG-UUID128" : "");
}


static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason: %d", reason);
}

static void on_sync(void)
{
    // Start services first (as you already do)
    int rc = ble_gatts_start();
    if (rc != 0) { ESP_LOGE(TAG, "ble_gatts_start rc=%d", rc); return; }
    ESP_LOGI(TAG, "All GATT services started");

    ble_img_xfer_resolve_handles();
    ESP_ERROR_CHECK(ble_hid_get_handles());

    ble_att_set_preferred_mtu(247);

    // Figure out own address type safely
    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        g_own_addr_type = BLE_OWN_ADDR_RANDOM; // last-ditch
    }

    // Log the actual address we'll use
    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Using own_addr_type=%u (%s) %02X:%02X:%02X:%02X:%02X:%02X",
                 g_own_addr_type,
                 (g_own_addr_type == BLE_OWN_ADDR_PUBLIC ? "PUBLIC" : "RANDOM/RPA"),
                 addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    } else {
        ESP_LOGW(TAG, "ble_hs_id_copy_addr rc=%d", rc);
    }

    start_advertising();
}

static void ble_clear_bonds_on_reset(void)
{
    static const char *NS_BOND = "nimble_bond";
    static const char *NS_SVC  = "nimble_svc";
    bool rtc_conn_active = false;
    bool nvs_conn_active = false;
    bool state_found = false;

    if (s_ble_conn_magic == BLE_CONN_MAGIC) {
        rtc_conn_active = (s_ble_conn_active != 0);
    } else {
        s_ble_conn_magic = BLE_CONN_MAGIC;
        s_ble_conn_active = 0;
    }

    nvs_conn_active = ble_get_persisted_conn_active(&state_found);
    if (!state_found) {
        if (ble_namespace_has_entries(NS_BOND) || ble_namespace_has_entries(NS_SVC)) {
            ESP_LOGW(TAG, "BLE state flag missing with existing NVS data; clearing for safety");
            nvs_conn_active = true;
        }
    }

    if (!rtc_conn_active && !nvs_conn_active) {
        return;
    }

    esp_reset_reason_t reason = esp_reset_reason();

    nvs_handle_t handle;
    esp_err_t err_bond = nvs_open(NS_BOND, NVS_READWRITE, &handle);
    if (err_bond == ESP_OK) {
        err_bond = nvs_erase_all(handle);
        if (err_bond == ESP_OK) {
            err_bond = nvs_commit(handle);
        }
        nvs_close(handle);
    }

    esp_err_t err_svc = nvs_open(NS_SVC, NVS_READWRITE, &handle);
    if (err_svc == ESP_OK) {
        err_svc = nvs_erase_all(handle);
        if (err_svc == ESP_OK) {
            err_svc = nvs_commit(handle);
        }
        nvs_close(handle);
    }

    if (err_bond == ESP_OK && err_svc == ESP_OK) {
        ESP_LOGW(TAG, "Cleared BLE bond/service store after reset (reason=%d)", reason);
    } else {
        if (err_bond != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear %s: %s", NS_BOND, esp_err_to_name(err_bond));
        }
        if (err_svc != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear %s: %s", NS_SVC, esp_err_to_name(err_svc));
        }
    }

    s_ble_conn_active = 0;
    ble_set_persisted_conn_active(false);
}

static bool ble_get_persisted_conn_active(bool *found)
{
    nvs_handle_t handle;
    uint8_t val = 0;
    if (found) {
        *found = false;
    }
    esp_err_t err = nvs_open(BLE_STATE_NS, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_u8(handle, BLE_STATE_KEY, &val);
        if (err == ESP_OK && found) {
            *found = true;
        }
        nvs_close(handle);
    }
    return val != 0;
}

static void ble_set_persisted_conn_active(bool active)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLE_STATE_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open %s: %s", BLE_STATE_NS, esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(handle, BLE_STATE_KEY, active ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to persist conn_active=%d: %s", active ? 1 : 0, esp_err_to_name(err));
    }
    nvs_close(handle);
}

static bool ble_namespace_has_entries(const char *ns)
{
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, ns, NVS_TYPE_ANY, &it);
    if (err == ESP_OK && it != NULL) {
        nvs_release_iterator(it);
        return true;
    }
    if (it != NULL) {
        nvs_release_iterator(it);
    }
    return false;
}


// Good ble_start_all(): register everything, set callbacks. Do NOT start or resolve yet.
static void ble_start_all(const char *device_name)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ble_clear_bonds_on_reset();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    nimble_port_init();
    ble_store_config_init();

    // Require bonding; Just Works (no IO)
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm    = 0;   // set 1 if you add passkey/pin UX later
    ble_hs_cfg.sm_sc      = 0;   // disable LE SC to avoid DHKey check failures
    ble_hs_cfg.sm_io_cap  = BLE_HS_IO_NO_INPUT_OUTPUT;

    // Distribute encryption and identity keys so iOS will remember you
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    // Configure NimBLE host callbacks BEFORE the host starts
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.reset_cb        = on_reset;
    ble_hs_cfg.sync_cb         = on_sync;

    //ble_app_set_random_addr();
    // Core GAP/GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (device_name && *device_name) {
        ble_svc_gap_device_name_set(device_name);
    }

    // 1) Register HID (count + add only)
    ESP_ERROR_CHECK(ble_hid_init());
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2) Register your FACE service (count + add only)
    sanity_check_tables(g_svcs);
    int rc = ble_gatts_count_cfg(g_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(g_svcs);
    assert(rc == 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    // (optional) sanity_check_tables(...) + dump_table(...)

    // 3) Register IMAGE XFER (count + add only; do not resolve yet)
    ble_img_xfer_init();
    vTaskDelay(pdMS_TO_TICKS(100));

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
