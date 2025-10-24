#include "ble_hid.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"

// ---------- Logging ----------
static const char *TAG = "HID_CC";

// ---------- UUIDs ----------
static const ble_uuid16_t UUID_HID_SVC           = BLE_UUID16_INIT(0x1812);
static const ble_uuid16_t UUID_HID_INFO          = BLE_UUID16_INIT(0x2A4A);
static const ble_uuid16_t UUID_HID_CTRL_POINT    = BLE_UUID16_INIT(0x2A4C);
static const ble_uuid16_t UUID_HID_REPORT_MAP    = BLE_UUID16_INIT(0x2A4B);
static const ble_uuid16_t UUID_HID_REPORT        = BLE_UUID16_INIT(0x2A4D); // Input
static const ble_uuid16_t UUID_HID_PROTO_MODE    = BLE_UUID16_INIT(0x2A4E);

static const ble_uuid16_t UUID_DSC_CCCD          = BLE_UUID16_INIT(0x2902);
static const ble_uuid16_t UUID_DSC_REPORT_REF    = BLE_UUID16_INIT(0x2908);

// ---------- HID constants ----------
#define REPORT_ID_CC  0x03
#define REPORT_TYPE_INPUT 0x01

// Same as your Arduino CONSUMER_REPORT_MAP: 7 bits → 1 byte payload.
static const uint8_t s_report_map[] = {
    0x05, 0x0C,              // Usage Page (Consumer)
    0x09, 0x01,              // Usage (Consumer Control)
    0xA1, 0x01,              // Collection (Application)
    0x85, REPORT_ID_CC,      //   Report ID (3)
    0x15, 0x00,              //   Logical Minimum (0)
    0x25, 0x01,              //   Logical Maximum (1)
    0x75, 0x01,              //   Report Size (1)
    0x95, 0x07,              //   Report Count (7 bits)
    0x09, 0xE9,              //   Volume Up
    0x09, 0xEA,              //   Volume Down
    0x09, 0xE2,              //   Mute
    0x09, 0xCD,              //   Play/Pause
    0x09, 0xB5,              //   Next
    0x09, 0xB6,              //   Previous
    0x09, 0xB7,              //   Stop
    0x81, 0x02,              //   Input (Data,Var,Abs)
    0x95, 0x01, 0x75, 0x01,  //   1 pad bit
    0x81, 0x03,              //   Input (Const,Var,Abs)
    0xC0                     // End Collection
};

// HID Information: ver 1.11, country=0, flags: normally connectable
static const uint8_t s_hid_info[4] = { 0x11, 0x01, 0x00, 0x00 };
static uint8_t       s_proto_mode  = 0x01; // Report mode only

// Report Reference (Report ID=3, Type=Input)
static const uint8_t s_report_ref[2] = { REPORT_ID_CC, REPORT_TYPE_INPUT };

// ---------- State ----------
static uint16_t s_conn              = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_input_val_handle  = 0;
static bool     s_input_notify_en   = false;

// ---------- Forward ----------
static int hid_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt*, void*);

// ---------- Descriptors / Characteristics ----------
static struct ble_gatt_dsc_def s_input_descs[] = {
    { .uuid = &UUID_DSC_CCCD.u,       .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE, .access_cb = hid_access },
    { .uuid = &UUID_DSC_REPORT_REF.u, .att_flags = BLE_ATT_F_READ,                   .access_cb = hid_access },
    { 0 }
};

static struct ble_gatt_chr_def s_hid_chrs[] = {
    { // Report Map
      .uuid = &UUID_HID_REPORT_MAP.u, .access_cb = hid_access, .flags = BLE_GATT_CHR_F_READ },
    { // Input Report (value handle stored for notify)
      .uuid = &UUID_HID_REPORT.u,     .access_cb = hid_access,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
      .val_handle = &s_input_val_handle, .descriptors = s_input_descs },
    { // HID Information
      .uuid = &UUID_HID_INFO.u,       .access_cb = hid_access, .flags = BLE_GATT_CHR_F_READ },
    { // Protocol Mode (Report only; allow write-no-rsp of 0x01)
      .uuid = &UUID_HID_PROTO_MODE.u, .access_cb = hid_access,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { // Control Point (Suspend/Exit Suspend; we just accept writes)
      .uuid = &UUID_HID_CTRL_POINT.u, .access_cb = hid_access,
      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
    { 0 }
};

static const struct ble_gatt_svc_def s_hid_svcs[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &UUID_HID_SVC.u, .characteristics = s_hid_chrs },
    { 0 }
};

// ---------- Access callback ----------
static int hid_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)attr; (void)arg;

    // Descriptor access
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        const ble_uuid_t *du = ctxt->dsc ? ctxt->dsc->uuid : NULL;
        if (!du) return BLE_ATT_ERR_UNLIKELY;

        // CCCD
        if (ble_uuid_cmp(du, &UUID_DSC_CCCD.u) == 0) {
            if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
                if (OS_MBUF_PKTLEN(ctxt->om) != 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                uint16_t cccd = ctxt->om->om_data[0] | (ctxt->om->om_data[1] << 8);
                s_input_notify_en = (cccd & 0x0001) != 0;
                ESP_LOGI(TAG, "Input CCCD %s", s_input_notify_en ? "ENABLED" : "DISABLED");
                return 0;
            } else {
                uint16_t cccd = s_input_notify_en ? 0x0001 : 0x0000;
                return os_mbuf_append(ctxt->om, &cccd, sizeof(cccd)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }

        // Report Reference
        if (ble_uuid_cmp(du, &UUID_DSC_REPORT_REF.u) == 0) {
            return os_mbuf_append(ctxt->om, s_report_ref, sizeof s_report_ref) == 0
                 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return BLE_ATT_ERR_UNLIKELY;
    }

    // Characteristic access
    const ble_uuid_t *cu = ctxt->chr ? ctxt->chr->uuid : NULL;
    if (!cu) return BLE_ATT_ERR_UNLIKELY;

    // Protocol Mode
    if (ble_uuid_cmp(cu, &UUID_HID_PROTO_MODE.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            return os_mbuf_append(ctxt->om, &s_proto_mode, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if (OS_MBUF_PKTLEN(ctxt->om) != 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            if (ctxt->om->om_data[0] != 0x01) return BLE_ATT_ERR_REQ_NOT_SUPPORTED; // report only
            s_proto_mode = 0x01;
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Report Map / HID Info / Input Report read
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (ble_uuid_cmp(cu, &UUID_HID_REPORT_MAP.u) == 0) {
            return os_mbuf_append(ctxt->om, s_report_map, sizeof s_report_map) == 0
                 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (ble_uuid_cmp(cu, &UUID_HID_INFO.u) == 0) {
            return os_mbuf_append(ctxt->om, s_hid_info, sizeof s_hid_info) == 0
                 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (ble_uuid_cmp(cu, &UUID_HID_REPORT.u) == 0) {
            uint8_t zero = 0x00; // one-byte report payload
            return os_mbuf_append(ctxt->om, &zero, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }

    // Control Point write: accept & ignore
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR &&
        ble_uuid_cmp(cu, &UUID_HID_CTRL_POINT.u) == 0) { return 0; }

    return BLE_ATT_ERR_UNLIKELY;
}

// ---------- Public: register service ----------
esp_err_t ble_hid_init(void)
{
    int rc = ble_gatts_count_cfg(s_hid_svcs);
    if (rc) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_hid_svcs);
    if (rc) { ESP_LOGE(TAG, "add_svcs rc=%d", rc);   return ESP_FAIL; }
    ESP_LOGI(TAG, "HID service registered");
    return ESP_OK;
}

// Resolve handles after ble_gatts_start() (called from your on_sync)
esp_err_t ble_hid_get_handles(void)
{
    if (s_input_val_handle == 0) {
        ESP_LOGE(TAG, "Input report handle unresolved");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "HID Input val handle=0x%04x", s_input_val_handle);
    return ESP_OK;
}

void ble_hid_set_conn(uint16_t conn_hdl) { s_conn = conn_hdl; }

// Map GAP SUBSCRIBE event to local notify flag (NimBLE gives value handle)
void ble_hid_check_cccd_subscribe(uint16_t attr_handle, uint16_t cur_notify)
{
    if (attr_handle == s_input_val_handle) {
        s_input_notify_en = (cur_notify != 0);
        ESP_LOGI(TAG, "Input notify %s", s_input_notify_en ? "ENABLED" : "DISABLED");
    }
}

// ---------- Send helper (1-byte payload, report ID is implied by characteristic+ReportRef) ----------
static esp_err_t cc_tap_bit(uint8_t bit_index)
{
    if (s_conn == BLE_HS_CONN_HANDLE_NONE)             return ESP_ERR_INVALID_STATE;
    if (!s_input_notify_en || s_input_val_handle == 0) return ESP_ERR_INVALID_STATE;

    // Press
    uint8_t v = (uint8_t)(1u << bit_index);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&v, 1);
    if (!om) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(s_conn, s_input_val_handle, om);
    if (rc) return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(90));

    // Release
    v = 0x00;
    om = ble_hs_mbuf_from_flat(&v, 1);
    if (!om) return ESP_ERR_NO_MEM;
    rc = ble_gatts_notify_custom(s_conn, s_input_val_handle, om);
    return rc ? ESP_FAIL : ESP_OK;
}

// Bit layout (must match s_report_map order)
#define BIT_VOL_UP       0
#define BIT_VOL_DOWN     1
#define BIT_MUTE         2
#define BIT_PLAY_PAUSE   3
#define BIT_NEXT         4
#define BIT_PREV         5
#define BIT_STOP         6
/*
esp_err_t ble_hid_cc_tap_vol_up(void)      { return cc_tap_bit(BIT_VOL_UP); }
esp_err_t ble_hid_cc_tap_vol_down(void)    { return cc_tap_bit(BIT_VOL_DOWN); }
esp_err_t ble_hid_cc_tap_play_pause(void)  { return cc_tap_bit(BIT_PLAY_PAUSE); }

// Demo task (optional, for dev)
static void demo_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_input_notify_en && s_conn != BLE_HS_CONN_HANDLE_NONE) {
            ble_hid_cc_tap_vol_up();     vTaskDelay(pdMS_TO_TICKS(800));
            ble_hid_cc_tap_vol_down();   vTaskDelay(pdMS_TO_TICKS(800));
            ble_hid_cc_tap_play_pause(); vTaskDelay(pdMS_TO_TICKS(1600));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
void ble_hid_start_demo_task(void)
{
    static bool started = false;
    if (!started) {
        xTaskCreate(demo_task, "hid_demo", 3072, NULL, 3, NULL);
        started = true;
    }
}
*/
esp_err_t ble_hid_tap_consumer_bits(uint8_t bits)
{
    if (s_conn == BLE_HS_CONN_HANDLE_NONE || !s_input_notify_en || s_input_val_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t v = (uint8_t)(bits & 0x7F);  // only 7 bits are valid per report map

    // press
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&v, 1);
    if (!om) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(s_conn, s_input_val_handle, om);
    if (rc) return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(60));

    // release
    v = 0x00;
    om = ble_hs_mbuf_from_flat(&v, 1);
    if (!om) return ESP_ERR_NO_MEM;
    rc = ble_gatts_notify_custom(s_conn, s_input_val_handle, om);
    return rc ? ESP_FAIL : ESP_OK;
}


/* -------------------------------------------------------------------------
 * Convenience wrappers that match the current REPORT MAP bit order
 * REPORT MAP order (16 bits):
 *  bit0:  Mute (0xE2)
 *  bit1:  Power (0x30)
 *  bit2:  Volume Up (0xE9)
 *  bit3:  Volume Down (0xEA)
 *  bit4:  Play/Pause (0xCD)
 *  bit5:  Next (0xB5)
 *  bit6:  Previous (0xB6)
 *  bit7:  Stop (0xB7)
 *  bit8:  Eject
 *  bit9:  Media Select
 *  bit10: AL Browser
 *  bit11: AL Calculator
 *  bit12: AC Bookmarks
 *  bit13: AC Search
 *  bit14: AC Home
 *  bit15: AC Back
 * ------------------------------------------------------------------------- */

esp_err_t ble_hid_cc_tap_vol_up(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 2));  // Volume Up
}

esp_err_t ble_hid_cc_tap_vol_down(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 3));  // Volume Down
}

esp_err_t ble_hid_cc_tap_mute(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 0));  // Mute
}

esp_err_t ble_hid_cc_tap_play_pause(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 4));  // Play/Pause
}

esp_err_t ble_hid_cc_tap_next(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 5));  // Next Track
}

esp_err_t ble_hid_cc_tap_prev(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 6));  // Previous Track
}

esp_err_t ble_hid_cc_tap_stop(void) {
    return ble_hid_tap_consumer_bits((uint8_t)(1u << 7));  // Stop
}
