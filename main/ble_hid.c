#include "ble_hid.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"
extern void ble_store_config_init(void);
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
#define REPORT_ID_CC      0x03
#define REPORT_TYPE_INPUT 0x01

// Report map: Consumer Control with 7 buttons (1 byte payload) + 1 pad bit.
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
// (Use 0x00 here since that was already working for you.)
static const uint8_t s_hid_info[4] = { 0x11, 0x01, 0x00, 0x03 };
static uint8_t       s_proto_mode  = 0x01; // Report protocol only

// Report Reference (Report ID=3, Type=Input)
static const uint8_t s_report_ref[2] = { REPORT_ID_CC, REPORT_TYPE_INPUT };

// ---------- State ----------
static uint16_t s_conn             = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_input_val_handle = 0;
static bool     s_input_notify_en  = false;

// ---------- Worker/Queue to serialize HID sends ----------
typedef enum {
    HID_ACT_BITS,     // send 1-byte bitmap (lower 7 bits used)
    HID_ACT_BITIDX    // send one specific bit by index (0..6)
} hid_act_type_t;

typedef struct {
    hid_act_type_t type;
    uint16_t bits;   // for HID_ACT_BITS
    uint8_t  bitidx; // for HID_ACT_BITIDX
} hid_act_t;

static QueueHandle_t s_hid_q = NULL;
static void          hid_worker(void *arg);

bool ble_hid_is_ready(void) {
    return (s_conn != BLE_HS_CONN_HANDLE_NONE) && s_input_notify_en && (s_input_val_handle != 0);
}

static void ensure_hid_worker_started(void)
{
    static bool started = false;
    if (!started) {
        s_hid_q = xQueueCreate(8, sizeof(hid_act_t));
        if (s_hid_q) {
            xTaskCreate(hid_worker, "hid_worker", 3072, NULL, 3, NULL);
            started = true;
        } else {
            ESP_LOGE(TAG, "Failed to create HID queue");
        }
    }
}

// ---------- Forward ----------
static int  hid_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt*, void*);
static esp_err_t send_bits_immediate(uint8_t v);
static esp_err_t cc_tap_bit_immediate(uint8_t bit_index);

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
    { // Control Point (Suspend/Exit Suspend; accept writes)
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
    
    // Start the HID worker/queue (safe to start early).
    ensure_hid_worker_started();

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

// ---------- Internal immediate send helpers (used by worker) ----------
static esp_err_t send_bits_immediate(uint8_t v)
{
    if (!ble_hid_is_ready()) return ESP_ERR_INVALID_STATE;

    // Press
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&v, 1);
    if (!om) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(s_conn, s_input_val_handle, om);
    if (rc) return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(60));

    // Release
    v = 0x00;
    om = ble_hs_mbuf_from_flat(&v, 1);
    if (!om) return ESP_ERR_NO_MEM;
    rc = ble_gatts_notify_custom(s_conn, s_input_val_handle, om);
    return rc ? ESP_FAIL : ESP_OK;
}

static esp_err_t cc_tap_bit_immediate(uint8_t bit_index)
{
    if (bit_index > 6) return ESP_ERR_INVALID_ARG;
    uint8_t v = (uint8_t)(1u << bit_index);
    return send_bits_immediate(v);
}

// ---------- Worker task (serializes and guards timing) ----------
static void hid_worker(void *arg)
{
    (void)arg;
    hid_act_t act;

    for (;;) {
        if (xQueueReceive(s_hid_q, &act, portMAX_DELAY) == pdTRUE) {
            // Wait for connection + CCCD, but don't busy-spin forever.
            int guard = 0;
            while (!ble_hid_is_ready() && guard++ < 100) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            if (!ble_hid_is_ready()) continue;

            if (act.type == HID_ACT_BITIDX) {
                (void)cc_tap_bit_immediate(act.bitidx);
            } else {
                uint8_t v = (uint8_t)(act.bits & 0x7F); // lower 7 bits valid
                (void)send_bits_immediate(v);
            }

            // Small inter-action gap so multiple gestures don't coalesce on phones
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    }
}

// ---------- Public API (enqueue; safe from any context/task) ----------
// Bit layout (must match s_report_map order)
#define BIT_VOL_UP       0
#define BIT_VOL_DOWN     1
#define BIT_MUTE         2
#define BIT_PLAY_PAUSE   3
#define BIT_NEXT         4
#define BIT_PREV         5
#define BIT_STOP         6

esp_err_t ble_hid_tap_consumer_bits(uint16_t bits)
{
    if (!s_hid_q) return ESP_ERR_INVALID_STATE;
    hid_act_t a = { .type = HID_ACT_BITS, .bits = (uint16_t)(bits & 0x7F), .bitidx = 0 };
    return xQueueSend(s_hid_q, &a, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}

static esp_err_t enqueue_bit(uint8_t idx)
{
    if (!s_hid_q) return ESP_ERR_INVALID_STATE;
    hid_act_t a = { .type = HID_ACT_BITIDX, .bits = 0, .bitidx = idx };
    return xQueueSend(s_hid_q, &a, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t ble_hid_cc_tap_vol_up(void)      { return enqueue_bit(BIT_VOL_UP); }
esp_err_t ble_hid_cc_tap_vol_down(void)    { return enqueue_bit(BIT_VOL_DOWN); }
esp_err_t ble_hid_cc_tap_mute(void)        { return enqueue_bit(BIT_MUTE); }
esp_err_t ble_hid_cc_tap_play_pause(void)  { return enqueue_bit(BIT_PLAY_PAUSE); }
esp_err_t ble_hid_cc_tap_next(void)        { return enqueue_bit(BIT_NEXT); }
esp_err_t ble_hid_cc_tap_prev(void)        { return enqueue_bit(BIT_PREV); }
esp_err_t ble_hid_cc_tap_stop(void)        { return enqueue_bit(BIT_STOP); }
