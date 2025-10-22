// ble_hid.c
#include "ble_hid.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include "ble_gatts_utils.h"

static const char *TAG = "HID";

/* ------------------------------- UUIDs ---------------------------------- */
static const ble_uuid16_t HID_SVC_UUID            = BLE_UUID16_INIT(0x1812);
static const ble_uuid16_t HID_REPORT_MAP_UUID     = BLE_UUID16_INIT(0x2A4B);
static const ble_uuid16_t HID_REPORT_UUID         = BLE_UUID16_INIT(0x2A4D); // Input Report
static const ble_uuid16_t HID_INFO_UUID           = BLE_UUID16_INIT(0x2A4A);
static const ble_uuid16_t HID_CTRL_POINT_UUID     = BLE_UUID16_INIT(0x2A4C);
static const ble_uuid16_t HID_PROTOCOL_MODE_UUID  = BLE_UUID16_INIT(0x2A4E);

/* Descriptor UUIDs as concrete objects (avoid compound literals in tables) */
static const ble_uuid16_t UUID_DSC_CCCD           = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16); // 0x2902
static const ble_uuid16_t UUID_DSC_REPORTREF      = BLE_UUID16_INIT(0x2908);                      // 0x2908

/* ------------------------------- Data ----------------------------------- */
// Report Reference (Report ID=1, Type=Input(1))
static uint8_t report_ref_input[2] = { 0x03, 0x01 };

/* Report Map: Consumer Control with 16 buttons (Report ID = 1) */
static const uint8_t hid_report_map[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x10,        //   Report Count (16)
    0x09, 0xE2,        //   Mute
    0x09, 0x30,        //   Power
    0x09, 0xE9,        //   Volume Up
    0x09, 0xEA,        //   Volume Down
    0x09, 0xCD,        //   Play/Pause
    0x09, 0xB5,        //   Next
    0x09, 0xB6,        //   Previous
    0x09, 0xB7,        //   Stop
    0x09, 0xB8,        //   Eject
    0x09, 0x83,        //   Media Select
    0x0A, 0x94, 0x01,  //   AL Browser
    0x0A, 0x92, 0x01,  //   AL Calculator
    0x0A, 0x2A, 0x02,  //   AC Bookmarks
    0x0A, 0x21, 0x02,  //   AC Search
    0x0A, 0x23, 0x02,  //   AC Home
    0x0A, 0x24, 0x02,  //   AC Back
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0xC0               // End Collection
};

static const uint8_t hid_info[] = { 0x11, 0x01, 0x00, 0x02 }; // ver 1.11, country 0, flags: RemoteWake
static uint8_t g_protocol_mode = 0x01; // Report protocol

/* ----------------------------- Global state ----------------------------- */
static uint16_t g_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_input_val_handle  = 0;
static uint16_t g_input_cccd_handle = 0;
static bool     g_cccd_subscribed   = false;

/* ------------------------ Forward declarations -------------------------- */
static int hid_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);

/* -------------------------- GATT declaration ---------------------------- */
/* Make descriptors array non-const; NimBLE stores pointers to it. */
static struct ble_gatt_dsc_def hid_input_descs[] = {
    {
        .uuid      = &UUID_DSC_CCCD.u,      // 0x2902
        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
        .access_cb = hid_chr_access_cb,
    },
    {
        .uuid      = &UUID_DSC_REPORTREF.u, // 0x2908
        .att_flags = BLE_ATT_F_READ,
        .access_cb = hid_chr_access_cb,
    },
    { 0 }
};

static struct ble_gatt_chr_def hid_chrs[] = {
    {   // Report Map
        .uuid       = &HID_REPORT_MAP_UUID.u,
        .access_cb  = hid_chr_access_cb,
        .flags      = BLE_GATT_CHR_F_READ,
    },
    {   // Input Report (Report ID 3), with CCCD + Report Reference descriptor
        .uuid       = &HID_REPORT_UUID.u,
        .access_cb  = hid_chr_access_cb,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &g_input_val_handle,
        .descriptors= hid_input_descs,
    },
    {   // HID Information (make read encrypted if you want to force pairing)
        .uuid       = &HID_INFO_UUID.u,
        .access_cb  = hid_chr_access_cb,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {   // Protocol Mode (Report=0x03; Boot=0x00 not supported)
        .uuid       = &HID_PROTOCOL_MODE_UUID.u,
        .access_cb  = hid_chr_access_cb,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {   // Control Point (Suspend/Exit Suspend)
        .uuid       = &HID_CTRL_POINT_UUID.u,
        .access_cb  = hid_chr_access_cb,
        .flags      = BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    { 0 }
};

static const struct ble_gatt_svc_def hid_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &HID_SVC_UUID.u,
        .characteristics = hid_chrs,
    },
    { 0 }
};



/* ------------------------------ Callbacks -------------------------------- */
static int hid_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Descriptors */
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        const ble_uuid_t *dsc_uuid = ctxt->dsc ? ctxt->dsc->uuid : NULL;
        if (!dsc_uuid) return BLE_ATT_ERR_UNLIKELY;

        if (ble_uuid_cmp(dsc_uuid, &UUID_DSC_CCCD.u) == 0) {
            if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
                if (OS_MBUF_PKTLEN(ctxt->om) != 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                uint16_t ccc = ctxt->om->om_data[0] | (ctxt->om->om_data[1] << 8);
                g_cccd_subscribed = (ccc & 0x0001) != 0;
                ESP_LOGI(TAG, "HID CCCD: %s", g_cccd_subscribed ? "SUBSCRIBED" : "UNSUBSCRIBED");
                return 0;
            } else {
                uint16_t ccc = g_cccd_subscribed ? 0x0001 : 0x0000;
                return os_mbuf_append(ctxt->om, &ccc, sizeof(ccc)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }

        if (ble_uuid_cmp(dsc_uuid, &UUID_DSC_REPORTREF.u) == 0) {
            return os_mbuf_append(ctxt->om, report_ref_input, sizeof(report_ref_input)) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return BLE_ATT_ERR_UNLIKELY;
    }

    /* Characteristics */
    const ble_uuid_t *uuid = ctxt->chr ? ctxt->chr->uuid : NULL;
    if (!uuid) return BLE_ATT_ERR_UNLIKELY;

    if (ble_uuid_cmp(uuid, &HID_PROTOCOL_MODE_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            return os_mbuf_append(ctxt->om, &g_protocol_mode, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if (OS_MBUF_PKTLEN(ctxt->om) != 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            uint8_t v = ctxt->om->om_data[0];
            if (v != 0x01) return BLE_ATT_ERR_REQ_NOT_SUPPORTED; // report mode only
            g_protocol_mode = 0x01;
            ESP_LOGI(TAG, "HID Protocol Mode set to REPORT (0x01)");
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (ble_uuid_cmp(uuid, &HID_REPORT_MAP_UUID.u) == 0) {
            return os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map)) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (ble_uuid_cmp(uuid, &HID_INFO_UUID.u) == 0) {
            return os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info)) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        // testing (1 instead of 3)
        if (ble_uuid_cmp(uuid, &HID_REPORT_UUID.u) == 0) {
            uint8_t report[3] = { 0x01, 0x00, 0x00 }; // ReportID=1, empty
            return os_mbuf_append(ctxt->om, report, sizeof(report)) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (ble_uuid_cmp(uuid, &HID_CTRL_POINT_UUID.u) == 0) {
            if (OS_MBUF_PKTLEN(ctxt->om) == 1) {
                uint8_t cmd = ctxt->om->om_data[0];
                ESP_LOGI(TAG, "HID Control Point: 0x%02x (%s)",
                         cmd, cmd==0 ? "SUSPEND" : cmd==1 ? "EXIT_SUSPEND" : "UNKNOWN");
            }
            return 0;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ------------------------------- API ------------------------------------ */
esp_err_t ble_hid_init(void)
{
    int rc = ble_gatts_count_cfg(hid_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(hid_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    sanity_check_tables(hid_svcs);
    dump_table("HID", hid_svcs);

    ESP_LOGI(TAG, "HID service initialized");
    return ESP_OK;
}

esp_err_t ble_hid_get_handles(void)
{
    ESP_LOGI(TAG, "Resolving HID handles...");
    ESP_LOGI(TAG, "g_input_val_handle from service definition = 0x%04x", g_input_val_handle);
    g_input_cccd_handle = g_input_val_handle + 1;
    ESP_LOGI(TAG, "HID Input Report value handle=0x%04x, CCCD=0x%04x",
             g_input_val_handle, g_input_cccd_handle);
    if (g_input_val_handle == 0) {
        ESP_LOGE(TAG, "Failed to get HID Input Report handle");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ble_hid_set_conn(uint16_t conn_handle)
{
    g_conn_handle = conn_handle;
    ESP_LOGI(TAG, "HID connection handle set: %u", conn_handle);
}

void ble_hid_check_cccd_subscribe(uint16_t attr_handle, uint16_t cur_notify)
{
    // NimBLE gives the VALUE handle here, not the CCCD handle.
    if (attr_handle == g_input_val_handle) {
        bool before = g_cccd_subscribed;
        g_cccd_subscribed = (cur_notify != 0);
        ESP_LOGI("HID", "🎯 SUBSCRIBE event on HID Input Report (val=0x%04x): %s",
                 g_input_val_handle, g_cccd_subscribed ? "SUBSCRIBED" : "UNSUBSCRIBED");
        if (before != g_cccd_subscribed) {
            ESP_LOGI("HID", "HID notify state changed via GAP event");
        }
    }
}


esp_err_t ble_hid_tap_consumer_bits(uint16_t bits)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "Not connected");
        return ESP_ERR_INVALID_STATE;
    }
    if (!g_cccd_subscribed) {
        ESP_LOGW(TAG, "Client not subscribed to HID notifications");
        return ESP_ERR_INVALID_STATE;
    }

    // Press
    // testing - 0x01 instead of 0x03
    uint8_t report[3] = { 0x01, (uint8_t)(bits & 0xFF), (uint8_t)((bits >> 8) & 0xFF) };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
    if (!om) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(g_conn_handle, g_input_val_handle, om);
    if (rc != 0) return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(100));

    // Release
    report[1] = 0x00; report[2] = 0x00;
    om = ble_hs_mbuf_from_flat(report, sizeof(report));
    if (!om) return ESP_ERR_NO_MEM;
    rc = ble_gatts_notify_custom(g_conn_handle, g_input_val_handle, om);

    return rc == 0 ? ESP_OK : ESP_FAIL;
}
