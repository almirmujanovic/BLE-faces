#include <assert.h>
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "ble_gatts_utils.h"
#include "esp_log.h"


void assert_uuid_ok(const ble_uuid_t *u) {
    assert(u);
    assert(u->type == BLE_UUID_TYPE_16 ||
           u->type == BLE_UUID_TYPE_32 ||
           u->type == BLE_UUID_TYPE_128);
}

void sanity_check_tables(const struct ble_gatt_svc_def *svcs) {
    assert(svcs);
    for (const struct ble_gatt_svc_def *s = svcs; s && s->type; ++s) {
        assert_uuid_ok(s->uuid);

        const struct ble_gatt_chr_def *c = s->characteristics;
        for (; c && c->uuid; ++c) {
            assert_uuid_ok(c->uuid);

            const struct ble_gatt_dsc_def *d = c->descriptors;
            if (d) {
                for (; d->uuid; ++d) {
                    assert_uuid_ok(d->uuid);
                }
                /* loop exits when d->uuid == NULL (the {0} terminator) */
            }
        }
        /* loop exits when c->uuid == NULL (the {0} terminator) */
    }
}

void dump_table(const char *name, const struct ble_gatt_svc_def *svcs) {
    ESP_LOGI("GATTDUMP", "=== %s ===", name);
    for (int si = 0; svcs[si].type; ++si) {
        const struct ble_gatt_svc_def *s = &svcs[si];
        ESP_LOGI("GATTDUMP", "svc[%d] uuid=%p type=%d", si, (void*)s->uuid, s->type);
        for (int ci = 0; s->characteristics && s->characteristics[ci].uuid; ++ci) {
            const struct ble_gatt_chr_def *c = &s->characteristics[ci];
            ESP_LOGI("GATTDUMP", "  chr[%d] uuid=%p flags=0x%04x val_handle=%p",
                     ci, (void*)c->uuid, c->flags, (void*)c->val_handle);
            for (int di = 0; c->descriptors && c->descriptors[di].uuid; ++di) {
                const struct ble_gatt_dsc_def *d = &c->descriptors[di];
                ESP_LOGI("GATTDUMP", "    dsc[%d] uuid=%p att_flags=0x%04x",
                         di, (void*)d->uuid, d->att_flags);
            }
        }
    }
}

