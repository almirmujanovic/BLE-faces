#pragma once

#include <assert.h>
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"

void assert_uuid_ok(const ble_uuid_t *u);
void sanity_check_tables(const struct ble_gatt_svc_def *svcs);
void dump_table(const char *name, const struct ble_gatt_svc_def *svcs) ;