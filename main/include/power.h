// power.h
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Public I2C + power init
esp_err_t power_init(void);

// Simple helpers (safe even if no battery is attached)
esp_err_t power_bq25155_read_id(uint8_t *out_id);
esp_err_t power_bq25155_read_battery(float *out_voltage, float *out_percent);

// Expose the I2C bus to other modules (e.g. LEDs)
#include "driver/i2c_master.h"
i2c_master_bus_handle_t power_get_i2c_bus(void);

#ifdef __cplusplus
}
#endif
