// power.h
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

// Shared I2C bus config (camera/PAJ/BQ/LEDs).
#define POWER_I2C_PORT      I2C_NUM_0
#define POWER_I2C_SDA       2
#define POWER_I2C_SCL       3
#define POWER_I2C_FREQ_HZ   100000

#ifdef __cplusplus
extern "C" {
#endif

// Public I2C + power init
esp_err_t power_init(void);

// Simple helpers (safe even if no battery is attached)
esp_err_t power_bq25155_read_id(uint8_t *out_id);
esp_err_t power_bq25155_read_battery(float *out_voltage, float *out_percent);
esp_err_t power_bq25155_read_charge_current(float *out_current_ma);
esp_err_t power_bq25155_read_status(uint8_t *stat0, uint8_t *stat1);
esp_err_t power_bq25155_read_config(void);

// Battery monitoring task
void power_start_battery_monitor(uint32_t interval_ms);
void power_stop_battery_monitor(void);

// Expose the I2C bus to other modules (e.g. LEDs)
i2c_master_bus_handle_t power_get_i2c_bus(void);

#ifdef __cplusplus
}
#endif
