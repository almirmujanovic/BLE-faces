// power.c
#include "power.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <math.h>
#include "freertos/FreeRTOS.h"

static const char *TAG = "POWER";

// ------- I2C pins / params (adjust in power.h if needed) -------

// ------- BQ25155 definitions (from your demo) -------
#define BQ25155_ADDR            0x6B

#define BQ25155_REG_DEVICE_ID   0x6F
#define BQ25155_REG_VBAT_ADC_H  0x42
#define BQ25155_REG_VBAT_ADC_L  0x43

// Battery voltage thresholds for percentage calculation
#define BATTERY_VOLTAGE_FULL    4.2f
#define BATTERY_VOLTAGE_EMPTY   3.0f

// ------- Static handles for new I2C driver -------
static i2c_master_bus_handle_t  s_power_bus   = NULL;
static i2c_master_dev_handle_t  s_bq_dev      = NULL;
static bool                     s_power_init  = false;

// ---------------------------------------------------
// Small helpers
// ---------------------------------------------------
static float calculate_battery_percentage(float voltage)
{
    if (voltage >= BATTERY_VOLTAGE_FULL) return 100.0f;
    if (voltage <= BATTERY_VOLTAGE_EMPTY) return 0.0f;

    float pct = (voltage - BATTERY_VOLTAGE_EMPTY) *
                100.0f / (BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY);
    return pct;
}

// ---------------------------------------------------
// Public: get shared I2C bus (for LEDs etc.)
// ---------------------------------------------------
i2c_master_bus_handle_t power_get_i2c_bus(void)
{
    return s_power_bus;
}

// ---------------------------------------------------
// Internal: init I2C bus and BQ25155 device handle
// ---------------------------------------------------
esp_err_t power_init(void)
{
    if (s_power_init) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing power I2C (new driver) ...");

    // --- Create I2C master bus (NEW API) ---
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = POWER_I2C_PORT,
        .scl_io_num = POWER_I2C_SCL,
        .sda_io_num = POWER_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = 1, // you can disable if you have external pull-ups
        }
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_power_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    // --- Add BQ25155 as one device on that bus ---
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BQ25155_ADDR,
        .scl_speed_hz    = POWER_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_power_bus, &dev_cfg, &s_bq_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add BQ25155 device (0x%02X): %s",
                 BQ25155_ADDR, esp_err_to_name(err));
        // Bus is still usable for other devices (e.g. LEDs)
        s_bq_dev = NULL;
    } else {
        ESP_LOGI(TAG, "BQ25155 device added at 0x%02X", BQ25155_ADDR);
    }

    s_power_init = true;
    return ESP_OK;
}

// ---------------------------------------------------
// Public: read BQ25155 device ID
// ---------------------------------------------------
esp_err_t power_bq25155_read_id(uint8_t *out_id)
{
    if (!s_power_init) {
        esp_err_t err = power_init();
        if (err != ESP_OK) return err;
    }
    if (!s_bq_dev) {
        ESP_LOGW(TAG, "BQ25155 device handle not available");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = BQ25155_REG_DEVICE_ID;
    uint8_t id  = 0;

    // write reg, then read 1 byte
    esp_err_t err = i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, &id, 1, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BQ25155 ID: %s", esp_err_to_name(err));
        return err;
    }

    if (out_id) {
        *out_id = id;
    }
    ESP_LOGI(TAG, "BQ25155 device ID: 0x%02X", id);
    return ESP_OK;
}

// ---------------------------------------------------
// Public: read battery voltage + percentage
// (safe to call even if no battery connected; will just
//  give some weird voltage or fail gracefully)
// ---------------------------------------------------
esp_err_t power_bq25155_read_battery(float *out_voltage, float *out_percent)
{
    if (!s_power_init) {
        esp_err_t err = power_init();
        if (err != ESP_OK) return err;
    }
    if (!s_bq_dev) {
        ESP_LOGW(TAG, "BQ25155 device handle not available");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2];
    uint8_t reg = BQ25155_REG_VBAT_ADC_H;

    // Read 2 bytes starting from VBAT_ADC_H
    esp_err_t err = i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, buf, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BQ25155 VBAT ADC: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t vbat_adc = ((uint16_t)buf[0] << 8) | buf[1];
    float vbat = (vbat_adc / 65536.0f) * 6.0f;  // same formula from your demo
    float pct  = calculate_battery_percentage(vbat);

    ESP_LOGI(TAG, "Battery VBAT: raw=0x%04X, voltage=%.3f V, %.1f%%",
             vbat_adc, vbat, pct);

    if (out_voltage) *out_voltage = vbat;
    if (out_percent) *out_percent = pct;

    return ESP_OK;
}
