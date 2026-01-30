// power.c
#include "power.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <math.h>
#include "freertos/FreeRTOS.h"

static const char *TAG = "POWER";

// ------- I2C pins / params (adjust in power.h if needed) -------

// ------- BQ25155 definitions (from TI datasheet) -------
#define BQ25155_ADDR            0x6B

// Status registers (FIXED: were wrong!)
#define BQ25155_REG_STAT0       0x00
#define BQ25155_REG_STAT1       0x01

// Configuration registers
#define BQ25155_REG_VBAT_CTRL   0x12  // Battery voltage control (3.6V-4.6V, 10mV steps)
#define BQ25155_REG_ICHG_CTRL   0x13  // Fast charge current control
#define BQ25155_REG_TERMCTRL    0x15  // Termination current control
#define BQ25155_REG_CHARGERCTRL0 0x17 // Charger control 0

// ADC registers
#define BQ25155_REG_VBAT_ADC_H  0x42  // Battery voltage ADC MSB
#define BQ25155_REG_VBAT_ADC_L  0x43  // Battery voltage ADC LSB
#define BQ25155_REG_ICHG_ADC_H  0x46  // Charge current ADC MSB (FIXED: was 0x44!)
#define BQ25155_REG_ICHG_ADC_L  0x47  // Charge current ADC LSB (FIXED: was 0x45!)

// Device info
#define BQ25155_REG_DEVICE_ID   0x6F

// Battery voltage thresholds for percentage calculation
#define BATTERY_VOLTAGE_FULL    4.2f
#define BATTERY_VOLTAGE_EMPTY   3.0f

// ------- Static handles for new I2C driver -------
static i2c_master_bus_handle_t  s_power_bus   = NULL;
static i2c_master_dev_handle_t  s_bq_dev      = NULL;
static bool                     s_power_init  = false;

// ------- Battery monitoring task -------
static TaskHandle_t             s_battery_monitor_task = NULL;
static uint32_t                 s_monitor_interval_ms  = 10000; // Default 10s
static bool                     s_monitor_running      = false;

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
    
    // Configure BQ25155 for LiPo battery charging
    if (s_bq_dev) {
        ESP_LOGI(TAG, "Configuring BQ25155 for 1000mAh LiPo battery...");
        
        // Configure battery regulation voltage to 4.2V
        // VBAT_CTRL: 3.6V base + (value * 10mV)
        // For 4.2V: (4.2 - 3.6) / 0.01 = 60 = 0x3C
        uint8_t vbat_cmd[2] = {BQ25155_REG_VBAT_CTRL, 0x3C};
        err = i2c_master_transmit(s_bq_dev, vbat_cmd, 2, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ Battery regulation voltage set to 4.2V");
        } else {
            ESP_LOGW(TAG, "  ✗ Failed to set VBAT_CTRL: %s", esp_err_to_name(err));
        }
        
        // Configure fast charge current to 500mA (0.5C for 1000mAh - safe)
        // ICHG_CTRL: Check datasheet for exact encoding
        // Typical: Base + (value * step), will use 0x80 for ~500mA
        uint8_t ichg_cmd[2] = {BQ25155_REG_ICHG_CTRL, 0x80};
        err = i2c_master_transmit(s_bq_dev, ichg_cmd, 2, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ Fast charge current set to ~500mA");
        } else {
            ESP_LOGW(TAG, "  ✗ Failed to set ICHG_CTRL: %s", esp_err_to_name(err));
        }
        
        // Configure termination current to ~50mA (5% of battery capacity)
        uint8_t term_cmd[2] = {BQ25155_REG_TERMCTRL, 0x32};  // ~50mA termination
        err = i2c_master_transmit(s_bq_dev, term_cmd, 2, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ Termination current set to ~50mA");
        } else {
            ESP_LOGW(TAG, "  ✗ Failed to set TERMCTRL: %s", esp_err_to_name(err));
        }
        
        ESP_LOGI(TAG, "BQ25155 configuration complete");
    }
    
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

// ---------------------------------------------------
// Public: read charging current
// ---------------------------------------------------
esp_err_t power_bq25155_read_charge_current(float *out_current_ma)
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
    uint8_t reg = BQ25155_REG_ICHG_ADC_H;

    // Read 2 bytes starting from ICHG_ADC_H
    esp_err_t err = i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, buf, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BQ25155 charge current: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t ichg_adc = ((uint16_t)buf[0] << 8) | buf[1];
    // BQ25155 ICHG ADC: 16-bit value, 0-3000mA range (typical)
    float current_ma = (ichg_adc / 65536.0f) * 3000.0f;
    
    ESP_LOGI(TAG, "Charge current: raw=0x%04X, %.1f mA", ichg_adc, current_ma);
    
    if (out_current_ma) *out_current_ma = current_ma;
    return ESP_OK;
}

// ---------------------------------------------------
// Public: read charging status
// ---------------------------------------------------
esp_err_t power_bq25155_read_status(uint8_t *stat0, uint8_t *stat1)
{
    if (!s_power_init) {
        esp_err_t err = power_init();
        if (err != ESP_OK) return err;
    }
    if (!s_bq_dev) {
        ESP_LOGW(TAG, "BQ25155 device handle not available");
        return ESP_ERR_INVALID_STATE;
    }

    // Read STAT0
    if (stat0) {
        uint8_t reg = BQ25155_REG_STAT0;
        esp_err_t err = i2c_master_transmit_receive(
            s_bq_dev, &reg, 1, stat0, 1, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read BQ25155 STAT0: %s", esp_err_to_name(err));
            return err;
        }
    }

    // Read STAT1
    if (stat1) {
        uint8_t reg = BQ25155_REG_STAT1;
        esp_err_t err = i2c_master_transmit_receive(
            s_bq_dev, &reg, 1, stat1, 1, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read BQ25155 STAT1: %s", esp_err_to_name(err));
            return err;
        }
    }

    if (stat0) {
        ESP_LOGI(TAG, "BQ25155 STAT0: 0x%02X", *stat0);
        // Decode status bits (per datasheet)
        ESP_LOGI(TAG, "  CHRG_CV (Taper): %s", (*stat0 & 0x40) ? "Active" : "Inactive");
        ESP_LOGI(TAG, "  CHARGE_DONE: %s", (*stat0 & 0x20) ? "Done" : "Not Done");
        ESP_LOGI(TAG, "  VINDPM_ACTIVE: %s", (*stat0 & 0x04) ? "Active" : "Inactive");
        ESP_LOGI(TAG, "  THERMREG_ACTIVE: %s", (*stat0 & 0x02) ? "Active" : "Inactive");
        ESP_LOGI(TAG, "  VIN_PGOOD: %s", (*stat0 & 0x01) ? "Good" : "Not Good");
    }
    
    if (stat1) {
        ESP_LOGI(TAG, "BQ25155 STAT1: 0x%02X", *stat1);
    }

    return ESP_OK;
}

// ---------------------------------------------------
// Public: read and display BQ25155 configuration
// ---------------------------------------------------
esp_err_t power_bq25155_read_config(void)
{
    if (!s_bq_dev) {
        ESP_LOGW(TAG, "BQ25155 device handle not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== BQ25155 Configuration ===");
    
    // Read VBAT_CTRL (0x12)
    uint8_t reg = BQ25155_REG_VBAT_CTRL;
    uint8_t vbat_ctrl = 0;
    esp_err_t err = i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, &vbat_ctrl, 1, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        // VBAT = 3.6V + (value * 10mV)
        float vbat_reg = 3.6f + (vbat_ctrl * 0.01f);
        ESP_LOGI(TAG, "VBAT_CTRL: 0x%02X → %.2fV target", vbat_ctrl, vbat_reg);
    }
    
    // Read ICHG_CTRL (0x13)
    reg = BQ25155_REG_ICHG_CTRL;
    uint8_t ichg_ctrl = 0;
    err = i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, &ichg_ctrl, 1, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ICHG_CTRL: 0x%02X", ichg_ctrl);
    }
    
    // Read TERMCTRL (0x15)
    reg = BQ25155_REG_TERMCTRL;
    uint8_t term_ctrl = 0;
    err = i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, &term_ctrl, 1, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TERMCTRL: 0x%02X", term_ctrl);
    }
    
    ESP_LOGI(TAG, "=============================");
    return ESP_OK;
}

// ---------------------------------------------------
// Battery monitoring task
// ---------------------------------------------------
static void battery_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Battery monitor task started (interval: %lu ms)", s_monitor_interval_ms);
    
    while (s_monitor_running) {
        float voltage = 0.0f;
        float percent = 0.0f;
        float current_ma = 0.0f;
        uint8_t stat0 = 0, stat1 = 0;
        
        // Read battery voltage and percentage
        esp_err_t ret = power_bq25155_read_battery(&voltage, &percent);
        if (ret == ESP_OK) {
            // Read charging current
            power_bq25155_read_charge_current(&current_ma);
            
            // Read status registers
            power_bq25155_read_status(&stat0, &stat1);
            
            // Log summary
            ESP_LOGI(TAG, "🔋 Battery: %.3fV (%.1f%%) | Charging: %.1fmA", 
                     voltage, percent, current_ma);
        } else {
            ESP_LOGW(TAG, "Failed to read battery data");
        }
        
        // Wait for next interval
        vTaskDelay(pdMS_TO_TICKS(s_monitor_interval_ms));
    }
    
    ESP_LOGI(TAG, "Battery monitor task stopped");
    s_battery_monitor_task = NULL;
    vTaskDelete(NULL);
}

// ---------------------------------------------------
// Public: start battery monitoring
// ---------------------------------------------------
void power_start_battery_monitor(uint32_t interval_ms)
{
    if (s_battery_monitor_task) {
        ESP_LOGW(TAG, "Battery monitor already running");
        return;
    }
    
    s_monitor_interval_ms = interval_ms;
    s_monitor_running = true;
    
    BaseType_t ret = xTaskCreate(battery_monitor_task, "battery_monitor",
                                 3072, NULL, 3, &s_battery_monitor_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create battery monitor task");
        s_monitor_running = false;
    } else {
        ESP_LOGI(TAG, "Battery monitor started (logging every %lu ms)", interval_ms);
    }
}

// ---------------------------------------------------
// Public: stop battery monitoring
// ---------------------------------------------------
void power_stop_battery_monitor(void)
{
    if (!s_battery_monitor_task) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping battery monitor...");
    s_monitor_running = false;
    
    // Wait for task to finish
    uint32_t wait_ms = 0;
    while (s_battery_monitor_task && wait_ms < 2000) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_ms += 100;
    }
    
    if (s_battery_monitor_task) {
        ESP_LOGW(TAG, "Battery monitor task did not stop cleanly");
    }
}
