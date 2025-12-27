// power.c
#include "power.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "POWER";

// -----------------------------------------------------------------------------
// I2C CONFIG
// -----------------------------------------------------------------------------
#define POWER_I2C_PORT      I2C_NUM_0
#define POWER_I2C_SDA       2
#define POWER_I2C_SCL       3
#define POWER_I2C_FREQ_HZ   100000

// -----------------------------------------------------------------------------
// BQ25155 REGISTER MAP
// -----------------------------------------------------------------------------
#define BQ25155_ADDR            0x6B

#define BQ25155_REG_STATUS0        0x00
#define BQ25155_REG_STATUS1        0x01
#define BQ25155_REG_STATUS2        0x02
#define BQ25155_REG_VBAT_CTRL      0x12
#define BQ25155_REG_ICHG_CTRL      0x13
#define BQ25155_REG_CHGCTRL0       0x17
#define BQ25155_REG_CHGCTRL1       0x18
#define BQ25155_REG_ILIMCTRL       0x19
#define BQ25155_REG_DEVICE_ID      0x6F
#define BQ25155_REG_VBAT_ADC_H     0x42
#define BQ25155_REG_VBAT_ADC_L     0x43

// Battery voltage thresholds
#define BATTERY_VOLTAGE_FULL    4.2f
#define BATTERY_VOLTAGE_EMPTY   3.0f

// -----------------------------------------------------------------------------
// INTERNAL STATE
// -----------------------------------------------------------------------------
static i2c_master_bus_handle_t  s_power_bus  = NULL;
static i2c_master_dev_handle_t  s_bq_dev     = NULL;
static bool                     s_power_init = false;

// -----------------------------------------------------------------------------
// HELPER: battery %
// -----------------------------------------------------------------------------
static float calculate_battery_percentage(float voltage)
{
    if (voltage >= BATTERY_VOLTAGE_FULL) return 100.0f;
    if (voltage <= BATTERY_VOLTAGE_EMPTY) return 0.0f;

    return ((voltage - BATTERY_VOLTAGE_EMPTY) *
            100.0f / (BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY));
}

// -----------------------------------------------------------------------------
// PUBLIC: get I2C bus
// -----------------------------------------------------------------------------
i2c_master_bus_handle_t power_get_i2c_bus(void)
{
    return s_power_bus;
}

// -----------------------------------------------------------------------------
// INTERNAL I2C UTILS
// -----------------------------------------------------------------------------
static esp_err_t bq_write(uint8_t reg, uint8_t val)
{
    if (!s_bq_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_bq_dev, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t bq_read(uint8_t reg, uint8_t *out)
{
    if (!s_bq_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(
        s_bq_dev, &reg, 1, out, 1, pdMS_TO_TICKS(100));
}

// -----------------------------------------------------------------------------
// CONFIGURE BQ25155 CHARGER (AUTO-CALLED FROM power_init)
// -----------------------------------------------------------------------------
static void power_bq25155_configure_charger(void)
{
    if (!s_bq_dev) return;

    uint8_t tmp;

    // ILIM = 500 mA
    bq_write(BQ25155_REG_ILIMCTRL, 0x7F);

    // VBAT regulation = 4.2V
    bq_write(BQ25155_REG_VBAT_CTRL, 0x3C);

    // Enable extended ICHG range
    if (bq_read(BQ25155_REG_CHGCTRL0, &tmp) == ESP_OK) {
        tmp |= (1 << 7);
        bq_write(BQ25155_REG_CHGCTRL0, tmp);
    }

        // 2) Maximize SYS load current limit = 400 mA
    if (bq_read(BQ25155_REG_CHGCTRL1, &tmp) == ESP_OK) {

        tmp &= ~(0b111 << 4);   // clear SYS load bits
        tmp |=  (0b111 << 4);   // set SYS load = 400 mA max

        bq_write(BQ25155_REG_CHGCTRL1, tmp);
    }

    // Charge current = 500mA approx
    bq_write(BQ25155_REG_ICHG_CTRL, 0xC7);

    // Enable charging
// Disable charging completely (power-only mode)
    if (bq_read(BQ25155_REG_CHGCTRL1, &tmp) == ESP_OK) {
        tmp |= (1 << 0);   // CHG_EN = 1 (disable charging)
        bq_write(BQ25155_REG_CHGCTRL1, tmp);
    }

    ESP_LOGI(TAG, "BQ25155 charger configured.");
}

// -----------------------------------------------------------------------------
// INIT (AUTO CONFIG + DEVICE SETUP)
// -----------------------------------------------------------------------------
esp_err_t power_init(void)
{
    if (s_power_init) return ESP_OK;

    ESP_LOGI(TAG, "Initializing power module...");

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = POWER_I2C_PORT,
        .scl_io_num = POWER_I2C_SCL,
        .sda_io_num = POWER_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = 1 }
    };

    i2c_new_master_bus(&bus_cfg, &s_power_bus);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BQ25155_ADDR,
        .scl_speed_hz    = POWER_I2C_FREQ_HZ,
    };

    if (i2c_master_bus_add_device(s_power_bus, &dev_cfg, &s_bq_dev) == ESP_OK) {
        ESP_LOGI(TAG, "BQ25155 detected at 0x6B.");

        // Auto-configure charger here so main.c needs no changes
        power_bq25155_configure_charger();
    }

    s_power_init = true;
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// READ DEVICE ID
// -----------------------------------------------------------------------------
esp_err_t power_bq25155_read_id(uint8_t *out_id)
{
    if (!s_power_init) power_init();
    if (!s_bq_dev) return ESP_ERR_INVALID_STATE;

    uint8_t id = 0;
    bq_read(BQ25155_REG_DEVICE_ID, &id);

    if (out_id) *out_id = id;
    ESP_LOGI(TAG, "BQ25155 Device ID = 0x%02X", id);

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// READ BATTERY VOLTAGE
// -----------------------------------------------------------------------------
esp_err_t power_bq25155_read_battery(float *out_voltage, float *out_percent)
{
    if (!s_power_init) power_init();
    if (!s_bq_dev) return ESP_ERR_INVALID_STATE;

    uint8_t buf[2];
    uint8_t reg = BQ25155_REG_VBAT_ADC_H;

    if (i2c_master_transmit_receive(s_bq_dev, &reg, 1, buf, 2, 100) != ESP_OK)
        return ESP_FAIL;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    float vbat = (raw / 65536.0f) * 6.0f;
    float pct = calculate_battery_percentage(vbat);

    if (out_voltage) *out_voltage = vbat;
    if (out_percent) *out_percent = pct;

    ESP_LOGI(TAG, "Battery %.3f V (%.1f%%)", vbat, pct);

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// READ STATUS (OPTIONAL)
// -----------------------------------------------------------------------------
esp_err_t power_bq25155_read_status(void)
{
    if (!s_bq_dev) return ESP_ERR_INVALID_STATE;

    uint8_t st0 = 0, st1 = 0;
    bq_read(BQ25155_REG_STATUS0, &st0);
    bq_read(BQ25155_REG_STATUS1, &st1);

    ESP_LOGI(TAG, "Status0=0x%02X Status1=0x%02X", st0, st1);
    return ESP_OK;
}
