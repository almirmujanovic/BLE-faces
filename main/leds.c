// leds.c
#include "leds.h"
#include "power.h"          // for power_get_i2c_bus()
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LEDS";

// PCA9536 address + registers
#define PCA9536_ADDR        0x41
#define PCA9536_REG_IN      0x00
#define PCA9536_REG_OUT     0x01
#define PCA9536_REG_POL     0x02
#define PCA9536_REG_CFG     0x03

// Bit positions
#define EIO0_BIT            0   // schematic LED A (idle)
#define EIO1_BIT            1   // button input
#define EIO2_BIT            2
#define EIO3_BIT            3   // schematic LED B (capture)


// Map logical LEDs to physical bits
#define LED_IDLE_BIT    EIO0_BIT
#define LED_BTN_BIT     EIO1_BIT
#define LED_CAPTURE_BIT EIO3_BIT

static i2c_master_dev_handle_t s_pca_dev = NULL;
static uint8_t                 s_output_state = 0x09; // EIO0=1, EIO3=1


static esp_err_t pca9536_read_reg(uint8_t reg, uint8_t *val)
{
    if (!s_pca_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_pca_dev, &reg, 1, val, 1, pdMS_TO_TICKS(100));
}

static esp_err_t pca9536_write_reg(uint8_t reg, uint8_t val)
{
    if (!s_pca_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_pca_dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}
// Set the cached output state and write it
static esp_err_t pca9536_write_output_state(uint8_t new_state)
{
    s_output_state = new_state;
    return pca9536_write_reg(PCA9536_REG_OUT, s_output_state);
}

// expose read inputs and configure one pin as input
esp_err_t leds_configure_button_input(void)
{
    if (!s_pca_dev) return ESP_ERR_INVALID_STATE;

    // PCA9536 CFG: 1=input, 0=output. Set IO1 (button) to input; others outputs (0).
    uint8_t cfg = (1 << LED_BTN_BIT);
    esp_err_t err = pca9536_write_reg(PCA9536_REG_CFG, cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set PCA9536 CFG: %s", esp_err_to_name(err));
        return err;
    }
    // read back verify
    uint8_t read_cfg = 0;
    if (pca9536_read_reg(PCA9536_REG_CFG, &read_cfg) == ESP_OK) {
        ESP_LOGI(TAG, "PCA9536 CFG set to 0x%02X", read_cfg);
    }
    return ESP_OK;
}

esp_err_t leds_read_inputs(uint8_t *out_inputs)
{
    if (!s_pca_dev || !out_inputs) return ESP_ERR_INVALID_ARG;
    return pca9536_read_reg(PCA9536_REG_IN, out_inputs);
}


bool leds_button_is_pressed(void)
{
    uint8_t in = 0;
    if (leds_read_inputs(&in) != ESP_OK) return false;
    // IO1 is button; polarity depends on wiring. 0=low,1=high
    bool pressed = ((in >> LED_BTN_BIT) & 0x01) ? true : false;
    // If your button pulls low when pressed, invert this return value.
    return pressed;
}


// ---------------------------------------------------
// Internal: set all 4 outputs at once
// ---------------------------------------------------
static esp_err_t pca9536_set_all(uint8_t eio0, uint8_t led1,
                                 uint8_t led2, uint8_t eio3)
{
    // Ensure only LSB used for each value and map to correct EIO bits
    s_output_state =
        ((eio0  & 0x01) << EIO0_BIT) |
        ((led1  & 0x01) << EIO1_BIT) |
        ((led2  & 0x01) << EIO2_BIT) |
        ((eio3  & 0x01) << EIO3_BIT);

    esp_err_t err = pca9536_write_reg(PCA9536_REG_OUT, s_output_state);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set PCA9536 outputs: %s", esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------
// Public: initialize PCA9536 on shared I2C bus
// ---------------------------------------------------
// Initialize PCA9536 and set initial outputs
esp_err_t leds_init(void)
{
    ESP_LOGI(TAG, "Initializing LEDs (PCA9536) ...");

    // Make sure power/I2C is ready
    esp_err_t err = power_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "power_init() failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_master_bus_handle_t bus = power_get_i2c_bus();
    if (!bus) {
        ESP_LOGE(TAG, "No I2C bus from power module");
        return ESP_ERR_INVALID_STATE;
    }

    // Add PCA9536 device on the same bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PCA9536_ADDR,
        .scl_speed_hz    = 100000,
    };

    err = i2c_master_bus_add_device(bus, &dev_cfg, &s_pca_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCA9536 device: %s", esp_err_to_name(err));
        s_pca_dev = NULL;
        return err;
    }

    // Probe the device to make sure it responds on the bus.
    uint8_t cfg = 0;
    err = pca9536_read_reg(PCA9536_REG_CFG, &cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PCA9536 not responding, disabling LED support: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_pca_dev);
        s_pca_dev = NULL;
        return err;
    }

    // Configure pins: set EIO0/EIO2/EIO3 as outputs; EIO1 will be reconfigured when leds_configure_button_input() is called
    err = pca9536_write_reg(PCA9536_REG_CFG, 0x00); // all outputs for now
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config PCA9536: %s", esp_err_to_name(err));
        return err;
    }

    // Initial state: LEDs are active-low, so HIGH = OFF
    s_output_state = (1 << EIO0_BIT) | (1 << EIO3_BIT);
    err = pca9536_write_output_state(s_output_state);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "PCA9536 initialized");
    return ESP_OK;
}
// ---------------------------------------------------
// Public: set LEDs (idle/capture)
// ---------------------------------------------------
void leds_set(bool idle_led, bool capture_led)
{
    if (!s_pca_dev) return;
    uint8_t new_state = s_output_state;
    // LEDs are active-low: set bit LOW (0) to turn LED ON
    if (idle_led) new_state &= ~(1 << LED_IDLE_BIT); else new_state |= (1 << LED_IDLE_BIT);
    if (capture_led) new_state &= ~(1 << LED_CAPTURE_BIT); else new_state |= (1 << LED_CAPTURE_BIT);
    pca9536_write_output_state(new_state);
}

// ---------------------------------------------------
// Public: simple blink helpers
// (blocking, but fine for status indications)
// ---------------------------------------------------
void leds_blink_both(uint8_t times, uint32_t delay_ms)
{
    if (!s_pca_dev) return;
    for (uint8_t i = 0; i < times; ++i) {
        leds_set(true, true);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        leds_set(false, false);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void leds_blink_alternate(uint8_t times, uint32_t delay_ms)
{
    if (!s_pca_dev) return;
    for (uint8_t i = 0; i < times; ++i) {
        leds_set(true, false);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        leds_set(false, true);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    leds_set(false, false);
}

void leds_set_idle(bool on)
{
    if (!s_pca_dev) return;
    uint8_t new_state = s_output_state;
    // LED is active-low: set bit LOW (0) to turn LED ON
    if (on) new_state &= ~(1 << LED_IDLE_BIT); else new_state |= (1 << LED_IDLE_BIT);
    pca9536_write_output_state(new_state);
}

void leds_set_capture(bool on)
{
    if (!s_pca_dev) return;
    uint8_t new_state = s_output_state;
    // LED is active-low: set bit LOW (0) to turn LED ON
    if (on) new_state &= ~(1 << LED_CAPTURE_BIT); else new_state |= (1 << LED_CAPTURE_BIT);
    pca9536_write_output_state(new_state);
}

void leds_blink_idle(uint8_t times, uint32_t delay_ms)
{
    if (!s_pca_dev) return;
    for (uint8_t i = 0; i < times; ++i) {
        leds_set_idle(true);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        leds_set_idle(false);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void leds_blink_capture(uint8_t times, uint32_t delay_ms)
{
    if (!s_pca_dev) return;
    for (uint8_t i = 0; i < times; ++i) {
        leds_set_capture(true);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        leds_set_capture(false);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
