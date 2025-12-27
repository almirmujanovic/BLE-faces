#include "gesture.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gesture.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <string.h>
#include "ble_hid.h"
#include "media.h"

static const char *TAG = "PAJ7620";

// Only a device handle now; bus comes from power.c
static i2c_master_dev_handle_t paj7620_dev_handle = NULL;

// Global state
static bool _gesture_high_rate = true;
static paj7620_gesture_t _current_gesture = PAJ7620_GESTURE_NONE;

// PAJ7620 initialization array (from DFRobot library)
static const uint8_t init_register_array[][2] = {
    {0xEF,0x00},
    {0x32,0x29},
    {0x33,0x01},
    {0x34,0x00},
    {0x35,0x01},
    {0x36,0x00},
    {0x37,0x07},
    {0x38,0x17},
    {0x39,0x06},
    {0x3A,0x12},
    {0x3F,0x00},
    {0x40,0x02},
    {0x41,0xFF},
    {0x42,0x01},
    {0x46,0x2D},
    {0x47,0x0F},
    {0x48,0x3C},
    {0x49,0x00},
    {0x4A,0x1E},
    {0x4B,0x00},
    {0x4C,0x20},
    {0x4D,0x00},
    {0x4E,0x1A},
    {0x4F,0x14},
    {0x50,0x00},
    {0x51,0x10},
    {0x52,0x00},
    {0x5C,0x02},
    {0x5D,0x00},
    {0x5E,0x10},
    {0x5F,0x3F},
    {0x60,0x27},
    {0x61,0x28},
    {0x62,0x00},
    {0x63,0x03},
    {0x64,0xF7},
    {0x65,0x03},
    {0x66,0xD9},
    {0x67,0x03},
    {0x68,0x01},
    {0x69,0xC8},
    {0x6A,0x40},
    {0x6D,0x04},
    {0x6E,0x00},
    {0x6F,0x00},
    {0x70,0x80},
    {0x71,0x00},
    {0x72,0x00},
    {0x73,0x00},
    {0x74,0xF0},
    {0x75,0x00},
    {0x80,0x42},
    {0x81,0x44},
    {0x82,0x04},
    {0x83,0x20},
    {0x84,0x20},
    {0x85,0x00},
    {0x86,0x10},
    {0x87,0x00},
    {0x88,0x05},
    {0x89,0x18},
    {0x8A,0x10},
    {0x8B,0x01},
    {0x8C,0x37},
    {0x8D,0x00},
    {0x8E,0xF0},
    {0x8F,0x81},
    {0x90,0x06},
    {0x91,0x06},
    {0x92,0x1E},
    {0x93,0x0D},
    {0x94,0x0A},
    {0x95,0x0A},
    {0x96,0x0C},
    {0x97,0x05},
    {0x98,0x0A},
    {0x99,0x41},
    {0x9A,0x14},
    {0x9B,0x0A},
    {0x9C,0x3F},
    {0x9D,0x33},
    {0x9E,0xAE},
    {0x9F,0xF9},
    {0xA0,0x48},
    {0xA1,0x13},
    {0xA2,0x10},
    {0xA3,0x08},
    {0xA4,0x30},
    {0xA5,0x19},
    {0xA6,0x10},
    {0xA7,0x08},
    {0xA8,0x24},
    {0xA9,0x04},
    {0xAA,0x1E},
    {0xAB,0x1E},
    {0xCC,0x19},
    {0xCD,0x0B},
    {0xCE,0x13},
    {0xCF,0x64},
    {0xD0,0x21},
    {0xD1,0x0F},
    {0xD2,0x88},
    {0xE0,0x01},
    {0xE1,0x04},
    {0xE2,0x41},
    {0xE3,0xD6},
    {0xE4,0x00},
    {0xE5,0x0C},
    {0xE6,0x0A},
    {0xE7,0x00},
    {0xE8,0x00},
    {0xE9,0x00},
    {0xEE,0x07},
    {0xEF,0x01},
    {0x00,0x1E},
    {0x01,0x1E},
    {0x02,0x0F},
    {0x03,0x10},
    {0x04,0x02},
    {0x05,0x00},
    {0x06,0xB0},
    {0x07,0x04},
    {0x08,0x0D},
    {0x09,0x0E},
    {0x0A,0x9C},
    {0x0B,0x04},
    {0x0C,0x05},
    {0x0D,0x0F},
    {0x0E,0x02},
    {0x0F,0x12},
    {0x10,0x02},
    {0x11,0x02},
    {0x12,0x00},
    {0x13,0x01},
    {0x14,0x05},
    {0x15,0x07},
    {0x16,0x05},
    {0x17,0x07},
    {0x18,0x01},
    {0x19,0x04},
    {0x1A,0x05},
    {0x1B,0x0C},
    {0x1C,0x2A},
    {0x1D,0x01},
    {0x1E,0x00},
    {0x21,0x00},
    {0x22,0x00},
    {0x23,0x00},
    {0x25,0x01},
    {0x26,0x00},
    {0x27,0x39},
    {0x28,0x7F},
    {0x29,0x08},
    {0x30,0x03},
    {0x31,0x00},
    {0x32,0x1A},
    {0x33,0x1A},
    {0x34,0x07},
    {0x35,0x07},
    {0x36,0x01},
    {0x37,0xFF},
    {0x38,0x36},
    {0x39,0x07},
    {0x3A,0x00},
    {0x3E,0xFF},
    {0x3F,0x00},
    {0x40,0x77},
    {0x41,0x40},
    {0x42,0x00},
    {0x43,0x30},
    {0x44,0xA0},
    {0x45,0x5C},
    {0x46,0x00},
    {0x47,0x00},
    {0x48,0x58},
    {0x4A,0x1E},
    {0x4B,0x1E},
    {0x4C,0x00},
    {0x4D,0x00},
    {0x4E,0xA0},
    {0x4F,0x80},
    {0x50,0x00},
    {0x51,0x00},
    {0x52,0x00},
    {0x53,0x00},
    {0x54,0x00},
    {0x57,0x80},
    {0x59,0x10},
    {0x5A,0x08},
    {0x5B,0x94},
    {0x5C,0xE8},
    {0x5D,0x08},
    {0x5E,0x3D},
    {0x5F,0x99},
    {0x60,0x45},
    {0x61,0x40},
    {0x63,0x2D},
    {0x64,0x02},
    {0x65,0x96},
    {0x66,0x00},
    {0x67,0x97},
    {0x68,0x01},
    {0x69,0xCD},
    {0x6A,0x01},
    {0x6B,0xB0},
    {0x6C,0x04},
    {0x6D,0x2C},
    {0x6E,0x01},
    {0x6F,0x32},
    {0x71,0x00},
    {0x72,0x01},
    {0x73,0x35},
    {0x74,0x00},
    {0x75,0x33},
    {0x76,0x31},
    {0x77,0x01},
    {0x7C,0x84},
    {0x7D,0x03},
    {0x7E,0x01}
};

// I2C timeout in ms (-1 = default, but we want explicit timeout for stability)
#define PAJ7620_I2C_TIMEOUT_MS  100

// Write a single byte to PAJ7620 register with explicit timeout
static esp_err_t paj7620_write_reg(uint8_t reg, uint8_t data) {
    if (!paj7620_dev_handle) return ESP_ERR_INVALID_STATE;
    uint8_t write_buf[2] = {reg, data};
    esp_err_t ret = i2c_master_transmit(paj7620_dev_handle, write_buf, 2, PAJ7620_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C write failed: reg=0x%02X, data=0x%02X, err=%s", reg, data, esp_err_to_name(ret));
    }
    return ret;
}

// Read bytes from PAJ7620 register with explicit timeout
static esp_err_t paj7620_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    if (!paj7620_dev_handle) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = i2c_master_transmit_receive(paj7620_dev_handle, &reg, 1, data, len, PAJ7620_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C read failed: reg=0x%02X, len=%d, err=%s", reg, (int)len, esp_err_to_name(ret));
    }
    return ret;
}

// Select register bank (with short delay for stability)
static esp_err_t paj7620_select_bank(paj7620_bank_t bank) {
    esp_err_t ret = paj7620_write_reg(PAJ7620_REGITER_BANK_SEL, (uint8_t)bank);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after bank switch
    }
    return ret;
}

// Wakeup the sensor - first I2C transaction wakes the device but is NOT processed
// Must send bank select TWICE: first to wake, second to actually select
static esp_err_t paj7620_wakeup(void) {
    if (!paj7620_dev_handle) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Sending wakeup sequence...");
    
    // Method 1: Send dummy bank select command to wake up the device
    // This transaction wakes the sensor but the command itself is ignored
    uint8_t wakeup_cmd[2] = {PAJ7620_REGITER_BANK_SEL, 0x00};
    esp_err_t ret = i2c_master_transmit(paj7620_dev_handle, wakeup_cmd, 2, PAJ7620_I2C_TIMEOUT_MS);
    
    // The wakeup transaction may fail with NACK - that's expected if sensor is in deep sleep
    // We don't check the return value here
    (void)ret;
    
    // Wait for sensor to wake up (datasheet recommends ~700us, we use 5ms for safety)
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Now send the real bank select command - this one should be processed
    ret = paj7620_select_bank(PAJ7620_BANK0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Second bank select failed, retrying after longer delay...");
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = paj7620_select_bank(PAJ7620_BANK0);
    }
    
    return ret;
}

// ------------------------------------------------------
// I2C init – attach to shared bus from power.c
// ------------------------------------------------------
esp_err_t paj7620_i2c_init(void) {
    esp_err_t ret;

    if (paj7620_dev_handle) {
        // Already initialized
        return ESP_OK;
    }

    // Ensure the shared bus is initialized
    ret = power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "power_init() failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_master_bus_handle_t bus = power_get_i2c_bus();
    if (!bus) {
        ESP_LOGE(TAG, "No I2C bus handle from power module");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure PAJ7620 device on the existing bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PAJ7620_I2C_ADDR,
        .scl_speed_hz    = PAJ7620_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus, &dev_config, &paj7620_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PAJ7620 device: %s", esp_err_to_name(ret));
        paj7620_dev_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "PAJ7620 I2C attached on shared bus, addr=0x%02X", PAJ7620_I2C_ADDR);
    return ESP_OK;
}

#define PAJ7620_INIT_MAX_RETRIES 5
#define PAJ7620_RETRY_DELAY_MS   300

esp_err_t paj7620_init(void) {
    esp_err_t ret = ESP_FAIL;
    uint8_t part_id[2];

    // Full initialization with retry logic
    for (int attempt = 1; attempt <= PAJ7620_INIT_MAX_RETRIES; attempt++) {
        ESP_LOGI(TAG, "PAJ7620 init attempt %d/%d", attempt, PAJ7620_INIT_MAX_RETRIES);
        
        // Step 1: Wait for sensor power-up (first attempt needs longer delay)
        if (attempt == 1) {
            vTaskDelay(pdMS_TO_TICKS(700));  // Initial power-up delay per datasheet
        } else {
            vTaskDelay(pdMS_TO_TICKS(PAJ7620_RETRY_DELAY_MS));
        }
        
        // Step 2: Wakeup sequence (sends bank select twice - first wakes, second processes)
        ret = paj7620_wakeup();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Wakeup failed on attempt %d: %s", attempt, esp_err_to_name(ret));
            continue;  // Retry
        }
        
        // Step 3: Read and verify Part ID
        ret = paj7620_read_reg(PAJ7620_ADDR_PART_ID_LOW, part_id, 2);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read Part ID on attempt %d: %s", attempt, esp_err_to_name(ret));
            continue;  // Retry
        }
        
        uint16_t pid = part_id[0] | (part_id[1] << 8);
        ESP_LOGI(TAG, "Part ID: 0x%04X", pid);
        
        if (pid != PAJ7620_PARTID) {
            ESP_LOGW(TAG, "Invalid Part ID on attempt %d: expected 0x%04X, got 0x%04X",
                     attempt, PAJ7620_PARTID, pid);
            ret = ESP_ERR_NOT_FOUND;
            continue;  // Retry
        }
        
        // Step 4: Write initialization array
        size_t init_array_size = sizeof(init_register_array) / sizeof(init_register_array[0]);
        ESP_LOGI(TAG, "Writing %d initialization registers...", (int)init_array_size);
        
        bool init_success = true;
        for (size_t i = 0; i < init_array_size; i++) {
            ret = paj7620_write_reg(init_register_array[i][0], init_register_array[i][1]);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to write reg %d (0x%02X) on attempt %d", 
                         (int)i, init_register_array[i][0], attempt);
                init_success = false;
                break;
            }
            // Small delay every 20 registers to prevent I2C bus overload
            if (i > 0 && (i % 20) == 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        if (!init_success) {
            continue;  // Retry full init
        }
        
        // Step 5: Select Bank 0 for gesture reading
        vTaskDelay(pdMS_TO_TICKS(10));  // Short delay after init sequence
        ret = paj7620_select_bank(PAJ7620_BANK0);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Final bank select failed on attempt %d", attempt);
            continue;  // Retry
        }
        
        // Success!
        ESP_LOGI(TAG, "✅ PAJ7620 initialized successfully on attempt %d", attempt);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "❌ PAJ7620 initialization failed after %d attempts", PAJ7620_INIT_MAX_RETRIES);
    return ret;
}

void paj7620_set_high_rate(bool high_rate) {
    _gesture_high_rate = high_rate;
}

paj7620_gesture_t paj7620_get_gesture(void) {
    uint8_t data_flag1 = 0;
    uint8_t data_flag0 = 0;
    esp_err_t ret;

    // Read FLAG_1 first (for wave detection)
    ret = paj7620_read_reg(PAJ7620_ADDR_GES_PS_DET_FLAG_1, &data_flag1, 1);
    if (ret != ESP_OK) {
        // I2C read failed - return no gesture rather than crash
        return PAJ7620_GESTURE_NONE;
    }
    _current_gesture = (paj7620_gesture_t)(((uint16_t)data_flag1) << 8);

    if (_current_gesture == PAJ7620_GESTURE_WAVE) {
        ESP_LOGD(TAG, "Wave detected (FLAG_1)");
        vTaskDelay(pdMS_TO_TICKS(GES_QUIT_TIME));
    } else {
        _current_gesture = PAJ7620_GESTURE_NONE;
        // Read FLAG_0 for other gestures
        ret = paj7620_read_reg(PAJ7620_ADDR_GES_PS_DET_FLAG_0, &data_flag0, 1);
        if (ret != ESP_OK) {
            return PAJ7620_GESTURE_NONE;
        }
        _current_gesture = (paj7620_gesture_t)(((uint16_t)data_flag0) & 0x00FF);

        if (!_gesture_high_rate && _current_gesture != PAJ7620_GESTURE_NONE) {
            // Slow mode: wait and read again to detect combined gestures
            uint8_t tmp = 0;
            vTaskDelay(pdMS_TO_TICKS(GES_ENTRY_TIME));
            paj7620_read_reg(PAJ7620_ADDR_GES_PS_DET_FLAG_0, &tmp, 1);
            _current_gesture = (paj7620_gesture_t)(((uint16_t)_current_gesture) | tmp);
        }

        if (_current_gesture != PAJ7620_GESTURE_NONE) {
            // Add delays for forward/backward gestures
            if (_current_gesture == PAJ7620_GESTURE_FORWARD ||
                _current_gesture == PAJ7620_GESTURE_BACKWARD) {
                if (!_gesture_high_rate) {
                    vTaskDelay(pdMS_TO_TICKS(GES_QUIT_TIME));
                } else {
                    vTaskDelay(pdMS_TO_TICKS(GES_QUIT_TIME / 5));
                }
            }

            // Check for combined wave gestures
            if (_current_gesture != PAJ7620_GESTURE_RIGHT &&
                _current_gesture != PAJ7620_GESTURE_LEFT &&
                _current_gesture != PAJ7620_GESTURE_UP &&
                _current_gesture != PAJ7620_GESTURE_DOWN &&
                _current_gesture != PAJ7620_GESTURE_FORWARD &&
                _current_gesture != PAJ7620_GESTURE_BACKWARD &&
                _current_gesture != PAJ7620_GESTURE_CLOCKWISE &&
                _current_gesture != PAJ7620_GESTURE_ANTICLOCKWISE) {

                // Check FLAG_1 again for wave
                paj7620_read_reg(PAJ7620_ADDR_GES_PS_DET_FLAG_1, &data_flag1, 1);
                if (data_flag1) {
                    _current_gesture = PAJ7620_GESTURE_WAVE;
                } else {
                    // Determine slow wave type
                    if (_current_gesture != PAJ7620_GESTURE_WAVE_SLOWLY_LEFT_RIGHT &&
                        _current_gesture != PAJ7620_GESTURE_WAVE_SLOWLY_UP_DOWN &&
                        _current_gesture != PAJ7620_GESTURE_WAVE_SLOWLY_FORWARD_BACKWARD) {
                        _current_gesture = PAJ7620_GESTURE_WAVE_SLOWLY_DISORDER;
                    }
                }
            }
        }
    }

    return _current_gesture;
}

const char* paj7620_gesture_name(paj7620_gesture_t gesture) {
    switch (gesture) {
        case PAJ7620_GESTURE_NONE: return "None";
        case PAJ7620_GESTURE_RIGHT: return "Right";
        case PAJ7620_GESTURE_LEFT: return "Left";
        case PAJ7620_GESTURE_UP: return "Up";
        case PAJ7620_GESTURE_DOWN: return "Down";
        case PAJ7620_GESTURE_FORWARD: return "Forward";
        case PAJ7620_GESTURE_BACKWARD: return "Backward";
        case PAJ7620_GESTURE_CLOCKWISE: return "Clockwise";
        case PAJ7620_GESTURE_ANTICLOCKWISE: return "Anti-Clockwise";
        case PAJ7620_GESTURE_WAVE: return "Wave";
        case PAJ7620_GESTURE_WAVE_SLOWLY_DISORDER: return "WaveSlowlyDisorder";
        case PAJ7620_GESTURE_WAVE_SLOWLY_LEFT_RIGHT: return "WaveSlowlyLeftRight";
        case PAJ7620_GESTURE_WAVE_SLOWLY_UP_DOWN: return "WaveSlowlyUpDown";
        case PAJ7620_GESTURE_WAVE_SLOWLY_FORWARD_BACKWARD: return "WaveSlowlyForwardBackward";

        default: return "Unknown";
    }
}

static void on_paj_gesture(paj7620_gesture_t g)
{
    if (g & PAJ7620_GESTURE_CLOCKWISE) {
        // PHOTO (VGA)
        ESP_LOGI(TAG, "📸 Taking photo (VGA)...");
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_err_t ret = media_save_photo(fb); // media_save_photo should handle VGA now
            esp_camera_fb_return(fb);
            (void)ret;
        } else {
            ESP_LOGE(TAG, "❌ Camera capture failed");
        }
        return;
    }

    if (g & PAJ7620_GESTURE_ANTICLOCKWISE) {
        // VIDEO (disabled for now, but kept for future)
        // esp_err_t res = media_capture_jpeg_burst(10000, 20, 200);
        // (void)res;
    }

    // BLE HID CONTROLS (only when connected)
    if (!ble_hid_is_ready()) {
        // Not connected - ignore other gestures
        return;
    }

    // BLE is connected - use other gestures for HID
    if (g & PAJ7620_GESTURE_UP)           (void)ble_hid_cc_tap_vol_up();
    else if (g & PAJ7620_GESTURE_DOWN)    (void)ble_hid_cc_tap_vol_down();
    else if (g & PAJ7620_GESTURE_WAVE)    (void)ble_hid_cc_tap_vol_down();
    else if (g & PAJ7620_GESTURE_LEFT)    (void)ble_hid_cc_tap_prev();
    else if (g & PAJ7620_GESTURE_RIGHT)   (void)ble_hid_cc_tap_next();
}

// Track consecutive I2C failures for recovery
static uint8_t s_i2c_fail_count = 0;
#define PAJ7620_MAX_I2C_FAILURES 10

void paj7620_task(void *pvParameters) {
    paj7620_gesture_t gesture;
    paj7620_gesture_t last_gesture = PAJ7620_GESTURE_NONE;

    ESP_LOGI(TAG, "Gesture reading task started (high_rate=%d)", _gesture_high_rate);

    while (1) {
        gesture = paj7620_get_gesture();

        // Track failures for potential recovery
        if (gesture == PAJ7620_GESTURE_NONE) {
            // Could be real "no gesture" or I2C failure - we can't distinguish easily
            // Reset failure count on any read (even if no gesture)
            s_i2c_fail_count = 0;
        }

        if (gesture != PAJ7620_GESTURE_NONE && gesture != last_gesture) {
            ESP_LOGI(TAG, "👋 Gesture: %s (0x%04X)", paj7620_gesture_name(gesture), gesture);
            on_paj_gesture(gesture);
            last_gesture = gesture;
            s_i2c_fail_count = 0;  // Successful read
        } else if (gesture == PAJ7620_GESTURE_NONE) {
            last_gesture = PAJ7620_GESTURE_NONE;
        }

        // Faster polling for better responsiveness (30ms in high-rate mode)
        vTaskDelay(pdMS_TO_TICKS(_gesture_high_rate ? 30 : 80));
    }
}
