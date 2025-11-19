#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gesture.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <string.h>
#include "ble_hid.h"

static const char *TAG = "PAJ7620";

// I2C master bus and device handles
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
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

// Write a single byte to PAJ7620 register
static esp_err_t paj7620_write_reg(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(paj7620_dev_handle, write_buf, 2, -1);
}

// Read bytes from PAJ7620 register
static esp_err_t paj7620_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(paj7620_dev_handle, &reg, 1, data, len, -1);
}

// Select register bank
static esp_err_t paj7620_select_bank(paj7620_bank_t bank) {
    return paj7620_write_reg(PAJ7620_REGITER_BANK_SEL, (uint8_t)bank);
}

esp_err_t paj7620_i2c_init(void) {
    esp_err_t ret;
    
    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = PAJ7620_I2C_NUM,
        .sda_io_num = PAJ7620_I2C_SDA_PIN,
        .scl_io_num = PAJ7620_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure PAJ7620 device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PAJ7620_I2C_ADDR,
        .scl_speed_hz = PAJ7620_I2C_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &paj7620_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PAJ7620 device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle);
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", PAJ7620_I2C_SDA_PIN, PAJ7620_I2C_SCL_PIN);
    return ESP_OK;
}

esp_err_t paj7620_init(void) {
    esp_err_t ret;
    uint8_t part_id[2];
    
    // Wait for sensor to wake up
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // Select Bank 0
    ret = paj7620_select_bank(PAJ7620_BANK0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select bank 0");
        return ret;
    }
    
    // Read Part ID (2 bytes)
    ret = paj7620_read_reg(PAJ7620_ADDR_PART_ID_LOW, part_id, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Part ID");
        return ret;
    }
    
    uint16_t pid = part_id[0] | (part_id[1] << 8);
    ESP_LOGI(TAG, "Part ID: 0x%04X", pid);
    
    if (pid != PAJ7620_PARTID) {
        ESP_LOGE(TAG, "Invalid Part ID! Expected 0x%04X, got 0x%04X", PAJ7620_PARTID, pid);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Write initialization array
    size_t init_array_size = sizeof(init_register_array) / sizeof(init_register_array[0]);
    ESP_LOGI(TAG, "Writing %d initialization registers...", init_array_size);
    
    for (size_t i = 0; i < init_array_size; i++) {
        ret = paj7620_write_reg(init_register_array[i][0], init_register_array[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write init register %d", i);
            return ret;
        }
    }
    
    // Select Bank 0 for gesture reading
    ret = paj7620_select_bank(PAJ7620_BANK0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select bank 0");
        return ret;
    }
    
    ESP_LOGI(TAG, "PAJ7620 initialized successfully");
    return ESP_OK;
}

void paj7620_set_high_rate(bool high_rate) {
    _gesture_high_rate = high_rate;
}

paj7620_gesture_t paj7620_get_gesture(void) {
    uint8_t data_flag1 = 0;
    uint8_t data_flag0 = 0;
    
    // Read FLAG_1 first (for wave detection)
    paj7620_read_reg(PAJ7620_ADDR_GES_PS_DET_FLAG_1, &data_flag1, 1);
    _current_gesture = (paj7620_gesture_t)(((uint16_t)data_flag1) << 8);
    
    if (_current_gesture == PAJ7620_GESTURE_WAVE) {
        ESP_LOGD(TAG, "Wave detected (FLAG_1)");
        vTaskDelay(pdMS_TO_TICKS(GES_QUIT_TIME));
    } else {
        _current_gesture = PAJ7620_GESTURE_NONE;
        // Read FLAG_0 for other gestures
        paj7620_read_reg(PAJ7620_ADDR_GES_PS_DET_FLAG_0, &data_flag0, 1);
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
    if (!ble_hid_is_ready()) {
        // Not connected or CCCD not enabled yet; ignore gestures quietly
        return;
    }

    if (g & PAJ7620_GESTURE_UP)           (void)ble_hid_cc_tap_vol_up();
    else if (g & PAJ7620_GESTURE_DOWN)    (void)ble_hid_cc_tap_vol_down();
    else if (g & PAJ7620_GESTURE_LEFT)    (void)ble_hid_cc_tap_prev();
    else if (g & PAJ7620_GESTURE_RIGHT)   (void)ble_hid_cc_tap_next();
    else if (g & PAJ7620_GESTURE_FORWARD) {
        // TODO: trigger PHOTO (placeholder)
     //   (void)ble_hid_cc_tap_play_pause(); // or leave commented for now
    } else if (g & PAJ7620_GESTURE_BACKWARD) {
        // TODO: trigger VIDEO (placeholder)
       // (void)ble_hid_cc_tap_mute(); // or leave commented
    }
}


void paj7620_task(void *pvParameters) {
    paj7620_gesture_t gesture;
    paj7620_gesture_t last_gesture = PAJ7620_GESTURE_NONE;

    ESP_LOGI(TAG, "Gesture reading task started (high_rate=%d)", _gesture_high_rate);

    while (1) {
        gesture = paj7620_get_gesture();

        if (gesture != PAJ7620_GESTURE_NONE && gesture != last_gesture) {
            ESP_LOGI(TAG, "GESTURE: %s (0x%04X)", paj7620_gesture_name(gesture), gesture);
            on_paj_gesture(gesture);
            last_gesture = gesture;
        } else if (gesture == PAJ7620_GESTURE_NONE) {
            last_gesture = PAJ7620_GESTURE_NONE;
        }

        vTaskDelay(pdMS_TO_TICKS(_gesture_high_rate ? 50 : 100));
    }
}
// ...existing code...