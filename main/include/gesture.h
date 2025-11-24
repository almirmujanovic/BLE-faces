#ifndef GESTURE_H
#define GESTURE_H

#include "power.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// I2C Configuration (pins/port now configured in power.c; kept here for reference)
#define PAJ7620_I2C_NUM         I2C_NUM_0
#define PAJ7620_I2C_SDA_PIN     21
#define PAJ7620_I2C_SCL_PIN     20
#define PAJ7620_I2C_FREQ_HZ     400000  // 400kHz

// DEVICE ID
#define PAJ7620_I2C_ADDR        0x73
#define PAJ7620_PARTID          0x7620

// REGISTER BANK SELECT
#define PAJ7620_REGITER_BANK_SEL  0xEF

// REGISTER BANK 0
#define PAJ7620_ADDR_PART_ID_LOW         0x00
#define PAJ7620_ADDR_PART_ID_HIGH        0x01
#define PAJ7620_ADDR_VERSION_ID          0x01
#define PAJ7620_ADDR_SUSPEND_CMD         0x03
#define PAJ7620_ADDR_GES_PS_DET_MASK_0   0x41
#define PAJ7620_ADDR_GES_PS_DET_MASK_1   0x42
#define PAJ7620_ADDR_GES_PS_DET_FLAG_0   0x43
#define PAJ7620_ADDR_GES_PS_DET_FLAG_1   0x44
#define PAJ7620_ADDR_STATE_INDICATOR     0x45
#define PAJ7620_ADDR_PS_HIGH_THRESHOLD   0x69
#define PAJ7620_ADDR_PS_LOW_THRESHOLD    0x6A
#define PAJ7620_ADDR_PS_APPROACH_STATE   0x6B
#define PAJ7620_ADDR_PS_RAW_DATA         0x6C

// REGISTER BANK 1
#define PAJ7620_ADDR_PS_GAIN             0x44
#define PAJ7620_ADDR_IDLE_S1_STEP_0      0x67
#define PAJ7620_ADDR_IDLE_S1_STEP_1      0x68
#define PAJ7620_ADDR_IDLE_S2_STEP_0      0x69
#define PAJ7620_ADDR_IDLE_S2_STEP_1      0x6A
#define PAJ7620_ADDR_OP_TO_S1_STEP_0     0x6B
#define PAJ7620_ADDR_OP_TO_S1_STEP_1     0x6C
#define PAJ7620_ADDR_OP_TO_S2_STEP_0     0x6D
#define PAJ7620_ADDR_OP_TO_S2_STEP_1     0x6E
#define PAJ7620_ADDR_OPERATION_ENABLE    0x72

// Timing constants
#define GES_REACTION_TIME  50    // You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME     2000  // When you want to recognize the Forward/Backward gestures
#define GES_QUIT_TIME      1000

/**
 * @enum paj7620_gesture_t
 * @brief Gesture information
 */
typedef enum {
    PAJ7620_GESTURE_NONE = 0x00,
    PAJ7620_GESTURE_RIGHT = 0x01 << 0,
    PAJ7620_GESTURE_LEFT = 0x01 << 1,
    PAJ7620_GESTURE_UP = 0x01 << 2,
    PAJ7620_GESTURE_DOWN = 0x01 << 3,
    PAJ7620_GESTURE_FORWARD = 0x01 << 4,
    PAJ7620_GESTURE_BACKWARD = 0x01 << 5,
    PAJ7620_GESTURE_CLOCKWISE = 0x01 << 6,
    PAJ7620_GESTURE_ANTICLOCKWISE = 0x01 << 7,
    PAJ7620_GESTURE_WAVE = 0x01 << 8,
    PAJ7620_GESTURE_WAVE_SLOWLY_DISORDER = 0x01 << 9,
    PAJ7620_GESTURE_WAVE_SLOWLY_LEFT_RIGHT = PAJ7620_GESTURE_LEFT + PAJ7620_GESTURE_RIGHT,
    PAJ7620_GESTURE_WAVE_SLOWLY_UP_DOWN = PAJ7620_GESTURE_UP + PAJ7620_GESTURE_DOWN,
    PAJ7620_GESTURE_WAVE_SLOWLY_FORWARD_BACKWARD = PAJ7620_GESTURE_FORWARD + PAJ7620_GESTURE_BACKWARD,
} paj7620_gesture_t;

/**
 * @enum paj7620_bank_t
 * @brief Register Bank
 */
typedef enum {
    PAJ7620_BANK0 = 0,
    PAJ7620_BANK1 = 1,
} paj7620_bank_t;

/**
 * @brief Initialize I2C device for PAJ7620 (uses shared I2C bus from power.c)
 */
esp_err_t paj7620_i2c_init(void);

/**
 * @brief Initialize PAJ7620 sensor
 */
esp_err_t paj7620_init(void);

/**
 * @brief Set gesture detection mode
 * @param high_rate true for fast detection (120Hz), false for slow detection with expanded gestures
 */
void paj7620_set_high_rate(bool high_rate);

/**
 * @brief Get gesture from PAJ7620
 * @return Detected gesture
 */
paj7620_gesture_t paj7620_get_gesture(void);

/**
 * @brief Get human-readable gesture name
 */
const char* paj7620_gesture_name(paj7620_gesture_t gesture);

/**
 * @brief Task to continuously read gestures
 */
void paj7620_task(void *pvParameters);

esp_err_t ble_hid_tap_key(uint8_t keycode, uint8_t modifier);

#endif // PAJ7620_H
