#include "camera_config.h"
#include "power.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAMERA_OV5640";

esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "esp_camera_init() for OV5640 YUV422...");

    // --- Power-down pin  ---
    if (CAM_PWDN_GPIO >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << CAM_PWDN_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(CAM_PWDN_GPIO, 0);  // power ON
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // --- Reset pin ---
    if (CAM_RESET_GPIO >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << CAM_RESET_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(CAM_RESET_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(CAM_RESET_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }


    // --------------------------------------------------------------------
    camera_config_t config = {
        .pin_pwdn      = CAM_PWDN_GPIO,
        .pin_reset     = CAM_RESET_GPIO,
        .pin_xclk      = CAM_XCLK_GPIO,
        // Use the shared I2C bus initialized in power.c
        .pin_sccb_sda  = -1,
        .pin_sccb_scl  = -1,

        .pin_d7        = CAM_D7_GPIO,
        .pin_d6        = CAM_D6_GPIO,
        .pin_d5        = CAM_D5_GPIO,
        .pin_d4        = CAM_D4_GPIO,
        .pin_d3        = CAM_D3_GPIO,
        .pin_d2        = CAM_D2_GPIO,
        .pin_d1        = CAM_D1_GPIO,
        .pin_d0        = CAM_D0_GPIO,

        .pin_vsync     = CAM_VSYNC_GPIO,
        .pin_href      = CAM_HREF_GPIO,
        .pin_pclk      = CAM_PCLK_GPIO,

        .xclk_freq_hz  = 20000000,
        .ledc_timer    = LEDC_TIMER_0,
        .ledc_channel  = LEDC_CHANNEL_0,

        .pixel_format  = PIXFORMAT_YUV422,
        .frame_size    = FRAMESIZE_QVGA,
        .jpeg_quality  = 12,
        .fb_count      = 2,
        .fb_location   = CAMERA_FB_IN_PSRAM,
        .grab_mode     = CAMERA_GRAB_LATEST,
        .sccb_i2c_port = POWER_I2C_PORT,
    };
    // --------------------------------------------------------------------

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init FAILED: 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Failed to get sensor");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Detected sensor PID: 0x%02X", s->id.PID);

    if (s->id.PID == OV5640_PID) {
        ESP_LOGI(TAG, "Configuring OV5640...");

        /* ---- BASIC, NEUTRAL SETTINGS ---- */

  // Make sure we are really RGB565 + VGA
    s->set_pixformat(s, PIXFORMAT_YUV422);
    s->set_framesize(s, FRAMESIZE_QVGA);

    // ---------- PROFILE A: “balanced defaults” (good starting point) ----------
    // Make sure auto-exposure and auto-gain are ON
    s->set_exposure_ctrl(s, 0);   // enable AEC (auto exposure)
    s->set_ae_level(s, 0);        // tweak -2..2, try 0 or +1 if still dark
    s->set_aec2(s, 1);            // improved AEC if supported

    s->set_gain_ctrl(s, 1);       // enable AGC (auto gain)
    s->set_agc_gain(s, 0);        // let it pick gain

    // Auto white balance
    s->set_whitebal(s, 1);        // enable AWB
    s->set_awb_gain(s, 1);        // enable AWB gain
    s->set_wb_mode(s, 0);         // 0 = auto (others: 1=sunny, 2=cloudy, etc.)

    // General look
    s->set_brightness(s, 1);      // -2..2   (1 brightens a bit)
    s->set_contrast(s, -1);        // -2..2
    s->set_saturation(s, 1);      // -2..2
    s->set_special_effect(s, 0);  // 0 = none (make sure no sepia/negative/etc.)

    // Lens / correction
    s->set_lenc(s, 1);            // lens correction
    s->set_bpc(s, 1);             // black pixel correction
    s->set_wpc(s, 1);             // white pixel correction
    s->set_raw_gma(s, 1);         // gamma curve on




        ESP_LOGI(TAG, "OV5640 is running ");
    }

    ESP_LOGI(TAG, "Camera init COMPLETE.");
    return ESP_OK;
}

camera_fb_t *capture_frame(void)
{
    return esp_camera_fb_get();
}

void release_frame(camera_fb_t *fb)
{
    if (fb) esp_camera_fb_return(fb);
}

void camera_discard_initial_frames(int count)
{
    for (int i = 0; i < count; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
