#include "include/ei_infer.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"

#include <cstdlib>
#include <cstring>

// Edge Impulse wrapper (brings in EI headers without Arduino conflicts)
#include "ei_wrapper.h"

static const char *TAG = "ei_infer";

// Raw camera frame size (QVGA)
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS    320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS    240
#define EI_CAMERA_FRAME_BYTE_SIZE          3    // RGB888

// State
static bool     ei_initialized     = false;
static bool     tflite_initialized = false;

// Single buffer: large enough for 320x240x3
static uint8_t *snapshot_buf       = nullptr;

// Persistent EI signal (reads pixels from snapshot_buf)
static ei::signal_t persistent_signal;

// Forward declarations
static bool ei_camera_capture(uint32_t out_width, uint32_t out_height);
static int  ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/**
 * Initialize EI buffers (call once at startup)
 */
extern "C" bool ei_infer_init(void)
{
    if (ei_initialized) {
        ESP_LOGW(TAG, "ei_infer_init called twice, ignoring");
        return true;
    }

    ESP_LOGI(TAG, "Initializing Edge Impulse inference...");

    // One big buffer for raw + resized image (320x240x3)
    const size_t snap_size =
        EI_CAMERA_RAW_FRAME_BUFFER_COLS *
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
        EI_CAMERA_FRAME_BYTE_SIZE;          // 320 * 240 * 3 = 230400

    snapshot_buf = (uint8_t *)heap_caps_malloc(
        snap_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!snapshot_buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for snapshot buffer", snap_size);
        return false;
    }

    ESP_LOGI(TAG, "Allocated snapshot buffer: %zu bytes (320x240x3) in PSRAM", snap_size);

    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "Heap corruption detected immediately after EI buffer allocation!");
        heap_caps_free(snapshot_buf);
        snapshot_buf = nullptr;
        return false;
    }

    // EI camera models use "pixels" as samples: width * height = 96*96 = 9216
    persistent_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;  // 9216
    persistent_signal.get_data     = &ei_camera_get_data;

    ei_initialized = true;

    size_t psram_free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Edge Impulse initialization complete");
    ESP_LOGI(TAG, "Memory - PSRAM: %zu KB, Internal: %zu KB",
             psram_free / 1024, internal_free / 1024);
    ESP_LOGI(TAG, "Model: %dx%d, DSP frame size: %d (width*height=%d)",
             EI_CLASSIFIER_INPUT_WIDTH,
             EI_CLASSIFIER_INPUT_HEIGHT,
             (int)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
             EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT);

    return true;
}

/**
 * Cleanup EI buffers
 */
extern "C" void ei_infer_deinit(void)
{
    if (snapshot_buf) {
        heap_caps_free(snapshot_buf);
        snapshot_buf = nullptr;
    }

    ei_initialized     = false;
    tflite_initialized = false;
}

/**
 * Capture camera frame into snapshot_buf (320x240), convert to RGB888,
 * then IN-PLACE resize snapshot_buf to 96x96 using Edge Impulse helper.
 *
 * After this, the first 96x96x3 bytes of snapshot_buf hold the resized image.
 */
static bool ei_camera_capture(uint32_t out_width, uint32_t out_height)
{
    if (!snapshot_buf) {
        ESP_LOGE(TAG, "snapshot_buf is NULL");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed (esp_camera_fb_get returned NULL)");
        return false;
    }

    ESP_LOGI(TAG, "Camera framebuffer:");
    ESP_LOGI(TAG, "  fb->buf   = %p", fb->buf);
    ESP_LOGI(TAG, "  fb->len   = %u", (unsigned)fb->len);
    ESP_LOGI(TAG, "  fb->width = %u", (unsigned)fb->width);
    ESP_LOGI(TAG, "  fb->height= %u", (unsigned)fb->height);
    ESP_LOGI(TAG, "  fb->format= %d", fb->format);

    bool converted = false;

    // Convert camera frame into snapshot_buf as RGB888 (320x240)
    if (fb->format == PIXFORMAT_JPEG) {
        ESP_LOGI(TAG, "Converting JPEG to RGB888...");
        converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    }
    else if (fb->format == PIXFORMAT_RGB565) {
        ESP_LOGI(TAG, "Converting RGB565 to RGB888...");
        converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, snapshot_buf);
    }
    else {
        ESP_LOGE(TAG, "Unsupported camera pixel format: %d", fb->format);
    }

    esp_camera_fb_return(fb);

    if (!converted) {
        ESP_LOGE(TAG, "fmt2rgb888 conversion failed");
        return false;
    }

    ESP_LOGI(TAG, "Conversion successful, checking heap...");
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "Heap corruption detected right after fmt2rgb888!");
        return false;
    }

    // Resize 320x240 -> 96x96 IN PLACE in snapshot_buf.
    // NOTE: snapshot_buf is large (320x240x3), so it's safe to use as both
    // input and output as done in official Edge Impulse examples.
    if (out_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
        out_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {

        ESP_LOGI(TAG, "Resizing from %dx%d to %dx%d",
                 (int)EI_CAMERA_RAW_FRAME_BUFFER_COLS,
                 (int)EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
                 (int)out_width, (int)out_height);

        size_t required = out_width * out_height * EI_CAMERA_FRAME_BYTE_SIZE;
        ESP_LOGI(TAG, "Required output buffer size: %u bytes", (unsigned)required);

        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_buf,                              // input buffer (320x240)
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            snapshot_buf,                              // output buffer (96x96) at same base
            out_width,
            out_height);

        ESP_LOGI(TAG, "Resize complete, checking heap...");
        if (!heap_caps_check_integrity_all(true)) {
            ESP_LOGE(TAG, "Heap corruption detected after resize!");
            return false;
        }
    }

    return true;
}

/**
 * EI SDK callback to read pixel data from snapshot_buf.
 *
 * After resize, the first 96x96x3 bytes of snapshot_buf hold the model input.
 * offset/length are in "pixels" (not bytes).
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    if (!snapshot_buf) {
        ESP_LOGE(TAG, "snapshot_buf is NULL in ei_camera_get_data");
        return -1;
    }

    size_t pixel_ix    = offset * 3;  // 3 bytes per pixel
    size_t out_ix      = 0;
    size_t pixels_left = length;

    while (pixels_left != 0) {
        // BGR->RGB fix: esp32-camera + fmt2rgb888 usually gives BGR.
        uint32_t r = snapshot_buf[pixel_ix + 2];
        uint32_t g = snapshot_buf[pixel_ix + 1];
        uint32_t b = snapshot_buf[pixel_ix + 0];

        uint32_t packed = (r << 16) | (g << 8) | b;

        out_ptr[out_ix++] = (float)packed;
        pixel_ix  += 3;
        pixels_left--;
    }

    return 0;
}

/**
 * Run Edge Impulse inference (called from gesture task)
 */
extern "C" const char* ei_infer_gesture(float *confidence_out)
{
    static char last_label[64] = {0};

    if (!ei_initialized || !snapshot_buf) {
        ESP_LOGE(TAG, "EI not initialized! Call ei_infer_init() first");
        if (confidence_out) *confidence_out = 0.0f;
        return nullptr;
    }

    ESP_LOGI(TAG, "Buffer addresses:");
    ESP_LOGI(TAG, "  snapshot_buf    = %p (size: %u)",
             snapshot_buf,
             (unsigned)(EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                        EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                        EI_CAMERA_FRAME_BYTE_SIZE));

    // Capture 320x240, convert to RGB888, THEN in-place resize to 96x96
    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH,
                           EI_CLASSIFIER_INPUT_HEIGHT)) {
        ESP_LOGE(TAG, "Camera capture/preprocess failed");
        if (confidence_out) *confidence_out = 0.0f;
        return nullptr;
    }

    // Sanity-check heap before NN
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "Heap corruption detected BEFORE run_classifier, aborting inference");
        if (confidence_out) *confidence_out = 0.0f;
        return nullptr;
    }

    ei_impulse_result_t result;
    memset(&result, 0, sizeof(result));

    bool verbose = !tflite_initialized; // NN debug only on first run

    EI_IMPULSE_ERROR err = run_classifier(&persistent_signal, &result, verbose);

    if (!tflite_initialized) {
        tflite_initialized = true;
        ESP_LOGI(TAG, "TFLite arena allocated on first successful inference");

        size_t psram_free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "Post-arena memory - PSRAM: %zu KB, Internal: %zu KB",
                 psram_free / 1024, internal_free / 1024);
    }

    if (err != EI_IMPULSE_OK) {
        ESP_LOGE(TAG, "run_classifier failed (%d)", err);
        ESP_LOGE(TAG, "  DSP time: %d ms", result.timing.dsp);
        ESP_LOGE(TAG, "  Classification time: %d ms", result.timing.classification);
        ESP_LOGE(TAG, "  Anomaly time: %d ms", result.timing.anomaly);
        if (confidence_out) *confidence_out = 0.0f;
        return nullptr;
    }

    // Find best & second-best predictions
    int   best_i    = -1;
    int   best_i2   = -1;
    float best_val  = -1.0f;
    float best_val2 = -1.0f;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float v = result.classification[i].value;

        if (v > best_val) {
            best_val2 = best_val;
            best_i2   = best_i;
            best_val  = v;
            best_i    = i;
        }
        else if (v > best_val2) {
            best_val2 = v;
            best_i2   = i;
        }
    }

    // Skip "no_gesture" if it wins
    if (best_i >= 0 &&
        strcmp(ei_classifier_inferencing_categories[best_i], "no_gesture") == 0) {
        best_i  = best_i2;
        best_val = best_val2;
    }

    // Threshold & output
    if (best_i >= 0 && best_val > 0.5f) {
        strncpy(last_label,
                ei_classifier_inferencing_categories[best_i],
                sizeof(last_label) - 1);
        last_label[sizeof(last_label) - 1] = '\0';

        if (confidence_out) {
            *confidence_out = best_val;
        }

        ESP_LOGI(TAG, "Gesture: %s (%.3f) [DSP:%d ms, Class:%d ms]",
                 last_label, best_val,
                 result.timing.dsp,
                 result.timing.classification);

        return last_label;
    }

    if (confidence_out) *confidence_out = 0.0f;
    return nullptr;
}
