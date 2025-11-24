#include "include/media.h"
#include "include/camera_config.h"

#include "esp_log.h"
#include "esp_spiffs.h"

#include "esp_camera.h"
#include "img_converters.h"   // for frame2jpg / frame2jpg_cb

#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "include/leds.h"


static const char *TAG = "MEDIA_STORAGE";

#define MEDIA_MOUNT_POINT "/media"
#define JPEG_QUALITY      75      // 0–100, higher = better (and bigger)

// -----------------------------------------------------------------------------
// Simple “camera busy” flag so other tasks can back off if you want
// (you can keep using this from face_detection, or ignore it)
// -----------------------------------------------------------------------------
static bool s_camera_busy = false;

bool media_is_camera_busy(void)
{
    return s_camera_busy;
}

void media_set_camera_busy(bool busy)
{
    s_camera_busy = busy;
}

// -----------------------------------------------------------------------------
// JPEG encoder helper (for non-JPEG formats, e.g. RGB565 from face camera)
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t *buffer;
    size_t   len;
    size_t   capacity;
} jpeg_encode_ctx_t;

static size_t jpeg_encode_output(void *arg, size_t index, const void *data, size_t len)
{
    (void)index;
    jpeg_encode_ctx_t *ctx = (jpeg_encode_ctx_t *)arg;

    if (!ctx->buffer) {
        ctx->capacity = 100 * 1024; // start with 100KB
        ctx->buffer   = (uint8_t *)malloc(ctx->capacity);
        if (!ctx->buffer) {
            return 0;
        }
        ctx->len = 0;
    }

    if (ctx->len + len > ctx->capacity) {
        size_t new_capacity = ctx->capacity * 2;
        uint8_t *new_buffer = (uint8_t *)realloc(ctx->buffer, new_capacity);
        if (!new_buffer) {
            return 0;
        }
        ctx->buffer   = new_buffer;
        ctx->capacity = new_capacity;
    }

    memcpy(ctx->buffer + ctx->len, data, len);
    ctx->len += len;
    return len;
}

static bool encode_to_jpeg(camera_fb_t *fb, uint8_t **out_jpeg, size_t *out_len)
{
    jpeg_encode_ctx_t ctx = {0};

    bool ok = frame2jpg_cb(fb, JPEG_QUALITY, jpeg_encode_output, &ctx);
    if (!ok || !ctx.buffer || ctx.len == 0) {
        ESP_LOGE(TAG, "JPEG encoding failed");
        if (ctx.buffer) {
            free(ctx.buffer);
        }
        return false;
    }

    *out_jpeg = ctx.buffer;
    *out_len  = ctx.len;
    return true;
}

// -----------------------------------------------------------------------------
// SPIFFS initialization / utilities
// -----------------------------------------------------------------------------
esp_err_t media_storage_init(void)
{
    ESP_LOGI(TAG, "Initializing media storage...");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = MEDIA_MOUNT_POINT,
        .partition_label = "media",
        .max_files = 10,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS partition 'media' not found");
        } else {
            ESP_LOGE(TAG, "Failed to init SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("media", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted successfully");
        ESP_LOGI(TAG, "  Total: %zu KB, Used: %zu KB, Free: %zu KB",
                 total / 1024, used / 1024, (total - used) / 1024);
    } else {
        ESP_LOGW(TAG, "esp_spiffs_info failed: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

esp_err_t media_get_storage_info(size_t *total_bytes, size_t *used_bytes, size_t *free_bytes)
{
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("media", &total, &used);
    if (ret != ESP_OK) {
        return ret;
    }

    if (total_bytes) *total_bytes = total;
    if (used_bytes)  *used_bytes  = used;
    if (free_bytes)  *free_bytes  = total - used;
    return ESP_OK;
}

esp_err_t media_list_files(void)
{
    DIR *dir = opendir(MEDIA_MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open '%s' directory", MEDIA_MOUNT_POINT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "=== Media Files ===");
    struct dirent *entry;
    int count = 0;
    size_t total_size = 0;

    while ((entry = readdir(dir)) != NULL) {
        char path[512];
        snprintf(path, sizeof(path), MEDIA_MOUNT_POINT "/%s", entry->d_name);

        struct stat st;
        if (stat(path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                ESP_LOGI(TAG, "  [DIR] %s", entry->d_name);
            } else if (S_ISREG(st.st_mode)) {
                ESP_LOGI(TAG, "  %s - %ld bytes", entry->d_name, st.st_size);
                total_size += st.st_size;
                count++;
            }
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Total regular files: %d, %zu KB", count, total_size / 1024);
    return ESP_OK;
}

esp_err_t media_delete_oldest(void)
{
    DIR *dir = opendir(MEDIA_MOUNT_POINT);
    if (!dir) {
        return ESP_FAIL;
    }

    char  oldest_path[256] = {0};
    time_t oldest_time     = time(NULL);

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        char path[256];
        int n = snprintf(path, sizeof(path), MEDIA_MOUNT_POINT "/%s", entry->d_name);
        if (n < 0 || n >= (int)sizeof(path)) {
            continue;
        }

        struct stat st;
        if (stat(path, &st) == 0 && S_ISREG(st.st_mode)) {
            if (st.st_mtime < oldest_time) {
                oldest_time = st.st_mtime;
                strncpy(oldest_path, path, sizeof(oldest_path) - 1);
                oldest_path[sizeof(oldest_path) - 1] = '\0';
            }
        }
    }

    closedir(dir);

    if (oldest_path[0]) {
        ESP_LOGI(TAG, "Deleting oldest file: %s", oldest_path);
        if (unlink(oldest_path) == 0) {
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

// -----------------------------------------------------------------------------
// Filename helper (single photo)
// -----------------------------------------------------------------------------
static void generate_photo_filename(char *buf, size_t len)
{
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);

    snprintf(buf, len, MEDIA_MOUNT_POINT "/photo_%04d%02d%02d_%02d%02d%02d.jpg",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
}

// -----------------------------------------------------------------------------
// Save one framebuffer as JPEG to a specific path
//  - If fb is already JPEG (PIXFORMAT_JPEG), write it directly.
//  - Otherwise, compress with software encoder (RGB565/YUV).
// -----------------------------------------------------------------------------
// ...existing code...


static esp_err_t save_fb_as_jpeg_to_path(camera_fb_t *fb, const char *path)
{
    if (!fb || !fb->buf || fb->len == 0) {
        ESP_LOGE(TAG, "save_fb_as_jpeg_to_path: invalid framebuffer");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Saving JPEG: %s (fmt=%d, len=%zu, %dx%d)",
             path, fb->format, fb->len, fb->width, fb->height);

    uint8_t *jpeg_buf = NULL;
    size_t   jpeg_len = 0;

    // Turn on capture LED while we prepare and write the file
    leds_set_capture(true);

    if (fb->format == PIXFORMAT_JPEG) {
        // Already JPEG from sensor
        jpeg_buf = fb->buf;
        jpeg_len = fb->len;
    } else {
        // Convert RGB565/YUV/etc to JPEG in software
        if (!encode_to_jpeg(fb, &jpeg_buf, &jpeg_len) || !jpeg_buf || jpeg_len == 0) {
            ESP_LOGE(TAG, "Failed to encode frame to JPEG");
            leds_set_capture(false);
            if (jpeg_buf) free(jpeg_buf);
            return ESP_FAIL;
        }
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open '%s' for writing", path);
        leds_set_capture(false);
        if (fb->format != PIXFORMAT_JPEG && jpeg_buf) {
            free(jpeg_buf);
        }
        return ESP_FAIL;
    }

    size_t written = fwrite(jpeg_buf, 1, jpeg_len, f);
    fclose(f);

    if (fb->format != PIXFORMAT_JPEG && jpeg_buf) {
        free(jpeg_buf);
    }

    // Turn off capture LED after done
    leds_set_capture(false);

    if (written != jpeg_len) {
        ESP_LOGE(TAG, "Short write when saving JPEG (%zu / %zu bytes)", written, jpeg_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✅ Saved JPEG: %s (%zu bytes)", path, jpeg_len);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Public: save a single photo with auto filename (from current camera mode)
//  - In your current RAW/RGB/YUV mode this uses software JPEG.
//  - If you ever switch to PIXFORMAT_JPEG for stills, it will write sensor JPEG.
// -----------------------------------------------------------------------------
esp_err_t media_save_photo(camera_fb_t *fb)
{
    char filename[128];
    generate_photo_filename(filename, sizeof(filename));
    return save_fb_as_jpeg_to_path(fb, filename);
}
