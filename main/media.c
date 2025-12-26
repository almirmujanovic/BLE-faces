#include "include/media.h"
#include "include/camera_config.h"

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "esp_camera.h"
#include "img_converters.h"   // for frame2jpg / frame2jpg_cb
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
#define JPEG_QUALITY      75      // 0-100, higher = better (and bigger)
#define VIDEO_JPEG_QUALITY 70      // better quality; keep fps target conservative
// -----------------------------------------------------------------------------
// Simple camera-busy flag so other tasks can back off if you want
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

static bool encode_to_jpeg_quality(camera_fb_t *fb, uint8_t quality, uint8_t **out_jpeg, size_t *out_len)
{
    jpeg_encode_ctx_t ctx = {0};

    bool ok = frame2jpg_cb(fb, quality, jpeg_encode_output, &ctx);
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

static bool encode_to_jpeg(camera_fb_t *fb, uint8_t **out_jpeg, size_t *out_len)
{
    return encode_to_jpeg_quality(fb, JPEG_QUALITY, out_jpeg, out_len);
}

static void swap_rgb565_bytes(const uint8_t *src, uint8_t *dst, size_t len)
{
    for (size_t i = 0; i + 1 < len; i += 2) {
        dst[i] = src[i + 1];
        dst[i + 1] = src[i];
    }
}

typedef struct {
    uint32_t offset;
    uint32_t size;
} avi_index_entry_t;

typedef struct {
    FILE *f;
    long movi_start;
    size_t frames;
    size_t max_frame_size;
    size_t index_capacity;
    avi_index_entry_t *index;
} avi_writer_ctx_t;

static void write_u16(FILE *fp, uint16_t v)
{
    uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
    fwrite(b, 1, 2, fp);
}

static void write_u32(FILE *fp, uint32_t v)
{
    uint8_t b[4] = {
        (uint8_t)(v & 0xFF),
        (uint8_t)((v >> 8) & 0xFF),
        (uint8_t)((v >> 16) & 0xFF),
        (uint8_t)((v >> 24) & 0xFF)
    };
    fwrite(b, 1, 4, fp);
}

static void avi_write_frame_chunk(avi_writer_ctx_t *ctx, const uint8_t *jpeg, size_t jpeg_len)
{
    uint32_t chunk_offset = (uint32_t)(ftell(ctx->f) - ctx->movi_start);
    fwrite("00dc", 1, 4, ctx->f);
    write_u32(ctx->f, (uint32_t)jpeg_len);
    fwrite(jpeg, 1, jpeg_len, ctx->f);
    if (jpeg_len & 1) {
        fputc(0, ctx->f);
    }
    if (ctx->frames < ctx->index_capacity) {
        ctx->index[ctx->frames].offset = chunk_offset;
        ctx->index[ctx->frames].size = (uint32_t)jpeg_len;
    }
    if (jpeg_len > ctx->max_frame_size) {
        ctx->max_frame_size = jpeg_len;
    }
    ctx->frames++;
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

esp_err_t media_delete_all(void)
{
    DIR *dir = opendir(MEDIA_MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open '%s' directory", MEDIA_MOUNT_POINT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleting all media files...");
    struct dirent *entry;
    int deleted = 0;
    int failed = 0;

    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        char path[512];
        snprintf(path, sizeof(path), MEDIA_MOUNT_POINT "/%s", entry->d_name);

        struct stat st;
        if (stat(path, &st) == 0 && S_ISREG(st.st_mode)) {
            if (unlink(path) == 0) {
                deleted++;
            } else {
                failed++;
                ESP_LOGW(TAG, "Failed to delete %s", path);
            }
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Deleted %d files, failed %d", deleted, failed);
    return (failed == 0) ? ESP_OK : ESP_FAIL;
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

static void generate_burst_prefix(char *buf, size_t len)
{
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);

    snprintf(buf, len, MEDIA_MOUNT_POINT "/video_%04d%02d%02d_%02d%02d%02d.avi",
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
    camera_fb_t rgb_fb = {0};

    // Turn on capture LED while we prepare and write the file
    leds_set_capture(true);

    camera_fb_t *encode_fb = fb;
    uint8_t *rgb565_le = NULL;
    if (encode_fb->format == PIXFORMAT_RGB565 && CAMERA_RGB565_BIG_ENDIAN) {
        rgb565_le = (uint8_t *)heap_caps_malloc(encode_fb->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!rgb565_le) {
            ESP_LOGE(TAG, "Failed to allocate RGB swap buffer (%u bytes)", (unsigned)encode_fb->len);
            leds_set_capture(false);
            return ESP_ERR_NO_MEM;
        }
        swap_rgb565_bytes(encode_fb->buf, rgb565_le, encode_fb->len);
        rgb_fb = *encode_fb;
        rgb_fb.buf = rgb565_le;
        encode_fb = &rgb_fb;
    }

    if (encode_fb->format == PIXFORMAT_JPEG) {
        // Already JPEG from sensor
        jpeg_buf = encode_fb->buf;
        jpeg_len = encode_fb->len;
    } else {
        // Convert RGB565/YUV/etc to JPEG in software
        if (!encode_to_jpeg(encode_fb, &jpeg_buf, &jpeg_len) || !jpeg_buf || jpeg_len == 0) {
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
        if (encode_fb->format != PIXFORMAT_JPEG && jpeg_buf) {
            free(jpeg_buf);
        }
        if (rgb565_le) {
            heap_caps_free(rgb565_le);
        }
        return ESP_FAIL;
    }

    size_t written = fwrite(jpeg_buf, 1, jpeg_len, f);
    fclose(f);

    if (encode_fb->format != PIXFORMAT_JPEG && jpeg_buf) {
        free(jpeg_buf);
    }
    if (rgb565_le) {
        heap_caps_free(rgb565_le);
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

esp_err_t media_capture_jpeg_burst(uint32_t duration_ms,
                                   uint32_t target_fps,
                                   size_t max_frames)
{
    if (target_fps == 0) {
        target_fps = 10;
    }
    const uint32_t period_ms = 1000 / target_fps;

    char filename[128];
    generate_burst_prefix(filename, sizeof(filename));

    FILE *f = fopen(filename, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open '%s' for video", filename);
        return ESP_FAIL;
    }

    media_set_camera_busy(true);
    camera_fb_t *first_fb = esp_camera_fb_get();
    if (!first_fb || !first_fb->buf || first_fb->len == 0) {
        if (first_fb) {
            esp_camera_fb_return(first_fb);
        }
        fclose(f);
        media_set_camera_busy(false);
        ESP_LOGE(TAG, "Video capture failed: no frame buffer");
        return ESP_FAIL;
    }

    uint8_t *first_jpeg = NULL;
    size_t first_jpeg_len = 0;
    if (first_fb->format == PIXFORMAT_JPEG) {
        first_jpeg = first_fb->buf;
        first_jpeg_len = first_fb->len;
    } else {
        if (!encode_to_jpeg_quality(first_fb, VIDEO_JPEG_QUALITY, &first_jpeg, &first_jpeg_len)) {
            esp_camera_fb_return(first_fb);
            fclose(f);
            media_set_camera_busy(false);
            return ESP_FAIL;
        }
    }

    const uint16_t width = first_fb->width;
    const uint16_t height = first_fb->height;
    const int64_t start_us = esp_timer_get_time();

    const long riff_size_pos = ftell(f) + 4;
    fwrite("RIFF", 1, 4, f);
    write_u32(f, 0);
    fwrite("AVI ", 1, 4, f);

    fwrite("LIST", 1, 4, f);
    const long hdrl_size_pos = ftell(f);
    write_u32(f, 0);
    fwrite("hdrl", 1, 4, f);

    fwrite("avih", 1, 4, f);
    write_u32(f, 56);
    const long avih_pos = ftell(f);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0x10);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 1);
    write_u32(f, 0);
    write_u32(f, width);
    write_u32(f, height);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);

    fwrite("LIST", 1, 4, f);
    const long strl_size_pos = ftell(f);
    write_u32(f, 0);
    fwrite("strl", 1, 4, f);

    fwrite("strh", 1, 4, f);
    write_u32(f, 56);
    fwrite("vids", 1, 4, f);
    fwrite("MJPG", 1, 4, f);
    write_u32(f, 0);
    write_u16(f, 0);
    write_u16(f, 0);
    write_u32(f, 0);
    const long strh_rate_pos = ftell(f);
    write_u32(f, 1);
    write_u32(f, target_fps);
    write_u32(f, 0);
    const long strh_len_pos = ftell(f);
    write_u32(f, 0);
    const long strh_suggest_pos = ftell(f);
    write_u32(f, 0);
    write_u32(f, 0xFFFFFFFF);
    write_u32(f, 0);
    write_u16(f, 0);
    write_u16(f, 0);
    write_u16(f, width);
    write_u16(f, height);

    fwrite("strf", 1, 4, f);
    write_u32(f, 40);
    write_u32(f, 40);
    write_u32(f, width);
    write_u32(f, height);
    write_u16(f, 1);
    write_u16(f, 24);
    fwrite("MJPG", 1, 4, f);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);

    const long strl_end = ftell(f);
    const long hdrl_end = strl_end;
    fseek(f, hdrl_size_pos, SEEK_SET);
    write_u32(f, (uint32_t)(hdrl_end - (hdrl_size_pos + 4)));
    fseek(f, strl_size_pos, SEEK_SET);
    write_u32(f, (uint32_t)(strl_end - (strl_size_pos + 4)));
    fseek(f, hdrl_end, SEEK_SET);

    fwrite("LIST", 1, 4, f);
    const long movi_size_pos = ftell(f);
    write_u32(f, 0);
    fwrite("movi", 1, 4, f);
    const long movi_start = ftell(f);

    avi_index_entry_t *index = NULL;
    size_t index_capacity = (max_frames > 0) ? max_frames : (duration_ms * target_fps / 1000 + 10);
    if (index_capacity < 32) {
        index_capacity = 32;
    }
    index = (avi_index_entry_t *)heap_caps_malloc(sizeof(avi_index_entry_t) * index_capacity,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!index) {
        if (first_fb->format != PIXFORMAT_JPEG && first_jpeg) {
            free(first_jpeg);
        }
        esp_camera_fb_return(first_fb);
        fclose(f);
        media_set_camera_busy(false);
        ESP_LOGE(TAG, "Failed to allocate AVI index");
        return ESP_FAIL;
    }

    avi_writer_ctx_t avi_ctx = {
        .f = f,
        .movi_start = movi_start,
        .frames = 0,
        .max_frame_size = 0,
        .index_capacity = index_capacity,
        .index = index
    };

    avi_write_frame_chunk(&avi_ctx, first_jpeg, first_jpeg_len);
    if (first_fb->format != PIXFORMAT_JPEG && first_jpeg) {
        free(first_jpeg);
    }
    esp_camera_fb_return(first_fb);

    while (1) {
        const int64_t now_us = esp_timer_get_time();
        const int64_t elapsed_ms = (now_us - start_us) / 1000;
        if (elapsed_ms >= (int64_t)duration_ms) {
            break;
        }
        if (max_frames > 0 && avi_ctx.frames >= max_frames) {
            break;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb || !fb->buf || fb->len == 0) {
            if (fb) {
                esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        uint8_t *jpeg_buf = NULL;
        size_t jpeg_len = 0;
        bool need_free = false;

        if (fb->format == PIXFORMAT_JPEG) {
            jpeg_buf = fb->buf;
            jpeg_len = fb->len;
        } else {
            if (!encode_to_jpeg_quality(fb, VIDEO_JPEG_QUALITY, &jpeg_buf, &jpeg_len)) {
                esp_camera_fb_return(fb);
                continue;
            }
            need_free = true;
        }

        if (avi_ctx.frames >= avi_ctx.index_capacity) {
            size_t new_capacity = index_capacity * 2;
            avi_index_entry_t *new_index = (avi_index_entry_t *)heap_caps_realloc(
                index, sizeof(avi_index_entry_t) * new_capacity,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (new_index) {
                index = new_index;
                index_capacity = new_capacity;
                avi_ctx.index = index;
                avi_ctx.index_capacity = index_capacity;
            } else {
                if (need_free && jpeg_buf) {
                    free(jpeg_buf);
                }
                esp_camera_fb_return(fb);
                break;
            }
        }

        avi_write_frame_chunk(&avi_ctx, jpeg_buf, jpeg_len);

        if (need_free && jpeg_buf) {
            free(jpeg_buf);
        }
        esp_camera_fb_return(fb);

        const int64_t target_ms = (int64_t)avi_ctx.frames * period_ms;
        const int64_t sleep_ms = target_ms - ((esp_timer_get_time() - start_us) / 1000);
        if (sleep_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)sleep_ms));
        }
    }

    const long movi_end = ftell(f);
    fseek(f, movi_size_pos, SEEK_SET);
    write_u32(f, (uint32_t)(movi_end - (movi_size_pos + 4)));
    fseek(f, movi_end, SEEK_SET);

    fwrite("idx1", 1, 4, f);
    write_u32(f, (uint32_t)(avi_ctx.frames * 16));
    for (size_t i = 0; i < avi_ctx.frames; ++i) {
        fwrite("00dc", 1, 4, f);
        write_u32(f, 0x10);
        write_u32(f, index[i].offset);
        write_u32(f, index[i].size);
    }

    const long file_end = ftell(f);
    const int64_t total_us = esp_timer_get_time() - start_us;
    uint32_t usec_per_frame = (avi_ctx.frames > 0) ? (uint32_t)(total_us / avi_ctx.frames) : 100000;
    if (usec_per_frame == 0) {
        usec_per_frame = 100000;
    }
    uint32_t actual_fps = (uint32_t)(1000000 / usec_per_frame);
    if (actual_fps == 0) {
        actual_fps = 1;
    }

    fseek(f, riff_size_pos, SEEK_SET);
    write_u32(f, (uint32_t)(file_end - 8));
    fseek(f, avih_pos, SEEK_SET);
    write_u32(f, usec_per_frame);
    fseek(f, avih_pos + 16, SEEK_SET);
    write_u32(f, (uint32_t)avi_ctx.frames);
    fseek(f, avih_pos + 32, SEEK_SET);
    write_u32(f, (uint32_t)avi_ctx.max_frame_size);
    fseek(f, strh_rate_pos, SEEK_SET);
    write_u32(f, 1);
    write_u32(f, actual_fps);
    fseek(f, strh_len_pos, SEEK_SET);
    write_u32(f, (uint32_t)avi_ctx.frames);
    fseek(f, strh_suggest_pos, SEEK_SET);
    write_u32(f, (uint32_t)avi_ctx.max_frame_size);
    fseek(f, file_end, SEEK_SET);

    fclose(f);
    media_set_camera_busy(false);
    if (index) {
        heap_caps_free(index);
    }

    ESP_LOGI(TAG, "Saved video: %s (%u frames, %u fps)", filename, (unsigned)avi_ctx.frames, (unsigned)actual_fps);
    return ESP_OK;
}




