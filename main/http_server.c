#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>

static const char *TAG = "HTTP_SERVER";
#define MEDIA_MOUNT_POINT "/media"

// HTML header/footer for consistent styling
static const char *html_header = 
    "<!DOCTYPE html>"
    "<html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>ESP32 Media Server</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; max-width: 800px; margin: 40px auto; padding: 20px; background: #f5f5f5; }"
    "h1 { color: #333; border-bottom: 3px solid #0066cc; padding-bottom: 10px; }"
    "h2 { color: #666; margin-top: 30px; }"
    ".storage-info { background: #e3f2fd; padding: 15px; border-radius: 8px; margin: 20px 0; }"
    ".storage-bar { background: #ddd; height: 30px; border-radius: 15px; overflow: hidden; margin: 10px 0; }"
    ".storage-fill { background: linear-gradient(90deg, #4CAF50, #45a049); height: 100%; text-align: center; line-height: 30px; color: white; font-weight: bold; }"
    ".nav-links { margin: 30px 0; }"
    ".nav-links a { display: inline-block; padding: 12px 24px; margin: 5px; background: #0066cc; color: white; text-decoration: none; border-radius: 5px; transition: 0.3s; }"
    ".nav-links a:hover { background: #0052a3; }"
    ".media-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }"
    ".media-item { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); text-align: center; }"
    ".media-item img { max-width: 100%; height: auto; border-radius: 5px; margin-bottom: 10px; }"
    ".media-item a { color: #0066cc; text-decoration: none; word-break: break-all; }"
    ".media-item a:hover { text-decoration: underline; }"
    ".media-item .size { color: #999; font-size: 0.9em; margin-top: 5px; }"
    ".empty { text-align: center; padding: 40px; color: #999; }"
    "</style>"
    "</head><body>";

static const char *html_footer = "</body></html>";

// Helper: Get storage info
static void get_storage_info(size_t *total, size_t *used, size_t *free, float *percent)
{
    esp_spiffs_info("media", total, used);
    *free = *total - *used;
    *percent = (*used * 100.0f) / *total;
}

// Helper: Format file size
static void format_size(size_t bytes, char *buf, size_t buf_len)
{
    if (bytes < 1024) {
        snprintf(buf, buf_len, "%zu B", bytes);
    } else if (bytes < 1024 * 1024) {
        snprintf(buf, buf_len, "%.1f KB", bytes / 1024.0f);
    } else {
        snprintf(buf, buf_len, "%.2f MB", bytes / (1024.0f * 1024.0f));
    }
}

// Landing page with storage info
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req, html_header);
    
    // Title
    httpd_resp_sendstr_chunk(req, "<h1>📷 ESP32 Media Server</h1>");
    
    // Storage info
    size_t total, used, free;
    float percent;
    get_storage_info(&total, &used, &free, &percent);
    
    char storage_html[512];
    char total_str[32], used_str[32], free_str[32];
    format_size(total, total_str, sizeof(total_str));
    format_size(used, used_str, sizeof(used_str));
    format_size(free, free_str, sizeof(free_str));
    
    snprintf(storage_html, sizeof(storage_html),
             "<div class='storage-info'>"
             "<h2>💾 Storage Status</h2>"
             "<div class='storage-bar'>"
             "<div class='storage-fill' style='width: %.1f%%;'>%.1f%% Used</div>"
             "</div>"
             "<p><strong>Total:</strong> %s | <strong>Used:</strong> %s | <strong>Free:</strong> %s</p>"
             "</div>",
             percent, percent, total_str, used_str, free_str);
    
    httpd_resp_sendstr_chunk(req, storage_html);
    
    // Count files
    int photo_count = 0, video_count = 0;
    DIR *dir = opendir(MEDIA_MOUNT_POINT);
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG) {
                if (strstr(entry->d_name, ".jpg") || strstr(entry->d_name, ".jpeg")) {
                    photo_count++;
                } else if (strstr(entry->d_name, ".avi") || strstr(entry->d_name, ".mjpg")) {  // Added .avi
                    video_count++;
                }
            }
        }
        closedir(dir);
    }
    
    char nav_html[512];
    snprintf(nav_html, sizeof(nav_html),
             "<h2>📂 Browse Media</h2>"
             "<div class='nav-links'>"
             "<a href='/images'>📸 Photos (%d)</a>"
             "<a href='/videos'>🎥 Videos (%d)</a>"
             "</div>"
             "<p style='color: #666; margin-top: 30px;'>"
             "<strong>Gesture Controls:</strong><br>"
             "• <strong>Forward</strong> - Take Photo<br>"
             "• <strong>Backward</strong> - Start/Stop Video Recording"
             "</p>",
             photo_count, video_count);
    
    httpd_resp_sendstr_chunk(req, nav_html);
    httpd_resp_sendstr_chunk(req, html_footer);
    httpd_resp_sendstr_chunk(req, NULL);
    
    return ESP_OK;
}

// Images gallery
static esp_err_t images_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req, html_header);
    
    httpd_resp_sendstr_chunk(req, "<h1>📸 Photos</h1>");
    httpd_resp_sendstr_chunk(req, "<div class='nav-links'><a href='/'>← Back to Home</a></div>");
    httpd_resp_sendstr_chunk(req, "<div class='media-grid'>");
    
    DIR *dir = opendir(MEDIA_MOUNT_POINT);
    int count = 0;
    
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG && 
                (strstr(entry->d_name, ".jpg") || strstr(entry->d_name, ".jpeg"))) {
                
                char filepath[512];
                snprintf(filepath, sizeof(filepath), MEDIA_MOUNT_POINT "/%s", entry->d_name);
                
                struct stat st;
                char size_str[32] = "Unknown";
                if (stat(filepath, &st) == 0) {
                    format_size(st.st_size, size_str, sizeof(size_str));
                }
                
                char item_html[2048];  // Increased from 1024 to 2048
                snprintf(item_html, sizeof(item_html),
                         "<div class='media-item'>"
                         "<img src='/download?file=%s' alt='%s' loading='lazy'>"
                         "<a href='/download?file=%s'>%s</a>"
                         "<div class='size'>%s</div>"
                         "</div>",
                         entry->d_name, entry->d_name,
                         entry->d_name, entry->d_name,
                         size_str);
                
                httpd_resp_sendstr_chunk(req, item_html);
                count++;
            }
        }
        closedir(dir);
    }
    
    if (count == 0) {
        httpd_resp_sendstr_chunk(req, "<div class='empty'>No photos found. Use FORWARD gesture to take a photo!</div>");
    }
    
    httpd_resp_sendstr_chunk(req, "</div>");
    httpd_resp_sendstr_chunk(req, html_footer);
    httpd_resp_sendstr_chunk(req, NULL);
    
    return ESP_OK;
}

// Videos gallery
static esp_err_t videos_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req, html_header);
    
    httpd_resp_sendstr_chunk(req, "<h1>🎥 Videos</h1>");
    httpd_resp_sendstr_chunk(req, "<div class='nav-links'><a href='/'>← Back to Home</a></div>");
    httpd_resp_sendstr_chunk(req, "<div class='media-grid'>");
    
    DIR *dir = opendir(MEDIA_MOUNT_POINT);
    int count = 0;
    
    if (dir) {
        struct dirent *entry;
         while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG && 
                (strstr(entry->d_name, ".avi") || strstr(entry->d_name, ".mjpg"))) {  // Added .avi

                
                char filepath[512];
                snprintf(filepath, sizeof(filepath), MEDIA_MOUNT_POINT "/%s", entry->d_name);
                
                struct stat st;
                char size_str[32] = "Unknown";
                if (stat(filepath, &st) == 0) {
                    format_size(st.st_size, size_str, sizeof(size_str));
                }
                
                char item_html[2048];  // Increased from 1024 to 2048
                snprintf(item_html, sizeof(item_html),
                         "<div class='media-item'>"
                         "🎬"
                         "<a href='/download?file=%s'>%s</a>"
                         "<div class='size'>%s</div>"
                         "</div>",
                         entry->d_name, entry->d_name,
                         size_str);
                
                httpd_resp_sendstr_chunk(req, item_html);
                count++;
            }
        }
        closedir(dir);
    }
    
    if (count == 0) {
        httpd_resp_sendstr_chunk(req, "<div class='empty'>No videos found. Use BACKWARD gesture to record a video!</div>");
    }
    
    httpd_resp_sendstr_chunk(req, "</div>");
    httpd_resp_sendstr_chunk(req, html_footer);
    httpd_resp_sendstr_chunk(req, NULL);
    
    return ESP_OK;
}

// Download/view file
static esp_err_t download_handler(httpd_req_t *req)
{
    char filename[256] = {0};
    
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0) {
        char *query = malloc(query_len + 1);
        httpd_req_get_url_query_str(req, query, query_len + 1);
        httpd_query_key_value(query, "file", filename, sizeof(filename));
        free(query);
    }
    
    if (strlen(filename) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    
    char filepath[512];
    snprintf(filepath, sizeof(filepath), MEDIA_MOUNT_POINT "/%s", filename);
    
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }
    
    // Set content type
    if (strstr(filename, ".jpg") || strstr(filename, ".jpeg")) {
        httpd_resp_set_type(req, "image/jpeg");
    } else if (strstr(filename, ".avi")) {
        httpd_resp_set_type(req, "video/x-msvideo");  // AVI MIME type
    } else if (strstr(filename, ".mjpg")) {
        httpd_resp_set_type(req, "video/x-motion-jpeg");
    } else {
        httpd_resp_set_type(req, "application/octet-stream");
    }

    char content_disp[320];
    snprintf(content_disp, sizeof(content_disp), "attachment; filename=\"%s\"", filename);
    httpd_resp_set_hdr(req, "Content-Disposition", content_disp);
    
    // Stream file
    char buf[1024];
    size_t read;
    while ((read = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, read) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }
    
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
}

// ...existing code at the end of file...

httpd_handle_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;  // Increase from default 4096 to 8192
    config.task_priority = 5;  // Lower priority to avoid mutex conflicts
    config.core_id = 1;        // Pin to Core 1
    
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Index page
        httpd_uri_t index = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_handler
        };
        httpd_register_uri_handler(server, &index);
        
        // Images gallery
        httpd_uri_t images = {
            .uri = "/images",
            .method = HTTP_GET,
            .handler = images_handler
        };
        httpd_register_uri_handler(server, &images);
        
        // Videos gallery
        httpd_uri_t videos = {
            .uri = "/videos",
            .method = HTTP_GET,
            .handler = videos_handler
        };
        httpd_register_uri_handler(server, &videos);
        
        // Download/view file
        httpd_uri_t download = {
            .uri = "/download",
            .method = HTTP_GET,
            .handler = download_handler
        };
        httpd_register_uri_handler(server, &download);
        
        ESP_LOGI(TAG, "HTTP server started on core %d with priority %d", 
                 config.core_id, config.task_priority);
    }
    
    return server;
}
