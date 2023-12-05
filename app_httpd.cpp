// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include "Arduino.h"
#include "icon_data.h"
extern bool isWebConnected; 

extern float nominal_pump_on_time;
extern float last_on_time;
extern float avg_period;
extern float since_last;
extern bool  pump_on_z;
extern bool  system_ok;
extern time_t system_restart_time;
 
extern bool get_init_chart_data(char * response);
extern bool get_new_chart_data(char * response);
extern void reset_pump_history();
extern void turn_flash_on(bool isOn);
extern void send_teletram_msg(char * msg);

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "camera_httpd";
#endif

// Face Detection will not work on boards without (or with disabled) PSRAM 
#ifdef BOARD_HAS_PSRAM
#define CONFIG_ESP_FACE_DETECT_ENABLED 0
// Face Recognition takes upward from 15 seconds per frame on chips other than ESP32S3
// Makes no sense to have it enabled for them
#if CONFIG_IDF_TARGET_ESP32S3
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 1
#else
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
#endif
#else
#define CONFIG_ESP_FACE_DETECT_ENABLED 0
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
#endif


typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;



typedef struct
{
    size_t size;  //number of values used for filtering
    size_t index; //current value index
    size_t count; //value count
    int sum;
    int *values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values)
    {
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values)
    {
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size)
    {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static esp_err_t bmp_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    uint64_t fr_start = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/x-windows-bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);


    uint8_t * buf = NULL;
    size_t buf_len = 0;
    bool converted = frame2bmp(fb, &buf, &buf_len);
    esp_camera_fb_return(fb);
    if(!converted){
        ESP_LOGE(TAG, "BMP Conversion failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_send(req, (const char *)buf, buf_len);
    free(buf);
    uint64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
    return res;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    isWebConnected = true;

    fb = esp_camera_fb_get();


    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);


        size_t fb_len = 0;
        if (fb->format == PIXFORMAT_JPEG)
        {
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        }
        else
        {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        ESP_LOGI(TAG, "JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
        return res;

}

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128];

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");



    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            _timestamp.tv_sec = fb->timestamp.tv_sec;
            _timestamp.tv_usec = fb->timestamp.tv_usec;
                if (fb->format != PIXFORMAT_JPEG)
                {
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if (!jpeg_converted)
                    {
                        ESP_LOGE(TAG, "JPEG compression failed");
                        res = ESP_FAIL;
                    }
                }
                else
                {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }

        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "send frame failed failed");
            break;
        }
        int64_t fr_end = esp_timer_get_time();


        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        ESP_LOGI(TAG, "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)"
                 ,
                 (uint32_t)(_jpg_buf_len),
                 (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
                 avg_frame_time, 1000.0 / avg_frame_time
        );
    }


    last_frame = 0;
    return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf)
{
    char *buf = NULL;
    size_t buf_len = 0;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            *obuf = buf;
            return ESP_OK;
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t index_handler(httpd_req_t *req)
{

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);

    } else {
        ESP_LOGE(TAG, "Camera sensor not found");
        return httpd_resp_send_500(req);
    }
}


static esp_err_t favicon_handler(httpd_req_t *req)
{
    //send_teletram_msg("favicon request");
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, (const char *)icon_data, sizeof(icon_data));

}


static esp_err_t pump_hist_data_handler(httpd_req_t *req)
{

    char json_response[1024];
    
    if(!get_init_chart_data(json_response)) return ESP_ERR_INVALID_STATE;
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t pump_new_data_handler(httpd_req_t *req)
{

    char json_response[150];
    
    if(!get_new_chart_data(json_response)) return ESP_ERR_INVALID_STATE;
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

extern float avg_period;
extern float since_last;
extern bool  pump_on;
extern bool  system_ok;

static esp_err_t pump_stat_handler(httpd_req_t *req)
{
     static char json_response[1024];
     char restart_time[30];
    // See http://www.cplusplus.com/reference/ctime/strftime/ for strftime functions
    strftime(restart_time, 30, "%D %T", localtime(&system_restart_time)); 
 
     sprintf(json_response,"{\"responseText\": [%.1f, %.1f, %.1f, %i, %i, \"%s\"]}",nominal_pump_on_time,avg_period, since_last, pump_on_z, system_ok,restart_time);
     httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}
static esp_err_t pump_reset_handler(httpd_req_t *req)
{
    reset_pump_history();
    return httpd_resp_send(req, NULL, 0);
}
static esp_err_t flash_on_handler(httpd_req_t *req)
{
    turn_flash_on(true);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t flash_off_handler(httpd_req_t *req)
{
    turn_flash_on(false);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t restart_handler(httpd_req_t *req)
{

    httpd_resp_send(req, NULL, 0);
    delay(1000);
    ESP.restart();
    return ESP_OK;
}

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL};

    httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_handler,
        .user_ctx = NULL};


    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL};

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};

    httpd_uri_t hist_uri = {
        .uri = "/get_pump_history_data",
        .method = HTTP_GET,
        .handler = pump_hist_data_handler,
        .user_ctx = NULL};

    httpd_uri_t pump_new_data_uri = {
        .uri = "/get_new_pump_data",
        .method = HTTP_GET,
        .handler = pump_new_data_handler,
        .user_ctx = NULL};
        
    httpd_uri_t pump_stat_uri = {
        .uri = "/get_pump_stat",
        .method = HTTP_GET,
        .handler = pump_stat_handler,
        .user_ctx = NULL};

    httpd_uri_t pump_reset_uri = {
        .uri = "/reset_baseline",
        .method = HTTP_GET,
        .handler = pump_reset_handler,
        .user_ctx = NULL};

    httpd_uri_t flash_on_uri = {
        .uri = "/flash_on",
        .method = HTTP_GET,
        .handler = flash_on_handler,
        .user_ctx = NULL};
        
    httpd_uri_t flash_off_uri = {
        .uri = "/flash_off",
        .method = HTTP_GET,
        .handler = flash_off_handler,
        .user_ctx = NULL};

    httpd_uri_t restart_uri = {
        .uri = "/restart",
        .method = HTTP_GET,
        .handler = restart_handler,
        .user_ctx = NULL};


    ra_filter_init(&ra_filter, 20);

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");

    // load ids from flash partition
    recognizer.set_ids_from_flash();
#endif
    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &hist_uri);
        httpd_register_uri_handler(camera_httpd, &pump_new_data_uri);
        httpd_register_uri_handler(camera_httpd, &pump_stat_uri);
        httpd_register_uri_handler(camera_httpd, &pump_reset_uri);
        httpd_register_uri_handler(camera_httpd, &flash_on_uri);
        httpd_register_uri_handler(camera_httpd, &flash_off_uri);        
        httpd_register_uri_handler(camera_httpd, &favicon_uri);        
        httpd_register_uri_handler(camera_httpd, &restart_uri);
    }


    config.server_port += 1;
    config.ctrl_port += 1;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
