/* ESP32-S3 USB UVC camera line following with ultrasonic obstacle avoidance. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "tjpgd.h"

#define CAM_W 640
#define CAM_H 480
#define CAM_FPS 25
#define IMG_W 320
#define IMG_H 240

/* Vision tuning copied from the tested esp32_car_vision approach. */
#define BLACK_THRESHOLD 110
#define ROI_TOP 168
#define NEAR_TOP 222
#define MID_TOP 198
#define MIN_COMPONENT_AREA 250
#define CONFIRM_FRAMES 3
#define BLIND_HOLD_FRAMES 6

/* Motor output is enabled for the live camera line-following test. */
#define MOTOR_OUTPUT_ENABLED 1
#define BASE_PWM 45
#define HOLD_PWM 24
#define MAX_CORRECTION 18

/* Existing Arduino wiring. GPIO19/20 remain reserved for native USB camera. */
#define STBY_PIN 10
#define PWMA_PIN 11
#define AIN1_PIN 12
#define AIN2_PIN 13
#define PWMB_PIN 14
#define BIN1_PIN 15
#define BIN2_PIN 16
#define PWMD_PIN 17
#define DIN1_PIN 18
#define DIN2_PIN 21
#define TRIG_PIN 7
#define ECHO_PIN 6

#define OBSTACLE_THRESHOLD_CM 15.0f
#define MAX_DISTANCE_CM 400.0f
#define ULTRASONIC_TIMEOUT_US 30000
#define ULTRASONIC_INTERVAL_MS 50
#define OBSTACLE_TURN_MS 1200
#define OBSTACLE_CHECK_MS 600
#define TURN_PWM 35

#define FRAME_BUFFERS 3
#define FRAME_SIZE (CAM_W * CAM_H * 2)
#define USB_PRIORITY 15

#define WIFI_AP_SSID "ESP32-CarVision"
#define WIFI_AP_PASSWORD "linecar123"
#define HTTP_STREAM_COPY_SIZE FRAME_SIZE

static const char *TAG = "car_vision";
static QueueHandle_t s_frame_q;
static uvc_host_stream_hdl_t s_stream;
static volatile bool s_connected;
static volatile bool s_vision_task_running;
static volatile float s_distance_cm = MAX_DISTANCE_CM;
static volatile bool s_distance_valid;
static uint8_t *s_gray;
static uint8_t *s_mask;
static uint32_t *s_flood_queue;
static uint8_t *s_jpeg_buffers[2];
static size_t s_jpeg_sizes[2];
static int s_jpeg_index;
static uint32_t s_jpeg_version;
static SemaphoreHandle_t s_jpeg_lock;
static SemaphoreHandle_t s_status_lock;
static httpd_handle_t s_http_server;

typedef struct { const uint8_t *data; size_t size, pos; } jpeg_src_t;
typedef struct { bool found; int area, near_x, mid_x, far_x, error; } line_result_t;
typedef enum { OBSTACLE_NONE, OBSTACLE_TURN, OBSTACLE_CHECK } obstacle_state_t;
typedef struct {
    bool decoded;
    bool found;
    int area;
    int near_x;
    int mid_x;
    int far_x;
    int error;
    float distance_cm;
    char state[16];
} vision_status_t;

static obstacle_state_t s_obstacle_state;
static uint32_t s_obstacle_started_ms;
static int s_turn_direction;
static vision_status_t s_vision_status = {.near_x = -1, .mid_x = -1, .far_x = -1};

static void publish_jpeg(const uvc_host_frame_t *frame)
{
    if (!frame || !frame->data || frame->data_len < 4 || frame->data_len > FRAME_SIZE ||
        frame->data[0] != 0xff || frame->data[1] != 0xd8 ||
        frame->data[frame->data_len - 2] != 0xff || frame->data[frame->data_len - 1] != 0xd9) {
        return;
    }

    if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(10)) != pdPASS) return;
    int next = s_jpeg_index ^ 1;
    memcpy(s_jpeg_buffers[next], frame->data, frame->data_len);
    s_jpeg_sizes[next] = frame->data_len;
    s_jpeg_index = next;
    s_jpeg_version++;
    xSemaphoreGive(s_jpeg_lock);
}

static void update_vision_status(bool decoded, const line_result_t *line, const char *state)
{
    if (xSemaphoreTake(s_status_lock, pdMS_TO_TICKS(10)) != pdPASS) return;
    s_vision_status.decoded = decoded;
    s_vision_status.found = line && line->found;
    s_vision_status.area = line ? line->area : 0;
    s_vision_status.near_x = line ? line->near_x : -1;
    s_vision_status.mid_x = line ? line->mid_x : -1;
    s_vision_status.far_x = line ? line->far_x : -1;
    s_vision_status.error = line && line->found ? line->error : 0;
    s_vision_status.distance_cm = s_distance_cm;
    snprintf(s_vision_status.state, sizeof(s_vision_status.state), "%s", state);
    xSemaphoreGive(s_status_lock);
}

static int clampi(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

static void motor_channel_init(ledc_channel_t channel, int pin)
{
    ledc_channel_config_t cfg = {
        .gpio_num = pin, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel, .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

static void motors_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << STBY_PIN) | (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                        (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN) |
                        (1ULL << DIN1_PIN) | (1ULL << DIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(STBY_PIN, 0);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 10000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    motor_channel_init(LEDC_CHANNEL_0, PWMA_PIN);
    motor_channel_init(LEDC_CHANNEL_1, PWMB_PIN);
    motor_channel_init(LEDC_CHANNEL_2, PWMD_PIN);
    ESP_LOGW(TAG, "motor output: %s", MOTOR_OUTPUT_ENABLED ? "ENABLED" : "DRY RUN");
}

static void set_motor(ledc_channel_t channel, int in1, int in2, int pwm)
{
    pwm = clampi(pwm, -255, 255);
    gpio_set_level(in1, pwm > 0);
    gpio_set_level(in2, pwm < 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, abs(pwm)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void drive(int left, int right)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    gpio_set_level(STBY_PIN, left != 0 || right != 0);
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, left);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, right);
}

static bool read_ultrasonic(float *distance_cm)
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start >= ULTRASONIC_TIMEOUT_US) {
            *distance_cm = MAX_DISTANCE_CM;
            return false;
        }
    }
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) != 0) {
        if (esp_timer_get_time() - echo_start >= ULTRASONIC_TIMEOUT_US) {
            *distance_cm = MAX_DISTANCE_CM;
            return false;
        }
    }
    uint32_t duration = (uint32_t)(esp_timer_get_time() - echo_start);
    *distance_cm = clampi((int)(duration * 0.0343f / 2.0f), 0, (int)MAX_DISTANCE_CM);
    return true;
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TRIG_PIN), .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    io.pin_bit_mask = (1ULL << ECHO_PIN);
    io.mode = GPIO_MODE_INPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    while (true) {
        float total = 0.0f;
        int valid = 0;
        for (int i = 0; i < 3; ++i) {
            float sample;
            if (read_ultrasonic(&sample)) { total += sample; valid++; }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        s_distance_valid = valid > 0;
        s_distance_cm = valid ? total / valid : MAX_DISTANCE_CM;
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
    }
}

static size_t jpeg_input(JDEC *jd, uint8_t *buf, size_t len)
{
    jpeg_src_t *src = jd->device;
    size_t available = src->size - src->pos;
    if (len > available) len = available;
    if (buf) memcpy(buf, src->data + src->pos, len);
    src->pos += len;
    return len;
}

static int jpeg_output(JDEC *jd, void *bitmap, JRECT *rect)
{
    (void)jd;
    uint8_t *src = bitmap;
    unsigned width = rect->right - rect->left + 1;
    for (unsigned y = rect->top; y <= rect->bottom && y < IMG_H; ++y) {
        unsigned copy = width;
        if (rect->left + copy > IMG_W) copy = IMG_W - rect->left;
        memcpy(s_gray + y * IMG_W + rect->left, src, copy);
        src += width;
    }
    return 1;
}

static bool decode_jpeg(const uint8_t *data, size_t size)
{
    uint8_t work[4096];
    JDEC decoder;
    jpeg_src_t source = {.data = data, .size = size, .pos = 0};
    memset(s_gray, 255, IMG_W * IMG_H);
    if (jd_prepare(&decoder, jpeg_input, work, sizeof(work), &source) != JDR_OK) return false;
    if (decoder.width != CAM_W || decoder.height != CAM_H) return false;
    return jd_decomp(&decoder, jpeg_output, 1) == JDR_OK;
}

static line_result_t detect_line(void)
{
    line_result_t out = {.near_x = -1, .mid_x = -1, .far_x = -1};
    memset(s_mask, 0, IMG_W * IMG_H);
    for (int y = ROI_TOP; y < IMG_H; ++y)
        for (int x = 0; x < IMG_W; ++x)
            s_mask[y * IMG_W + x] = s_gray[y * IMG_W + x] < BLACK_THRESHOLD;

    int best_area = 0, best_near_sum = 0, best_near_n = 0;
    int best_mid_sum = 0, best_mid_n = 0, best_far_sum = 0, best_far_n = 0;
    for (int sy = NEAR_TOP; sy < IMG_H; ++sy) {
        for (int sx = 0; sx < IMG_W; ++sx) {
            int start = sy * IMG_W + sx;
            if (s_mask[start] != 1) continue;
            size_t head = 0, tail = 0;
            int area = 0, ns = 0, nn = 0, ms = 0, mn = 0, fs = 0, fn = 0;
            s_mask[start] = 2;
            s_flood_queue[tail++] = start;
            while (head < tail) {
                int p = (int)s_flood_queue[head++], x = p % IMG_W, y = p / IMG_W;
                area++;
                if (y >= NEAR_TOP) { ns += x; nn++; }
                else if (y >= MID_TOP) { ms += x; mn++; }
                else { fs += x; fn++; }
                if (x > 0 && s_mask[p - 1] == 1) { s_mask[p - 1] = 2; s_flood_queue[tail++] = p - 1; }
                if (x + 1 < IMG_W && s_mask[p + 1] == 1) { s_mask[p + 1] = 2; s_flood_queue[tail++] = p + 1; }
                if (y > ROI_TOP && s_mask[p - IMG_W] == 1) { s_mask[p - IMG_W] = 2; s_flood_queue[tail++] = p - IMG_W; }
                if (y + 1 < IMG_H && s_mask[p + IMG_W] == 1) { s_mask[p + IMG_W] = 2; s_flood_queue[tail++] = p + IMG_W; }
            }
            if (area > best_area) {
                best_area = area; best_near_sum = ns; best_near_n = nn;
                best_mid_sum = ms; best_mid_n = mn; best_far_sum = fs; best_far_n = fn;
            }
        }
    }
    if (best_area < MIN_COMPONENT_AREA || best_near_n == 0) return out;
    out.found = true; out.area = best_area;
    out.near_x = best_near_sum / best_near_n;
    out.mid_x = best_mid_n ? best_mid_sum / best_mid_n : out.near_x;
    out.far_x = best_far_n ? best_far_sum / best_far_n : out.mid_x;
    int target_x = (60 * out.near_x + 25 * out.mid_x + 15 * out.far_x) / 100;
    out.error = target_x - IMG_W / 2;
    return out;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>ESP32 Car Vision</title><style>"
        "body{margin:0;background:#eef2f1;color:#162321;font:16px Arial,sans-serif}"
        "main{max-width:980px;margin:0 auto;padding:18px 14px}h1{font-size:22px;margin:0 0 12px}"
        ".views{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.panel{min-width:0}"
        ".title{font-size:13px;font-weight:bold;margin:0 0 5px;color:#334541}.view{position:relative;width:100%;aspect-ratio:4/3;background:#111}"
        ".view img,.view canvas{position:absolute;inset:0;width:100%;height:100%;display:block}.view canvas{pointer-events:none}"
        ".process{width:100%;aspect-ratio:4/3;background:#111;image-rendering:pixelated;display:block}"
        ".stats{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin-top:12px}"
        ".stat{background:#fff;border:1px solid #c8d2cf;padding:10px}.label{font-size:12px;color:#596764}"
        ".value{font-size:19px;margin-top:3px;font-weight:bold}@media(max-width:680px){.views{grid-template-columns:1fr}}"
        "@media(max-width:420px){.stats{grid-template-columns:1fr}}</style></head><body><main><h1>ESP32-S3 Camera Processing</h1>"
        "<div class=\"views\"><section class=\"panel\"><div class=\"title\">Original + line overlay</div><div class=\"view\">"
        "<img id=\"stream\" src=\"/stream\" alt=\"Camera stream\"><canvas id=\"overlay\" width=\"640\" height=\"480\"></canvas></div></section>"
        "<section class=\"panel\"><div class=\"title\">Grayscale preview</div><canvas id=\"gray\" class=\"process\" width=\"320\" height=\"240\"></canvas></section>"
        "<section class=\"panel\"><div class=\"title\">Binary mask: grayscale &lt; 110</div><canvas id=\"binary\" class=\"process\" width=\"320\" height=\"240\"></canvas></section>"
        "<section class=\"panel\"><div class=\"title\">Detected line centers and connection</div><canvas id=\"line\" class=\"process\" width=\"320\" height=\"240\"></canvas></section></div>"
        "<section class=\"stats\"><div class=\"stat\"><div class=\"label\">Control state</div>"
        "<div class=\"value\" id=\"state\">Connecting</div></div><div class=\"stat\"><div class=\"label\">Obstacle distance</div>"
        "<div class=\"value\" id=\"distance\">--</div></div><div class=\"stat\"><div class=\"label\">Line area</div>"
        "<div class=\"value\" id=\"area\">--</div></div><div class=\"stat\"><div class=\"label\">Direction error</div>"
        "<div class=\"value\" id=\"error\">--</div></div></section></main><script>"
        "const stream=document.getElementById('stream'),c=document.getElementById('overlay'),g=c.getContext('2d');"
        "const grayCanvas=document.getElementById('gray'),binaryCanvas=document.getElementById('binary'),lineCanvas=document.getElementById('line');"
        "const gg=grayCanvas.getContext('2d'),bg=binaryCanvas.getContext('2d'),lg=lineCanvas.getContext('2d'),src=document.createElement('canvas');"
        "src.width=320;src.height=240;const sg=src.getContext('2d');"
        "let s={near_x:-1,mid_x:-1,far_x:-1,state:'NO_FRAME',distance_cm:0,area:0,error:0,found:false};"
        "function dot(x,y,col){if(x<0)return;g.fillStyle=col;g.beginPath();g.arc(x*2,y,7,0,Math.PI*2);g.fill()}"
        "function points(ctx,scale){const p=[[s.far_x,183,'#53d9ff'],[s.mid_x,210,'#ffd45a'],[s.near_x,231,'#71e79b']];"
        "ctx.lineWidth=2;ctx.strokeStyle='#ff695e';ctx.beginPath();let started=false;p.forEach(v=>{if(v[0]>=0){if(!started){ctx.moveTo(v[0]*scale,v[1]*scale);started=true}else ctx.lineTo(v[0]*scale,v[1]*scale)}});ctx.stroke();"
        "p.forEach(v=>{if(v[0]>=0){ctx.fillStyle=v[2];ctx.beginPath();ctx.arc(v[0]*scale,v[1]*scale,4*scale,0,Math.PI*2);ctx.fill()}})}"
        "function drawProcessed(){if(!stream.naturalWidth)return;sg.drawImage(stream,0,0,320,240);let p=sg.getImageData(0,0,320,240).data;"
        "let gray=gg.createImageData(320,240),binary=bg.createImageData(320,240);for(let i=0;i<320*240;i++){let q=i*4;"
        "let y=(77*p[q]+150*p[q+1]+29*p[q+2])>>8;gray.data[q]=gray.data[q+1]=gray.data[q+2]=y;gray.data[q+3]=255;"
        "let black=(i>=168*320&&y<110);let v=black?0:255;binary.data[q]=binary.data[q+1]=binary.data[q+2]=v;binary.data[q+3]=255}"
        "gg.putImageData(gray,0,0);bg.putImageData(binary,0,0);lg.drawImage(binaryCanvas,0,0);lg.strokeStyle='#f0bd3e';lg.strokeRect(0,168,320,72);"
        "lg.strokeStyle='rgba(255,255,255,.7)';[198,222].forEach(y=>{lg.beginPath();lg.moveTo(0,y);lg.lineTo(320,y);lg.stroke()});points(lg,1)}"
        "function draw(){g.clearRect(0,0,640,480);g.lineWidth=2;g.strokeStyle='rgba(255,190,45,.8)';g.strokeRect(0,336,640,144);"
        "g.strokeStyle='rgba(255,255,255,.55)';[396,444].forEach(y=>{g.beginPath();g.moveTo(0,y);g.lineTo(640,y);g.stroke()});"
        "g.strokeStyle='rgba(65,210,145,.9)';g.beginPath();g.moveTo(320,336);g.lineTo(320,480);g.stroke();"
        "dot(s.far_x,366,'#53d9ff');dot(s.mid_x,420,'#ffd45a');dot(s.near_x,462,'#71e79b');"
        "if(s.found){g.strokeStyle='#ff695e';g.beginPath();g.moveTo((160+s.error)*2,336);g.lineTo((160+s.error)*2,480);g.stroke()}"
        "points(g,2);"
        "g.font='18px Arial';g.fillStyle='#fff';g.fillText(s.state,12,28);g.fillText((s.distance_cm||0).toFixed(1)+' cm',12,52);drawProcessed()}"
        "async function poll(){try{const r=await fetch('/status',{cache:'no-store'});s=await r.json();"
        "document.getElementById('state').textContent=s.state;document.getElementById('distance').textContent=s.distance_cm.toFixed(1)+' cm';"
        "document.getElementById('area').textContent=s.area;document.getElementById('error').textContent=s.error;draw()}catch(e){s.state='LINK LOST';draw()}}"
        "setInterval(poll,100);poll();</script></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    vision_status_t status;
    if (xSemaphoreTake(s_status_lock, pdMS_TO_TICKS(50)) != pdPASS) return ESP_FAIL;
    status = s_vision_status;
    xSemaphoreGive(s_status_lock);

    char response[256];
    int length = snprintf(response, sizeof(response),
                          "{\"decoded\":%s,\"found\":%s,\"area\":%d,\"near_x\":%d,"
                          "\"mid_x\":%d,\"far_x\":%d,\"error\":%d,\"distance_cm\":%.1f,\"state\":\"%s\"}",
                          status.decoded ? "true" : "false", status.found ? "true" : "false",
                          status.area, status.near_x, status.mid_x, status.far_x, status.error,
                          status.distance_cm, status.state);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, response, length);
}

static void stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    uint8_t *copy = heap_caps_malloc(HTTP_STREAM_COPY_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) {
        httpd_req_async_handler_complete(req);
        vTaskDelete(NULL);
        return;
    }

    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    uint32_t sent_version = 0;
    esp_err_t result = ESP_OK;

    while (result == ESP_OK) {
        size_t size = 0;
        if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(100)) == pdPASS) {
            if (s_jpeg_version != sent_version && s_jpeg_sizes[s_jpeg_index] > 0) {
                size = s_jpeg_sizes[s_jpeg_index];
                memcpy(copy, s_jpeg_buffers[s_jpeg_index], size);
                sent_version = s_jpeg_version;
            }
            xSemaphoreGive(s_jpeg_lock);
        }
        if (!size) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        char header[96];
        int header_len = snprintf(header, sizeof(header),
                                  "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                  (unsigned)size);
        result = httpd_resp_send_chunk(req, header, header_len);
        if (result == ESP_OK) result = httpd_resp_send_chunk(req, (const char *)copy, size);
        if (result == ESP_OK) result = httpd_resp_send_chunk(req, "\r\n", 2);
    }
    heap_caps_free(copy);
    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_req_t *async_req = NULL;
    esp_err_t err = httpd_req_async_handler_begin(req, &async_req);
    if (err != ESP_OK) return err;
    if (xTaskCreate(stream_task, "http_stream", 8192, async_req, 4, NULL) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void wifi_start_ap(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASSWORD,
            .channel = 1,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi AP ready: %s", WIFI_AP_SSID);
}

static void start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 4;
    ESP_ERROR_CHECK(httpd_start(&s_http_server, &config));
    const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    const httpd_uri_t status = {.uri = "/status", .method = HTTP_GET, .handler = status_handler};
    const httpd_uri_t stream = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &stream));
    ESP_LOGI(TAG, "open http://192.168.4.1 after joining Wi-Fi %s", WIFI_AP_SSID);
}

static void begin_obstacle(void)
{
    s_obstacle_state = OBSTACLE_TURN;
    s_obstacle_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_turn_direction = (esp_random() & 1) ? 1 : -1;
    drive(0, 0);
    ESP_LOGW(TAG, "obstacle %.1f cm, turning %s", s_distance_cm,
             s_turn_direction > 0 ? "right" : "left");
}

static bool handle_obstacle(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_obstacle_state == OBSTACLE_NONE && s_distance_valid &&
        s_distance_cm < OBSTACLE_THRESHOLD_CM) begin_obstacle();

    if (s_obstacle_state == OBSTACLE_TURN) {
        if (now - s_obstacle_started_ms < OBSTACLE_TURN_MS) {
            drive(s_turn_direction * TURN_PWM, -s_turn_direction * TURN_PWM);
        } else {
            s_obstacle_state = OBSTACLE_CHECK;
            s_obstacle_started_ms = now;
            drive(0, 0);
        }
        return true;
    }
    if (s_obstacle_state == OBSTACLE_CHECK) {
        if (now - s_obstacle_started_ms < OBSTACLE_CHECK_MS) drive(BASE_PWM / 2, BASE_PWM / 2);
        else { s_obstacle_state = OBSTACLE_NONE; drive(0, 0); }
        return true;
    }
    return false;
}

static bool frame_callback(const uvc_host_frame_t *frame, void *ctx)
{
    uvc_host_frame_t *mutable_frame = (uvc_host_frame_t *)frame;
    return xQueueSend((QueueHandle_t)ctx, &mutable_frame, 0) != pdPASS;
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        s_connected = false;
        drive(0, 0);
        ESP_LOGW(TAG, "camera disconnected; motors stopped");
    } else if (event->type == UVC_HOST_FRAME_BUFFER_OVERFLOW) {
        ESP_LOGW(TAG, "camera frame overflow");
    } else if (event->type == UVC_HOST_FRAME_BUFFER_UNDERFLOW) {
        ESP_LOGW(TAG, "camera frame buffer underflow");
    }
}

static void vision_task(void *arg)
{
    (void)arg;
    int confirmed = 0, missing = 0, last_error = 0;
    unsigned frame_no = 0;
    while (s_connected) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(500)) != pdPASS) {
            drive(0, 0);
            update_vision_status(false, NULL, "NO_FRAME");
            continue;
        }
        publish_jpeg(frame);

        line_result_t line = {.near_x = -1, .mid_x = -1, .far_x = -1};
        bool decoded = decode_jpeg(frame->data, frame->data_len);
        if (decoded) line = detect_line();
        const bool obstacle_active = handle_obstacle();
        const char *state;
        if (obstacle_active) {
            state = s_obstacle_state == OBSTACLE_TURN ? "AVOID_TURN" : "AVOID_CHECK";
        } else {
            if (line.found) {
                missing = 0;
                if (confirmed < CONFIRM_FRAMES) confirmed++;
                if (confirmed >= CONFIRM_FRAMES) {
                    last_error = (3 * last_error + line.error) / 4;
                    int correction = clampi(last_error * MAX_CORRECTION / (IMG_W / 2), -MAX_CORRECTION, MAX_CORRECTION);
                    drive(BASE_PWM - correction, BASE_PWM + correction);
                } else drive(0, 0);
            } else {
                confirmed = 0;
                missing++;
                if (missing <= BLIND_HOLD_FRAMES) {
                    int correction = clampi(last_error * MAX_CORRECTION / (IMG_W / 2), -MAX_CORRECTION, MAX_CORRECTION);
                    drive(HOLD_PWM - correction, HOLD_PWM + correction);
                } else drive(0, 0);
            }
            state = line.found ? (confirmed >= CONFIRM_FRAMES ? "TRACK" : "CONFIRM")
                               : (missing <= BLIND_HOLD_FRAMES ? "BLIND_HOLD" : "LOST_STOP");
        }
        update_vision_status(decoded, &line, state);
        if (++frame_no % 5 == 0) {
            ESP_LOGI(TAG, "state=%s decode=%d area=%d x=%d/%d/%d error=%d distance=%.1f motor=%s",
                     state, decoded, line.area, line.near_x, line.mid_x, line.far_x,
                     line.found ? line.error : last_error, s_distance_cm,
                     MOTOR_OUTPUT_ENABLED ? "ON" : "DRY");
        }
        uvc_host_frame_return(s_stream, frame);
        vTaskDelay(1);
    }
    drive(0, 0);
    uvc_host_frame_t *pending = NULL;
    while (xQueueReceive(s_frame_q, &pending, 0) == pdPASS)
        uvc_host_frame_return(s_stream, pending);
    s_vision_task_running = false;
    vTaskDelete(NULL);
}

static void camera_task(void *arg)
{
    (void)arg;
    while (true) {
        const uvc_host_stream_config_t cfg = {
            .event_cb = stream_callback, .frame_cb = frame_callback, .user_ctx = s_frame_q,
            .usb = {.vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0},
            .vs_format = {.h_res = CAM_W, .v_res = CAM_H, .fps = CAM_FPS, .format = UVC_VS_FORMAT_MJPEG},
            .advanced = {.number_of_frame_buffers = FRAME_BUFFERS, .frame_size = FRAME_SIZE,
                         .frame_heap_caps = MALLOC_CAP_SPIRAM, .number_of_urbs = 6, .urb_size = 16 * 1024},
        };
        ESP_LOGI(TAG, "opening UVC camera GPIO19=D- GPIO20=D+");
        if (uvc_host_stream_open(&cfg, pdMS_TO_TICKS(5000), &s_stream) != ESP_OK) {
            ESP_LOGW(TAG, "camera open failed; retrying"); vTaskDelay(pdMS_TO_TICKS(3000)); continue;
        }
        s_connected = true;
        ESP_ERROR_CHECK(uvc_host_stream_start(s_stream));
        ESP_LOGI(TAG, "camera streaming %dx%d@%d MJPEG", CAM_W, CAM_H, CAM_FPS);
        s_vision_task_running = true;
        if (xTaskCreatePinnedToCore(vision_task, "vision", 16384, NULL,
                                   USB_PRIORITY - 2, NULL, 1) != pdPASS) {
            s_vision_task_running = false;
            s_connected = false;
            drive(0, 0);
            ESP_LOGE(TAG, "failed to create vision task");
        }
        while (s_connected) vTaskDelay(pdMS_TO_TICKS(200));
        while (s_vision_task_running) vTaskDelay(pdMS_TO_TICKS(10));
        uvc_host_stream_close(s_stream);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void uvc_event_callback(const uvc_host_driver_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED)
        ESP_LOGI(TAG, "camera connected, USB address=%d", event->device_connected.dev_addr);
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    while (true) { uint32_t flags; usb_host_lib_handle_events(portMAX_DELAY, &flags); }
}

void app_main(void)
{
    motors_init();
    s_gray = heap_caps_malloc(IMG_W * IMG_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_mask = heap_caps_malloc(IMG_W * IMG_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_flood_queue = heap_caps_malloc(IMG_W * IMG_H * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_jpeg_buffers[0] = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_jpeg_buffers[1] = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_jpeg_lock = xSemaphoreCreateMutex();
    s_status_lock = xSemaphoreCreateMutex();
    assert(s_gray && s_mask && s_flood_queue && s_jpeg_buffers[0] && s_jpeg_buffers[1] &&
           s_jpeg_lock && s_status_lock);
    s_frame_q = xQueueCreate(FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
    assert(s_frame_q);

    update_vision_status(false, NULL, "BOOT");
    wifi_start_ap();
    start_web_server();

    const usb_host_config_t usb_cfg = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LOWMED};
    ESP_ERROR_CHECK(usb_host_install(&usb_cfg));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, USB_PRIORITY, NULL);
    const uvc_host_driver_config_t uvc_cfg = {
        .driver_task_stack_size = 4096, .driver_task_priority = USB_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY, .create_background_task = true, .event_cb = uvc_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_cfg));

    xTaskCreate(ultrasonic_task, "ultrasonic", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "camera line following + ultrasonic avoidance ready");
    ESP_LOGI(TAG, "camera D-=GPIO19 D+=GPIO20; TRIG=GPIO7 ECHO=GPIO6");
    xTaskCreatePinnedToCore(camera_task, "camera", 6144, NULL, USB_PRIORITY - 1, NULL, 0);
}
