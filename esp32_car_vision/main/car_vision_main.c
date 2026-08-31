/* ESP32-S3 + USB UVC camera predictive line following, first safe version. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "tjpgd.h"

#define CAM_W 640
#define CAM_H 480
#define CAM_FPS 25
#define IMG_W 320
#define IMG_H 240

/* Current on-car tuning; ROI_TOP=168 corresponds to ROI=0.70 at 240 px. */
#define BLACK_THRESHOLD 110
#define ROI_TOP 168
#define NEAR_TOP 222
#define MID_TOP 198
#define MIN_COMPONENT_AREA 250
#define CONFIRM_FRAMES 3
#define BLIND_HOLD_FRAMES 6

/* Keep this at 0 for serial-only dry-run. Change to 1 only after wheel-up tests. */
#define MOTOR_OUTPUT_ENABLED 1
#define BASE_PWM 45
#define HOLD_PWM 24
#define MAX_CORRECTION 18

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

#define FRAME_BUFFERS 3
#define USB_PRIORITY 15

static const char *TAG = "car_vision";
static QueueHandle_t s_frame_q;
static volatile bool s_connected;
static uvc_host_stream_hdl_t s_stream;
static uint8_t *s_gray, *s_mask;
static uint32_t *s_queue;
static volatile bool s_frame_task_running;

typedef struct { const uint8_t *data; size_t size, pos; } jpeg_src_t;
typedef struct { bool found; int area, near_x, mid_x, far_x, error; } line_result_t;

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
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << STBY_PIN) | (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                        (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN) |
                        (1ULL << DIN1_PIN) | (1ULL << DIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(STBY_PIN, 0);
    ledc_timer_config_t timer = {
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
    gpio_set_level(STBY_PIN, (left != 0 || right != 0));
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, left);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0); /* rear motor stays stopped */
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, right);
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
    for (unsigned y = rect->top; y <= rect->bottom && y < IMG_H; y++) {
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
    jpeg_src_t src = {.data = data, .size = size, .pos = 0};
    memset(s_gray, 255, IMG_W * IMG_H);
    JRESULT result = jd_prepare(&decoder, jpeg_input, work, sizeof(work), &src);
    if (result != JDR_OK) return false;
    if (decoder.width != CAM_W || decoder.height != CAM_H) return false;
    return jd_decomp(&decoder, jpeg_output, 1) == JDR_OK; /* 1/2 -> 320x240 */
}

static line_result_t detect_line(void)
{
    line_result_t out = {.near_x = -1, .mid_x = -1, .far_x = -1};
    memset(s_mask, 0, IMG_W * IMG_H);
    for (int y = ROI_TOP; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++)
            s_mask[y * IMG_W + x] = s_gray[y * IMG_W + x] < BLACK_THRESHOLD ? 1 : 0;

    int best_area = 0, best_near_sum = 0, best_near_n = 0;
    int best_mid_sum = 0, best_mid_n = 0, best_far_sum = 0, best_far_n = 0;
    for (int sy = NEAR_TOP; sy < IMG_H; sy++) {
        for (int sx = 0; sx < IMG_W; sx++) {
            int start = sy * IMG_W + sx;
            if (s_mask[start] != 1) continue;
            size_t head = 0, tail = 0;
            int area = 0, ns = 0, nn = 0, ms = 0, mn = 0, fs = 0, fn = 0;
            s_mask[start] = 2;
            s_queue[tail++] = start;
            while (head < tail) {
                int p = (int)s_queue[head++], x = p % IMG_W, y = p / IMG_W;
                area++;
                if (y >= NEAR_TOP) { ns += x; nn++; }
                else if (y >= MID_TOP) { ms += x; mn++; }
                else { fs += x; fn++; }
                const int next[4] = {p - 1, p + 1, p - IMG_W, p + IMG_W};
                if (x > 0 && s_mask[next[0]] == 1) { s_mask[next[0]] = 2; s_queue[tail++] = next[0]; }
                if (x + 1 < IMG_W && s_mask[next[1]] == 1) { s_mask[next[1]] = 2; s_queue[tail++] = next[1]; }
                if (y > ROI_TOP && s_mask[next[2]] == 1) { s_mask[next[2]] = 2; s_queue[tail++] = next[2]; }
                if (y + 1 < IMG_H && s_mask[next[3]] == 1) { s_mask[next[3]] = 2; s_queue[tail++] = next[3]; }
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

static bool frame_callback(const uvc_host_frame_t *frame, void *ctx)
{
    QueueHandle_t queue = ctx;
    uvc_host_frame_t *mutable_frame = (uvc_host_frame_t *)frame;
    if (xQueueSend(queue, &mutable_frame, 0) != pdPASS) return true;
    return false;
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        s_connected = false;
    }
}

static void frame_task(void *arg)
{
    (void)arg;
    int confirmed = 0, missing = 0, last_error = 0;
    unsigned frame_no = 0;
    while (s_connected) {
        uvc_host_frame_t *frame;
        if (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(500)) != pdPASS) {
            drive(0, 0); continue;
        }
        line_result_t line = {0};
        bool decoded = decode_jpeg(frame->data, frame->data_len);
        if (decoded) line = detect_line();
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
        if (++frame_no % 5 == 0) {
            const char *state = line.found ? (confirmed >= CONFIRM_FRAMES ? "TRACK" : "CONFIRM")
                                           : (missing <= BLIND_HOLD_FRAMES ? "BLIND_HOLD" : "LOST_STOP");
            ESP_LOGI(TAG, "state=%s decode=%d area=%d x=%d/%d/%d error=%d motor=%s",
                     state, decoded, line.area, line.near_x, line.mid_x, line.far_x,
                     line.found ? line.error : last_error,
                     MOTOR_OUTPUT_ENABLED ? "ON" : "DRY");
        }
        uvc_host_frame_return(s_stream, frame);
        /* JPEG decoding is CPU-heavy; let IDLE1 run so its watchdog is fed. */
        vTaskDelay(1);
    }
    drive(0, 0);
    uvc_host_frame_t *pending;
    while (xQueueReceive(s_frame_q, &pending, 0) == pdPASS)
        uvc_host_frame_return(s_stream, pending);
    s_frame_task_running = false;
    vTaskDelete(NULL);
}

static void camera_task(void *arg)
{
    (void)arg;
    while (true) {
        uvc_host_stream_config_t cfg = {
            .event_cb = stream_callback, .frame_cb = frame_callback, .user_ctx = s_frame_q,
            .usb = {.vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0},
            .vs_format = {.h_res = CAM_W, .v_res = CAM_H, .fps = CAM_FPS, .format = UVC_VS_FORMAT_MJPEG},
            .advanced = {.number_of_frame_buffers = FRAME_BUFFERS, .frame_size = 0,
                         .number_of_urbs = 4, .urb_size = 10 * 1024},
        };
        if (uvc_host_stream_open(&cfg, pdMS_TO_TICKS(5000), &s_stream) != ESP_OK) {
            ESP_LOGW(TAG, "camera open failed; retrying"); vTaskDelay(pdMS_TO_TICKS(3000)); continue;
        }
        s_connected = true;
        ESP_ERROR_CHECK(uvc_host_stream_start(s_stream));
        ESP_LOGI(TAG, "camera streaming: %dx%d@%d MJPEG", CAM_W, CAM_H, CAM_FPS);
        s_frame_task_running = true;
        xTaskCreatePinnedToCore(frame_task, "vision", 16384, NULL, USB_PRIORITY - 2, NULL, 1);
        while (s_connected) vTaskDelay(pdMS_TO_TICKS(200));
        drive(0, 0);
        while (s_frame_task_running) vTaskDelay(pdMS_TO_TICKS(10));
        esp_err_t close_err = uvc_host_stream_close(s_stream);
        if (close_err != ESP_OK) ESP_LOGW(TAG, "stream close failed: %s", esp_err_to_name(close_err));
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
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

void app_main(void)
{
    motors_init();
    s_gray = heap_caps_malloc(IMG_W * IMG_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_mask = heap_caps_malloc(IMG_W * IMG_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_queue = heap_caps_malloc(IMG_W * IMG_H * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_gray && s_mask && s_queue);
    s_frame_q = xQueueCreate(FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
    assert(s_frame_q);
    const usb_host_config_t usb_cfg = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LOWMED};
    ESP_ERROR_CHECK(usb_host_install(&usb_cfg));
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, USB_PRIORITY, NULL, tskNO_AFFINITY);
    const uvc_host_driver_config_t uvc_cfg = {
        .driver_task_stack_size = 4096, .driver_task_priority = USB_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY, .create_background_task = true, .event_cb = uvc_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_cfg));
    ESP_LOGI(TAG, "USB camera pins: GPIO19=D- GPIO20=D+; waiting for camera");
    xTaskCreatePinnedToCore(camera_task, "camera", 6144, NULL, USB_PRIORITY - 1, NULL, 0);
}
