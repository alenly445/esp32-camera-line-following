/* ESP32-S3 + USB UVC 摄像头预测巡线：第一版安全实现。 */
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
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "car_display.h"
#include "tjpgd.h"

#define CAM_W 640
#define CAM_H 480
#define CAM_FPS 25
#define IMG_W 320
#define IMG_H 240

/* 当前实车调参值：图像高度为240像素时，ROI_TOP=168对应ROI=0.70。 */
#define BLACK_THRESHOLD 110
#define ROI_TOP 168
#define NEAR_TOP 222
#define MID_TOP 198
#define MIN_COMPONENT_AREA 250
#define CONFIRM_FRAMES 3
#define BLIND_HOLD_FRAMES 6

/* 仅进行串口空运行测试时保持为0；架空小车确认车轮方向后才改为1。 */
#define MOTOR_OUTPUT_ENABLED 1
#define BASE_PWM 42
#define HOLD_PWM 36
#define MAX_CORRECTION 30

/* 直角转弯参数：路口进入摄像头盲区前先记住方向，再按编码器计数前进。 */
#define TURN_SCAN_AREA_MIN 300
#define TURN_SIDE_EXTENT_MIN 65
#define TURN_SIDE_DOMINANCE 30
#define TURN_STRONG_SIDE_X 45
#define TURN_STRONG_ERROR 60
#define TURN_HINT_CONFIRM_FRAMES 2
#define TURN_APPROACH_PWM 40
#define TURN_APPROACH_ENCODER_COUNTS 200
#define TURN_APPROACH_SAFETY_MS 5000
#define TURN_SPIN_PWM 36
#define TURN_CENTER_TOLERANCE 40
#define TURN_NEAR_CENTER_TOLERANCE 48
#define TURN_REACQUIRE_CONFIRM_FRAMES 1
#define TURN_REACQUIRE_PWM 36
#define TURN_SAFETY_MAX_FRAMES 800

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

/* 霍尔编码器接线与v20_line_car_vscode.ino一致。 */
#define ENCODER_A_PHASE_A_PIN 48
#define ENCODER_A_PHASE_B_PIN 47
#define ENCODER_D_PHASE_A_PIN 42
#define ENCODER_D_PHASE_B_PIN 41

#define FRAME_BUFFERS 3
#define USB_PRIORITY 15

static const char *TAG = "car_vision";
static QueueHandle_t s_frame_q;
static volatile bool s_connected;
static uvc_host_stream_hdl_t s_stream;
static uint8_t *s_gray, *s_mask;
static uint32_t *s_queue;
static volatile bool s_frame_task_running;
static volatile int32_t s_encoder_a_count;
static volatile int32_t s_encoder_d_count;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct { const uint8_t *data; size_t size, pos; } jpeg_src_t;

typedef enum {
    TURN_DIR_NONE = 0,
    TURN_DIR_LEFT = -1,
    TURN_DIR_RIGHT = 1,
} turn_direction_t;

typedef enum {
    NAV_FOLLOW,
    NAV_APPROACH_TURN,
    NAV_SPIN_TURN,
    NAV_TURN_FAILED,
} navigation_state_t;

typedef struct {
    bool found;
    int area;
    int near_x;
    int mid_x;
    int far_x;
    int error;
    int scan_area;
    int left_extent;
    int right_extent;
    turn_direction_t turn_hint;
} line_result_t;

/* 将整数限制在闭区间[low, high]内。 */
static int clampi(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

/* A轮霍尔编码器A相边沿中断，读取B相后确定计数方向。 */
static void IRAM_ATTR encoder_a_isr(void *argument)
{
    (void)argument;
    int direction = gpio_get_level(ENCODER_A_PHASE_A_PIN) ==
                    gpio_get_level(ENCODER_A_PHASE_B_PIN) ? 1 : -1;
    portENTER_CRITICAL_ISR(&s_encoder_lock);
    s_encoder_a_count += direction;
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

/* D轮霍尔编码器A相边沿中断，读取B相后确定计数方向。 */
static void IRAM_ATTR encoder_d_isr(void *argument)
{
    (void)argument;
    int direction = gpio_get_level(ENCODER_D_PHASE_A_PIN) ==
                    gpio_get_level(ENCODER_D_PHASE_B_PIN) ? 1 : -1;
    portENTER_CRITICAL_ISR(&s_encoder_lock);
    s_encoder_d_count += direction;
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

/* 原子读取A、D两轮编码器累计计数。 */
static void read_encoder_counts(int32_t *a_count, int32_t *d_count)
{
    portENTER_CRITICAL(&s_encoder_lock);
    *a_count = s_encoder_a_count;
    *d_count = s_encoder_d_count;
    portEXIT_CRITICAL(&s_encoder_lock);
}

/* 初始化A、D轮霍尔编码器；只在A相双边沿触发中断。 */
static void encoders_init(void)
{
    gpio_config_t input = {
        .pin_bit_mask = (1ULL << ENCODER_A_PHASE_A_PIN) |
                        (1ULL << ENCODER_A_PHASE_B_PIN) |
                        (1ULL << ENCODER_D_PHASE_A_PIN) |
                        (1ULL << ENCODER_D_PHASE_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_A_PHASE_A_PIN, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_D_PHASE_A_PIN, GPIO_INTR_ANYEDGE));
    esp_err_t service_error = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (service_error != ESP_OK && service_error != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(service_error);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_A_PHASE_A_PIN, encoder_a_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_D_PHASE_A_PIN, encoder_d_isr, NULL));
    ESP_LOGI(TAG, "hall encoders: A=%d/%d D=%d/%d approach_target=%d",
             ENCODER_A_PHASE_A_PIN, ENCODER_A_PHASE_B_PIN,
             ENCODER_D_PHASE_A_PIN, ENCODER_D_PHASE_B_PIN,
             TURN_APPROACH_ENCODER_COUNTS);
}

/* 将一个PWM输出引脚连接到ESP-IDF的LEDC通道。 */
static void motor_channel_init(ledc_channel_t channel, int pin)
{
    ledc_channel_config_t cfg = {
        .gpio_num = pin, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel, .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

/* 初始化TB6612方向引脚和三个电机PWM通道。 */
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

/* 驱动一个TB6612通道：正值表示正转，负值表示反转。 */
static void set_motor(ledc_channel_t channel, int in1, int in2, int pwm)
{
    pwm = clampi(pwm, -255, 255);
    gpio_set_level(in1, pwm > 0);
    gpio_set_level(in2, pwm < 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, abs(pwm)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

/* 驱动左侧A电机和右侧D电机，后方B电机保持停止。 */
static void drive(int left, int right)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    gpio_set_level(STBY_PIN, (left != 0 || right != 0));
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, left);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0); /* 后方B电机保持停止 */
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, right);
}

/* 将UVC帧中的压缩JPEG数据提供给TJpgDec。 */
static size_t jpeg_input(JDEC *jd, uint8_t *buf, size_t len)
{
    jpeg_src_t *src = jd->device;
    size_t available = src->size - src->pos;
    if (len > available) len = available;
    if (buf) memcpy(buf, src->data + src->pos, len);
    src->pos += len;
    return len;
}

/* 将解码后的每个灰度JPEG图块复制到320x240图像缓冲区。 */
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

/* 将一帧640x480的MJPEG图像解码为缩小一半的灰度图像。 */
static bool decode_jpeg(const uint8_t *data, size_t size)
{
    uint8_t work[4096];
    JDEC decoder;
    jpeg_src_t src = {.data = data, .size = size, .pos = 0};
    memset(s_gray, 255, IMG_W * IMG_H);
    JRESULT result = jd_prepare(&decoder, jpeg_input, work, sizeof(work), &src);
    if (result != JDR_OK) return false;
    if (decoder.width != CAM_W || decoder.height != CAM_H) return false;
    return jd_decomp(&decoder, jpeg_output, 1) == JDR_OK; /* 缩小为原图的1/2，即320x240 */
}

/* 查找与图像底部相连的黑线，并检测向左或向右延伸的宽分支。
 * 分支提示只代表几何特征，frame_task还会通过连续多帧进行确认。 */
static line_result_t detect_line(void)
{
    line_result_t out = {.near_x = -1, .mid_x = -1, .far_x = -1};
    memset(s_mask, 0, IMG_W * IMG_H);
    for (int y = ROI_TOP; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++)
            s_mask[y * IMG_W + x] = s_gray[y * IMG_W + x] < BLACK_THRESHOLD ? 1 : 0;

    int best_area = 0, best_near_sum = 0, best_near_n = 0;
    int best_mid_sum = 0, best_mid_n = 0, best_far_sum = 0, best_far_n = 0;
    int best_scan_area = 0, best_scan_min_x = IMG_W, best_scan_max_x = -1;
    for (int sy = NEAR_TOP; sy < IMG_H; sy++) {
        for (int sx = 0; sx < IMG_W; sx++) {
            int start = sy * IMG_W + sx;
            /* 对一个延伸到近端窄条的黑色连通区域进行洪水填充。 */
            if (s_mask[start] != 1) continue;
            size_t head = 0, tail = 0;
            int area = 0, ns = 0, nn = 0, ms = 0, mn = 0, fs = 0, fn = 0;
            int scan_area = 0, scan_min_x = IMG_W, scan_max_x = -1;
            s_mask[start] = 2;
            s_queue[tail++] = start;
            while (head < tail) {
                int p = (int)s_queue[head++], x = p % IMG_W, y = p / IMG_W;
                area++;
                if (y >= NEAR_TOP) { ns += x; nn++; }
                else if (y >= MID_TOP) { ms += x; mn++; }
                else { fs += x; fn++; }
                if (y < NEAR_TOP) {
                    scan_area++;
                    if (x < scan_min_x) scan_min_x = x;
                    if (x > scan_max_x) scan_max_x = x;
                }
                const int next[4] = {p - 1, p + 1, p - IMG_W, p + IMG_W};
                if (x > 0 && s_mask[next[0]] == 1) { s_mask[next[0]] = 2; s_queue[tail++] = next[0]; }
                if (x + 1 < IMG_W && s_mask[next[1]] == 1) { s_mask[next[1]] = 2; s_queue[tail++] = next[1]; }
                if (y > ROI_TOP && s_mask[next[2]] == 1) { s_mask[next[2]] = 2; s_queue[tail++] = next[2]; }
                if (y + 1 < IMG_H && s_mask[next[3]] == 1) { s_mask[next[3]] = 2; s_queue[tail++] = next[3]; }
            }

            if (area > best_area) {
                best_area = area; best_near_sum = ns; best_near_n = nn;
                best_mid_sum = ms; best_mid_n = mn; best_far_sum = fs; best_far_n = fn;
                best_scan_area = scan_area;
                best_scan_min_x = scan_min_x;
                best_scan_max_x = scan_max_x;
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
    out.scan_area = best_scan_area;
    out.left_extent = best_scan_max_x >= 0 ? out.near_x - best_scan_min_x : 0;
    out.right_extent = best_scan_max_x >= 0 ? best_scan_max_x - out.near_x : 0;

    /* 仅接受面积较大且明显偏向单侧的延伸区域，从而在连续帧确认前排除
     * 普通居中直线和大多数斜向弯线。 */
    int center_x = IMG_W / 2;
    bool strongly_right = out.near_x >= center_x + TURN_STRONG_SIDE_X &&
                          out.mid_x >= center_x + TURN_STRONG_SIDE_X &&
                          out.far_x >= center_x + TURN_STRONG_SIDE_X &&
                          out.error >= TURN_STRONG_ERROR;
    bool strongly_left = out.near_x <= center_x - TURN_STRONG_SIDE_X &&
                         out.mid_x <= center_x - TURN_STRONG_SIDE_X &&
                         out.far_x <= center_x - TURN_STRONG_SIDE_X &&
                         out.error <= -TURN_STRONG_ERROR;

    /* 近、中、远中心明显位于同一侧时，优先采用线路中心方向。
     * 这可以避免120度转弯中，透视线向反方向延伸导致ext判断颠倒。 */
    if (out.scan_area >= TURN_SCAN_AREA_MIN && strongly_right) {
        out.turn_hint = TURN_DIR_RIGHT;
    } else if (out.scan_area >= TURN_SCAN_AREA_MIN && strongly_left) {
        out.turn_hint = TURN_DIR_LEFT;
    } else if (out.scan_area >= TURN_SCAN_AREA_MIN &&
        out.left_extent >= TURN_SIDE_EXTENT_MIN &&
        out.left_extent - out.right_extent >= TURN_SIDE_DOMINANCE) {
        out.turn_hint = TURN_DIR_LEFT;
    } else if (out.scan_area >= TURN_SCAN_AREA_MIN &&
               out.right_extent >= TURN_SIDE_EXTENT_MIN &&
               out.right_extent - out.left_extent >= TURN_SIDE_DOMINANCE) {
        out.turn_hint = TURN_DIR_RIGHT;
    }
    return out;
}

/* 将UVC帧放入视觉处理队列；队列已满时丢弃新帧。 */
static bool frame_callback(const uvc_host_frame_t *frame, void *ctx)
{
    QueueHandle_t queue = ctx;
    uvc_host_frame_t *mutable_frame = (uvc_host_frame_t *)frame;
    if (xQueueSend(queue, &mutable_frame, 0) != pdPASS) return true;
    return false;
}

/* USB摄像头断开时立即停止小车。 */
static void stream_callback(const uvc_host_stream_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        s_connected = false;
        car_display_set(0, 0, 0, CAR_DISPLAY_STOP);
    }
}

/* 返回已记忆转弯方向的简短可打印名称。 */
static const char *turn_direction_name(turn_direction_t direction)
{
    if (direction == TURN_DIR_LEFT) return "LEFT";
    if (direction == TURN_DIR_RIGHT) return "RIGHT";
    return "NONE";
}

/* 检查新找到的线路是否已足够居中，以便结束原地转弯。 */
static bool line_centered_after_turn(const line_result_t *line)
{
    return line->found &&
           abs(line->error) <= TURN_CENTER_TOLERANCE &&
           abs(line->near_x - IMG_W / 2) <= TURN_NEAR_CENTER_TOLERANCE;
}

/* 为向左或向右原地转动产生方向相反的左右轮速度。 */
static void get_spin_pwm(turn_direction_t direction, int *left_pwm, int *right_pwm)
{
    if (direction == TURN_DIR_LEFT) {
        *left_pwm = -TURN_SPIN_PWM;
        *right_pwm = TURN_SPIN_PWM;
    } else {
        *left_pwm = TURN_SPIN_PWM;
        *right_pwm = -TURN_SPIN_PWM;
    }
}

/* 把导航状态、当前黑线和新线确认进度转换为屏幕中文提示。 */
static car_display_state_t display_state_from_navigation(const char *state_name,
                                                          const line_result_t *line,
                                                          int spin_center)
{
    if (strcmp(state_name, "PEND_LEFT") == 0)
        return line->found ? CAR_DISPLAY_DETECTED_LEFT : CAR_DISPLAY_LOST_FORWARD;
    if (strcmp(state_name, "PEND_RIGHT") == 0)
        return line->found ? CAR_DISPLAY_DETECTED_RIGHT : CAR_DISPLAY_LOST_FORWARD;
    if (strcmp(state_name, "TURN_LEFT") == 0)
        return spin_center > 0 ? CAR_DISPLAY_NEW_LINE : CAR_DISPLAY_TURNING_LEFT;
    if (strcmp(state_name, "TURN_RIGHT") == 0)
        return spin_center > 0 ? CAR_DISPLAY_NEW_LINE : CAR_DISPLAY_TURNING_RIGHT;
    if (strcmp(state_name, "REACQUIRED") == 0) return CAR_DISPLAY_NEW_LINE;
    if (strcmp(state_name, "LOST_HOLD") == 0) return CAR_DISPLAY_LOST_FORWARD;
    if (strcmp(state_name, "DECODE_FAIL") == 0 ||
        strcmp(state_name, "CONFIRM") == 0 ||
        strcmp(state_name, "LOST_STOP") == 0 ||
        strcmp(state_name, "TURN_FAILED") == 0) return CAR_DISPLAY_STOP;
    return CAR_DISPLAY_STRAIGHT;
}

/* 处理摄像头图像并运行完整导航状态机：
 * 正常巡线 -> 记住路口方向 -> 向前接近 -> 原地转弯 -> 找回线路并恢复巡线。 */
static void frame_task(void *arg)
{
    (void)arg;
    int confirmed = 0, missing = 0, last_error = 0;
    int hint_confirm = 0;
    int spin_center = 0, spin_frames = 0;
    int32_t approach_start_a = 0, approach_start_d = 0;
    int64_t approach_start_us = 0;
    unsigned frame_no = 0;
    navigation_state_t nav_state = NAV_FOLLOW;
    turn_direction_t hint_candidate = TURN_DIR_NONE;
    turn_direction_t pending_turn = TURN_DIR_NONE;
    turn_direction_t next_hint_candidate = TURN_DIR_NONE;
    turn_direction_t queued_turn = TURN_DIR_NONE;
    int next_hint_confirm = 0;

    while (s_connected) {
        int correction = 0;
        int left_pwm = 0;
        int right_pwm = 0;
        int32_t approach_a = 0;
        int32_t approach_d = 0;
        bool reacquired_now = false;
        const char *state_name = "UNKNOWN";
        uvc_host_frame_t *frame;

        if (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(500)) != pdPASS) {
            drive(0, 0);
            car_display_set(0, 0, 0, CAR_DISPLAY_STOP);
            continue;
        }

        line_result_t line = {.near_x = -1, .mid_x = -1, .far_x = -1};
        bool decoded = decode_jpeg(frame->data, frame->data_len);
        if (decoded) line = detect_line();

        /* JPEG解码失败时，不据此判断丢线或检测路口。 */
        if (!decoded) {
            drive(0, 0);
            state_name = "DECODE_FAIL";
        } else if (nav_state == NAV_FOLLOW) {
            if (queued_turn != TURN_DIR_NONE) {
                pending_turn = queued_turn;
                queued_turn = TURN_DIR_NONE;
                nav_state = NAV_APPROACH_TURN;
                read_encoder_counts(&approach_start_a, &approach_start_d);
                approach_start_us = esp_timer_get_time();
                left_pwm = right_pwm = TURN_APPROACH_PWM;
                drive(left_pwm, right_pwm);
                state_name = pending_turn == TURN_DIR_LEFT ? "PEND_LEFT" : "PEND_RIGHT";
                ESP_LOGW(TAG, "continuous corner activated: %s; encoder target=%d",
                         turn_direction_name(pending_turn),
                         TURN_APPROACH_ENCODER_COUNTS);
            } else if (line.found) {
                missing = 0;
                if (confirmed < CONFIRM_FRAMES) confirmed++;

                /* 侧向分支必须在连续多帧中指向同一方向，才能记住该方向。 */
                if (line.turn_hint == TURN_DIR_NONE) {
                    hint_candidate = TURN_DIR_NONE;
                    hint_confirm = 0;
                } else if (line.turn_hint == hint_candidate) {
                    if (hint_confirm < TURN_HINT_CONFIRM_FRAMES) hint_confirm++;
                } else {
                    hint_candidate = line.turn_hint;
                    hint_confirm = 1;
                }

                if (confirmed >= CONFIRM_FRAMES) {
                    last_error = (3 * last_error + line.error) / 4;
                    correction = clampi(last_error * MAX_CORRECTION / (IMG_W / 3),
                                        -MAX_CORRECTION, MAX_CORRECTION);
                    left_pwm = BASE_PWM + correction;
                    right_pwm = BASE_PWM - correction;
                    drive(left_pwm, right_pwm);
                    state_name = "TRACK";
                } else {
                    left_pwm = right_pwm = BASE_PWM;
                    drive(left_pwm, right_pwm);
                    state_name = "CONFIRM";
                }

                if (hint_confirm >= TURN_HINT_CONFIRM_FRAMES) {
                    pending_turn = hint_candidate;
                    nav_state = NAV_APPROACH_TURN;
                    read_encoder_counts(&approach_start_a, &approach_start_d);
                    approach_start_us = esp_timer_get_time();
                    left_pwm = right_pwm = TURN_APPROACH_PWM;
                    drive(left_pwm, right_pwm);
                    state_name = pending_turn == TURN_DIR_LEFT ? "PEND_LEFT" : "PEND_RIGHT";
                    ESP_LOGW(TAG,
                             "corner memorized: %s; approach by hall encoders A/D target=%d",
                             turn_direction_name(pending_turn),
                             TURN_APPROACH_ENCODER_COUNTS);
                }
            } else {
                confirmed = 0;
                hint_candidate = TURN_DIR_NONE;
                hint_confirm = 0;
                missing++;
                correction = clampi(last_error * MAX_CORRECTION / (IMG_W / 3),
                                    -MAX_CORRECTION, MAX_CORRECTION);
                left_pwm = HOLD_PWM + correction;
                right_pwm = HOLD_PWM - correction;
                drive(left_pwm, right_pwm);
                state_name = missing <= BLIND_HOLD_FRAMES ? "BLIND_HOLD" : "LOST_HOLD";
            }
        } else if (nav_state == NAV_APPROACH_TURN) {
            /* 摄像头看到约10厘米外的区域，因此先按A、D轮编码器计数前进。
             * A轮前进计数为正，D轮前进计数为负，所以D轮差值需要取反。 */
            left_pwm = right_pwm = TURN_APPROACH_PWM;
            drive(left_pwm, right_pwm);
            state_name = pending_turn == TURN_DIR_LEFT ? "PEND_LEFT" : "PEND_RIGHT";

            int32_t current_a, current_d;
            read_encoder_counts(&current_a, &current_d);
            approach_a = current_a - approach_start_a;
            approach_d = -(current_d - approach_start_d);
            if (approach_a < 0) approach_a = 0;
            if (approach_d < 0) approach_d = 0;

            if (approach_a >= TURN_APPROACH_ENCODER_COUNTS &&
                approach_d >= TURN_APPROACH_ENCODER_COUNTS) {
                nav_state = NAV_SPIN_TURN;
                spin_frames = 0;
                spin_center = 0;
                next_hint_candidate = TURN_DIR_NONE;
                next_hint_confirm = 0;
                get_spin_pwm(pending_turn, &left_pwm, &right_pwm);
                drive(left_pwm, right_pwm);
                state_name = pending_turn == TURN_DIR_LEFT ? "TURN_LEFT" : "TURN_RIGHT";
                ESP_LOGW(TAG,
                         "approach complete: enc=A%ld/D%ld; start in-place turn: %s",
                         (long)approach_a, (long)approach_d,
                         turn_direction_name(pending_turn));
            } else if ((esp_timer_get_time() - approach_start_us) / 1000 >=
                       TURN_APPROACH_SAFETY_MS) {
                nav_state = NAV_TURN_FAILED;
                left_pwm = right_pwm = 0;
                drive(0, 0);
                state_name = "TURN_FAILED";
                ESP_LOGE(TAG,
                         "approach encoder timeout: enc=A%ld/D%ld target=%d; stop",
                         (long)approach_a, (long)approach_d,
                         TURN_APPROACH_ENCODER_COUNTS);
            }
        } else if (nav_state == NAV_SPIN_TURN) {
            /* 原地转弯不使用固定时间或角度，只有新线路在连续多帧中保持居中
             * 才会结束转弯。 */
            spin_frames++;
            get_spin_pwm(pending_turn, &left_pwm, &right_pwm);
            drive(left_pwm, right_pwm);
            state_name = pending_turn == TURN_DIR_LEFT ? "TURN_LEFT" : "TURN_RIGHT";

            /* 连续路口可能在本次转弯尚未结束时已经进入画面。
             * 暂存与当前转向相反且连续出现的提示，供REACQUIRED后的下一轮使用。 */
            if (line.turn_hint != TURN_DIR_NONE && line.turn_hint != pending_turn) {
                if (line.turn_hint == next_hint_candidate) {
                    if (next_hint_confirm < TURN_HINT_CONFIRM_FRAMES) next_hint_confirm++;
                } else {
                    next_hint_candidate = line.turn_hint;
                    next_hint_confirm = 1;
                }
            } else if (line.turn_hint == TURN_DIR_NONE) {
                next_hint_candidate = TURN_DIR_NONE;
                next_hint_confirm = 0;
            }

            if (line_centered_after_turn(&line)) {
                spin_center++;
            } else {
                spin_center = 0;
            }

            if (spin_center >= TURN_REACQUIRE_CONFIRM_FRAMES) {
                if (next_hint_confirm >= TURN_HINT_CONFIRM_FRAMES) {
                    queued_turn = next_hint_candidate;
                }
                nav_state = NAV_FOLLOW;
                pending_turn = TURN_DIR_NONE;
                hint_candidate = TURN_DIR_NONE;
                hint_confirm = 0;
                confirmed = CONFIRM_FRAMES;
                missing = 0;
                last_error = line.error;
                left_pwm = right_pwm = TURN_REACQUIRE_PWM;
                drive(left_pwm, right_pwm);
                state_name = "REACQUIRED";
                reacquired_now = true;
                ESP_LOGW(TAG, "new path centered; resume line following; queued=%s",
                         turn_direction_name(queued_turn));
            } else if (spin_frames >= TURN_SAFETY_MAX_FRAMES) {
                /* 此超时只用于安全停车，绝不会仅根据经过时间判定转弯成功；
                 * 超时后需要复位。 */
                nav_state = NAV_TURN_FAILED;
                left_pwm = right_pwm = 0;
                drive(0, 0);
                state_name = "TURN_FAILED";
                ESP_LOGE(TAG, "turn safety stop: new centered path not found");
            }
        } else {
            drive(0, 0);
            state_name = "TURN_FAILED";
        }

        car_display_set(left_pwm, 0, right_pwm,
                        display_state_from_navigation(state_name, &line, spin_center));

        if (++frame_no % 5 == 0 || reacquired_now) {
            ESP_LOGI(TAG,
                     "state=%s decode=%d area=%d x=N%d/M%d/F%d raw_err=%d ctrl_err=%d "
                     "hint=%s scan=%d ext=L%d/R%d turn=%s next=%s/%d enc=A%ld/D%ld "
                     "turn_ok=%d pwm=L%d/R%d motor=%s",
                     state_name, decoded, line.area,
                     line.near_x, line.mid_x, line.far_x,
                     line.found ? line.error : 9999, last_error,
                     turn_direction_name(line.turn_hint), line.scan_area,
                     line.left_extent, line.right_extent,
                     turn_direction_name(pending_turn),
                     turn_direction_name(next_hint_candidate), next_hint_confirm,
                     (long)approach_a, (long)approach_d, spin_center,
                     left_pwm, right_pwm,
                     MOTOR_OUTPUT_ENABLED ? "ON" : "DRY");
        }

        uvc_host_frame_return(s_stream, frame);
        /* JPEG解码占用较多CPU时间，主动让出处理器以便IDLE1喂看门狗。 */
        vTaskDelay(1);
    }

    drive(0, 0);
    car_display_set(0, 0, 0, CAR_DISPLAY_STOP);
    uvc_host_frame_t *pending;
    while (xQueueReceive(s_frame_q, &pending, 0) == pdPASS)
        uvc_host_frame_return(s_stream, pending);
    s_frame_task_running = false;
    vTaskDelete(NULL);
}

/* 打开指定的UVC图像格式，并在摄像头重新连接后恢复视频流。 */
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
        car_display_set(0, 0, 0, CAR_DISPLAY_STOP);
        while (s_frame_task_running) vTaskDelay(pdMS_TO_TICKS(10));
        esp_err_t close_err = uvc_host_stream_close(s_stream);
        if (close_err != ESP_OK) ESP_LOGW(TAG, "stream close failed: %s", esp_err_to_name(close_err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* 将新连接的UVC设备信息输出到应用日志。 */
static void uvc_event_callback(const uvc_host_driver_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED)
        ESP_LOGI(TAG, "camera connected, USB address=%d", event->device_connected.dev_addr);
}

/* 持续处理ESP-IDF底层USB主机库事件。 */
static void usb_lib_task(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

/* 分配视觉缓冲区，并启动电机、USB、UVC和摄像头处理任务。 */
void app_main(void)
{
    motors_init();
    encoders_init();
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
    /* 先启动摄像头和电机控制，再加载可选显示模块。 */
    esp_err_t display_error = car_display_init();
    if (display_error != ESP_OK) {
        ESP_LOGE(TAG, "display disabled after initialization failure: %s",
                 esp_err_to_name(display_error));
    }
}
