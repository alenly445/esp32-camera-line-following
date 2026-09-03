/*
 * ESP32-S3 摄像头帧驱动的直线走停测试。
 *
 * 本文件只保留当前工程已经调试过的：
 *   1. USB UVC 摄像头取帧；
 *   2. A、D 两个驱动轮直行；
 *   3. 行驶 2 帧、短刹车 2 帧的循环；
 *   4. 短刹车后恢复直行时持续 1 帧的启动增强。
 *
 * 不包含 JPEG 解码、黑线识别、转弯、Approach、避障和显示屏。
 * 该文件默认不参与编译，不能和 car_vision_main.c 同时编译，因为两者都有 app_main()。
 * 测试时，把 main/CMakeLists.txt 的 SRCS 临时改为：
 *     SRCS "straight_drive_test.c"
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#define CAM_W 640
#define CAM_H 480
#define CAM_FPS 25
#define FRAME_BUFFERS 3
#define FRAME_QUEUE_DEPTH 1
#define FRAME_SIZE (CAM_W * CAM_H * 2)
#define USB_PRIORITY 15

/* 以下数值直接取自当前 car_vision_main.c。 */
#define MOTOR_OUTPUT_ENABLED 1
#define BASE_PWM 44
#define START_BOOST_PWM 33
#define START_BOOST_FRAMES 1
#define TRACK_DRIVE_FRAMES 2
#define TRACK_PAUSE_FRAMES 2

/* 当前三轮小车电机接线；直线测试只驱动 A、D，B 轮保持停止。 */
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

static const char *TAG = "straight_test";
static QueueHandle_t s_frame_queue;
static uvc_host_stream_hdl_t s_stream;
static volatile bool s_connected;
static volatile bool s_control_task_running;
static bool s_brake_active;
static int s_start_boost_frames;

static int clampi(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

static void motor_channel_init(ledc_channel_t channel, int pin)
{
    const ledc_channel_config_t config = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

static void motors_init(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << STBY_PIN) |
                        (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                        (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN) |
                        (1ULL << DIN1_PIN) | (1ULL << DIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));
    gpio_set_level(STBY_PIN, 0);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    motor_channel_init(LEDC_CHANNEL_0, PWMA_PIN);
    motor_channel_init(LEDC_CHANNEL_1, PWMB_PIN);
    motor_channel_init(LEDC_CHANNEL_2, PWMD_PIN);
}

static void set_motor(ledc_channel_t channel, int in1, int in2, int pwm)
{
    pwm = clampi(pwm, -255, 255);
    gpio_set_level(in1, pwm > 0);
    gpio_set_level(in2, pwm < 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, abs(pwm)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

/* 与当前主程序一致：只有从短刹车恢复时才增加启动 PWM。 */
static void drive(int left, int right)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    if (left == 0 && right == 0) {
        s_start_boost_frames = 0;
        s_brake_active = false;
    } else if (s_brake_active) {
        s_start_boost_frames = START_BOOST_FRAMES;
        s_brake_active = false;
    }
    if (s_start_boost_frames > 0) {
        if (left) left += left > 0 ? START_BOOST_PWM : -START_BOOST_PWM;
        if (right) right += right > 0 ? START_BOOST_PWM : -START_BOOST_PWM;
        s_start_boost_frames--;
    }
    gpio_set_level(STBY_PIN, left != 0 || right != 0);
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, left);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, right);
}

static void brake_motor(ledc_channel_t channel, int in1, int in2)
{
    gpio_set_level(in1, 1);
    gpio_set_level(in2, 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void brake_drive(void)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    s_brake_active = true;
    gpio_set_level(STBY_PIN, 1);
    brake_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN);
    brake_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN);
    brake_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN);
}

/* 回调只把最新摄像头帧交给控制任务，不进行图像识别。 */
static bool frame_callback(const uvc_host_frame_t *frame, void *context)
{
    uvc_host_frame_t *mutable_frame = (uvc_host_frame_t *)frame;
    return xQueueSend((QueueHandle_t)context, &mutable_frame, 0) != pdPASS;
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *context)
{
    (void)context;
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

/* 每成功收到一帧，走停状态只前进一步，因此节奏由摄像头处理循环决定。 */
static void frame_control_task(void *argument)
{
    (void)argument;
    unsigned control_frame = 0;
    const unsigned cycle_frames = TRACK_DRIVE_FRAMES + TRACK_PAUSE_FRAMES;

    while (s_connected) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, pdMS_TO_TICKS(500)) != pdPASS) {
            drive(0, 0);
            ESP_LOGW(TAG, "no camera frame; motors stopped");
            continue;
        }

        unsigned phase = control_frame % cycle_frames;
        if (phase < TRACK_DRIVE_FRAMES) {
            drive(BASE_PWM, BASE_PWM);
            ESP_LOGI(TAG, "frame=%u state=DRIVE pwm=%d", control_frame, BASE_PWM);
        } else {
            brake_drive();
            ESP_LOGI(TAG, "frame=%u state=BRAKE", control_frame);
        }
        control_frame++;
        uvc_host_frame_return(s_stream, frame);
    }

    drive(0, 0);
    uvc_host_frame_t *pending = NULL;
    while (xQueueReceive(s_frame_queue, &pending, 0) == pdPASS)
        uvc_host_frame_return(s_stream, pending);
    s_control_task_running = false;
    vTaskDelete(NULL);
}

static void camera_task(void *argument)
{
    (void)argument;
    while (true) {
        const uvc_host_stream_config_t config = {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = s_frame_queue,
            .usb = {
                .vid = UVC_HOST_ANY_VID,
                .pid = UVC_HOST_ANY_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = CAM_W,
                .v_res = CAM_H,
                .fps = CAM_FPS,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = FRAME_BUFFERS,
                .frame_size = FRAME_SIZE,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
                .number_of_urbs = 6,
                .urb_size = 16 * 1024,
            },
        };

        ESP_LOGI(TAG, "opening UVC camera GPIO19=D- GPIO20=D+");
        if (uvc_host_stream_open(&config, pdMS_TO_TICKS(5000), &s_stream) != ESP_OK) {
            ESP_LOGW(TAG, "camera open failed; retrying");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        s_connected = true;
        ESP_ERROR_CHECK(uvc_host_stream_start(s_stream));
        ESP_LOGI(TAG, "camera streaming %dx%d@%d MJPEG", CAM_W, CAM_H, CAM_FPS);
        s_control_task_running = true;
        if (xTaskCreatePinnedToCore(frame_control_task, "frame_control", 4096, NULL,
                                   USB_PRIORITY - 2, NULL, 1) != pdPASS) {
            s_control_task_running = false;
            s_connected = false;
            drive(0, 0);
            ESP_LOGE(TAG, "failed to create frame control task");
        }

        while (s_connected) vTaskDelay(pdMS_TO_TICKS(200));
        while (s_control_task_running) vTaskDelay(pdMS_TO_TICKS(10));
        uvc_host_stream_close(s_stream);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void uvc_event_callback(const uvc_host_driver_event_data_t *event, void *context)
{
    (void)context;
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED)
        ESP_LOGI(TAG, "camera connected, USB address=%d",
                 event->device_connected.dev_addr);
}

static void usb_lib_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    }
}

void app_main(void)
{
    motors_init();
    s_frame_queue = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(uvc_host_frame_t *));
    assert(s_frame_queue != NULL);

    const usb_host_config_t usb_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&usb_config));
    assert(xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL,
                       USB_PRIORITY, NULL) == pdPASS);

    const uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = USB_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_config));

    ESP_LOGI(TAG,
             "camera-frame straight loop ready: drive=%d frames, brake=%d frames, pwm=%d",
             TRACK_DRIVE_FRAMES, TRACK_PAUSE_FRAMES, BASE_PWM);
    assert(xTaskCreatePinnedToCore(camera_task, "camera", 6144, NULL,
                                   USB_PRIORITY - 1, NULL, 0) == pdPASS);
}
