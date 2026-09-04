/*
 * ESP32-S3 摄像头帧驱动的原地转弯走停测试。
 *
 * 仅保留：USB UVC 摄像头取帧、A/D 两轮反向旋转、转弯启动增强，
 * 以及“旋转若干帧 -> 短刹车若干帧”的循环。
 * 不包含 JPEG 解码、黑线识别、自动退出转弯、Approach、避障和显示屏。
 *
 * 本文件默认不参与编译，不能和 car_vision_main.c 同时编译。
 * 测试时将 main/CMakeLists.txt 的 SRCS 临时改为：
 *     SRCS "in_place_turn_test.c"
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

/* 以下转弯参数直接取自当前 car_vision_main.c。 */
#define MOTOR_OUTPUT_ENABLED 1
#define TURN_START_BOOST_PWM 10
#define START_BOOST_FRAMES 1
#define CORNER_SEARCH_PWM 36
#define CORNER_ALIGN_PWM 34
#define CORNER_SEARCH_TURN_FRAMES 1
#define CORNER_SEARCH_PAUSE_FRAMES 3

/* +1 测试右转，-1 测试左转。 */
#define TEST_TURN_DIRECTION 1

/* 当前三轮小车电机接线；原地转弯只驱动 A、D，B 轮保持停止。 */
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

static const char *TAG = "turn_test";
static QueueHandle_t s_frame_queue;
static uvc_host_stream_hdl_t s_stream;
static volatile bool s_connected;
static volatile bool s_control_task_running;
static int s_turn_start_boost_frames;

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

static void stop_drive(void)
{
    s_turn_start_boost_frames = 0;
    gpio_set_level(STBY_PIN, 0);
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, 0);
}

/* pulse_start=true 时，为本次旋转脉冲的第一帧增加独立转弯 Boost。 */
static void drive_turn(int steering, bool pulse_start)
{
    int left = steering;
    int right = -steering;
    if (!MOTOR_OUTPUT_ENABLED) return;

    if (pulse_start) s_turn_start_boost_frames = START_BOOST_FRAMES;
    if (s_turn_start_boost_frames > 0) {
        if (left) left += left > 0 ? TURN_START_BOOST_PWM : -TURN_START_BOOST_PWM;
        if (right) right += right > 0 ? TURN_START_BOOST_PWM : -TURN_START_BOOST_PWM;
        s_turn_start_boost_frames--;
    }

    gpio_set_level(STBY_PIN, 1);
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
    gpio_set_level(STBY_PIN, 1);
    brake_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN);
    brake_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN);
    brake_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN);
}

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
        stop_drive();
        ESP_LOGW(TAG, "camera disconnected; motors stopped");
    } else if (event->type == UVC_HOST_FRAME_BUFFER_OVERFLOW) {
        ESP_LOGW(TAG, "camera frame overflow");
    } else if (event->type == UVC_HOST_FRAME_BUFFER_UNDERFLOW) {
        ESP_LOGW(TAG, "camera frame buffer underflow");
    }
}

/* 每收到一帧推进一次原地转弯走停循环。 */
static void frame_control_task(void *argument)
{
    (void)argument;
    unsigned control_frame = 0;
    const unsigned cycle_frames =
        CORNER_SEARCH_TURN_FRAMES + CORNER_SEARCH_PAUSE_FRAMES;

    while (s_connected) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_queue, &frame, pdMS_TO_TICKS(500)) != pdPASS) {
            stop_drive();
            ESP_LOGW(TAG, "no camera frame; motors stopped");
            continue;
        }

        unsigned phase = control_frame % cycle_frames;
        if (phase < CORNER_SEARCH_TURN_FRAMES) {
            int steering = TEST_TURN_DIRECTION * CORNER_SEARCH_PWM;
            drive_turn(steering, phase == 0);
            ESP_LOGI(TAG, "frame=%u state=TURN_%s pwm=%d boost=%d",
                     control_frame,
                     TEST_TURN_DIRECTION < 0 ? "LEFT" : "RIGHT",
                     CORNER_SEARCH_PWM, TURN_START_BOOST_PWM);
        } else {
            brake_drive();
            ESP_LOGI(TAG, "frame=%u state=TURN_CHECK", control_frame);
        }

        control_frame++;
        uvc_host_frame_return(s_stream, frame);
    }

    stop_drive();
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
            stop_drive();
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
             "camera-frame turn loop ready: direction=%s turn=%d frames pause=%d frames",
             TEST_TURN_DIRECTION < 0 ? "LEFT" : "RIGHT",
             CORNER_SEARCH_TURN_FRAMES, CORNER_SEARCH_PAUSE_FRAMES);
    assert(xTaskCreatePinnedToCore(camera_task, "camera", 6144, NULL,
                                   USB_PRIORITY - 1, NULL, 0) == pdPASS);
}
