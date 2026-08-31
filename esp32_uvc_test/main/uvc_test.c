/*
 * 阶段B：ESP32-S3 UVC 取流最小验证（JQ-CAM12-720P 接 GPIO19=D- / GPIO20=D+）
 *
 * 基于 espressif/esp-usb 的 basic_uvc_stream 示例（Apache-2.0）改造：
 *   - 单路 MJPEG 流；上线时自动挑 1280x720（没有则挑面积最大的 MJPEG 格式）
 *   - 摄像头接入时打印它报告的全部支持格式（相当于电脑端 step2 的 ESP32 版）
 *   - 以 5 秒为窗口统计实测帧率、平均帧大小、带宽，判断取流是否稳定
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#define USB_HOST_PRIORITY   (15)
#define FRAME_BUFFERS       (3)
#define WINDOW_SECONDS      (5)

static const char *TAG = "uvc_test";

static QueueHandle_t frame_q;
static bool dev_connected = false;

/* 摄像头上报的格式列表与选中项 */
static uint16_t sel_h = 0, sel_v = 0;
static float sel_fps = 15.0f;

static const char *FORMAT_STR[] = {
    "UNDEFINED", "MJPEG", "YUY2", "H264", "H265",
};

/* 帧回调：把帧指针丢进队列，交给处理任务；队列满则立刻退还丢帧 */
bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    QueueHandle_t q = *((QueueHandle_t *)user_ctx);
    if (xQueueSendToBack(q, &frame, 0) != pdPASS) {
        return true;    /* 本帧不处理，立即归还 */
    }
    return false;       /* 所有权移交，稍后 uvc_host_frame_return() */
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB transfer error, errno = %i", event->transfer_error.error);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Camera disconnected");
        dev_connected = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "Frame buffer underflow");
        break;
    default:
        ESP_LOGW(TAG, "Unexpected stream event: %d", event->type);
        break;
    }
}

/* USB 库系统事件任务（固定写法） */
static void usb_lib_task(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            /* 继续处理事件，允许设备重新接入 */
        }
    }
}

static void frame_handling_task(void *arg)
{
    (void)arg;
    while (1) {
        /* 每次打开前重建配置：断线重连后能拿到最新选中的格式 */
        uvc_host_stream_config_t stream_config = {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = &frame_q,
            .usb = {
                .vid = UVC_HOST_ANY_VID,
                .pid = UVC_HOST_ANY_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = sel_h,
                .v_res = sel_v,
                .fps = sel_fps,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = FRAME_BUFFERS,
                .frame_size = 0,            /* 按格式自动 */
                .number_of_urbs = 4,
                .urb_size = 10 * 1024,
            },
        };

        uvc_host_stream_hdl_t stream = NULL;
        ESP_LOGI(TAG, "Opening UVC stream: MJPEG %ux%u @%.1ffps ...", sel_h, sel_v, sel_fps);
        if (uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(5000), &stream) != ESP_OK) {
            ESP_LOGW(TAG, "Open failed (no camera?), retry in 5s ...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        ESP_LOGI(TAG, "Camera OPENED");
        dev_connected = true;

        unsigned iter = 0;
        while (dev_connected) {
            iter++;
            ESP_LOGI(TAG, "=== window %u: streaming for %ds ===", iter, WINDOW_SECONDS);
            uvc_host_stream_start(stream);

            uint32_t frames = 0, bytes = 0;
            TimeOut_t timeout;
            vTaskSetTimeOutState(&timeout);
            TickType_t remaining = pdMS_TO_TICKS(WINDOW_SECONDS * 1000);
            while (xTaskCheckForTimeOut(&timeout, &remaining) == pdFALSE) {
                uvc_host_frame_t *frame;
                if (xQueueReceive(frame_q, &frame, pdMS_TO_TICKS(1000)) == pdPASS) {
                    frames++;
                    bytes += frame->data_len;
                    if (frames % 30 == 1) {   /* 每30帧打印一行，避免刷屏 */
                        ESP_LOGI(TAG, "frame %u: %ux%u len=%u", (unsigned)frames,
                                 frame->vs_format.h_res, frame->vs_format.v_res,
                                 (unsigned)frame->data_len);
                    }
                    uvc_host_frame_return(stream, frame);
                } else if (!dev_connected) {
                    break;                  /* 流被断开回调关闭 */
                } else {
                    ESP_LOGW(TAG, "no frame within 1s ...");
                }
            }

            if (dev_connected) {
                uvc_host_stream_stop(stream);
                float fps = (float)frames / WINDOW_SECONDS;
                float mbps = (float)bytes * 8.0f / WINDOW_SECONDS / 1e6f;
                ESP_LOGI(TAG, "window %u summary: %u frames (%.1f fps), avg %u KB/frame, %.2f Mbit/s",
                         iter, (unsigned)frames, fps,
                         frames ? (unsigned)(bytes / frames / 1024) : 0, mbps);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
        ESP_LOGW(TAG, "waiting for camera reconnection ...");
    }
}

static void uvc_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED: {
        ESP_LOGI(TAG, "Camera connected, addr %d", event->device_connected.dev_addr);
        size_t list_size = event->device_connected.frame_info_num;
        uvc_host_frame_info_t *list = calloc(list_size, sizeof(uvc_host_frame_info_t));
        assert(list);
        uvc_host_get_frame_list(event->device_connected.dev_addr, 0,
                                (uvc_host_frame_info_t (*)[])list, &list_size);

        ESP_LOGI(TAG, "--- camera supported formats (stream 0) ---");
        int chosen_exact = -1, chosen_biggest = -1;
        uint32_t area_max = 0;
        for (int i = 0; i < (int)list_size; i++) {
            uvc_host_frame_info_t *fi = &list[i];
            float fps = fi->default_interval ? 10000000.0f / fi->default_interval : 0.0f;
            ESP_LOGI(TAG, "  [%d] %-9s %ux%u @%.1ffps", i, FORMAT_STR[fi->format],
                     fi->h_res, fi->v_res, fps);
            if (fi->format == UVC_VS_FORMAT_MJPEG) {
                if (fi->h_res == 1280 && fi->v_res == 720) {
                    chosen_exact = i;
                }
                uint32_t area = (uint32_t)fi->h_res * fi->v_res;
                if (area > area_max) {
                    area_max = area;
                    chosen_biggest = i;
                }
            }
        }
        int chosen = (chosen_exact >= 0) ? chosen_exact
                     : (chosen_biggest >= 0) ? chosen_biggest : 0;
        sel_h = list[chosen].h_res;
        sel_v = list[chosen].v_res;
        sel_fps = list[chosen].default_interval
                  ? 10000000.0f / list[chosen].default_interval : 15.0f;
        ESP_LOGI(TAG, "chosen: [%d] %ux%u @%.1ffps", chosen, sel_h, sel_v, sel_fps);
        free(list);

        static bool task_started = false;
        if (!task_started) {
            task_started = true;
            BaseType_t ok = xTaskCreatePinnedToCore(frame_handling_task, "uvc_frame", 4096,
                                                    NULL, USB_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
            assert(ok == pdTRUE);
        }
        break;
    }
    default:
        break;
    }
}

void app_main(void)
{
    frame_q = xQueueCreate(FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
    assert(frame_q);

    ESP_LOGI(TAG, "Installing USB Host (GPIO19=D- GPIO20=D+, camera needs 5V+GND too)");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    BaseType_t ok = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096,
                                            NULL, USB_HOST_PRIORITY, NULL, tskNO_AFFINITY);
    assert(ok == pdTRUE);

    ESP_LOGI(TAG, "Installing UVC driver");
    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = USB_HOST_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_event_cb,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));
}
