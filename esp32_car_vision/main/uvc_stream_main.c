/*
 * WiFi MJPEG 实时视频流：JQ-CAM12-720P 接 GPIO19=D- / GPIO20=D+，浏览器看画面
 *
 * 浏览器访问：
 *   http://<板子的IP>/          带播放器的页面
 *   http://<板子的IP>/stream    裸 MJPEG 流（10~25fps）
 *   http://<板子的IP>/snapshot  单帧 JPEG 快照
 *
 * 流程：UVC 取流(640x480@25 MJPEG) → 有浏览器客户端时把最新帧经 WiFi 发出，
 * 没有客户端时立即丢帧（省带宽省内存）。只支持一个观看客户端。
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_eap_client.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "wifi_credentials.h"

#define USB_HOST_PRIORITY   (15)
#define FRAME_BUFFERS       (5)

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

/* 巡线甜点档位；想要别的分辨率改这里（摄像头支持：1280x720@15 / 800x480@20 / 640x480@25 / 480x320@25） */
#define CAM_WIDTH           (640)
#define CAM_HEIGHT          (480)
#define CAM_FPS             (25.0f)

static const char *TAG = "uvc_stream";

static QueueHandle_t s_frame_q;         /* 驱动 -> 帧任务 */
static QueueHandle_t s_send_q;          /* 帧任务 -> HTTP 线程（仅在有观众时使用） */
static volatile int s_clients = 0;      /* 正在观看的浏览器数量 */
static volatile bool s_stream_ok = false;
static volatile uint32_t s_rx_count = 0, s_send_count = 0;
static uvc_host_stream_hdl_t s_stream = NULL;

static EventGroupHandle_t s_wifi_events;

/* 单帧 HTTP 发送缓冲（边界行+头+JPEG 拼成一整包，避免 Nagle 小包 stalls） */
#define PART_BUF_SIZE   (96 * 1024)
static uint8_t *s_part_buf;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static const char *RESP_HTML =
    "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
    "<title>ESP32-S3 摄像头</title>"
    "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0}"
    "img{max-width:100%;max-height:92vh}p a{color:#8cf}</style></head><body>"
    "<h3>ESP32-S3 + JQ-CAM12 实时画面 " STRINGIFY(CAM_WIDTH) "x" STRINGIFY(CAM_HEIGHT) "@25</h3>"
    "<img src=\"/stream\" alt=\"live\">"
    "<p><a href=\"/snapshot\">单帧快照 /snapshot</a></p></body></html>";

/* ---------------- UVC 取流部分（与 esp32_uvc_test 相同模式） ---------------- */

bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    QueueHandle_t q = *((QueueHandle_t *)user_ctx);
    /* 与已验证的取流测试相同：帧永远先进队列，由帧任务统一处理 */
    if (xQueueSendToBack(q, &frame, 0) != pdPASS) {
        return true;    /* 队列满，丢最慢的帧 */
    }
    return false;       /* 所有权移交，稍后归还 */
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
        s_stream_ok = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "Frame buffer underflow");
        break;
    default:
        break;
    }
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void frame_handling_task(void *arg)
{
    (void)arg;
    while (1) {
        uvc_host_stream_config_t stream_config = {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = &s_frame_q,
            .usb = {
                .vid = UVC_HOST_ANY_VID,
                .pid = UVC_HOST_ANY_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = CAM_WIDTH,
                .v_res = CAM_HEIGHT,
                .fps = CAM_FPS,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = FRAME_BUFFERS,
                .frame_size = 0,            /* 按格式自动 */
                .number_of_urbs = 4,
                .urb_size = 10 * 1024,
            },
        };

        ESP_LOGI(TAG, "Opening camera %ux%u@%.0f MJPEG ...", CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
        if (uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(5000), &s_stream) != ESP_OK) {
            ESP_LOGW(TAG, "Open failed (camera plugged into GPIO19/20?), retry in 5s ...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        ESP_LOGI(TAG, "Camera OPENED");
        s_stream_ok = true;
        uvc_host_stream_start(s_stream);

        TickType_t last_stat = xTaskGetTickCount();
        while (s_stream_ok) {
            uvc_host_frame_t *frame;
            if (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(100)) == pdPASS) {
                s_rx_count++;
                if (s_clients > 0 && xQueueSendToBack(s_send_q, &frame, 0) == pdPASS) {
                    s_send_count++;     /* 所有权转给 HTTP 线程 */
                } else if (s_stream_ok) {
                    uvc_host_frame_return(s_stream, frame);     /* 没观众或发送队列满：丢帧 */
                }
            }
            /* 观众刚走：清空积压的发送队列，防止帧缓冲泄漏 */
            while (s_clients == 0 && xQueueReceive(s_send_q, &frame, 0) == pdPASS) {
                if (s_stream_ok) {
                    uvc_host_frame_return(s_stream, frame);
                }
            }
            if (xTaskGetTickCount() - last_stat >= pdMS_TO_TICKS(5000)) {
                last_stat = xTaskGetTickCount();
                ESP_LOGI(TAG, "stat: stream_ok=%d clients=%d rx=%u sent=%u",
                         (int)s_stream_ok, s_clients,
                         (unsigned)s_rx_count, (unsigned)s_send_count);
            }
        }
        ESP_LOGW(TAG, "waiting for camera reconnection ...");
    }
}

static void uvc_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "Camera connected, addr %d", event->device_connected.dev_addr);
        /* 取流生命周期由 frame_handling_task 管理 */
    }
}

/* ---------------- HTTP 服务 ---------------- */

#define STREAM_BOUNDARY "frameboundary"

static esp_err_t handler_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, RESP_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handler_snapshot(httpd_req_t *req)
{
    s_clients++;
    uvc_host_frame_t *frame = NULL;
    esp_err_t ret = ESP_FAIL;
    if (xQueueReceive(s_send_q, &frame, pdMS_TO_TICKS(2000)) == pdPASS) {
        httpd_resp_set_type(req, "image/jpeg");
        ret = httpd_resp_send(req, (const char *)frame->data, frame->data_len);
        if (s_stream_ok) {
            uvc_host_frame_return(s_stream, frame);
        }
    } else {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        ret = httpd_resp_send(req, "no frame (camera not streaming?)", HTTPD_RESP_USE_STRLEN);
    }
    s_clients--;
    return ret;
}

static esp_err_t handler_stream(httpd_req_t *req)
{
    s_clients++;
    ESP_LOGI(TAG, "Browser client connected (%d watching)", s_clients);

    /* 必须用 chunked API：它会自动先发 HTTP 状态行+响应头，再逐块发数据 */
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=" STREAM_BOUNDARY);
    /* 关闭 Nagle：小包立即发出，否则和延迟 ACK 互相等待会把帧率拖到 ~1fps */
    int sockfd = httpd_req_to_sockfd(req);
    int nodelay = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    esp_err_t ret = ESP_OK;

    while (ret == ESP_OK && s_stream_ok) {
        uvc_host_frame_t *frame;
        if (xQueueReceive(s_send_q, &frame, pdMS_TO_TICKS(5000)) != pdPASS) {
            break;  /* 5 秒没帧，断开这个客户端 */
        }
        int n = snprintf((char *)s_part_buf, PART_BUF_SIZE,
                         "--" STREAM_BOUNDARY "\r\n"
                         "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                         (unsigned)frame->data_len);
        if (n > 0 && (size_t)n + frame->data_len + 2 <= PART_BUF_SIZE) {
            memcpy(s_part_buf + n, frame->data, frame->data_len);
            n += frame->data_len;
            s_part_buf[n++] = '\r';
            s_part_buf[n++] = '\n';
            ret = httpd_resp_send_chunk(req, (const char *)s_part_buf, n);
        } else {
            ret = ESP_ERR_NO_MEM;
        }
        if (s_stream_ok) {
            uvc_host_frame_return(s_stream, frame);
        }
    }

    httpd_resp_send_chunk(req, NULL, 0);    /* 结束 chunked 响应，关闭连接 */
    s_clients--;
    ESP_LOGI(TAG, "Browser client left (%d watching)", s_clients);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;          /* 发大帧需要足够的任务栈 */
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_index =    { .uri = "/",         .method = HTTP_GET, .handler = handler_index };
        httpd_uri_t uri_stream =   { .uri = "/stream",   .method = HTTP_GET, .handler = handler_stream };
        httpd_uri_t uri_snapshot = { .uri = "/snapshot", .method = HTTP_GET, .handler = handler_snapshot };
        httpd_register_uri_handler(server, &uri_index);
        httpd_register_uri_handler(server, &uri_stream);
        httpd_register_uri_handler(server, &uri_snapshot);
        return server;
    }
    return NULL;
}

/* ---------------- WiFi ---------------- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi lost, reason=%d, reconnecting ...", disc->reason);
        esp_wifi_connect();
        xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  WiFi connected! IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
        ESP_LOGI(TAG, "  Browser open: http://" IPSTR "/", IP2STR(&ip_event->ip_info.ip));
        ESP_LOGI(TAG, "========================================");
        xEventGroupClearBits(s_wifi_events, WIFI_FAIL_BIT);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .threshold.authmode = WIFI_AUTH_WPA_PSK,
        },
    };
#if WIFI_USE_ENTERPRISE
    /* 802.1X 企业网（Tsinghua-Secure 等）：PEAP + MSCHAPv2，学号+校园网密码 */
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE;
    ESP_ERROR_CHECK(esp_eap_client_set_identity((const unsigned char *)EAP_IDENTITY, strlen(EAP_IDENTITY)));
    ESP_ERROR_CHECK(esp_eap_client_set_username((const unsigned char *)EAP_USERNAME, strlen(EAP_USERNAME)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((const unsigned char *)EAP_PASSWORD, strlen(EAP_PASSWORD)));
    ESP_ERROR_CHECK(esp_eap_client_set_eap_methods(ESP_EAP_TYPE_PEAP));
    /* ESP32 没有 RTC 电池，上电时间是 1970 年，关掉证书时间校验避免 PEAP 因此失败 */
    ESP_ERROR_CHECK(esp_eap_client_set_disable_time_check(true));
    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
    ESP_LOGI(TAG, "WiFi mode: 802.1X enterprise (PEAP/MSCHAPv2), identity \"%s\"", EAP_IDENTITY);
#else
    strlcpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    ESP_LOGI(TAG, "WiFi mode: WPA-PSK");
#endif
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));     /* 关省电，否则推流吞吐掉一个数量级 */
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi \"%s\" ...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "WiFi not connected yet (check SSID/password in main/wifi_credentials.h). Will keep retrying.");
    }
}

/* ---------------- app_main ---------------- */

void app_main(void)
{
    /* NVS：WiFi 校准数据要存这里 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Installing USB Host (camera on GPIO19=D- GPIO20=D+)");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    BaseType_t ok = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096,
                                            NULL, USB_HOST_PRIORITY, NULL, tskNO_AFFINITY);
    assert(ok == pdTRUE);

    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = USB_HOST_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_event_cb,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));

    s_frame_q = xQueueCreate(FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
    assert(s_frame_q);
    s_send_q = xQueueCreate(2, sizeof(uvc_host_frame_t *));
    assert(s_send_q);
    s_part_buf = heap_caps_malloc(PART_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_part_buf);
    ok = xTaskCreatePinnedToCore(frame_handling_task, "uvc_frame", 4096,
                                 NULL, USB_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
    assert(ok == pdTRUE);

    wifi_init();
    start_webserver();
}
