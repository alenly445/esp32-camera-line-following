# WiFi 实时画面：esp32_uvc_stream

ESP32-S3 从 JQ-CAM12 取 MJPEG 流，通过 WiFi 提供网页视频流。电脑/手机浏览器直接看画面。**这也是以后调巡线的主力工具。**

实测帧率取决于 WiFi 环境：手机热点宿舍环境约 5~8fps（640x480，峰值 ~500KB/s）；信号好的路由器可到 10~15fps。固件已开大 TCP 窗口（23KB）并关闭 WiFi 省电，这两个是流媒体必做项。想要更高帧率把 `CAM_WIDTH/HEIGHT` 改成 480x320（帧体积约减半，帧率约翻倍）。

## 首次使用：填 WiFi

编辑 [main/wifi_credentials.h](main/wifi_credentials.h)（该文件在 .gitignore 里，不会把密码推上 GitHub；模板见 `wifi_credentials.example.h`）。两种模式：

- **校园网 802.1X**（Tsinghua-Secure 等）：`WIFI_USE_ENTERPRISE = 1`，填学号和校园网密码（即登录 net.tsinghua.edu.cn 的密码）。固件用 PEAP/MSCHAPv2 认证
- **家庭 WiFi / 手机热点**：`WIFI_USE_ENTERPRISE = 0`，填 `WIFI_SSID` + `WIFI_PASSWORD`（必须是 2.4GHz 频段）

改完重新烧录。

## 编译烧录

用 VSCode 的 ESP-IDF 插件单独打开本文件夹（或 ESP-IDF PowerShell）：

```powershell
idf.py set-target esp32s3     # 仅第一次
idf.py build
idf.py -p COM7 flash monitor
```

上电后串口会打印：

```
I (xxxx) uvc_stream: ========================================
I (xxxx) uvc_stream:   WiFi connected! IP: 192.168.1.23
I (xxxx) uvc_stream:   Browser open: http://192.168.1.23/
I (xxxx) uvc_stream: ========================================
```

## 看画面

电脑（连同一个 WiFi）浏览器打开串口里显示的地址：

| 地址 | 用途 |
|---|---|
| `http://<IP>/` | 带播放器的页面 |
| `http://<IP>/stream` | 裸 MJPEG 流（可直接贴进 `<img>` 标签） |
| `http://<IP>/snapshot` | 单帧 JPEG（写脚本抓图方便） |

同时只支持一个观看客户端；第二个打开 `/stream` 的会显示失败，关掉第一个再开。

## 改分辨率

[main/uvc_stream_main.c](main/uvc_stream_main.c) 顶部 `CAM_WIDTH / CAM_HEIGHT / CAM_FPS`。摄像头实测支持：1280x720@15、800x480@20、640x480@25（默认，巡线甜点档）、480x320@25、480x854@25。

## 常见问题

- **一直 "WiFi lost, reconnecting"**：检查 SSID/密码；确认路由器是 2.4GHz；有些路由器把新设备隔离在访客网络，看路由器后台
- **IP 打不开**：电脑和板子必须连**同一个**路由器；公司/学校 WiFi 常开设备隔离，家里的一般没问题
- **`Open failed` 反复重试**：摄像头没接好（19/20/5V/GND）
- **画面卡顿**：正常现象与 WiFi 信号相关；`/snapshot` 不受影响
