# 阶段 B：ESP32-S3 UVC 取流测试（esp32_uvc_test）

用 ESP32-S3 当 USB 主机，直接枚举并读取 JQ-CAM12-720P 摄像头。这是路线 A 的第一个里程碑：**只验证"能枚举、能出流"，不控制小车、不解码图像**。

## 硬件准备（先断电再接线）

| 摄像头线 | 接到 | 说明 |
|---|---|---|
| 5V | 开发板 5V | 别接 3.3V |
| GND | GND | 共地 |
| D- | GPIO19 | |
| D+ | GPIO20 | |

同时：
1. **红外板从 GPIO4~7 拔掉**（TRIG 已挪到 6，会顶牛）
2. 超声波已换到 `TRIG=6 / ECHO=7`（v26，Arduino 固件对应已改）
3. **本测试只插 USB（COM7）供电，别接电池驱动电机**——USB 口 500mA 要同时喂板子和摄像头

## 编译烧录（两种方式）

**方式一：VSCode 插件（推荐）**
用 VSCode 单独打开 `esp32_uvc_test` 文件夹，ESP-IDF 插件会自动识别，点底部状态栏的 🔨Build → ⚡Flash → 🖥️Monitor（端口选 COM7）。

**方式二：命令行**（PowerShell，需要 ESP-IDF 环境，可用插件创建的 "ESP-IDF PowerShell" 终端）：

```powershell
cd esp32_uvc_test
idf.py set-target esp32s3     # 仅第一次
idf.py build
idf.py -p COM7 flash monitor  # 退出 monitor: Ctrl+]
```

首次 build 会自动从组件仓库下载 `espressif/usb_host_uvc`（需要网络）。

## 怎么判断结果

| 串口输出 | 含义 |
|---|---|
| `Camera connected, addr N` | USB 枚举成功——**接线正确、摄像头活着**（阶段 B 第一关） |
| `--- camera supported formats ---` + 一串列表 | 摄像头报告的格式清单（记下来，这就是 ESP32 版的 step2 结果） |
| `chosen: [n] 1280x720 @30fps` | 自动选中了 720p MJPEG |
| `frame 1: 1280x720 len=xxxxx` | **真实出流了**，len 是每帧 MJPEG 字节数 |
| `window 1 summary: 14x frames (28.x fps), avg 45 KB/frame` | 5 秒窗口统计，fps 接近标称即稳定 |
| 一直 `Open failed (no camera?), retry in 5s` | 枚举失败：查 5V/GND 是否接对、D± 是否接反、红外板是否已拔 |

## 常见故障

- **完全没有 `Camera connected`**：5V/GND 没接好或摄像头没上电；用万用表量摄像头 5V 线上有没有 5V
- **枚举失败/反复重试**：D+ 和 D− 接反（对调试试）；杜邦线太长（换短线）
- **`no frame within 1s` 反复出现**：帧缓冲不够或供电不稳，先换 USB 口
- **monitor 打不开 COM7**：先关掉 Arduino 的串口监视器，串口不能同时被两个程序占用

## 通过之后

把串口里的**格式列表**和**窗口统计**发我 → 决定巡线用哪个分辨率/帧率 → 下一步搭"Arduino 作为 IDF 组件"的正式工程，把小车 v25 控制代码迁进来。
