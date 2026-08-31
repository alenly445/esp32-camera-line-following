# ESP32-S3 单摄像头黑线巡线

本仓库是小车摄像头巡线的精简共享版本。JQ-CAM12 通过 USB Host 连接 ESP32-S3，由芯片本地完成 MJPEG 解码、黑线检测、前视预测和 TB6612 电机控制；正常运行不依赖电脑或 Python。

## 目录

```text
esp32_car_vision/          ESP-IDF 摄像头巡线主工程
  main/car_vision_main.c   识别、预测、状态机和电机控制
  main/tjpgd.*             Tiny JPEG Decoder
  README.md                编译、烧录和调参说明
test/line_detect.py        电脑端 OpenCV 阈值标定工具
v20_line_car_vscode.ino    原 v20 小车控制程序，保留作参考
摄像头型号与使用说明.md     摄像头接线与硬件记录
```

## 当前硬件映射

- 摄像头：`GPIO19=D-`、`GPIO20=D+`，另接稳定的 `5V/GND`
- TB6612：`STBY=GPIO10`
- 左前 A：`PWM/IN1/IN2 = GPIO11/12/13`
- 后轮 B：`GPIO14/15/16`，摄像头巡线时保持停止
- 右前 D：`PWM/IN1/IN2 = GPIO17/18/21`

## 快速开始

安装 ESP-IDF 5.4.x，在 ESP-IDF PowerShell 中运行：

```powershell
cd esp32_car_vision
idf.py set-target esp32s3
idf.py build
idf.py -p COM7 flash monitor
```

退出串口监视器按 `Ctrl+]`。固件当前启用了电机，烧录或复位前必须将小车架空。具体调参方法见 [esp32_car_vision/README.md](esp32_car_vision/README.md)。

## 当前控制策略

- 灰度阈值分割黑线，只选择与画面底部相连的主要区域
- 近、中、远三个观察区联合估计方向
- 连续多帧确认后驱动
- 短时丢线保持最后方向，超过限制立即停车
- 提前识别左/右直角分支并记忆方向，进入摄像头盲区后原地转向
- 不按固定转弯时间退出；连续确认新线居中后恢复巡线
- 当前实车调参值以 `car_vision_main.c` 顶部宏定义为准

## 安全说明

首次使用或修改转向参数后，必须先架空检查左右轮方向和 `LOST_STOP`。摄像头断线、电机方向异常或串口出现崩溃日志时，不要直接落地运行。
