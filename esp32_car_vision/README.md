# ESP32-S3 摄像头巡黑线 + 超声避障

这是新的 ESP-IDF 工程，取代红外巡线模块。JQ-CAM12 通过 ESP32-S3 USB Host 接入，图像在板端解码、识别黑线并输出差速控制；HC-SR04 继续负责近距离避障。运行时不依赖电脑。

## 接线

- JQ-CAM12：`D- -> GPIO19`、`D+ -> GPIO20`、稳定 `5V`、与主板共地。
- HC-SR04：`TRIG -> GPIO7`、`ECHO -> GPIO6`、共地。
- TB6612：`STBY=10`；左 A `PWM/IN1/IN2=11/12/13`；后 B `14/15/16` 始终停止；右 D `17/18/21`。

红外巡线模块不再参与控制，应从 `GPIO4~7` 断开。`GPIO6/7` 现在仅供超声波使用。

## 工作逻辑

- MJPEG 解码为 `320x240` 灰度图，在画面下方寻找与近端区域连通的最大黑色区域。
- 近、中、远三段黑线中心按 `60% / 25% / 15%` 合成转向偏差。
- 连续 `3` 帧识别到线后进入 `TRACK`；短时丢线保持上一方向 `6` 帧，之后 `LOST_STOP`。
- 超声波距离小于 `15 cm` 时，巡线控制让位给避障状态机：原地转向 `1.2 s`，低速检查 `0.6 s`，再恢复摄像头巡线。
- 摄像头拔出、视频流异常或长期丢线时，电机停车。

## 首次验证

固件默认不驱动电机：`main/car_vision_main.c` 中的 `MOTOR_OUTPUT_ENABLED` 为 `0`。先将车轮架空，编译、烧录并观察串口：

```bash
cd "/Users/mengyangzu/Documents/ChatGPT/ardui/esp32_car_vision"
./idf-run.sh build
./idf-run.sh -p /dev/cu.usbserial-0001 flash monitor
```

正常状态示例：

```text
camera connected, USB address=...
camera streaming 640x480@25 MJPEG
state=TRACK decode=1 area=... x=近/中/远 error=... distance=... motor=DRY
```

黑线位于画面左侧时 `error` 应为负数，右侧应为正数。遮挡黑线后，应依次出现 `BLIND_HOLD` 和 `LOST_STOP`。退出监视器按 `Ctrl+]`。

## 实时画面与识别标注

烧录并运行后，电脑连接开发板建立的 Wi-Fi：

```text
Wi-Fi 名称：ESP32-CarVision
密码：linecar123
```

连接成功后，在电脑浏览器打开：

```text
http://192.168.4.1
```

页面显示 640x480 的实时摄像头画面，并同时叠加：黄色巡线 ROI、近中远三条扫描带、三段黑线中心点、画面中心线、方向目标线、避障/巡线状态和超声距离。图像通过 Wi-Fi MJPEG 传输，标注数据由开发板独立发送；网络较慢时不会阻塞摄像头帧回收或板端巡线控制。

建议只开一个浏览器页面观看视频。浏览器显示 `NO_FRAME` 时，先确认 JQ-CAM12 的 USB 供电、`GPIO19=D-` 和 `GPIO20=D+`，再检查串口是否已出现 `camera streaming 640x480@25 MJPEG`。

## 开启电机

仅在上述识别方向、电机方向和超声距离全部确认后，将：

```c
#define MOTOR_OUTPUT_ENABLED 0
```

改为：

```c
#define MOTOR_OUTPUT_ENABLED 1
```

重新执行 `build` 和 `flash monitor`。首次落地测试建议把 `BASE_PWM` 从 `45` 调低到 `32`。若右转/左转与图像偏差相反，架空状态下先交换该侧电机方向或修改 `drive()` 映射，不要直接落地运行。
