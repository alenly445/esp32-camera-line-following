# ESP32-S3 红球 -> 左侧蓝色球门

这是一个独立的 ESP-IDF 测试工程，只实现红球进入左前方蓝色球门。

## 逻辑

- 找球、接近球、收球和找球门阶段完全不使用巡线逻辑。
- 远距离以红球中心对准摄像头中心线。
- 红球接近铲斗后，允许从画面消失；红球消失且超声距离不大于 10 cm，连续确认后认为红球已进入铲斗。
- 收球后持续监测超声近距离状态。
- 找球门时搜索左侧蓝色区域，并直接把蓝色区域作为球门入口。
- 已删除 `GOAL_FINAL_ALIGN`，直接执行 `GOAL_ALIGN -> GOAL_APPROACH -> GOAL_PUSH`。
- `GOAL_VERIFY` 当前只停车短暂保持，然后默认进球成功，进入 `ALL_DONE`。

## 接线

- JQ-CAM12：`D- -> GPIO19`、`D+ -> GPIO20`、稳定 5V、共地。
- TB6612：`STBY=10`；左 A `PWM/IN1/IN2=11/12/13`；右 D `17/18/21`。
- 当前先复用现有单路超声：`TRIG=GPIO7`、`ECHO=GPIO6`。
- 后部 B 电机通道保持停止。第二路 HC-SR04P 的 GPIO 尚未在现有工程中定义，本版本不擅自占用引脚。

## 编译和烧录

```bash
cd "/Users/mengyangzu/Documents/ChatGPT/ardui/esp32_red_ball_left_goal"
./idf-run.sh build
./idf-run.sh -p /dev/cu.usbserial-0001 flash monitor
```

默认 `MOTOR_OUTPUT_ENABLED` 为 `0`，先架空车轮观察串口和浏览器画面；确认方向后再将其改为 `1` 重新编译烧录。

Wi-Fi：`ESP32-RedBall`，密码：`redball123`。连接后打开 `http://192.168.4.1`。
