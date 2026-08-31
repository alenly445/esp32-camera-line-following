# 电脑端黑线参数标定

`line_detect.py` 仅用于在电脑上观察 mask、验证阈值和 ROI，不控制电机。参数验证完成后，把结果手动写入 `esp32_car_vision/main/car_vision_main.c`。

安装依赖：

```powershell
python -m pip install opencv-python numpy
```

读取 ESP32 Wi-Fi 串流工程提供的画面时：

```powershell
python test\line_detect.py --url http://192.168.43.7/stream --threshold 110 --roi 0.70
```

也可以读取电脑本地摄像头：

```powershell
python test\line_detect.py --camera 0 --threshold 110 --roi 0.70
```

运行窗口中按 `q` 退出，按 `s` 保存当前画面。当前独立巡线固件不提供 Wi-Fi 视频流；若板上已经烧录 `esp32_car_vision`，`--url` 模式不可用。
