# ESP32-S3 三轮巡线避障小车 v20（VS Code 版）

本工程使用 **VS Code + Arduino CLI**。核心程序与已记录的 Arduino v20 源码逐字节一致，没有改动巡线、转弯、避障、编码器、终点停车或 TFT 显示逻辑。

## 为什么没有改成纯 ESP-IDF

原程序直接使用 Arduino API、`Adafruit_GFX` 和 `Adafruit_ST7735`。改成纯 ESP-IDF 需要重写 PWM、串口、GPIO、中断、任务和屏幕驱动，风险较大，也可能改变已经实测成功的小车行为。因此本工程只更换编辑、编译和烧录入口，不重写控制逻辑。

## 工程结构

```text
v20_line_car_vscode/
├─ .vscode/
│  ├─ extensions.json    推荐安装的 VS Code 扩展
│  ├─ settings.json      编辑器设置
│  └─ tasks.json         编译、烧录、串口监视任务
├─ .gitignore
├─ README.md
└─ v20_line_car_vscode.ino   完整小车程序
```

## 已知环境要求

- VS Code
- Arduino CLI：`C:\Program Files\Arduino CLI\arduino-cli.exe`
- ESP32 Arduino Core，开发板 FQBN：`esp32:esp32:esp32s3`
- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library
- Adafruit BusIO
- ESP32-S3 对应的 USB 转串口驱动

## 第一次打开

1. 在 VS Code 中选择“文件 → 打开文件夹”。
2. 打开本 `v20_line_car_vscode` 文件夹，不要只打开 `.ino` 文件。
3. 如果 VS Code 提示推荐扩展，可安装：
   - C/C++
   - Arduino Community Edition
4. 按 `Ctrl+Shift+P`，输入“运行任务”。
5. 先运行 `Arduino: 查看开发板和串口`，确认开发板对应哪个 COM 口。

## 编译

按 `Ctrl+Shift+B`，默认执行 `Arduino: 编译 ESP32-S3`。

编译产物放在 `.build` 目录，不会污染源码目录。

## 烧录

1. 用小车开发板的 UART USB 口连接电脑。
2. 关闭 Arduino IDE 串口监视器以及其他占用串口的软件。
3. 按 `Ctrl+Shift+P` → “运行任务”。
4. 选择 `Arduino: 烧录 ESP32-S3`。
5. 输入实际串口，例如 `COM7`。

如果提示无法连接：按住 `BOOT`，短按一次 `RESET`，松开 `RESET`，再松开 `BOOT`，然后重新烧录。

## 查看串口日志

运行任务 `Arduino: 打开串口监视器`，输入实际 COM 口。波特率已经设置为 `115200`。

退出串口监视器时，在终端中按 `Ctrl+C`。

## 烧录后独立运行

烧录完成后可以拔掉电脑 USB，但必须保留小车自己的稳定供电。重新上电或短按 `RESET` 后，程序会从 `setup()` 开始自动运行。

## 当前验证状态

- 已确认 VS Code 主程序存在。
- 已确认 Arduino CLI 位于预期路径。
- 已确认三个 Adafruit 依赖库能够被 Arduino CLI 列出。
- 已生成完整 VS Code 工程和任务配置。
- 核心 `.ino` 与 Arduino v20 存档 SHA-256 完全一致。
- 当前受权限限制，尚未成功读取 ESP32 Core 清单和 VS Code 扩展清单，也尚未完成本工程的实际编译。
- 当前只看到 COM3、COM4、COM5、COM6，均显示为 `Serial Port Unknown`；这不能证明其中任何一个就是 ESP32-S3。烧录前应重新插拔开发板并确认新增的 COM 口。

