# 🤖 树莓派智能小车项目

## 目录结构

```
rpi-car-project/
├── main/                          # 主程序（推荐使用）
│   ├── gesture_opencv.py         # OpenCV手势识别 + 舵机控制
│   └── gesture_robot_simple.py   # MediaPipe简化版
│
├── gesture_recognition/           # 手势识别实验版本
│   ├── gesture_camera_oled.py    # 摄像头+OLED基础版
│   ├── gesture_camera_oled_smooth.py  # 平滑处理版
│   ├── gesture_debug.py          # 调试用（显示参数）
│   ├── gesture_emoji.py          # 表情包显示版
│   ├── gesture_final.py          # 综合最终版
│   ├── gesture_mediapipe.py      # MediaPipe完整版
│   └── gesture_emoji_mpu.py      # 带MPU6050版本
│
├── test_tools/                    # 测试工具
│   ├── test_camera_simple.py     # 摄像头测试
│   ├── test_oled_debug.py        # OLED测试
│   ├── test_servo.py             # 舵机测试
│   └── test_servo_original.py    # 舵机原方法测试
│
├── tools/                         # 工具脚本
│   ├── car_control.py            # 小车控制
│   ├── display_cme12864.py       # LCD显示屏驱动
│   ├── display_oled.py           # OLED显示屏驱动
│   ├── emoji_oled.py             # OLED表情包显示
│   └── hello_oled*.py            # OLED测试程序
│
├── archive/                       # 旧版本/归档
│   └── gesture_robot_interaction.py  # 复杂版本（不稳定）
│
├── examples/                      # 示例代码（原始）
│   └── 树莓派wifi智能小车python版本源代码/
│       ├── 1.七彩探照灯/
│       ├── 2.小车前进/
│       ├── 3.小车前后左右综合实验/
│       ├── 4.舵机旋转控制七彩灯/
│       ├── 5.按键控制小车启动/
│       ├── 6.红外避障/
│       ├── 7.红外跟随/
│       ├── 8.寻光行走/
│       ├── 9.寻迹实验/
│       ├── 10.超声波避障/
│       ├── 11.带舵机云台的超声波避障/
│       ├── 12.上位机控制智能小车综合实验/
│       ├── 13.蓝牙控制智能小车综合实验/
│       ├── 14.PS2游戏手柄控制智能小车综合实验/
│       ├── 15.TCP控制智能小车实验/
│       └── 16.微信控制小车/
│
├── 资料/                          # 硬件文档资料
│   ├── 6.硬件资料/
│   │   ├── 七彩探照灯/
│   │   ├── 巡线传感器/
│   │   ├── 红外+寻光传感器/
│   │   ├── 蓝牙BLE4.0模块/
│   │   └── MPU6050/
│   ├── AI视觉教程.docx
│   └── README.txt
│
├── run_gesture_daemon.sh         # 守护进程启动脚本
├── test_servo.py                 # 舵机硬件测试
└── .venv/                        # Python虚拟环境

```

## 快速开始

### 1. 运行推荐版本（OpenCV + 舵机）
```bash
cd ~/rpi-car-project/main
~/.local/bin/uv run python3 gesture_opencv.py --headless
```

### 2. 运行MediaPipe版本
```bash
cd ~/rpi-car-project/main
~/.local/bin/uv run python3 gesture_robot_simple.py --headless
```

### 3. 测试硬件
```bash
cd ~/rpi-car-project/test_tools
~/.local/bin/uv run python3 test_camera_simple.py      # 测试摄像头
~/.local/bin/uv run python3 test_servo.py              # 测试舵机
~/.local/bin/uv run python3 test_oled_debug.py         # 测试OLED
```

## 手势说明

| 手势 | OLED显示 | 舵机动作 |
|------|---------|---------|
| 🖐️ 张开手掌 | (✧ω✧) 你好 | 环顾扫描 |
| ✊ 握拳 | (◣_◢) 加油 | 锁定 |
| ✌️ 剪刀手 | (◠‿◠) 耶 | 摇摆 |

## 硬件连接

### 摄像头
- 使用 `/dev/video0` 或 `/dev/video1`

### OLED (I2C)
- VCC → 3.3V (物理引脚1)
- GND → GND (物理引脚6)
- SCL → GPIO 3/SCL (物理引脚5)
- SDA → GPIO 2/SDA (物理引脚3)
- 地址: 0x3C

### 舵机云台
- GPIO 9: 上下舵机
- GPIO 11: 左右舵机
- 电源: 5V (建议使用外部电源)

## 依赖安装

```bash
cd ~/rpi-car-project
~/.local/bin/uv pip install mediapipe opencv-python luma.oled pillow numpy RPi.GPIO
```

## 注意事项

1. 运行前确保摄像头、OLED、舵机正确连接
2. 使用 `--headless` 参数在无显示器环境下运行
3. 如遇摄像头问题，重启USB: `sudo sh -c 'echo 1-1.4 > /sys/bus/usb/drivers/usb/unbind; echo 1-1.4 > /sys/bus/usb/drivers/usb/bind'`
4. 舵机需要足够电流，建议使用外部5V电源

## 版本说明

- **v2.0 (OpenCV版)**: 当前推荐版本，使用肤色检测，轻量级，响应快
- **v1.0 (MediaPipe版)**: 使用Google MediaPipe，识别精度高但资源占用大
- **v0.x**: 实验版本，已归档到 archive/

## 日志查看

```bash
# 实时查看运行日志
tail -f /tmp/gesture.log
```
