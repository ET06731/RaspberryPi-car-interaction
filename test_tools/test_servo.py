#!/usr/bin/env python3
"""
舵机测试脚本 - 检查硬件连接
"""

import time
import sys

print("=" * 60)
print("舵机硬件测试")
print("=" * 60)
print()

# 检查 RPi.GPIO
try:
    import RPi.GPIO as GPIO

    print("✓ RPi.GPIO 导入成功")
except ImportError:
    print("✗ RPi.GPIO 未安装")
    print("  安装: sudo apt install python3-rpi.gpio")
    sys.exit(1)

# 配置
SERVO_PIN = 23  # BCM 编码的 GPIO 23 (物理引脚 16)

print(f"\n测试配置:")
print(f"  舵机信号引脚: GPIO {SERVO_PIN} (物理引脚 16)")
print(f"  电源: 5V (物理引脚 2 或 4)")
print(f"  地线: GND (物理引脚 6, 9, 14, 20, 25, 30, 34, 39)")
print()

# 检查接线说明
print("请确认接线:")
print("  舵机红线   → 5V")
print("  舵机黑/棕线 → GND")
print("  舵机黄/橙线 → GPIO 23 (物理引脚 16)")
print()

print("等待 3 秒...")
time.sleep(3)
print()

# 初始化 GPIO
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    print("✓ GPIO 初始化成功")
except Exception as e:
    print(f"✗ GPIO 初始化失败: {e}")
    sys.exit(1)

# 创建 PWM
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz
pwm.start(0)
print("✓ PWM 启动成功 (50Hz)")
print()


def set_angle(angle):
    """设置舵机角度 - 参考原版代码"""
    duty = 2.5 + 10 * angle / 180.0
    print(f"  角度: {angle}°, 占空比: {duty:.1f}%")
    # 循环发送 18 次，每次 20ms，参考原版代码
    for i in range(18):
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.02)
        pwm.ChangeDutyCycle(0)
        time.sleep(0.02)


# 测试不同角度
print("开始测试舵机运动...")
print("如果舵机正常，你会看到它转动")
print()

try:
    print("1. 归零位 (0°)...")
    set_angle(0)
    time.sleep(1)

    print("2. 中间位 (90°)...")
    set_angle(90)
    time.sleep(1)

    print("3. 最大位 (180°)...")
    set_angle(180)
    time.sleep(1)

    print("4. 回到中间 (90°)...")
    set_angle(90)
    time.sleep(1)

    print()
    print("✓ 测试完成！")
    print()
    print("如果舵机没有转动，请检查:")
    print("  1. 电源是否接通 (5V)")
    print("  2. 地线是否连接 (GND)")
    print("  3. 信号线是否接在 GPIO 23")
    print("  4. 舵机是否损坏 (换一个试试)")

except KeyboardInterrupt:
    print("\n测试中断")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("\n已清理 GPIO")
