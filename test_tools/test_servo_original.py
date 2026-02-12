#!/usr/bin/env python3
"""舵机测试 - 使用原版方法"""

import RPi.GPIO as GPIO
import time

ServoPin = 23

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ServoPin, GPIO.OUT)

pwm = GPIO.PWM(ServoPin, 50)
pwm.start(0)


def servo_appointed_detection(pos):
    """原版舵机控制函数"""
    for i in range(18):
        pwm.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)
        pwm.ChangeDutyCycle(0)
        time.sleep(0.02)


print("舵机测试开始...")
print("0度...")
servo_appointed_detection(0)
time.sleep(1)

print("90度...")
servo_appointed_detection(90)
time.sleep(1)

print("180度...")
servo_appointed_detection(180)
time.sleep(1)

print("90度...")
servo_appointed_detection(90)
time.sleep(1)

print("测试完成！")
pwm.stop()
GPIO.cleanup()
