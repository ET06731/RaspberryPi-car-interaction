#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
小车控制模块
基于树莓派GPIO控制智能小车运动
"""

import RPi.GPIO as GPIO
import time


class CarController:
    """智能小车控制器"""

    # 引脚定义
    IN1, IN2 = 20, 21  # 左电机方向
    IN3, IN4 = 19, 26  # 右电机方向
    ENA, ENB = 16, 13  # PWM使能

    def __init__(self, speed=80):
        self.speed = speed
        self.pwm_ENA = None
        self.pwm_ENB = None
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def init(self):
        """初始化GPIO"""
        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN4, GPIO.OUT, initial=GPIO.LOW)

        self.pwm_ENA = GPIO.PWM(self.ENA, 2000)
        self.pwm_ENB = GPIO.PWM(self.ENB, 2000)
        self.pwm_ENA.start(0)
        self.pwm_ENB.start(0)
        print("🚗 小车初始化完成")

    def set_speed(self, speed):
        """设置速度"""
        self.speed = max(0, min(100, speed))

    def forward(self, duration=None, speed=None):
        """前进"""
        s = speed or self.speed
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(s)
        self.pwm_ENB.ChangeDutyCycle(s)
        if duration:
            time.sleep(duration)
            self.stop()

    def backward(self, duration=None, speed=None):
        """后退"""
        s = speed or self.speed
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(s)
        self.pwm_ENB.ChangeDutyCycle(s)
        if duration:
            time.sleep(duration)
            self.stop()

    def left(self, duration=None, speed=None):
        """左转 (左轮停，右轮转)"""
        s = speed or self.speed
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(0)
        self.pwm_ENB.ChangeDutyCycle(s)
        if duration:
            time.sleep(duration)
            self.stop()

    def right(self, duration=None, speed=None):
        """右转 (左轮转，右轮停)"""
        s = speed or self.speed
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(s)
        self.pwm_ENB.ChangeDutyCycle(0)
        if duration:
            time.sleep(duration)
            self.stop()

    def spin_left(self, duration=None, speed=None):
        """原地左转"""
        s = speed or self.speed
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(s)
        self.pwm_ENB.ChangeDutyCycle(s)
        if duration:
            time.sleep(duration)
            self.stop()

    def spin_right(self, duration=None, speed=None):
        """原地右转"""
        s = speed or self.speed
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(s)
        self.pwm_ENB.ChangeDutyCycle(s)
        if duration:
            time.sleep(duration)
            self.stop()

    def stop(self):
        """停止"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(0)
        self.pwm_ENB.ChangeDutyCycle(0)

    def move_towards_object(self, object_x, frame_center, frame_width, threshold=50):
        """
        根据物体位置移动小车
        object_x: 物体在画面中的x坐标
        frame_center: 画面中心x坐标
        frame_width: 画面宽度
        threshold: 居中阈值
        """
        # 计算物体相对于中心的位置
        relative_x = object_x - frame_center

        if abs(relative_x) < threshold:
            # 物体在中间，前进
            self.forward(duration=0.3)
        elif relative_x < 0:
            # 物体在左，转向左边
            self.spin_left(duration=0.1)
        else:
            # 物体在右，转向右边
            self.spin_right(duration=0.1)

    def cleanup(self):
        """清理GPIO"""
        self.stop()
        if self.pwm_ENA:
            self.pwm_ENA.stop()
        if self.pwm_ENB:
            self.pwm_ENB.stop()
        GPIO.cleanup()
        print("🔌 GPIO已清理")


if __name__ == "__main__":
    # 测试代码
    car = CarController(speed=70)
    car.init()

    print("测试：小车运动")
    print("前进...")
    car.forward(duration=1)
    print("后退...")
    car.backward(duration=1)
    print("左转...")
    car.left(duration=0.5)
    print("右转...")
    car.right(duration=0.5)
    print("停止...")
    car.stop()

    car.cleanup()
    print("测试完成")
