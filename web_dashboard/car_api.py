#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
小车控制API封装模块
提供线程安全的GPIO控制接口
"""

import RPi.GPIO as GPIO
import time
import threading
from typing import Optional

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 超声波引脚定义
TRIG_PIN = 1
ECHO_PIN = 0

# 默认速度
DEFAULT_SPEED = 80


class CarController:
    """小车控制器类 - 单例模式确保GPIO安全"""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self.pwm_ena = None
        self.pwm_enb = None
        self.current_speed = DEFAULT_SPEED
        self._motor_lock = threading.Lock()

        self._init_gpio()
        self._initialized = True

    def _init_gpio(self):
        """初始化GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # 电机引脚初始化
        GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)

        # 超声波引脚初始化
        GPIO.setup(TRIG_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)

        # 设置PWM
        self.pwm_ena = GPIO.PWM(ENA, 2000)
        self.pwm_enb = GPIO.PWM(ENB, 2000)
        self.pwm_ena.start(0)
        self.pwm_enb.start(0)

    def set_speed(self, speed: int):
        """设置速度 (0-100)"""
        self.current_speed = max(0, min(100, speed))
        with self._motor_lock:
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def get_speed(self) -> int:
        """获取当前速度"""
        return self.current_speed

    def forward(self):
        """前进"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def backward(self):
        """后退"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def turn_left(self):
        """左转"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def turn_right(self):
        """右转"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def spin_left(self):
        """原地左转"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def spin_right(self):
        """原地右转"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            self.pwm_ena.ChangeDutyCycle(self.current_speed)
            self.pwm_enb.ChangeDutyCycle(self.current_speed)

    def brake(self):
        """停止"""
        with self._motor_lock:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
            self.pwm_ena.ChangeDutyCycle(0)
            self.pwm_enb.ChangeDutyCycle(0)

    def get_distance(self) -> float:
        """获取超声波测距"""
        try:
            # 发送触发信号
            GPIO.output(TRIG_PIN, GPIO.HIGH)
            time.sleep(0.000015)
            GPIO.output(TRIG_PIN, GPIO.LOW)

            # 等待ECHO变高
            timeout = time.time()
            while not GPIO.input(ECHO_PIN):
                t1 = time.time()
                if t1 - timeout > 0.1:
                    return -1

            # 等待ECHO变低
            timeout = time.time()
            while GPIO.input(ECHO_PIN):
                t2 = time.time()
                if t2 - timeout > 0.1:
                    return -1

            # 计算距离
            distance = ((t2 - t1) * 340 / 2) * 100
            return round(distance, 1)
        except Exception:
            return -1

    def cleanup(self):
        """清理GPIO资源"""
        self.brake()
        if self.pwm_ena:
            self.pwm_ena.stop()
        if self.pwm_enb:
            self.pwm_enb.stop()
        GPIO.cleanup()


# 全局控制器实例
car = CarController()
