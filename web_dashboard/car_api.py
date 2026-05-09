#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
小车控制API封装模块
提供线程安全的GPIO控制接口
"""

import RPi.GPIO as GPIO
import time
import threading
import atexit
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

# 蜂鸣器引脚定义
BUZZER_PIN = 8

# 风扇引脚定义 (使用 GPIO 12，避免与 I2C 冲突)
FAN_PIN = 12

# 舵机引脚定义 (云台)
SERVO_UP_DOWN_PIN = 11  # 上下舵机
SERVO_LEFT_RIGHT_PIN = 9  # 左右舵机

# RGB LED 引脚定义
RGB_RED_PIN = 22
RGB_GREEN_PIN = 27
RGB_BLUE_PIN = 24

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
        atexit.register(self.cleanup)

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

        # 蜂鸣器引脚初始化
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        GPIO.output(BUZZER_PIN, GPIO.HIGH)  # 默认关闭

        # 风扇引脚初始化
        GPIO.setup(FAN_PIN, GPIO.OUT)
        GPIO.output(FAN_PIN, GPIO.LOW)  # 默认关闭

        # 设置PWM
        self.pwm_ena = GPIO.PWM(ENA, 2000)
        self.pwm_enb = GPIO.PWM(ENB, 2000)
        self.pwm_ena.start(0)
        self.pwm_enb.start(0)

    def set_speed(self, speed: int):
        """设置速度 (0-100)"""
        with self._motor_lock:
            self.current_speed = max(0, min(100, speed))
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
            t1 = time.time()
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

    # ===== 蜂鸣器控制 =====
    def buzzer_beep(self, duration=0.1, times=1):
        """蜂鸣器发声 - duration: 单次持续时间, times: 次数"""
        for _ in range(times):
            GPIO.output(BUZZER_PIN, GPIO.LOW)
            time.sleep(duration)
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            time.sleep(0.05)

    def buzzer_short(self):
        """短促一声"""
        self.buzzer_beep(0.05, 1)

    def buzzer_double(self):
        """两声"""
        self.buzzer_beep(0.08, 2)

    def buzzer_long(self):
        """长声"""
        self.buzzer_beep(0.3, 1)

    def buzzer_alert(self, pattern="short"):
        """不同模式的蜂鸣反馈"""
        if pattern == "short":
            self.buzzer_short()
        elif pattern == "double":
            self.buzzer_double()
        elif pattern == "long":
            self.buzzer_long()
        elif pattern == "on":
            GPIO.output(BUZZER_PIN, GPIO.LOW)
        elif pattern == "off":
            GPIO.output(BUZZER_PIN, GPIO.HIGH)

    # ===== 风扇控制 =====
    def fan_on(self):
        """风扇开启"""
        GPIO.output(FAN_PIN, GPIO.HIGH)

    def fan_off(self):
        """风扇关闭"""
        GPIO.output(FAN_PIN, GPIO.LOW)

    def fan_status(self) -> bool:
        """获取风扇状态"""
        return GPIO.input(FAN_PIN) == GPIO.HIGH

    # ===== 舵机控制 =====
    def _init_servo(self):
        """初始化舵机PWM"""
        if hasattr(self, "_servo_initialized"):
            return
        try:
            GPIO.setup(SERVO_UP_DOWN_PIN, GPIO.OUT)
            GPIO.setup(SERVO_LEFT_RIGHT_PIN, GPIO.OUT)
            self.pwm_servo_up = GPIO.PWM(SERVO_UP_DOWN_PIN, 50)
            self.pwm_servo_lr = GPIO.PWM(SERVO_LEFT_RIGHT_PIN, 50)
            self.pwm_servo_up.start(0)
            self.pwm_servo_lr.start(0)
            self._servo_initialized = True
        except Exception as e:
            print(f"[警告] 舵机初始化失败: {e}")

    def _servo_angle_to_duty(self, angle):
        """角度转换为PWM占空比"""
        return 2.5 + 10 * angle / 180.0

    def servo_up_down(self, angle: int):
        """控制上下舵机角度 (0-180)"""
        self._init_servo()
        angle = max(0, min(180, angle))
        duty = self._servo_angle_to_duty(angle)
        self.pwm_servo_up.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.pwm_servo_up.ChangeDutyCycle(0)

    def servo_left_right(self, angle: int):
        """控制左右舵机角度 (0-180)"""
        self._init_servo()
        angle = max(0, min(180, angle))
        duty = self._servo_angle_to_duty(angle)
        self.pwm_servo_lr.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.pwm_servo_lr.ChangeDutyCycle(0)

    def servo_center(self):
        """舵机回中"""
        self.servo_up_down(90)
        self.servo_left_right(90)

    # ===== RGB LED 控制 =====
    def _init_rgb(self):
        """初始化RGB LED"""
        if hasattr(self, "_rgb_initialized"):
            return
        try:
            GPIO.setup(RGB_RED_PIN, GPIO.OUT)
            GPIO.setup(RGB_GREEN_PIN, GPIO.OUT)
            GPIO.setup(RGB_BLUE_PIN, GPIO.OUT)
            self.pwm_rgb_red = GPIO.PWM(RGB_RED_PIN, 1000)
            self.pwm_rgb_green = GPIO.PWM(RGB_GREEN_PIN, 1000)
            self.pwm_rgb_blue = GPIO.PWM(RGB_BLUE_PIN, 1000)
            self.pwm_rgb_red.start(0)
            self.pwm_rgb_green.start(0)
            self.pwm_rgb_blue.start(0)
            self._rgb_initialized = True
        except Exception as e:
            print(f"[警告] RGB LED初始化失败: {e}")

    def set_rgb(self, red: int, green: int, blue: int):
        """设置RGB颜色 (0-255)"""
        self._init_rgb()
        red_duty = max(0, min(100, red * 100 // 255))
        green_duty = max(0, min(100, green * 100 // 255))
        blue_duty = max(0, min(100, blue * 100 // 255))
        self.pwm_rgb_red.ChangeDutyCycle(red_duty)
        self.pwm_rgb_green.ChangeDutyCycle(green_duty)
        self.pwm_rgb_blue.ChangeDutyCycle(blue_duty)

    def rgb_off(self):
        """关闭RGB LED"""
        self._init_rgb()
        self.pwm_rgb_red.ChangeDutyCycle(0)
        self.pwm_rgb_green.ChangeDutyCycle(0)
        self.pwm_rgb_blue.ChangeDutyCycle(0)

    def rgb_color(self, color: str):
        """预设颜色"""
        colors = {
            "red": (255, 0, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
            "yellow": (255, 255, 0),
            "cyan": (0, 255, 255),
            "magenta": (255, 0, 255),
            "white": (255, 255, 255),
            "purple": (128, 0, 128),
            "orange": (255, 165, 0),
            "pink": (255, 192, 203),
        }
        if color.lower() in colors:
            r, g, b = colors[color.lower()]
            self.set_rgb(r, g, b)
        else:
            self.rgb_off()


# 全局控制器实例
car = CarController()
