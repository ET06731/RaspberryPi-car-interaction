#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
树莓派WiFi智能小车 - 键盘控制版
控制指令:
  W - 前进
  S - 后退
  A - 左转
  D - 右转
  Q - 原地左转
  E - 原地右转
  空格 - 停止
  X - 退出程序
"""

import RPi.GPIO as GPIO
import time
import sys
import tty
import termios

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# GPIO设置
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 电机引脚初始化
pwm_ENA = None
pwm_ENB = None

def motor_init():
    global pwm_ENA, pwm_ENB
    
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    
    # 设置PWM频率为2000Hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

def set_motor_speed(speed=80):
    """设置电机速度 (0-100)"""
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)

def forward():
    """前进"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed()

def backward():
    """后退"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_motor_speed()

def left():
    """左转 (左轮停，右轮转)"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed()

def right():
    """右转 (左轮转，右轮停)"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed()

def spin_left():
    """原地左转"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed()

def spin_right():
    """原地右转"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_motor_speed()

def brake():
    """停止"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(0)

def cleanup():
    """清理GPIO"""
    brake()
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.cleanup()

def getch():
    """获取单个字符"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_help():
    print("\n" + "="*50)
    print("树莓派WiFi智能小车控制")
    print("="*50)
    print("控制指令:")
    print("  W - 前进")
    print("  S - 后退")
    print("  A - 左转")
    print("  D - 右转")
    print("  Q - 原地左转")
    print("  E - 原地右转")
    print("  空格 - 停止")
    print("  X - 退出程序")
    print("  H - 显示帮助")
    print("="*50)

if __name__ == "__main__":
    try:
        print("初始化电机控制...")
        motor_init()
        print("初始化完成!")
        print_help()
        
        while True:
            try:
                key = getch().upper()
                
                if key == 'X':
                    print("\n退出程序...")
                    break
                elif key == 'H':
                    print_help()
                elif key == 'W':
                    print("前进 ↑")
                    forward()
                elif key == 'S':
                    print("后退 ↓")
                    backward()
                elif key == 'A':
                    print("左转 ←")
                    left()
                elif key == 'D':
                    print("右转 →")
                    right()
                elif key == 'Q':
                    print("原地左转 ↺")
                    spin_left()
                elif key == 'E':
                    print("原地右转 ↻")
                    spin_right()
                elif key == ' ':
                    print("停止 ■")
                    brake()
                    
            except Exception as e:
                print(f"\n错误: {e}")
                continue
                
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在退出...")
    finally:
        cleanup()
        print("程序已退出 GPIO已清理")
