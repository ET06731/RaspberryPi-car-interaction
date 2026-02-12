#!/usr/bin/env python3
import time
import random
import threading
import sys
import select
import math
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=64)

class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.address = address
        self.bus = bus
        self.connected = False
        try:
            import smbus2
            self.smbus = smbus2.SMBus(bus)
            self.smbus.write_byte_data(self.address, 0x6B, 0)
            self.connected = True
        except Exception as e:
            print(f"MPU6050 未连接: {e}")
    
    def read_word(self, reg):
        high = self.smbus.read_byte_data(self.address, reg)
        low = self.smbus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    def get_accel_data(self):
        if not self.connected:
            return None
        try:
            x = self.read_word(0x3B) / 16384.0
            y = self.read_word(0x3D) / 16384.0
            z = self.read_word(0x3F) / 16384.0
            return {'x': x, 'y': y, 'z': z}
        except:
            return None
    
    def get_gyro_data(self):
        if not self.connected:
            return None
        try:
            x = self.read_word(0x43) / 131.0
            y = self.read_word(0x45) / 131.0
            z = self.read_word(0x47) / 131.0
            return {'x': x, 'y': y, 'z': z}
        except:
            return None

gesture_emojis = {
    'up': ('(✧ω✧)', '向上', '抬头'),
    'down': ('(◕︵◕)', '向下', '低头'),
    'left': ('(◣_◢)', '向左', '左转'),
    'right': ('(◢_◣)', '向右', '右转'),
    'shake': ('(⊙_⊙)', '摇晃', '摇一摇'),
    'tilt_left': ('(¬‿¬)', '左倾', '左倾斜'),
    'tilt_right': ('(‿¬¬)', '右倾', '右倾斜'),
    'still': ('(-_-)', '静止', '等待中'),
}

current_gesture = 'still'
lock = threading.Lock()
use_mpu6050 = False
mpu = None

def detect_gesture_from_mpu():
    global current_gesture
    
    accel_history = []
    gyro_history = []
    
    while True:
        if not mpu or not mpu.connected:
            time.sleep(0.5)
            continue
        
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        
        if accel and gyro:
            accel_history.append(accel)
            gyro_history.append(gyro)
            
            if len(accel_history) > 10:
                accel_history.pop(0)
                gyro_history.pop(0)
            
            ax, ay, az = accel['x'], accel['y'], accel['z']
            
            if az < 0.3:
                if ay > 0.5:
                    gesture = 'up'
                elif ay < -0.5:
                    gesture = 'down'
                elif ax > 0.5:
                    gesture = 'right'
                elif ax < -0.5:
                    gesture = 'left'
                else:
                    gesture = 'still'
            else:
                if ax > 0.4:
                    gesture = 'tilt_right'
                elif ax < -0.4:
                    gesture = 'tilt_left'
                else:
                    gesture = 'still'
            
            gyro_z_values = [g['z'] for g in gyro_history[-5:]]
            if len(gyro_z_values) >= 5:
                gyro_variance = max(gyro_z_values) - min(gyro_z_values)
                if gyro_variance > 50:
                    gesture = 'shake'
            
            with lock:
                current_gesture = gesture
        
        time.sleep(0.1)

def check_keyboard_input():
    global current_gesture
    
    key_map = {
        'w': 'up',
        's': 'down',
        'a': 'left',
        'd': 'right',
        'q': 'tilt_left',
        'e': 'tilt_right',
        ' ': 'shake',
    }
    
    while True:
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                if key in key_map:
                    with lock:
                        current_gesture = key_map[key]
                        time.sleep(0.5)
                        current_gesture = 'still'
                elif key == '\x03':
                    break
        except:
            break

def draw_emoji(draw, emoji_text, gesture_name, action_name, font_emoji, font_text):
    draw.rectangle((0, 0, 127, 63), outline=0, fill=0)
    
    bbox = draw.textbbox((0, 0), emoji_text, font=font_emoji)
    text_width = bbox[2] - bbox[0]
    x = (128 - text_width) // 2
    draw.text((x, 5), emoji_text, font=font_emoji, fill=255)
    
    bbox_name = draw.textbbox((0, 0), gesture_name, font=font_text)
    name_width = bbox_name[2] - bbox_name[0]
    x_name = (128 - name_width) // 2
    draw.text((x_name, 32), gesture_name, font=font_text, fill=255)
    
    bbox_action = draw.textbbox((0, 0), action_name, font=font_text)
    action_width = bbox_action[2] - bbox_action[0]
    x_action = (128 - action_width) // 2
    draw.text((x_action, 48), f"[{action_name}]", font=font_text, fill=255)

mpu = MPU6050()
use_mpu6050 = mpu.connected

try:
    font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    font_text = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
except:
    font_emoji = ImageFont.load_default()
    font_text = ImageFont.load_default()

print("=" * 50)
print("手势识别表情显示")
print("=" * 50)

if use_mpu6050:
    print("\n✓ MPU6050 传感器已连接")
    print("  自动识别手势...")
    mpu_thread = threading.Thread(target=detect_gesture_from_mpu, daemon=True)
    mpu_thread.start()
else:
    print("\n⚠ MPU6050 未连接")
    print("  使用键盘控制手势:")
    print("    W - 向上  |  S - 向下")
    print("    A - 向左  |  D - 向右")
    print("    Q - 左倾  |  E - 右倾")
    print("    空格 - 摇晃")
    
    input_thread = threading.Thread(target=check_keyboard_input, daemon=True)
    input_thread.start()

print("\n按 Ctrl+C 退出")
print("=" * 50)
print()

try:
    last_gesture = None
    
    while True:
        with lock:
            gesture = current_gesture
        
        if gesture != last_gesture:
            emoji_text, gesture_name, action_name = gesture_emojis[gesture]
            
            image = Image.new('1', (128, 64))
            draw = ImageDraw.Draw(image)
            
            draw_emoji(draw, emoji_text, gesture_name, action_name, font_emoji, font_text)
            device.display(image)
            
            if gesture != 'still':
                print(f"识别到手势: {gesture_name} {emoji_text}")
            
            last_gesture = gesture
        
        time.sleep(0.05)
        
except KeyboardInterrupt:
    device.clear()
    print("\n已退出")
