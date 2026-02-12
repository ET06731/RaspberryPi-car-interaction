#!/usr/bin/env python3
import time
import random
import threading
import sys
import select
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=64)

gesture_emojis = {
    'up': ('(✧ω✧)', '向上', '抬头'),
    'down': ('(◕︵◕)', '向下', '低头'),
    'left': ('(◣_◢)', '向左', '左转'),
    'right': ('(◢_◣)', '向右', '右转'),
    'shake': ('(⊙_⊙)', '摇晃', '摇一摇'),
    'still': ('(-_-)', '静止', '等待'),
    'tilt_left': ('(¬‿¬)', '左倾', '左歪头'),
    'tilt_right': ('(‿¬¬)', '右倾', '右歪头'),
}

current_gesture = 'still'
lock = threading.Lock()

def get_fonts():
    try:
        font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
        font_label = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 14)
        font_action = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 11)
    except Exception as e:
        print(f"Font load error: {e}")
        font_emoji = ImageFont.load_default()
        font_label = ImageFont.load_default()
        font_action = ImageFont.load_default()
    return font_emoji, font_label, font_action

def draw_emoji(draw, emoji_text, gesture_label, action_text, font_emoji, font_label, font_action):
    draw.rectangle((0, 0, 127, 63), outline=0, fill=0)
    
    bbox = draw.textbbox((0, 0), emoji_text, font=font_emoji)
    text_width = bbox[2] - bbox[0]
    x = (128 - text_width) // 2
    draw.text((x, 2), emoji_text, font=font_emoji, fill=255)
    
    bbox_label = draw.textbbox((0, 0), gesture_label, font=font_label)
    label_width = bbox_label[2] - bbox_label[0]
    x_label = (128 - label_width) // 2
    draw.text((x_label, 30), gesture_label, font=font_label, fill=255)
    
    bbox_action = draw.textbbox((0, 0), action_text, font=font_action)
    action_width = bbox_action[2] - bbox_action[0]
    x_action = (128 - action_width) // 2
    draw.text((x_action, 48), action_text, font=font_action, fill=255)

def check_keyboard_input():
    global current_gesture
    
    key_map = {
        'w': 'up', 's': 'down', 'a': 'left', 'd': 'right',
        'q': 'tilt_left', 'e': 'tilt_right', ' ': 'shake',
    }
    
    while True:
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                if key in key_map:
                    with lock:
                        current_gesture = key_map[key]
                elif key == '\x03':
                    break
        except:
            break

def simulate_gesture():
    global current_gesture
    gestures = list(gesture_emojis.keys())
    weights = [1, 1, 2, 2, 3, 10, 2, 2]
    
    while True:
        time.sleep(random.uniform(2, 4))
        with lock:
            if current_gesture == 'still':
                current_gesture = random.choices(gestures, weights=weights)[0]
                time.sleep(1.5)
                current_gesture = 'still'

font_emoji, font_label, font_action = get_fonts()

print("=" * 50)
print("手势表情显示")
print("=" * 50)
print()
print("控制方式:")
print("  W - 向上    |  S - 向下")
print("  A - 向左    |  D - 向右")
print("  Q - 左倾    |  E - 右倾")
print("  空格 - 摇晃")
print()
print("自动模式: 每2-4秒随机切换表情")
print()
print("按 Ctrl+C 退出")
print("=" * 50)
print()

input_thread = threading.Thread(target=check_keyboard_input, daemon=True)
input_thread.start()

simulate_thread = threading.Thread(target=simulate_gesture, daemon=True)
simulate_thread.start()

try:
    while True:
        with lock:
            gesture = current_gesture
        
        emoji_text, gesture_label, action_text = gesture_emojis[gesture]
        
        image = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(image)
        
        draw_emoji(draw, emoji_text, gesture_label, action_text, font_emoji, font_label, font_action)
        device.display(image)
        
        if gesture != 'still':
            print(f"手势: {gesture_label} {emoji_text}")
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    device.clear()
    print("\n已退出")
