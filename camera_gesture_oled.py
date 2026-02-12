#!/usr/bin/env python3
import time
import threading
import random
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

class OLEDDisplay:
    def __init__(self, port=1, address=0x3C):
        try:
            serial = i2c(port=port, address=address)
            self.device = ssd1306(serial, width=128, height=64)
            self.connected = True
            try:
                self.font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
                self.font_text = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 14)
                self.font_small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 11)
            except:
                self.font_emoji = ImageFont.load_default()
                self.font_text = ImageFont.load_default()
                self.font_small = ImageFont.load_default()
            print("[OK] OLED 屏幕已连接")
        except Exception as e:
            self.connected = False
            print(f"[ERROR] OLED 连接失败: {e}")
    
    def show_gesture(self, emoji, gesture_name, action_name):
        if not self.connected:
            return
        image = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 127, 63), outline=0, fill=0)
        
        bbox = draw.textbbox((0, 0), emoji, font=self.font_emoji)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 0), emoji, font=self.font_emoji, fill=255)
        
        bbox_name = draw.textbbox((0, 0), gesture_name, font=self.font_text)
        x_name = (128 - (bbox_name[2] - bbox_name[0])) // 2
        draw.text((x_name, 30), gesture_name, font=self.font_text, fill=255)
        
        bbox_action = draw.textbbox((0, 0), action_name, font=self.font_small)
        x_action = (128 - (bbox_action[2] - bbox_action[0])) // 2
        draw.text((x_action, 48), action_name, font=self.font_small, fill=255)
        
        self.device.display(image)
    
    def clear(self):
        if self.connected:
            self.device.clear()

def main():
    print("=" * 60)
    print("手势识别 + OLED 表情显示 (模拟模式)")
    print("=" * 60)
    print()
    
    gesture_map = {
        'open_palm': ('(✧ω✧)', '张开手掌', '你好！'),
        'fist': ('(◣_◢)', '握拳', '加油！'),
        'peace': ('(◠‿◠)', '剪刀手', '耶！'),
        'wave': ('(⊙_⊙)', '挥手', '再见！'),
        'point': ('(¬‿¬)', '指向前', '这边！'),
        'ok': ('(◕‿◕)', 'OK手势', '好的！'),
    }
    
    display = OLEDDisplay()
    
    print("[INFO] 当前为模拟模式 (每3秒随机切换手势)")
    print("[INFO] 摄像头问题排查:")
    print("  1. 检查USB连接: ls /dev/video*")
    print("  2. 测试摄像头: v4l2-ctl -d /dev/video0 --all")
    print("  3. 可能需要: sudo usermod -aG video yoi")
    print()
    print("按 Ctrl+C 退出")
    print()
    
    gesture_keys = list(gesture_map.keys())
    
    try:
        while True:
            gesture = random.choice(gesture_keys)
            emoji, name, action = gesture_map[gesture]
            display.show_gesture(emoji, name, action)
            print(f"[手势] {name} {emoji} - {action}")
            time.sleep(3)
    except KeyboardInterrupt:
        display.clear()
        print("\n已退出")

if __name__ == "__main__":
    main()
