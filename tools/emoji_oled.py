#!/usr/bin/env python3
import time
import random
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=64)

emojis = [
    ("(◕‿◕)", "开心"),
    ("(◣_◢)", "生气"),
    ("(｡♥‿♥｡)", "爱心"),
    ("(◕︵◕)", "难过"),
    ("(⊙_⊙)", "惊讶"),
    ("(-_-)", "无语"),
    ("(｡◕‿◕｡)", "微笑"),
    ("(✧ω✧)", "期待"),
    ("(◠‿◠)", "满足"),
    ("(╥﹏╥)", "哭泣"),
    ("(¬‿¬)", "得意"),
    ("(◉_◉)", "震惊"),
]

def draw_emoji(draw, emoji_text, mood, font_emoji, font_mood):
    draw.rectangle((0, 0, 127, 63), outline=0, fill=0)
    
    bbox = draw.textbbox((0, 0), emoji_text, font=font_emoji)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    x = (128 - text_width) // 2
    y = 10
    draw.text((x, y), emoji_text, font=font_emoji, fill=255)
    
    bbox_mood = draw.textbbox((0, 0), mood, font=font_mood)
    mood_width = bbox_mood[2] - bbox_mood[0]
    x_mood = (128 - mood_width) // 2
    draw.text((x_mood, 45), mood, font=font_mood, fill=255)

try:
    font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    font_mood = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
except:
    font_emoji = ImageFont.load_default()
    font_mood = ImageFont.load_default()

print("OLED 表情循环播放")
print("按 Ctrl+C 退出")
print()

try:
    while True:
        emoji_text, mood = random.choice(emojis)
        
        image = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(image)
        
        draw_emoji(draw, emoji_text, mood, font_emoji, font_mood)
        device.display(image)
        
        print(f"显示: {emoji_text} - {mood}")
        
        time.sleep(2)
        
except KeyboardInterrupt:
    device.clear()
    print("\n已退出")
