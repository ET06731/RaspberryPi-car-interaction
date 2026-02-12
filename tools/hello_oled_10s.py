#!/usr/bin/env python3
import time
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=64)

image = Image.new('1', (128, 64))
draw = ImageDraw.Draw(image)

try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
except:
    font = ImageFont.load_default()

draw.rectangle((0, 0, 127, 63), outline=255, fill=0)
draw.text((10, 18), "Hello World!", font=font, fill=255)

device.display(image)

print("✓ OLED 显示中...")
print("  屏幕会保持显示 10 秒")
time.sleep(10)

print("✓ 测试完成，屏幕已关闭")
