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
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
except:
    font = ImageFont.load_default()
    font_small = ImageFont.load_default()

draw.rectangle((0, 0, 127, 63), outline=255, fill=0)
draw.text((15, 20), "Hello World!", font=font, fill=255)
draw.text((25, 45), "Running...", font=font_small, fill=255)

device.display(image)
device.show()

print("OLED 已点亮，按 Ctrl+C 退出")
print("屏幕会一直保持显示")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n退出")
