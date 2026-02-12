#!/usr/bin/env python3
import time
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)

image = Image.new('1', (128, 64))
draw = ImageDraw.Draw(image)

try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
except:
    font = ImageFont.load_default()

draw.rectangle((0, 0, 127, 63), outline=255, fill=0)
draw.text((5, 22), "Hello World!", font=font, fill=255)

device.display(image)

print("OLED 显示中: Hello World!")
print("按 Ctrl+C 退出")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("退出")
