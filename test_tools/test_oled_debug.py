#!/usr/bin/env python3
import time
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

print("OLED 诊断测试")
print("=" * 40)

try:
    print("1. 连接 I2C 设备...")
    serial = i2c(port=1, address=0x3C)
    print("   ✓ I2C 连接成功")
    
    print("2. 初始化 OLED...")
    device = ssd1306(serial, width=128, height=64)
    print("   ✓ OLED 初始化成功")
    
    print("3. 清屏...")
    device.clear()
    time.sleep(0.5)
    
    print("4. 测试全屏点亮...")
    image = Image.new('1', (128, 64), 1)
    device.display(image)
    print("   ✓ 全屏点亮 (应该看到全白屏幕)")
    time.sleep(2)
    
    print("5. 测试文字显示...")
    device.clear()
    image = Image.new('1', (128, 64))
    draw = ImageDraw.Draw(image)
    
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
    except:
        font = ImageFont.load_default()
    
    draw.text((10, 20), "Hello!", font=font, fill=255)
    draw.text((10, 40), "OLED OK", font=font, fill=255)
    device.display(image)
    print("   ✓ 文字显示完成")
    
    print("\n" + "=" * 40)
    print("诊断完成!")
    print("如果全屏亮但文字不显示: OLED 正常, 检查字体")
    print("如果全屏不亮: 检查硬件连接和电源")
    
except Exception as e:
    print(f"\n✗ 错误: {e}")
    print("\n排查步骤:")
    print("1. 检查硬件连接:")
    print("   VCC → 树莓派 Pin 1 (3.3V) 或 Pin 2 (5V)")
    print("   GND → 树莓派 Pin 6 (GND)")
    print("   SCL → 树莓派 Pin 5 (GPIO3)")
    print("   SDA → 树莓派 Pin 3 (GPIO2)")
    print("\n2. 检查 I2C 设备:")
    print("   sudo i2cdetect -y 1")
    print("\n3. 检查电源:")
    print("   有些 OLED 需要 5V, 有些需要 3.3V")
