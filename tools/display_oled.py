#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
CME12864 OLED 显示模块测试程序 (128x64 I2C OLED)
使用 luma.oled 驱动库

硬件连接:
  VCC - 3.3V (Pin 1)
  GND - GND  (Pin 6)
  SCL - GPIO3 (Pin 5, SCL)
  SDA - GPIO2 (Pin 3, SDA)

在树莓派上运行前请先:
  1. 启用 I2C: sudo raspi-config -> Interface Options -> I2C
  2. 检查设备: i2cdetect -y 1
"""

import time
from PIL import Image, ImageDraw, ImageFont

I2C_PORT = 1
I2C_ADDRESS = 0x3C


class CME12864OLED:
    def __init__(self, port=I2C_PORT, address=I2C_ADDRESS):
        from luma.core.interface.serial import i2c
        from luma.oled.device import ssd1306
        
        self.port = port
        self.address = address
        self.device = None
        self.connected = False
        
        try:
            serial = i2c(port=port, address=address)
            self.device = ssd1306(serial, width=128, height=64)
            self.connected = True
            print(f"[OK] CME12864 OLED 已连接 (I2C:{port}, 0x{address:02X})")
        except Exception as e:
            print(f"[ERROR] OLED 连接失败: {e}")
            print("请检查:")
            print(f"  1. I2C 是否启用: ls /dev/i2c*")
            print(f"  2. 设备地址: i2cdetect -y {port}")
            print(f"  3. 硬件连接是否正确")
    
    def clear(self):
        if self.device:
            self.device.clear()
    
    def show_text(self, text, x=0, y=0, fill=255):
        if not self.device:
            return
        
        image = Image.new('1', (self.device.width, self.device.height))
        draw = ImageDraw.Draw(image)
        
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        except:
            font = ImageFont.load_default()
        
        draw.text((x, y), text, font=font, fill=fill)
        self.device.display(image)
    
    def show_status(self, line1="Ready", line2="Waiting..."):
        if not self.device:
            return
        
        image = Image.new('1', (self.device.width, self.device.height))
        draw = ImageDraw.Draw(image)
        
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
        except:
            font = ImageFont.load_default()
        
        draw.rectangle((0, 0, self.device.width, self.device.height), outline=0, fill=0)
        draw.text((5, 5), line1[:20], font=font, fill=255)
        draw.text((5, 25), line2[:20], font=font, fill=255)
        
        self.device.display(image)
    
    def show_car_status(self, direction, speed, target=""):
        if not self.device:
            return
        
        image = Image.new('1', (self.device.width, self.device.height))
        draw = ImageDraw.Draw(image)
        
        try:
            font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
            font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        except:
            font_large = ImageFont.load_default()
            font_small = ImageFont.load_default()
        
        draw.rectangle((0, 0, self.device.width, self.device.height), outline=0, fill=0)
        
        draw.text((5, 2), "Car Status", font=font_small, fill=255)
        draw.line((0, 18, 128, 18), fill=255)
        
        status_text = f"{direction} {speed}%"
        draw.text((5, 22), status_text, font=font_large, fill=255)
        
        if target:
            draw.text((5, 44), f"Target: {target}", font=font_small, fill=255)
        
        self.device.display(image)
    
    def draw_welcome(self):
        if not self.device:
            return
        
        image = Image.new('1', (self.device.width, self.device.height))
        draw = ImageDraw.Draw(image)
        
        try:
            font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
            font_text = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
        except:
            font_title = ImageFont.load_default()
            font_text = ImageFont.load_default()
        
        draw.rectangle((0, 0, self.device.width, self.device.height), outline=0, fill=0)
        
        draw.text((15, 10), "CME12864", font=font_title, fill=255)
        draw.text((10, 30), "128x64 OLED", font=font_text, fill=255)
        draw.text((10, 45), "Test Program", font=font_text, fill=255)
        
        self.device.display(image)


def scan_i2c_devices(port=I2C_PORT):
    import smbus2 as smbus
    
    print(f"\nI2C Bus {port} 扫描:")
    print("-" * 40)
    
    try:
        bus = smbus.SMBus(port)
        devices = []
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                devices.append(addr)
            except:
                pass
        
        if devices:
            print(f"找到 {len(devices)} 个设备:")
            for addr in devices:
                name = get_device_name(addr)
                print(f"  0x{addr:02X} - {name}")
        else:
            print("未找到 I2C 设备")
        
        return devices
    except Exception as e:
        print(f"扫描失败: {e}")
        return []


def get_device_name(addr):
    names = {
        0x27: "LCD1602/2004 (PCF8574)",
        0x3C: "SSD1306 OLED (128x64)",
        0x3D: "SSD1306 OLED (128x64)",
        0x68: "MPU6050/DS1307",
        0x76: "BMP280/BME280",
        0x77: "BMP180/BME280",
    }
    return names.get(addr, "Unknown")


def test_display():
    print("\n" + "=" * 50)
    print("CME12864 OLED 显示模块测试")
    print("=" * 50)
    
    devices = scan_i2c_devices()
    
    oled_addrs = [a for a in devices if a in [0x3C, 0x3D]]
    if oled_addrs:
        addr = oled_addrs[0]
        print(f"\n检测到 OLED 设备: 0x{addr:02X}")
    else:
        addr = I2C_ADDRESS
        print(f"\n未检测到 OLED，使用默认地址: 0x{addr:02X}")
    
    display = CME12864OLED(address=addr)
    
    if not display.connected:
        print("\n[FAILED] OLED 显示模块连接失败")
        return
    
    print("\n测试 1: 显示欢迎画面")
    display.draw_welcome()
    time.sleep(2)
    
    print("测试 2: 显示状态信息")
    display.show_status("System Ready", "Waiting input...")
    time.sleep(1.5)
    
    print("测试 3: 显示小车状态")
    directions = [("Forward", 80), ("Left", 60), ("Right", 60), ("Stop", 0)]
    for direction, speed in directions:
        display.show_car_status(direction, speed, "纸巾")
        time.sleep(1)
    
    print("测试 4: 动态文本")
    for i in range(5):
        display.show_status(f"Count: {i}", f"Speed: {i*20}%")
        time.sleep(0.5)
    
    print("\n测试完成")
    display.show_status("Test Complete!", "All OK")
    time.sleep(2)
    display.clear()


if __name__ == "__main__":
    test_display()
