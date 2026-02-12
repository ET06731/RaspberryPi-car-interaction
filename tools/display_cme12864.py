#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
CME12864 I2C LCD 显示模块测试程序

硬件连接:
  VCC - 5V 或 3.3V
  GND - GND
  SCL - GPIO3 (SCL)
  SDA - GPIO2 (SDA)

在树莓派上运行前请先:
  1. 启用 I2C: sudo raspi-config -> Interface Options -> I2C
  2. 安装工具: sudo apt-get install i2c-tools
  3. 检查设备: i2cdetect -y 1
"""

import time
import smbus2 as smbus

# I2C 配置
I2C_BUS = 20  # 树莓派 I2C 总线编号 (使用 /dev/i2c-20)
DEFAULT_ADDRESS = 0x27  # 常见 LCD I2C 地址 (PCF8574)

# CME12864 / LCD1602/2004 指令定义
LCD_CMD = {
    'CLEAR': 0x01,
    'HOME': 0x02,
    'ENTRY_MODE': 0x06,
    'DISPLAY_ON': 0x0C,
    'DISPLAY_OFF': 0x08,
    'FUNCTION_SET': 0x28,  # 4位模式, 2行, 5x8字体
    'SET_DDRAM_ADDR': 0x80,
}

# 背光控制
LCD_BACKLIGHT = 0x08
LCD_ENABLE = 0x04
LCD_READ = 0x02
LCD_WRITE = 0x00


class CME12864Display:
    """CME12864 / LCD I2C 显示模块驱动"""

    def __init__(self, address=DEFAULT_ADDRESS, bus=None):
        """
        初始化显示模块

        Args:
            address: I2C 设备地址 (默认 0x27)
            bus: I2C 总线编号 (默认 1)
        """
        self.address = address
        self.bus = bus if bus is not None else I2C_BUS
        self.lcd_handle = None
        self.connected = False

        try:
            self.lcd_handle = smbus.SMBus(self.bus)
            self._write_byte(0x00)  # 唤醒设备
            time.sleep(0.05)
            self._lcd_init()
            self.connected = True
            print(f"[OK] CME12864 显示模块已连接, I2C 地址: 0x{address:02X}")
        except Exception as e:
            print(f"[ERROR] 连接显示模块失败: {e}")
            print(f"提示: 运行 'i2cdetect -y 1' 查看可用设备地址")

    def _write_byte(self, data):
        """写入一个字节到 I2C 设备"""
        if self.lcd_handle:
            self.lcd_handle.write_byte(self.address, data)

    def _lcd_send_byte(self, data, mode=0):
        """
        发送一个字节到 LCD

        Args:
            data: 要发送的数据
            mode: 0 = 命令, 1 = 数据
        """
        high_nibble = data & 0xF0
        low_nibble = (data << 4) & 0xF0

        # 发送高位
        self._write_byte(high_nibble | LCD_BACKLIGHT | (mode << 0))
        self._write_byte(high_nibble | LCD_BACKLIGHT | LCD_ENABLE | (mode << 0))
        time.sleep(0.0005)
        self._write_byte(high_nibble | LCD_BACKLIGHT | (mode << 0))

        # 发送低位
        self._write_byte(low_nibble | LCD_BACKLIGHT | (mode << 0))
        self._write_byte(low_nibble | LCD_BACKLIGHT | LCD_ENABLE | (mode << 0))
        time.sleep(0.0005)
        self._write_byte(low_nibble | LCD_BACKLIGHT | (mode << 0))

    def _lcd_init(self):
        """初始化 LCD"""
        time.sleep(0.05)
        # 初始化序列
        self._lcd_send_byte(0x33)  # 4位模式开始
        self._lcd_send_byte(0x32)  # 4位模式
        self._lcd_send_byte(0x06)  # 入口模式
        self._lcd_send_byte(0x0C)  # 显示开启
        self._lcd_send_byte(0x28)  # 4位模式, 2行
        self._lcd_send_byte(0x01)  # 清屏
        time.sleep(0.002)

    def clear(self):
        """清屏"""
        self._lcd_send_byte(LCD_CMD['CLEAR'])
        time.sleep(0.002)

    def home(self):
        """光标归位"""
        self._lcd_send_byte(LCD_CMD['HOME'])
        time.sleep(0.002)

    def display_on(self):
        """开启显示"""
        self._lcd_send_byte(LCD_CMD['DISPLAY_ON'])

    def display_off(self):
        """关闭显示"""
        self._lcd_send_byte(LCD_CMD['DISPLAY_OFF'])

    def set_cursor(self, row, col):
        """
        设置光标位置

        Args:
            row: 行 (0-1 对于 16x2, 0-3 对于 20x4)
            col: 列 (0-15 或 0-19)
        """
        # LCD1602/2004 行地址偏移
        row_offsets = [0x00, 0x40, 0x14, 0x54]
        if row < len(row_offsets):
            self._lcd_send_byte(LCD_CMD['SET_DDRAM_ADDR'] | (row_offsets[row] + col))

    def write_char(self, char):
        """写入单个字符"""
        self._lcd_send_byte(ord(char), mode=1)

    def write_string(self, text, row=0, col=0):
        """
        写入字符串

        Args:
            text: 要显示的字符串
            row: 行位置
            col: 列位置
        """
        self.set_cursor(row, col)
        for char in text[:16]:  # 限制每行16字符
            self.write_char(char)

    def show_status(self, line1="Ready", line2="Waiting..."):
        """
        显示两行状态信息

        Args:
            line1: 第一行文本
            line2: 第二行文本
        """
        self.clear()
        self.write_string(line1[:16], row=0, col=0)
        self.write_string(line2[:16], row=1, col=0)

    def show_car_status(self, direction, speed, target=""):
        """
        显示小车状态

        Args:
            direction: 方向 (前进/后退/左转/右转/停止)
            speed: 速度 (0-100)
            target: 目标物体 (可选)
        """
        self.clear()
        # 第一行: 方向 + 速度
        status = direction[:8] + f" {speed:3d}%"
        self.write_string(status[:16], row=0, col=0)
        # 第二行: 目标
        target_display = target if target else "---"
        self.write_string(target_display[:16], row=1, col=0)

    def cleanup(self):
        """关闭显示并清理"""
        self.display_off()
        print("[OK] 显示模块已关闭")


def scan_i2c_devices():
    """扫描 I2C 总线上的所有设备"""
    print("\n" + "=" * 40)
    print("I2C 设备扫描")
    print("=" * 40)
    try:
        bus = smbus.SMBus(I2C_BUS)
        devices = []
        for addr in range(0x03, 0x78):
            try:
                bus.write_quick(addr)
                devices.append(addr)
            except:
                pass

        if devices:
            print(f"找到 {len(devices)} 个设备:")
            for addr in devices:
                name = get_device_name(addr)
                print(f"  - 0x{addr:02X}: {name}")
        else:
            print("未找到任何 I2C 设备")
            print("请检查:")
            print("  1. I2C 是否在 raspi-config 中启用")
            print("  2. 硬件连接是否正确")
            print("  3. 显示模块供电是否正常")
        return devices
    except Exception as e:
        print(f"扫描失败: {e}")
        return []


def get_device_name(addr):
    """获取设备名称"""
    names = {
        0x27: "LCD1602/2004/CME12864 (PCF8574)",
        0x3F: "LCD1602/2004 (PCF8574)",
        0x20: "PCF8574 IO Expander",
        0x68: "DS1307/AT24C32/MPU6050",
        0x76: "BMP280/BME280",
        0x77: "BMP180/BME280",
    }
    return names.get(addr, "Unknown Device")


def test_display():
    """显示模块测试程序"""
    print("\n" + "=" * 40)
    print("CME12864 显示模块测试")
    print("=" * 40)

    # 扫描 I2C 设备
    devices = scan_i2c_devices()

    if not devices:
        print("\n[WARNING] 未找到 I2C 设备，尝试默认地址...")
        display = CME12864Display()
    else:
        # 尝试使用找到的 LCD 常见地址
        lcd_addrs = [a for a in devices if a in [0x27, 0x3F]]
        if lcd_addrs:
            display = CME12864Display(address=lcd_addrs[0])
        else:
            print(f"\n[INFO] 未找到 LCD 设备地址，使用默认 0x27")
            display = CME12864Display()

    if not display.connected:
        print("\n[ERROR] 显示模块连接失败")
        print("请在树莓派上运行以下命令检查:")
        print("  ls /dev/i2c*")
        print("  i2cdetect -y 1")
        return

    # 测试显示
    print("\n--- 测试 1: 基本显示 ---")
    display.show_status("CME12864", "Test Started")
    time.sleep(2)

    print("\n--- 测试 2: 状态滚动 ---")
    directions = ["Forward", "Backward", "Left", "Right", "Stop"]
    for i, direction in enumerate(directions):
        display.show_car_status(direction, 50 + i * 10)
        time.sleep(1)

    print("\n--- 测试 3: 目标检测显示 ---")
    targets = ["纸巾", "萝卜", "苹果", "香蕉"]
    for target in targets:
        display.show_car_status("Forward", 80, target)
        time.sleep(1)

    print("\n--- 测试完成 ---")
    display.clear()
    display.show_status("Test", "Complete!")
    time.sleep(1)
    display.cleanup()

    print("\n[SUCCESS] 所有测试完成!")


if __name__ == "__main__":
    test_display()
