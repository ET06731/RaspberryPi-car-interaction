#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
树莓派系统资源监控模块
"""

import psutil
import time
import os
from datetime import datetime


def get_cpu_usage():
    """获取CPU使用率"""
    return psutil.cpu_percent(interval=0.5, percpu=False)


def get_cpu_temp():
    """获取CPU温度"""
    try:
        temp = psutil.sensors_temperatures()
        if 'cpu_thermal' in temp:
            return temp['cpu_thermal'][0].current
        elif 'soc_thermal' in temp:
            return temp['soc_thermal'][0].current
        for key in temp:
            if 'cpu' in key.lower() or 'soc' in key.lower():
                return temp[key][0].current
        return None
    except:
        return None


def get_memory_info():
    """获取内存信息"""
    mem = psutil.virtual_memory()
    return {
        'total': round(mem.total / (1024**3), 2),
        'available': round(mem.available / (1024**3), 2),
        'used': round(mem.used / (1024**3), 2),
        'percent': mem.percent
    }


def get_disk_info():
    """获取磁盘信息"""
    disk = psutil.disk_usage('/')
    return {
        'total': round(disk.total / (1024**3), 2),
        'used': round(disk.used / (1024**3), 2),
        'free': round(disk.free / (1024**3), 2),
        'percent': disk.percent
    }


def get_network_info():
    """获取网络信息"""
    net = psutil.net_io_counters()
    return {
        'bytes_sent': net.bytes_sent,
        'bytes_recv': net.bytes_recv,
        'packets_sent': net.packets_sent,
        'packets_recv': net.packets_recv
    }


def get_uptime():
    """获取系统运行时间"""
    boot_time = psutil.boot_time()
    uptime_seconds = time.time() - boot_time
    
    days = int(uptime_seconds // 86400)
    hours = int((uptime_seconds % 86400) // 3600)
    minutes = int((uptime_seconds % 3600) // 60)
    
    if days > 0:
        return f"{days}天 {hours}小时"
    elif hours > 0:
        return f"{hours}小时 {minutes}分钟"
    else:
        return f"{minutes}分钟"


def get_load_average():
    """获取负载平均值"""
    try:
        load1, load5, load15 = os.getloadavg()
        return {
            '1min': round(load1, 2),
            '5min': round(load5, 2),
            '15min': round(load15, 2)
        }
    except:
        return None


def get_system_info():
    """获取完整系统信息"""
    return {
        'cpu': {
            'usage': get_cpu_usage(),
            'temp': get_cpu_temp()
        },
        'memory': get_memory_info(),
        'disk': get_disk_info(),
        'network': get_network_info(),
        'uptime': get_uptime(),
        'load': get_load_average(),
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    }


def format_bytes(bytes_val):
    """格式化字节数"""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if bytes_val < 1024:
            return f"{bytes_val:.2f} {unit}"
        bytes_val /= 1024
    return f"{bytes_val:.2f} PB"


if __name__ == '__main__':
    info = get_system_info()
    print(info)