#!/bin/bash
# 守护进程启动手势识别程序

cd /home/yoi/rpi-car-project

while true; do
    echo "[$(date)] 启动手势识别程序..."
    timeout 300 ~/.local/bin/uv run python3 gesture_robot_simple.py --headless 2>&1 | tee -a /tmp/gesture.log
    echo "[$(date)] 程序退出，5秒后重启..."
    sleep 5
done
