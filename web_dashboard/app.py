#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
小车Web仪表盘 - Flask应用
端口: 5000
"""

from flask import Flask, render_template, jsonify, request
from car_api import car
import atexit
import signal
import sys

app = Flask(__name__)


# 注册清理函数
@atexit.register
def cleanup():
    """程序退出时清理GPIO"""
    print("清理GPIO资源...")
    car.cleanup()


def signal_handler(sig, frame):
    """处理信号"""
    print("\n收到停止信号...")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# ===== 页面路由 =====


@app.route("/")
def index():
    """主页面"""
    return render_template("index.html")


# ===== API路由 =====


@app.route("/api/status", methods=["GET"])
def get_status():
    """获取小车状态"""
    return jsonify({"speed": car.get_speed(), "distance": car.get_distance()})


@app.route("/api/control/move", methods=["POST"])
def move():
    """控制小车移动"""
    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400
    direction = data.get("direction", "stop")

    actions = {
        "forward": car.forward,
        "backward": car.backward,
        "left": car.turn_left,
        "right": car.turn_right,
        "spin_left": car.spin_left,
        "spin_right": car.spin_right,
        "stop": car.brake,
    }

    if direction in actions:
        actions[direction]()
        return jsonify({"status": "success", "direction": direction})
    else:
        return jsonify({"status": "error", "message": "Invalid direction"}), 400


@app.route("/api/control/speed", methods=["POST"])
def set_speed():
    """设置速度"""
    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400
    speed = data.get("speed", 80)

    try:
        speed = int(speed)
        if speed < 0 or speed > 100:
            return jsonify(
                {"status": "error", "message": "Speed must be between 0 and 100"}
            ), 400
        car.set_speed(speed)
        return jsonify({"status": "success", "speed": speed})
    except (ValueError, TypeError):
        return jsonify({"status": "error", "message": "Invalid speed value"}), 400


@app.route("/api/sensor/distance", methods=["GET"])
def get_distance():
    """获取超声波距离"""
    distance = car.get_distance()
    return jsonify({"status": "success", "distance": distance, "unit": "cm"})


if __name__ == "__main__":
    print("=" * 50)
    print("小车Web仪表盘启动中...")
    print("访问地址: http://<raspberry-pi-ip>:5000")
    print("=" * 50)

    # 启动Flask服务器，监听所有网络接口
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
