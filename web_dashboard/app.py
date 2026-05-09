#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
小车Web仪表盘 - Flask应用
端口: 5000
"""

from flask import Flask, render_template, jsonify, request, Response
from car_api import car
from system_monitor import get_system_info
import atexit
import signal
import sys
import subprocess
import threading
import time
import cv2

app = Flask(__name__)


# ===== 摄像头视频流 =====


def generate_video_frames():
    """生成视频帧"""
    camera = cv2.VideoCapture(1)
    camera.set(3, 320)
    camera.set(4, 240)

    while True:
        success, frame = camera.read()
        if not success:
            break

        # 添加时间戳
        cv2.putText(
            frame,
            time.strftime("%H:%M:%S"),
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        # 压缩为JPEG
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        frame_bytes = buffer.tobytes()

        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")


@app.route("/video_feed")
def video_feed():
    """视频流路由"""
    return Response(
        generate_video_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


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


@app.route("/api/control", methods=["POST"])
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


@app.route("/api/speed", methods=["POST"])
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


@app.route("/api/sensors", methods=["GET"])
def get_sensors():
    """获取所有传感器数据"""
    return jsonify({"speed": car.get_speed(), "distance": car.get_distance()})


# ===== 蜂鸣器控制 =====


@app.route("/api/buzzer", methods=["POST"])
def buzzer_control():
    """控制蜂鸣器"""
    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    pattern = data.get("pattern", "short")

    patterns = {
        "short": car.buzzer_short,
        "double": car.buzzer_double,
        "long": car.buzzer_long,
        "on": lambda: car.buzzer_alert("on"),
        "off": lambda: car.buzzer_alert("off"),
    }

    if pattern in patterns:
        patterns[pattern]()
        return jsonify({"status": "success", "pattern": pattern})
    else:
        return jsonify({"status": "error", "message": "Invalid pattern"}), 400


# ===== 风扇控制 =====


@app.route("/api/fan", methods=["POST"])
def fan_control():
    """控制风扇"""
    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    action = data.get("action", "status")

    if action == "on":
        car.fan_on()
        return jsonify({"status": "success", "action": "on"})
    elif action == "off":
        car.fan_off()
        return jsonify({"status": "success", "action": "off"})
    elif action == "status":
        return jsonify({"status": "success", "running": car.fan_status()})
    else:
        return jsonify({"status": "error", "message": "Invalid action"}), 400


# ===== 舵机控制 =====


@app.route("/api/servo", methods=["POST"])
def servo_control():
    """控制舵机"""
    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    servo_type = data.get("type", "up_down")  # up_down 或 left_right
    angle = data.get("angle", 90)

    try:
        angle = int(angle)
        if angle < 0 or angle > 180:
            return jsonify({"status": "error", "message": "Angle must be 0-180"}), 400

        if servo_type == "up_down":
            car.servo_up_down(angle)
        elif servo_type == "left_right":
            car.servo_left_right(angle)
        elif servo_type == "center":
            car.servo_center()
        else:
            return jsonify({"status": "error", "message": "Invalid servo type"}), 400

        return jsonify({"status": "success", "type": servo_type, "angle": angle})
    except (ValueError, TypeError):
        return jsonify({"status": "error", "message": "Invalid angle value"}), 400


# ===== RGB LED 控制 =====


@app.route("/api/rgb", methods=["POST"])
def rgb_control():
    """控制RGB LED"""
    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    action = data.get("action", "color")

    if action == "color":
        red = data.get("red", 0)
        green = data.get("green", 0)
        blue = data.get("blue", 0)
        try:
            car.set_rgb(int(red), int(green), int(blue))
            return jsonify(
                {"status": "success", "red": red, "green": green, "blue": blue}
            )
        except (ValueError, TypeError):
            return jsonify({"status": "error", "message": "Invalid color values"}), 400

    elif action == "preset":
        color = data.get("color", "red")
        car.rgb_color(color)
        return jsonify({"status": "success", "color": color})

    elif action == "off":
        car.rgb_off()
        return jsonify({"status": "success", "action": "off"})

    else:
        return jsonify({"status": "error", "message": "Invalid action"}), 400


# ===== 摄像头控制 =====

camera_active = False


@app.route("/api/camera", methods=["POST"])
def camera_control():
    """控制摄像头（视频流始终在主端口）"""
    global camera_active

    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    action = data.get("action", "status")

    if action == "start":
        camera_active = True
        return jsonify({"status": "success", "action": "started"})
    elif action == "stop":
        camera_active = False
        return jsonify({"status": "success", "action": "stopped"})
    elif action == "status":
        return jsonify({"status": "success", "running": camera_active})
    else:
        return jsonify({"status": "error", "message": "Invalid action"}), 400


# ===== 系统资源监控 =====


@app.route("/monitor")
def monitor():
    """系统资源监控页面"""
    return render_template("monitor.html")


@app.route("/api/system", methods=["GET"])
def get_system_stats():
    """获取系统资源数据"""
    return jsonify(get_system_info())


# ===== 手势程序控制 =====

gesture_process = None
gesture_lock = threading.Lock()
gesture_output_thread = None


def read_gesture_output(proc):
    """后台读取手势程序输出"""
    import os

    while proc.poll() is None:
        try:
            line = proc.stdout.readline()
            if line:
                print(f"[手势] {line.rstrip()}")
        except:
            break


@app.route("/api/gesture", methods=["POST"])
def gesture_control():
    """控制手势互动程序"""
    global gesture_process

    data = request.get_json()
    if data is None:
        return jsonify({"status": "error", "message": "Invalid JSON body"}), 400

    action = data.get("action", "status")

    with gesture_lock:
        if action == "start":
            if gesture_process and gesture_process.poll() is None:
                return jsonify({"status": "already_running"})

            # 启动手势互动程序 (使用虚拟环境中的Python)
            try:
                venv_python = "/home/yoi/rpi-car-project/.venv/bin/python3"
                gesture_process = subprocess.Popen(
                    [
                        venv_python,
                        "/home/yoi/rpi-car-project/main/gesture_opencv.py",
                        "--headless",
                    ],
                    cwd="/home/yoi/rpi-car-project",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )
                # 等待1秒让进程启动
                time.sleep(1)
                # 检查进程是否成功启动
                if gesture_process.poll() is not None:
                    stdout, stderr = gesture_process.communicate()
                    print(f"[手势程序] 启动失败: {stderr[:200]}")
                    return jsonify(
                        {
                            "status": "error",
                            "message": "Process exited immediately",
                            "stderr": stderr[:200],
                        }
                    ), 500
                print(f"[手势程序] 已启动, PID: {gesture_process.pid}")

                # 启动后台线程读取输出
                global gesture_output_thread
                gesture_output_thread = threading.Thread(
                    target=read_gesture_output, args=(gesture_process,), daemon=True
                )
                gesture_output_thread.start()

                return jsonify(
                    {
                        "status": "success",
                        "action": "started",
                        "pid": gesture_process.pid,
                    }
                )
            except Exception as e:
                return jsonify({"status": "error", "message": str(e)}), 500

        elif action == "stop":
            if gesture_process and gesture_process.poll() is None:
                gesture_process.terminate()
                gesture_process.wait(timeout=5)
                return jsonify({"status": "success", "action": "stopped"})
            else:
                return jsonify({"status": "not_running"})

        elif action == "status":
            is_running = gesture_process and gesture_process.poll() is None
            return jsonify({"status": "success", "running": is_running})

        else:
            return jsonify({"status": "error", "message": "Invalid action"}), 400


if __name__ == "__main__":
    print("=" * 50)
    print("小车Web仪表盘启动中...")
    print("访问地址: http://<raspberry-pi-ip>:5000")
    print("=" * 50)

    # 启动Flask服务器，监听所有网络接口
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
