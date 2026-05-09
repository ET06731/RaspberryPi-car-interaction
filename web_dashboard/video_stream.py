#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
独立视频流服务器 - 端口 5001
"""

import cv2
import time
from flask import Flask, Response

app = Flask(__name__)


def generate_frames():
    """生成视频帧"""
    camera = cv2.VideoCapture(0)
    camera.set(3, 640)
    camera.set(4, 480)

    while True:
        success, frame = camera.read()
        if not success:
            break

        # 添加时间戳
        cv2.putText(
            frame,
            time.strftime("%Y-%m-%d %H:%M:%S"),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        # 压缩为JPEG
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        frame_bytes = buffer.tobytes()

        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")


@app.route("/")
def index():
    """主页面 - 直接显示视频"""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>摄像头实时画面</title>
        <style>
            body { margin: 0; background: #000; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
            img { max-width: 100%; max-height: 100vh; }
        </style>
    </head>
    <body>
        <img src="/video" style="width:100%;max-width:800px;">
    </body>
    </html>
    """


@app.route("/video")
def video_feed():
    """视频流路由"""
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    print("=" * 50)
    print("视频流服务器启动中...")
    print("手机访问: http://<raspberry-pi-ip>:5001")
    print("=" * 50)
    app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)
