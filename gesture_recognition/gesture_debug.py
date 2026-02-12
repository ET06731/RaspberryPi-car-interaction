#!/usr/bin/env python3
"""
手势识别诊断版 - 实时显示检测参数
用于调试握拳识别问题
"""

import time
import threading
import cv2
import numpy as np
from collections import Counter
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont


class GestureSmoother:
    def __init__(self, window_size=5, min_confidence=3):
        self.window_size = window_size
        self.min_confidence = min_confidence
        self.history = []
        self.stable_gesture = "none"

    def update(self, raw_gesture):
        self.history.append(raw_gesture)
        if len(self.history) > self.window_size:
            self.history.pop(0)

        if len(self.history) < self.window_size:
            return None

        counter = Counter(self.history)
        most_common = counter.most_common(1)[0]
        gesture, count = most_common

        if count >= self.min_confidence:
            if gesture != self.stable_gesture:
                self.stable_gesture = gesture
                return gesture
        return None

    def get_stable(self):
        return self.stable_gesture


class GestureCamera:
    """带诊断信息的手势识别"""

    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.running = False
        self.current_gesture = "none"
        self.lock = threading.Lock()
        self.frame = None

        # 诊断信息
        self.debug_info = {
            "area": 0,
            "aspect_ratio": 0.0,
            "defects": 0,
            "hull_points": 0,
        }

        if self.cap.isOpened():
            print("[OK] 摄像头已连接")
        else:
            print("[ERROR] 无法打开摄像头")

    def detect_gesture_with_debug(self, frame):
        """识别手势并返回诊断信息"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV 肤色
        lower_skin = np.array([0, 20, 70])
        upper_skin = np.array([20, 255, 255])
        mask1 = cv2.inRange(hsv, lower_skin, upper_skin)

        lower_skin2 = np.array([170, 20, 70])
        upper_skin2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_skin2, upper_skin2)

        mask = mask1 + mask2
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 显示mask调试用
        self.mask_frame = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            if area > 5000:
                x, y, w, h = cv2.boundingRect(max_contour)
                aspect_ratio = float(w) / h

                hull = cv2.convexHull(max_contour, returnPoints=False)
                hull_points = len(hull) if hull is not None else 0

                finger_count = 0
                if hull_points > 3:
                    defects = cv2.convexityDefects(max_contour, hull)
                    if defects is not None:
                        for i in range(defects.shape[0]):
                            s, e, f, d = defects[i, 0]
                            if d > 10000:
                                finger_count += 1

                # 更新诊断信息
                self.debug_info = {
                    "area": int(area),
                    "aspect_ratio": round(aspect_ratio, 2),
                    "defects": finger_count,
                    "hull_points": hull_points,
                    "w": w,
                    "h": h,
                }

                # 改进的识别逻辑
                # 手掌：缺陷多
                if finger_count >= 4:
                    return "open_palm"

                # 剪刀手：2个缺陷
                elif finger_count == 2:
                    return "peace"

                # 握拳：缺陷少 (0-1) 或 面积大但紧凑 (长宽比接近1)
                # 放宽条件：缺陷<=2 或 (缺陷<=3 且 长宽比>0.6)
                elif finger_count <= 1:
                    return "fist"
                elif finger_count <= 2 and aspect_ratio > 0.6:
                    return "fist"
                elif area > 20000 and aspect_ratio > 0.7 and aspect_ratio < 1.3:
                    # 大而紧凑的区域，可能是握拳
                    return "fist"

        self.debug_info = {"area": 0, "aspect_ratio": 0, "defects": 0, "hull_points": 0}
        return "none"

    def run(self):
        self.running = True
        print("[INFO] 摄像头运行中，请展示手势...")

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            self.frame = frame

            gesture = self.detect_gesture_with_debug(frame)

            with self.lock:
                self.current_gesture = gesture

        self.cap.release()
        print("[OK] 摄像头已关闭")

    def get_gesture(self):
        with self.lock:
            return self.current_gesture

    def get_frame(self):
        return self.frame

    def get_debug_info(self):
        return self.debug_info

    def stop(self):
        self.running = False


class OLEDDisplay:
    def __init__(self, port=1, address=0x3C):
        try:
            serial = i2c(port=port, address=address)
            self.device = ssd1306(serial, width=128, height=64)
            self.connected = True
            try:
                self.font_emoji = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24
                )
                self.font_text = ImageFont.truetype(
                    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 14
                )
                self.font_small = ImageFont.truetype(
                    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 11
                )
            except:
                self.font_emoji = ImageFont.load_default()
                self.font_text = ImageFont.load_default()
                self.font_small = ImageFont.load_default()
            print("[OK] OLED 已连接")
        except Exception as e:
            self.connected = False
            print(f"[ERROR] OLED 连接失败: {e}")

    def show(self, emoji, name, action):
        if not self.connected:
            return

        image = Image.new("1", (128, 64))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 127, 63), outline=0, fill=0)

        bbox = draw.textbbox((0, 0), emoji, font=self.font_emoji)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 0), emoji, font=self.font_emoji, fill=255)

        bbox = draw.textbbox((0, 0), name, font=self.font_text)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 30), name, font=self.font_text, fill=255)

        bbox = draw.textbbox((0, 0), action, font=self.font_small)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 48), action, font=self.font_small, fill=255)

        self.device.display(image)

    def clear(self):
        if self.connected:
            self.device.clear()


def main():
    import sys

    headless = "--headless" in sys.argv or "-h" in sys.argv

    print("=" * 60)
    print("  🤚 手势识别诊断版")
    print("=" * 60)
    print()
    print("调试信息:")
    print("  - Area: 手部区域面积")
    print("  - Aspect: 长宽比 (宽/高)")
    print("  - Defects: 凸包缺陷数 (手指间凹陷)")
    print("  - Hull: 凸包点数")
    print()
    print("握拳检测条件:")
    print("  缺陷<=1 或 (缺陷<=2 且 长宽比>0.6)")
    print()
    if headless:
        print("无显示模式")
        print("按 Ctrl+C 退出")
    else:
        print("按 Q 退出，按 S 保存当前帧")
    print()

    gestures = {
        "none": ("(-_-)", "未识别", "请展示手势"),
        "open_palm": ("(✧ω✧)", "张开手掌", "你好！"),
        "fist": ("(◣_◢)", "握拳", "加油！"),
        "peace": ("(◠‿◠)", "剪刀手", "耶！"),
    }

    # 自动检测摄像头
    camera = None
    for idx in [0, 1]:
        print(f"[信息] 尝试 /dev/video{idx}...")
        cam = GestureCamera(idx)
        if cam.cap.isOpened():
            ret, frame = cam.cap.read()
            if ret and frame is not None:
                camera = cam
                print(f"[OK] 使用 /dev/video{idx}")
                break
            cam.cap.release()

    if camera is None:
        print("[ERROR] 无法打开摄像头")
        exit(1)

    oled = OLEDDisplay()
    smoother = GestureSmoother(window_size=5, min_confidence=3)

    if not camera.cap.isOpened():
        print("[ERROR] 摄像头初始化失败")
        exit(1)

    camera_thread = threading.Thread(target=camera.run)
    camera_thread.start()

    current_display = "none"
    frame_count = 0

    try:
        while True:
            raw_gesture = camera.get_gesture()
            stable_gesture = smoother.update(raw_gesture)
            debug = camera.get_debug_info()

            if stable_gesture is not None and stable_gesture != current_display:
                emoji, name, action = gestures.get(stable_gesture, gestures["none"])
                oled.show(emoji, name, action)
                print(f"\n[识别] {name} {emoji}")
                current_display = stable_gesture

            frame = camera.get_frame()
            if frame is not None:
                # 显示调试信息
                y_offset = 30
                color = (0, 255, 0)

                cv2.putText(
                    frame,
                    f"Raw: {raw_gesture}",
                    (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )
                cv2.putText(
                    frame,
                    f"Stable: {smoother.get_stable()}",
                    (10, y_offset + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

                # 显示诊断数据
                info_text = (
                    f"A:{debug['area']} R:{debug['aspect_ratio']} D:{debug['defects']}"
                )
                cv2.putText(
                    frame,
                    info_text,
                    (10, y_offset + 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    1,
                )

                # 显示握拳判断条件
                is_fist_candidate = (
                    debug["defects"] <= 2 and debug["aspect_ratio"] > 0.6
                )
                if is_fist_candidate and debug["area"] > 5000:
                    cv2.putText(
                        frame,
                        "FIST CANDIDATE!",
                        (10, y_offset + 75),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2,
                    )

                cv2.imshow("Gesture Debug", frame)

                # 显示mask调试用
                if hasattr(camera, "mask_frame"):
                    cv2.imshow("Mask", camera.mask_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    filename = f"gesture_debug_{frame_count}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"[保存] {filename}")
                    frame_count += 1

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n退出中...")

    finally:
        camera.stop()
        camera_thread.join()
        oled.clear()
        cv2.destroyAllWindows()
        print("[OK] 已退出")


if __name__ == "__main__":
    main()
