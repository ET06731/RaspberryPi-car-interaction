#!/usr/bin/env python3
"""
手势识别 - OpenCV版本 (带舵机控制)
使用肤色检测进行手势识别 + 双舵机云台
"""

import time
import threading
import sys
import cv2
import numpy as np
from collections import Counter
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

# 导入 RPi.GPIO
try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[警告] RPi.GPIO 不可用")


# ==================== 双舵机云台控制 ====================
class ServoController:
    def __init__(self, up_down_pin=9, left_right_pin=11):
        self.up_down_pin = up_down_pin
        self.left_right_pin = left_right_pin
        self.pwm_up_down = None
        self.pwm_left_right = None
        self._is_busy = False  # 添加忙标志，防止动作重叠

        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)

                # 初始化上下舵机 (GPIO 9)
                GPIO.setup(self.up_down_pin, GPIO.OUT)
                self.pwm_up_down = GPIO.PWM(self.up_down_pin, 50)
                self.pwm_up_down.start(0)
                self._set_angle_single(self.pwm_up_down, 90)

                # 初始化左右舵机 (GPIO 11)
                GPIO.setup(self.left_right_pin, GPIO.OUT)
                self.pwm_left_right = GPIO.PWM(self.left_right_pin, 50)
                self.pwm_left_right.start(0)
                self._set_angle_single(self.pwm_left_right, 90)

                print("[OK] 双舵机云台就绪 (GPIO 9=上下, GPIO 11=左右)")
            except Exception as e:
                print(f"[错误] 舵机: {e}")

    def _set_angle_single(self, pwm, angle, duration=0.5):
        """设置单个舵机角度，持续发送信号保证流畅"""
        angle = max(0, min(180, angle))
        if pwm:
            duty = 2.5 + 10 * angle / 180.0
            pwm.ChangeDutyCycle(duty)
            time.sleep(duration)  # 持续发送信号一段时间
            pwm.ChangeDutyCycle(0)  # 然后停止，防止抖动

    def execute_action(self, action_name):
        def action():
            if not GPIO_AVAILABLE:
                return

            # 如果正在执行动作，跳过
            if self._is_busy:
                return

            self._is_busy = True
            print(f"[舵机] 执行动作: {action_name}")

            if action_name == "nod":  # 点头 - 上下舵机动 (70°-100°)
                for _ in range(2):
                    self._set_angle_single(self.pwm_up_down, 100, duration=0.3)  # 抬头
                    self._set_angle_single(self.pwm_up_down, 70, duration=0.3)  # 低头
                self._set_angle_single(self.pwm_up_down, 90, duration=0.3)  # 回到原位

            elif action_name == "shake":  # 摇头 - 左右舵机动 (60°-120°)
                for _ in range(3):
                    self._set_angle_single(
                        self.pwm_left_right, 60, duration=0.25
                    )  # 左转
                    self._set_angle_single(
                        self.pwm_left_right, 120, duration=0.25
                    )  # 右转
                self._set_angle_single(
                    self.pwm_left_right, 90, duration=0.3
                )  # 回到原位

            elif action_name == "sway":  # 摇摆 - 左右舵机
                for _ in range(2):
                    self._set_angle_single(self.pwm_left_right, 75, duration=0.4)
                    self._set_angle_single(self.pwm_left_right, 105, duration=0.4)
                self._set_angle_single(
                    self.pwm_left_right, 90, duration=0.3
                )  # 回到原位

            elif action_name == "scan":  # 扫描 - 左右舵机环顾
                for a in range(0, 181, 10):
                    self._set_angle_single(self.pwm_left_right, a, duration=0.08)
                for a in range(180, -1, -10):
                    self._set_angle_single(self.pwm_left_right, a, duration=0.08)
                self._set_angle_single(self.pwm_left_right, 90, duration=0.3)

            elif action_name == "look_left":  # 看左边
                self._set_angle_single(self.pwm_left_right, 60)

            elif action_name == "look_right":  # 看右边
                self._set_angle_single(self.pwm_left_right, 120)

            elif action_name == "look_up":  # 抬头
                self._set_angle_single(self.pwm_up_down, 70)

            elif action_name == "look_down":  # 低头
                self._set_angle_single(self.pwm_up_down, 100)

            elif action_name == "center":  # 回中
                self._set_angle_single(self.pwm_up_down, 90)
                self._set_angle_single(self.pwm_left_right, 90)

            elif action_name == "lock":  # 锁定
                pass

            self._is_busy = False  # 动作完成，释放标志

        threading.Thread(target=action, daemon=True).start()

    def stop(self):
        if self.pwm_up_down:
            self.pwm_up_down.stop()
        if self.pwm_left_right:
            self.pwm_left_right.stop()
        if GPIO_AVAILABLE:
            GPIO.cleanup()


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
    """OpenCV手势识别"""

    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.running = False
        self.current_gesture = "none"
        self.lock = threading.Lock()
        self.frame = None
        self.mask_frame = None
        self.debug_info = {"area": 0, "aspect_ratio": 0.0, "defects": 0}

        if self.cap.isOpened():
            print("[OK] 摄像头已连接")
        else:
            print("[ERROR] 无法打开摄像头")

    def detect_gesture(self, frame):
        """使用肤色检测识别手势"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV肤色范围
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

                self.debug_info = {
                    "area": int(area),
                    "aspect_ratio": round(aspect_ratio, 2),
                    "defects": finger_count,
                    "hull_points": hull_points,
                }

                # 手势判断
                if finger_count >= 4:
                    return "open_palm"
                elif finger_count == 2:
                    return "peace"
                elif finger_count <= 1:
                    return "fist"
                elif finger_count <= 2 and aspect_ratio > 0.6:
                    return "fist"

        self.debug_info = {"area": 0, "aspect_ratio": 0.0, "defects": 0}
        return "none"

    def run(self):
        self.running = True
        print("[INFO] 摄像头运行中...")

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            self.frame = frame

            gesture = self.detect_gesture(frame)

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
                    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 14
                )
                self.font_small = ImageFont.truetype(
                    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 11
                )
            except:
                self.font_emoji = ImageFont.load_default()
                self.font_text = ImageFont.load_default()
                self.font_small = ImageFont.load_default()
            print("[OK] OLED 已连接")
        except Exception as e:
            self.connected = False
            print(f"[警告] OLED: {e}")

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
    headless = "--headless" in sys.argv or "-h" in sys.argv

    print("=" * 60)
    print("  🤚 OpenCV 手势识别")
    print("=" * 60)
    print()
    print("手势: 手掌(张开) / 拳头(握紧) / 剪刀手(2指)")
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
        return

    oled = OLEDDisplay()
    smoother = GestureSmoother()
    servo = ServoController()  # 初始化舵机

    # 手势对应的舵机动作
    servo_actions = {
        "open_palm": "scan",  # 手掌 -> 环顾
        "fist": "lock",  # 握拳 -> 锁定
        "peace": "sway",  # 剪刀手 -> 摇摆
    }

    camera_thread = threading.Thread(target=camera.run)
    camera_thread.start()

    current_display = "none"
    frame_count = 0

    print("\n[运行中] 请在摄像头前展示手势...")
    print("按 Ctrl+C 退出" if headless else "按 Q 退出")
    print()

    try:
        while True:
            raw_gesture = camera.get_gesture()
            stable_gesture = smoother.update(raw_gesture)
            debug = camera.get_debug_info()

            if stable_gesture is not None and stable_gesture != current_display:
                emoji, name, action = gestures.get(stable_gesture, gestures["none"])
                oled.show(emoji, name, action)
                print(
                    f"[识别] {name} {emoji} (面积:{debug['area']} 缺陷:{debug['defects']})"
                )
                # 执行舵机动作
                servo.execute_action(servo_actions.get(stable_gesture, "center"))
                time.sleep(2)  # 暂停2秒给舵机做反馈动作
                current_display = stable_gesture

            if not headless:
                frame = camera.get_frame()
                if frame is not None:
                    # 添加调试信息到画面
                    cv2.putText(
                        frame,
                        f"Gesture: {raw_gesture}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                    info = f"A:{debug['area']} D:{debug['defects']}"
                    cv2.putText(
                        frame,
                        info,
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                    )

                    cv2.imshow("Gesture", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            else:
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"[调试] 已处理 {frame_count} 帧")
                time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n退出中...")

    finally:
        camera.stop()
        camera_thread.join()
        oled.clear()
        servo.stop()  # 停止舵机
        if not headless:
            cv2.destroyAllWindows()
        print("[OK] 已退出")


if __name__ == "__main__":
    main()
