#!/usr/bin/env python3
"""
🤖 智能手势交互小车 - 简化稳定版
使用 MediaPipe 进行手势识别 + 舵机 + OLED
"""

import sys

sys.stdout.reconfigure(line_buffering=True)
print("程序启动...", flush=True)

import os
import cv2
import time
import threading
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

# 导入 MediaPipe
print("导入 MediaPipe...", flush=True)
import mediapipe as mp

print("MediaPipe 导入完成", flush=True)


# ==================== 表情配置 ====================
EXPRESSIONS = {
    "idle": ("(-_-)", "待机", "等待中"),
    "palm": ("(✧ω✧)", "手掌", "你好！"),
    "fist": ("(◣_◢)", "握拳", "锁定"),
    "peace": ("(◠‿◠)", "剪刀手", "耶！"),
    "thumbs_up": ("(｡◕‿◕｡)", "点赞", "收到"),
    "thumbs_down": ("(◕︵◕)", "倒赞", "不认同"),
    "point_up": ("(⊙_⊙)", "指上", "抬头"),
    "point_down": ("(¬_¬)", "指下", "低头"),
    "point_left": ("(◣_◢)", "指左", "左转"),
    "point_right": ("(◢_◣)", "指右", "右转"),
    "ok": ("(⌒‿⌒)", "OK", "没问题"),
}


# ==================== 双舵机云台控制 ====================
class ServoController:
    def __init__(self, up_down_pin=11, left_right_pin=9):  # GPIO 11=上下, GPIO 9=左右
        self.up_down_pin = up_down_pin
        self.left_right_pin = left_right_pin
        self.current_up_down = 90
        self.current_left_right = 90
        self.pwm_up_down = None
        self.pwm_left_right = None
        self._is_busy = False  # 添加忙标志，防止动作重叠

        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)

                # 初始化上下舵机 (GPIO 11)
                GPIO.setup(self.up_down_pin, GPIO.OUT)
                self.pwm_up_down = GPIO.PWM(self.up_down_pin, 50)
                self.pwm_up_down.start(0)
                self._set_angle_single(self.pwm_up_down, 90)

                # 初始化左右舵机 (GPIO 9)
                GPIO.setup(self.left_right_pin, GPIO.OUT)
                self.pwm_left_right = GPIO.PWM(self.left_right_pin, 50)
                self.pwm_left_right.start(0)
                self._set_angle_single(self.pwm_left_right, 90)

                print("[OK] 双舵机云台就绪 (GPIO 11=上下, GPIO 9=左右)")
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
                print(f"[舵机] GPIO 不可用")
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

            elif action_name == "sway":  # 摇摆 - 左右舵机 (轻微摆动)
                for _ in range(2):
                    self._set_angle_single(self.pwm_left_right, 75, duration=0.4)
                    self._set_angle_single(self.pwm_left_right, 105, duration=0.4)
                self._set_angle_single(
                    self.pwm_left_right, 90, duration=0.3
                )  # 回到原位

            elif action_name == "scan":  # 扫描 - 左右舵机环顾
                for a in range(0, 161, 10):
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

            time.sleep(2)  # 暂停2秒给舵机做反馈动作
            self._is_busy = False  # 动作完成，释放标志

        threading.Thread(target=action, daemon=True).start()

    def stop(self):
        if self.pwm_up_down:
            self.pwm_up_down.stop()
        if self.pwm_left_right:
            self.pwm_left_right.stop()


# ==================== OLED ====================
class OLED:
    def __init__(self):
        try:
            serial = i2c(port=1, address=0x3C)
            self.device = ssd1306(serial, width=128, height=64)
            try:
                self.font_e = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24
                )
                self.font_t = ImageFont.truetype(
                    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 14
                )
                self.font_a = ImageFont.truetype(
                    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 11
                )
            except:
                self.font_e = ImageFont.load_default()
                self.font_t = ImageFont.load_default()
                self.font_a = ImageFont.load_default()
            print("[OK] OLED 就绪")
        except Exception as e:
            self.device = None
            print(f"[警告] OLED: {e}")

    def show(self, emoji, name, action):
        if not self.device:
            return
        img = Image.new("1", (128, 64))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, 127, 63), outline=0, fill=0)

        bbox = draw.textbbox((0, 0), emoji, font=self.font_e)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 0), emoji, font=self.font_e, fill=255)

        bbox = draw.textbbox((0, 0), name, font=self.font_t)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 30), name, font=self.font_t, fill=255)

        bbox = draw.textbbox((0, 0), action, font=self.font_a)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 48), action, font=self.font_a, fill=255)

        self.device.display(img)

    def clear(self):
        if self.device:
            self.device.clear()


# ==================== 手势识别 ====================
class GestureRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils
        print("[OK] MediaPipe 就绪")

    def recognize(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture = "idle"

        if results.multi_hand_landmarks:
            for hand in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)
                gesture = self._classify(hand)

        return gesture, frame

    def _classify(self, hand):
        pts = [(lm.x, lm.y, lm.z) for lm in hand.landmark]

        def extended(tip, mcp):
            return (
                np.linalg.norm(np.array(pts[tip]) - np.array(pts[0]))
                > np.linalg.norm(np.array(pts[mcp]) - np.array(pts[0])) * 1.3
            )

        idx = extended(8, 5)
        mid = extended(12, 9)
        rng = extended(16, 13)
        pnk = extended(20, 17)

        ext = sum([idx, mid, rng, pnk])

        if ext == 0:
            return "fist"
        elif ext == 1:
            if idx:
                dx = pts[8][0] - pts[0][0]
                dy = pts[8][1] - pts[0][1]
                if abs(dx) > abs(dy):
                    return "point_right" if dx > 0 else "point_left"
                return "point_down" if dy > 0 else "point_up"
            return "thumbs_up"
        elif ext == 2:
            if idx and mid:
                return "peace"
            return "ok"
        elif ext >= 4:
            return "palm"
        return "idle"

    def close(self):
        self.hands.close()


# ==================== 主程序 ====================
class Robot:
    def __init__(self, camera_idx=1, headless=False):
        print("=" * 60)
        print("  🤖 手势交互机器人")
        print("=" * 60)
        print()

        self.headless = headless

        # 初始化摄像头 - 自动检测可用设备
        print("[信息] 打开摄像头...")
        self.cap = None

        for idx in [0, 1]:
            print(f"[信息] 尝试 /dev/video{idx}...")
            for attempt in range(3):
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    print(f"[信息] 摄像头已打开，读取测试帧...")
                    # 增加超时等待
                    time.sleep(0.5)
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.cap = cap
                        print(f"[OK] 摄像头 /dev/video{idx} 可用")
                        break
                    else:
                        print(f"[警告] 无法读取帧，重试...")
                        cap.release()
                        time.sleep(1)
                else:
                    time.sleep(0.5)
            if self.cap:
                break

        if not self.cap:
            raise RuntimeError("无法打开摄像头 (尝试 /dev/video0 和 /dev/video1)")

        # 先不设置 MJPEG，使用默认格式测试
        print("[信息] 配置摄像头参数...")
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 10)

        print("[OK] 摄像头配置完成")

        self.recognizer = GestureRecognizer()
        self.oled = OLED()
        self.servo = ServoController()

        self.history = []
        self.stable = "idle"
        self.running = False
        self.lock = threading.Lock()

        print("[OK] 初始化完成")
        print("按 Ctrl+C 退出")
        print()

    def update(self, gesture):
        self.history.append(gesture)
        if len(self.history) > 5:
            self.history.pop(0)

        if len(self.history) >= 3:
            c = Counter(self.history)
            g, n = c.most_common(1)[0]
            if n >= 3 and g != self.stable:
                self.stable = g
                return g
        return None

    def show(self, gesture):
        expr = EXPRESSIONS.get(gesture, EXPRESSIONS["idle"])
        self.oled.show(*expr)

        servo_actions = {
            "palm": "scan",
            "fist": "lock",
            "peace": "sway",
            "thumbs_up": "nod",
            "thumbs_down": "shake",
            "point_up": "look_up",
            "point_down": "look_down",
            "point_left": "look_left",
            "point_right": "look_right",
            "ok": "nod",
            "idle": "center",
        }
        self.servo.execute_action(servo_actions.get(gesture, "center"))
        print(f"[识别] {expr[1]} {expr[0]}")

    def run(self):
        self.running = True
        self.show("idle")
        last = "idle"
        last_action = 0

        consecutive_failures = 0
        frame_count = 0
        last_status_print = time.time()

        try:
            print("[运行] 进入主循环...")
            while self.running:
                try:
                    ret, frame = self.cap.read()
                    if not ret:
                        consecutive_failures += 1
                        if consecutive_failures % 10 == 0:
                            print(f"[调试] 摄像头读取失败 {consecutive_failures} 次")

                        # 每100次失败尝试重启一次，但不退出程序
                        if consecutive_failures >= 100:
                            print(
                                f"[警告] 摄像头连续失败 {consecutive_failures} 次，尝试重启..."
                            )
                            try:
                                self.cap.release()
                            except:
                                pass
                            time.sleep(2)
                            self.cap = cv2.VideoCapture(1)
                            if self.cap.isOpened():
                                print("[OK] 摄像头重启成功")
                                consecutive_failures = 0
                            else:
                                print("[警告] 摄像头重启失败，将继续尝试...")
                                consecutive_failures = 0
                                time.sleep(5)
                        time.sleep(0.05)
                        continue

                    consecutive_failures = 0
                    frame_count += 1

                    # 每5秒打印一次状态
                    if time.time() - last_status_print >= 5:
                        print(f"[调试] 运行中，已处理 {frame_count} 帧")
                        last_status_print = time.time()
                    frame = cv2.flip(frame, 1)
                    gesture, frame = self.recognizer.recognize(frame)

                    stable = self.update(gesture)

                    if stable and stable != last:
                        now = time.time()
                        if now - last_action > 2:  # 2秒冷却
                            self.show(stable)
                            last = stable
                            last_action = now

                    # 3秒无手势回到待机
                    if (
                        last != "idle"
                        and gesture != last
                        and time.time() - last_action > 3
                    ):
                        self.show("idle")
                        last = "idle"
                        self.history = []

                    if not self.headless:
                        cv2.putText(
                            frame,
                            f"{gesture}",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )
                        cv2.imshow("Robot", frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
                    else:
                        time.sleep(0.03)

                except Exception as e:
                    print(f"[错误] 循环异常: {e}")
                    time.sleep(0.5)

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        print("\n[信息] 关闭中...")
        self.running = False
        self.cap.release()
        self.recognizer.close()
        self.oled.clear()
        self.servo.stop()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        cv2.destroyAllWindows()
        print("[OK] 已退出")


if __name__ == "__main__":
    import sys

    headless = "--headless" in sys.argv or "-h" in sys.argv

    try:
        robot = Robot(camera_idx=1, headless=headless)
        robot.run()
    except Exception as e:
        print(f"[错误] {e}")
        import traceback

        traceback.print_exc()
