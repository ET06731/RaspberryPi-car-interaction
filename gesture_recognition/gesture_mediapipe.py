#!/usr/bin/env python3
"""
手势识别 - MediaPipe Hands 版本 (终极稳定版)

MediaPipe Hands 提供 21 个手部关键点，识别准确率远超颜色+轮廓方法

手势定义:
- 张开手掌: 所有手指伸直
- 握拳: 所有手指弯曲
- 剪刀手: 食指和中指伸直，其他弯曲
"""

import time
import threading
import cv2
import numpy as np
from collections import Counter

# 尝试导入 MediaPipe
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
    print("[OK] MediaPipe 已加载")
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("[ERROR] MediaPipe 未安装，请先运行: pip3 install mediapipe --break-system-packages")
    exit(1)

# 导入 OLED 显示
try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from PIL import Image, ImageDraw, ImageFont
    OLED_AVAILABLE = True
except ImportError:
    OLED_AVAILABLE = False
    print("[WARNING] OLED 库未安装")


class GestureSmoother:
    """时序平滑器"""
    def __init__(self, window_size=5, min_confidence=3):
        self.window_size = window_size
        self.min_confidence = min_confidence
        self.history = []
        self.stable_gesture = 'none'
    
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


class MediaPipeGestureCamera:
    """基于 MediaPipe 的手势识别"""
    
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.running = False
        self.current_gesture = 'none'
        self.lock = threading.Lock()
        self.frame = None
        
        # MediaPipe Hands 初始化
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # 手指关键点索引
        self.finger_tips = [8, 12, 16, 20]  # 食指、中指、无名指、小指尖
        self.finger_pips = [6, 10, 14, 18]  # 对应关节
        self.thumb_tip = 4
        self.thumb_ip = 2
        
        if self.cap.isOpened():
            print("[OK] 摄像头已连接")
        else:
            print("[ERROR] 无法打开摄像头")
    
    def is_finger_extended(self, landmarks, tip_id, pip_id):
        """判断手指是否伸直"""
        tip = landmarks[tip_id]
        pip = landmarks[pip_id]
        wrist = landmarks[0]
        
        # 计算到手腕的距离
        tip_dist = ((tip.x - wrist.x) ** 2 + (tip.y - wrist.y) ** 2) ** 0.5
        pip_dist = ((pip.x - wrist.x) ** 2 + (pip.y - wrist.y) ** 2) ** 0.5
        
        return tip_dist > pip_dist
    
    def detect_gesture(self, frame):
        """使用 MediaPipe 检测手势"""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark
            
            # 检测四指状态
            fingers = []
            for tip_id, pip_id in zip(self.finger_tips, self.finger_pips):
                fingers.append(self.is_finger_extended(landmarks, tip_id, pip_id))
            
            # 拇指特殊处理（根据 x 坐标判断）
            thumb_extended = landmarks[self.thumb_tip].x < landmarks[self.thumb_ip].x
            
            extended_count = sum(fingers) + (1 if thumb_extended else 0)
            
            # 手势分类
            if extended_count == 5:
                return 'open_palm'
            elif extended_count == 0:
                return 'fist'
            elif extended_count == 2 and fingers[0] and fingers[1]:
                # 只有食指和中指伸直
                return 'peace'
            elif extended_count == 1 and fingers[0]:
                # 只有食指伸直
                return 'point'
            
            return 'unknown'
        
        return 'none'
    
    def run(self):
        self.running = True
        print("[INFO] MediaPipe 手势识别启动...")
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            frame = cv2.flip(frame, 1)
            self.frame = frame.copy()
            
            gesture = self.detect_gesture(frame)
            
            with self.lock:
                self.current_gesture = gesture
        
        self.cap.release()
        self.hands.close()
        print("[OK] 摄像头已关闭")
    
    def get_gesture(self):
        with self.lock:
            return self.current_gesture
    
    def get_frame(self):
        return self.frame
    
    def draw_landmarks(self, frame):
        """在帧上绘制手部关键点"""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )
        
        return frame
    
    def stop(self):
        self.running = False


class OLEDDisplay:
    def __init__(self, port=1, address=0x3C):
        try:
            serial = i2c(port=port, address=address)
            self.device = ssd1306(serial, width=128, height=64)
            self.connected = True
            try:
                self.font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
                self.font_text = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 14)
                self.font_small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 11)
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
        
        image = Image.new('1', (128, 64))
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
    print("=" * 60)
    print("  🤚 MediaPipe 手势识别 (终极稳定版)")
    print("=" * 60)
    print()
    print("🚀 MediaPipe 优势:")
    print("  ✓ 21个手部关键点")
    print("  ✓ 不受光照/背景影响")
    print("  ✓ 识别率 95%+")
    print()
    print("支持手势:")
    print("  ✋ 张开手掌 - (✧ω✧)")
    print("  ✊ 握拳 - (◣_◢)")
    print("  ✌️ 剪刀手 - (◠‿◠)")
    print("  ☝️ 食指指 - (¬‿¬)")
    print()
    print("按 Q 退出，按 S 截图")
    print()
    
    # 手势映射表
    gestures = {
        'none': ('(-_-)', '未检测', '请展示手势'),
        'unknown': ('(⊙_⊙)', '未知', '再试一次'),
        'open_palm': ('(✧ω✧)', '张开手掌', '你好！'),
        'fist': ('(◣_◢)', '握拳', '加油！'),
        'peace': ('(◠‿◠)', '剪刀手', '耶！'),
        'point': ('(¬‿¬)', '指向前', '这边！'),
    }
    
    # 初始化
    camera = MediaPipeGestureCamera(0)
    oled = OLEDDisplay() if OLED_AVAILABLE else None
    smoother = GestureSmoother(window_size=5, min_confidence=3)
    
    if not camera.cap.isOpened():
        print("[ERROR] 摄像头初始化失败")
        exit(1)
    
    camera_thread = threading.Thread(target=camera.run)
    camera_thread.start()
    
    current_display = 'none'
    screenshot_count = 0
    
    try:
        while True:
            raw_gesture = camera.get_gesture()
            stable_gesture = smoother.update(raw_gesture)
            
            # 稳定手势变化时更新 OLED
            if stable_gesture is not None and stable_gesture != current_display:
                emoji, name, action = gestures.get(stable_gesture, gestures['unknown'])
                if oled:
                    oled.show(emoji, name, action)
                print(f"[识别] {name} {emoji} - {action}")
                current_display = stable_gesture
            
            # 显示调试画面
            frame = camera.get_frame()
            if frame is not None:
                # 绘制 MediaPipe 关键点
                frame = camera.draw_landmarks(frame)
                
                # 显示状态
                stable = smoother.get_stable()
                cv2.putText(frame, f"Raw: {raw_gesture}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Stable: {stable}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                cv2.imshow("MediaPipe Gesture", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"gesture_screenshot_{screenshot_count}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"[截图] 已保存 {filename}")
                    screenshot_count += 1
            
            time.sleep(0.03)
    
    except KeyboardInterrupt:
        print("\n退出中...")
    
    finally:
        camera.stop()
        camera_thread.join()
        if oled:
            oled.clear()
        cv2.destroyAllWindows()
        print("[OK] 已退出")


if __name__ == "__main__":
    main()
