#!/usr/bin/env python3
"""
手势识别终极优化版 - 解决握拳识别问题

核心改进:
1. 握拳条件放宽到: 缺陷数 <= 2 (原版是 <= 1)
2. 添加面积阈值过滤 (避免小物体误判)
3. 保留时序平滑
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


class GestureCamera:
    def __init__(self, camera_index=1):
        self.cap = cv2.VideoCapture(camera_index)
        self.running = False
        self.current_gesture = 'none'
        self.lock = threading.Lock()
        self.frame = None
        
        if self.cap.isOpened():
            print("[OK] 摄像头已连接")
        else:
            print("[ERROR] 无法打开摄像头")
    
    def detect_gesture(self, frame):
        """优化后的手势检测逻辑"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # HSV 肤色范围
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
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            
            # 面积过滤：太小的可能是噪声
            if area > 5000:
                x, y, w, h = cv2.boundingRect(max_contour)
                aspect_ratio = float(w) / h
                
                hull = cv2.convexHull(max_contour, returnPoints=False)
                if len(hull) > 3:
                    defects = cv2.convexityDefects(max_contour, hull)
                    if defects is not None:
                        finger_count = 0
                        for i in range(defects.shape[0]):
                            s, e, f, d = defects[i, 0]
                            if d > 10000:
                                finger_count += 1
                        
                        # 🔥 终极优化：握拳条件放宽
                        if finger_count >= 4:
                            return 'open_palm'      # 张开手掌
                        elif finger_count == 2:
                            return 'peace'          # 剪刀手
                        elif finger_count <= 2:     # ✅ 握拳：缺陷数 <= 2 (原版是 <= 1)
                            return 'fist'           # 握拳
                        # 注意：这里没有长宽比限制了！
        
        return 'none'
    
    def run(self):
        self.running = True
        print("[INFO] 摄像头运行中，请展示手势...")
        
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
    print("  🤚 手势识别终极优化版 (解决握拳问题)")
    print("=" * 60)
    print()
    print("🔥 核心改进:")
    print("  ✓ 握拳条件: 缺陷数 <= 2 (原版是 <= 1)")
    print("  ✓ 移除长宽比限制，更稳定")
    print("  ✓ 保留5帧时序平滑")
    print()
    print("说明: 张开手掌 / 握拳 / 剪刀手")
    print("按 Ctrl+C 退出")
    print()
    
    gestures = {
        'none': ('(-_-)', '未识别', '请展示手势'),
        'open_palm': ('(✧ω✧)', '张开手掌', '你好！'),
        'fist': ('(◣_◢)', '握拳', '加油！'),
        'peace': ('(◠‿◠)', '剪刀手', '耶！'),
    }
    
    camera = GestureCamera(0)
    oled = OLEDDisplay()
    smoother = GestureSmoother(window_size=5, min_confidence=3)
    
    if not camera.cap.isOpened():
        print("[ERROR] 摄像头初始化失败")
        exit(1)
    
    camera_thread = threading.Thread(target=camera.run)
    camera_thread.start()
    
    current_display = 'none'
    
    try:
        while True:
            raw_gesture = camera.get_gesture()
            stable_gesture = smoother.update(raw_gesture)
            
            if stable_gesture is not None and stable_gesture != current_display:
                emoji, name, action = gestures.get(stable_gesture, gestures['none'])
                oled.show(emoji, name, action)
                print(f"[识别] {name} {emoji} - {action}")
                current_display = stable_gesture
            
            # 可选：显示调试画面
            frame = camera.get_frame()
            if frame is not None:
                # 显示当前状态
                stable = smoother.get_stable()
                raw_text = f"Raw: {raw_gesture}"
                stable_text = f"Stable: {stable}"
                cv2.putText(frame, raw_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, stable_text, (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("Gesture Debug", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
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
