#!/usr/bin/env python3
import time
import threading
import cv2
import numpy as np
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

class GestureCamera:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.running = False
        self.current_gesture = 'none'
        self.lock = threading.Lock()
        
        if self.cap.isOpened():
            print("[OK] 摄像头已连接")
        else:
            print("[ERROR] 无法打开摄像头")
    
    def detect_gesture(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
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
                        
                        if finger_count >= 4:
                            return 'open_palm'
                        elif finger_count <= 1 and aspect_ratio < 0.8:
                            return 'fist'
                        elif finger_count == 2:
                            return 'peace'
        
        return 'none'
    
    def run(self):
        self.running = True
        print("[INFO] 摄像头运行中，请展示手势...")
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            frame = cv2.flip(frame, 1)
            gesture = self.detect_gesture(frame)
            
            with self.lock:
                self.current_gesture = gesture
        
        self.cap.release()
        print("[OK] 摄像头已关闭")
    
    def get_gesture(self):
        with self.lock:
            return self.current_gesture
    
    def stop(self):
        self.running = False

class OLEDDisplay:
    def __init__(self):
        serial = i2c(port=1, address=0x3C)
        self.device = ssd1306(serial, width=128, height=64)
        try:
            self.font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
            self.font_text = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 14)
            self.font_small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 11)
        except:
            self.font_emoji = ImageFont.load_default()
            self.font_text = ImageFont.load_default()
            self.font_small = ImageFont.load_default()
        print("[OK] OLED 已连接")
    
    def show(self, emoji, name, action):
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
        self.device.clear()

print("=" * 50)
print("手势识别 + OLED 表情")
print("=" * 50)
print()

gestures = {
    'none': ('(-_-)', '未识别', '请展示手势'),
    'open_palm': ('(✧ω✧)', '张开手掌', '你好！'),
    'fist': ('(◣_◢)', '握拳', '加油！'),
    'peace': ('(◠‿◠)', '剪刀手', '耶！'),
}

camera = GestureCamera(0)
oled = OLEDDisplay()

if not camera.cap.isOpened():
    print("[ERROR] 摄像头初始化失败")
    exit(1)

print("说明: 张开手掌/握拳/剪刀手")
print("按 Ctrl+C 退出")
print()

camera_thread = threading.Thread(target=camera.run)
camera_thread.start()

last_gesture = None

try:
    while True:
        gesture = camera.get_gesture()
        
        if gesture != last_gesture:
            emoji, name, action = gestures.get(gesture, gestures['none'])
            oled.show(emoji, name, action)
            
            if gesture != 'none':
                print(f"识别到: {name} {emoji}")
            
            last_gesture = gesture
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n退出中...")

finally:
    camera.stop()
    camera_thread.join()
    oled.clear()
    print("[OK] 已退出")
