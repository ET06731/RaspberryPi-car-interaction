#!/usr/bin/env python3
"""
手势识别 + OLED 表情显示 (带时序平滑优化版)

改进点:
- 添加滑动窗口时序平滑，减少识别抖动
- 连续3帧相同才确认手势变化
- 保持原有的摄像头+OLED联动
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
    """时序平滑器 - 减少识别抖动"""
    
    def __init__(self, window_size=5, min_confidence=3):
        """
        Args:
            window_size: 滑动窗口大小 (帧数)
            min_confidence: 确认手势所需的最少相同帧数
        """
        self.window_size = window_size
        self.min_confidence = min_confidence
        self.history = []
        self.stable_gesture = 'none'  # 当前稳定的手势
        self.last_confirmed = 'none'   # 上次确认的手势（用于去重显示）
    
    def update(self, raw_gesture):
        """
        更新手势历史，返回平滑后的结果
        
        Args:
            raw_gesture: 当前帧的原始识别结果
            
        Returns:
            str: 稳定后的手势，或 None（如果还不够稳定）
        """
        self.history.append(raw_gesture)
        
        # 保持窗口大小
        if len(self.history) > self.window_size:
            self.history.pop(0)
        
        # 窗口填满后才做判断
        if len(self.history) < self.window_size:
            return None
        
        # 投票：取众数
        counter = Counter(self.history)
        most_common = counter.most_common(1)[0]
        gesture, count = most_common
        
        # 需要达到最小置信度
        if count >= self.min_confidence:
            # 只有当稳定手势改变时才返回
            if gesture != self.stable_gesture:
                self.stable_gesture = gesture
                return gesture
        
        return None  # 不够稳定或没变化
    
    def get_stable(self):
        """获取当前稳定的手势（不触发更新）"""
        return self.stable_gesture
    
    def reset(self):
        """重置历史"""
        self.history.clear()
        self.stable_gesture = 'none'


class GestureCamera:
    """手势摄像头 - 原始检测逻辑不变"""
    
    def __init__(self, camera_index=0):
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
        """原始检测逻辑（HSV + 凸包缺陷）"""
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
        """摄像头运行线程"""
        self.running = True
        print("[INFO] 摄像头运行中，请展示手势...")
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            frame = cv2.flip(frame, 1)
            self.frame = frame  # 保存当前帧供调试显示
            
            gesture = self.detect_gesture(frame)
            
            with self.lock:
                self.current_gesture = gesture
        
        self.cap.release()
        print("[OK] 摄像头已关闭")
    
    def get_gesture(self):
        """获取当前原始识别结果"""
        with self.lock:
            return self.current_gesture
    
    def get_frame(self):
        """获取当前帧（用于调试显示）"""
        return self.frame
    
    def stop(self):
        self.running = False


class OLEDDisplay:
    """OLED 显示模块"""
    
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
        """显示手势对应的颜文字"""
        if not self.connected:
            return
        
        image = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 127, 63), outline=0, fill=0)
        
        # 颜文字（居中上方）
        bbox = draw.textbbox((0, 0), emoji, font=self.font_emoji)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 0), emoji, font=self.font_emoji, fill=255)
        
        # 手势名称（居中）
        bbox = draw.textbbox((0, 0), name, font=self.font_text)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 30), name, font=self.font_text, fill=255)
        
        # 动作描述（底部）
        bbox = draw.textbbox((0, 0), action, font=self.font_small)
        x = (128 - (bbox[2] - bbox[0])) // 2
        draw.text((x, 48), action, font=self.font_small, fill=255)
        
        self.device.display(image)
    
    def clear(self):
        if self.connected:
            self.device.clear()


# ==================== 主程序 ====================

def main():
    print("=" * 60)
    print("  🤚 手势识别 + OLED 表情 (时序平滑版)")
    print("=" * 60)
    print()
    print("改进点:")
    print("  ✓ 5帧滑动窗口平滑")
    print("  ✓ 至少3帧相同才确认手势")
    print("  ✓ 大幅减少识别抖动")
    print()
    print("说明: 张开手掌 / 握拳 / 剪刀手")
    print("按 Ctrl+C 退出")
    print()
    
    # 手势映射表
    gestures = {
        'none': ('(-_-)', '未识别', '请展示手势'),
        'open_palm': ('(✧ω✧)', '张开手掌', '你好！'),
        'fist': ('(◣_◢)', '握拳', '加油！'),
        'peace': ('(◠‿◠)', '剪刀手', '耶！'),
    }
    
    # 初始化硬件
    camera = GestureCamera(0)
    oled = OLEDDisplay()
    smoother = GestureSmoother(window_size=5, min_confidence=3)
    
    if not camera.cap.isOpened():
        print("[ERROR] 摄像头初始化失败")
        exit(1)
    
    # 启动摄像头线程
    camera_thread = threading.Thread(target=camera.run)
    camera_thread.start()
    
    # 当前显示的手势（用于避免重复刷新OLED）
    current_display = 'none'
    
    try:
        while True:
            # 1. 获取原始识别结果
            raw_gesture = camera.get_gesture()
            
            # 2. 时序平滑
            stable_gesture = smoother.update(raw_gesture)
            
            # 3. 只有当稳定手势变化时才更新显示
            if stable_gesture is not None and stable_gesture != current_display:
                emoji, name, action = gestures.get(stable_gesture, gestures['none'])
                oled.show(emoji, name, action)
                print(f"[识别] {name} {emoji} - {action}")
                current_display = stable_gesture
            
            # 4. 可选：显示调试画面
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
            
            time.sleep(0.03)  # ~30fps
    
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
