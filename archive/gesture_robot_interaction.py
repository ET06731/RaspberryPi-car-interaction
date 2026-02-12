#!/usr/bin/env python3
"""
🤖 智能手势交互小车 - MediaPipe 版

功能:
1. 使用 MediaPipe 进行手势识别 (手掌/拳头/剪刀手/点赞/数字)
2. OLED 显示对应的表情和反馈
3. 舵机云台跟随手势移动 (抬头/低头/左转/右转)
4. 像一个有情感的机器人

手势对应:
- ✋ 张开手掌: 友好模式，舵机环顾四周
- ✊ 握拳: 警惕模式，舵机锁定
- ✌️ 剪刀手: 开心，舵机摇摆
- 👍 点赞: 确认，点头
- 👎 倒赞: 摇头
- ☝️ 食指向上: 抬头看
- 👇 食指向下: 低头看
- 👈 👉 左右指: 左右转头
- 🖐️ 五指张开: 打招呼，左右摇摆

运行: cd ~/rpi-car-project && ~/.local/bin/uv run python3 gesture_robot_interaction.py
"""

import os
import cv2
import time
import threading
import numpy as np
from collections import Counter
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

# 尝试导入 RPi.GPIO，如果在非树莓派环境则使用模拟模式
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[警告] RPi.GPIO 不可用，使用模拟模式")

# 尝试导入 MediaPipe
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("[错误] MediaPipe 未安装")


# ==================== 硬件配置 ====================
# 舵机引脚 (BCM 编码)
SERVO_PIN = 23

# 舵机角度范围
SERVO_CENTER = 90      # 中间位置
SERVO_LEFT_MAX = 0     # 最左
SERVO_RIGHT_MAX = 180  # 最右
SERVO_UP_MAX = 60      # 抬头 (水平舵机的话是另一个轴)
SERVO_DOWN_MAX = 120   # 低头

# 手势平滑窗口大小
GESTURE_WINDOW_SIZE = 5


# ==================== 表情配置 ====================
ROBOT_EXPRESSIONS = {
    'idle': {
        'emoji': '(-_-)',
        'name': '待机',
        'action': '等待指令...',
        'servo_action': 'center'
    },
    'palm': {
        'emoji': '(✧ω✧)',
        'name': '手掌',
        'action': '你好！朋友',
        'servo_action': 'scan'  # 扫描四周
    },
    'fist': {
        'emoji': '(◣_◢)',
        'name': '握拳',
        'action': '检测到威胁',
        'servo_action': 'lock'  # 锁定
    },
    'peace': {
        'emoji': '(◠‿◠)',
        'name': '剪刀手',
        'action': '耶！开心',
        'servo_action': 'sway'  # 摇摆
    },
    'thumbs_up': {
        'emoji': '(｡◕‿◕｡)',
        'name': '点赞',
        'action': '收到！',
        'servo_action': 'nod'  # 点头
    },
    'thumbs_down': {
        'emoji': '(◕︵◕)',
        'name': '倒赞',
        'action': '不太认同',
        'servo_action': 'shake'  # 摇头
    },
    'point_up': {
        'emoji': '(⊙_⊙)',
        'name': '指上',
        'action': '往上看',
        'servo_action': 'look_up'  # 抬头
    },
    'point_down': {
        'emoji': '(¬_¬)',
        'name': '指下',
        'action': '往下看',
        'servo_action': 'look_down'  # 低头
    },
    'point_left': {
        'emoji': '(◣_◢)',
        'name': '指左',
        'action': '看左边',
        'servo_action': 'look_left'  # 左转
    },
    'point_right': {
        'emoji': '(◢_◣)',
        'name': '指右',
        'action': '看右边',
        'servo_action': 'look_right'  # 右转
    },
    'ok': {
        'emoji': '(⌒‿⌒)',
        'name': 'OK',
        'action': '没问题',
        'servo_action': 'nod_twice'  # 点两下头
    },
    'unknown': {
        'emoji': '(⊙_⊙)？',
        'name': '未知',
        'action': '不太明白',
        'servo_action': 'confused'  # 疑惑摇头
    }
}


# ==================== 舵机控制器 ====================
class ServoController:
    """舵机云台控制器"""
    
    def __init__(self, pin=SERVO_PIN):
        self.pin = pin
        self.current_angle = SERVO_CENTER
        self.target_angle = SERVO_CENTER
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.action_lock = threading.Lock()  # 动作执行锁
        self.is_busy = False  # 是否正在执行动作
        
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(self.pin, GPIO.OUT)
                self.pwm = GPIO.PWM(self.pin, 50)  # 50Hz
                self.pwm.start(0)
                self._set_angle(SERVO_CENTER)
                print("[OK] 舵机初始化成功")
            except Exception as e:
                print(f"[错误] 舵机初始化失败: {e}")
                self.pwm = None
        else:
            self.pwm = None
            print("[模拟] 舵机控制器 (无硬件)")
    
    def _angle_to_duty_cycle(self, angle):
        """将角度转换为占空比 (2.5-12.5 对应 0-180度)"""
        return 2.5 + (angle / 180.0) * 10.0
    
    def _set_angle(self, angle):
        """直接设置角度"""
        angle = max(SERVO_LEFT_MAX, min(SERVO_RIGHT_MAX, angle))
        if self.pwm:
            duty = self._angle_to_duty_cycle(angle)
            self.pwm.ChangeDutyCycle(duty)
            time.sleep(0.02)
            self.pwm.ChangeDutyCycle(0)  # 停止信号，防止抖动
        self.current_angle = angle
    
    def set_angle(self, angle, smooth=True):
        """设置目标角度"""
        with self.lock:
            self.target_angle = max(SERVO_LEFT_MAX, min(SERVO_RIGHT_MAX, angle))
    
    def get_current_angle(self):
        """获取当前角度"""
        with self.lock:
            return self.current_angle
    
    def _smooth_move_thread(self):
        """平滑移动线程"""
        while self.running:
            with self.lock:
                diff = self.target_angle - self.current_angle
            
            if abs(diff) > 1:
                step = 2 if diff > 0 else -2
                new_angle = self.current_angle + step
                self._set_angle(new_angle)
            else:
                time.sleep(0.02)
    
    def start(self):
        """启动平滑控制线程"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._smooth_move_thread, daemon=True)
            self.thread.start()
    
    def stop(self):
        """停止舵机"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.pwm:
            self.pwm.stop()
    
    # ==================== 情感动作 ====================
    def action_center(self):
        """回中"""
        self.set_angle(SERVO_CENTER)
    
    def action_look_left(self):
        """向左看"""
        self.set_angle(SERVO_LEFT_MAX + 30)
    
    def action_look_right(self):
        """向右看"""
        self.set_angle(SERVO_RIGHT_MAX - 30)
    
    def action_nod(self):
        """点头 (快速左右摆动)"""
        if not GPIO_AVAILABLE:
            return
        for _ in range(2):
            self._set_angle(SERVO_CENTER - 20)
            time.sleep(0.15)
            self._set_angle(SERVO_CENTER + 20)
            time.sleep(0.15)
        self._set_angle(SERVO_CENTER)
    
    def action_shake(self):
        """摇头 (左右摆动)"""
        if not GPIO_AVAILABLE:
            return
        for _ in range(3):
            self._set_angle(SERVO_LEFT_MAX + 45)
            time.sleep(0.2)
            self._set_angle(SERVO_RIGHT_MAX - 45)
            time.sleep(0.2)
        self._set_angle(SERVO_CENTER)
    
    def action_sway(self):
        """摇摆 (慢速摆动)"""
        if not GPIO_AVAILABLE:
            return
        for _ in range(2):
            self._set_angle(SERVO_CENTER - 30)
            time.sleep(0.3)
            self._set_angle(SERVO_CENTER + 30)
            time.sleep(0.3)
        self._set_angle(SERVO_CENTER)
    
    def action_scan(self):
        """扫描 (环顾四周)"""
        if not GPIO_AVAILABLE:
            return
        for angle in range(SERVO_LEFT_MAX, SERVO_RIGHT_MAX + 1, 5):
            self._set_angle(angle)
            time.sleep(0.05)
        for angle in range(SERVO_RIGHT_MAX, SERVO_LEFT_MAX - 1, -5):
            self._set_angle(angle)
            time.sleep(0.05)
        self._set_angle(SERVO_CENTER)
    
    def action_lock(self):
        """锁定 (居中不动)"""
        self._set_angle(SERVO_CENTER)
    
    def action_confused(self):
        """疑惑 (快速小幅度摆动)"""
        if not GPIO_AVAILABLE:
            return
        for _ in range(4):
            self._set_angle(SERVO_CENTER - 10)
            time.sleep(0.1)
            self._set_angle(SERVO_CENTER + 10)
            time.sleep(0.1)
        self._set_angle(SERVO_CENTER)
    
    def execute_action(self, action_name):
        """执行情感动作"""
        action_method = getattr(self, f'action_{action_name}', None)
        if action_method:
            # 在新线程中执行，避免阻塞主循环
            thread = threading.Thread(target=action_method, daemon=True)
            thread.start()


# ==================== OLED 显示器 ====================
class OLEDDisplay:
    """OLED 表情显示器"""
    
    def __init__(self, port=1, address=0x3C):
        try:
            serial = i2c(port=port, address=address)
            self.device = ssd1306(serial, width=128, height=64)
            self.connected = True
            self._load_fonts()
            print("[OK] OLED 已连接")
        except Exception as e:
            self.connected = False
            self.device = None
            print(f"[警告] OLED 连接失败: {e}")
    
    def _load_fonts(self):
        """加载字体"""
        try:
            self.font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
            self.font_name = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 14)
            self.font_action = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 11)
        except:
            try:
                self.font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
                self.font_name = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
                self.font_action = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
            except:
                self.font_emoji = ImageFont.load_default()
                self.font_name = ImageFont.load_default()
                self.font_action = ImageFont.load_default()
    
    def show_expression(self, emoji, name, action):
        """显示表情"""
        if not self.connected:
            return
        
        image = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(image)
        
        # 清屏
        draw.rectangle((0, 0, 127, 63), outline=0, fill=0)
        
        # 绘制表情 (居中)
        bbox = draw.textbbox((0, 0), emoji, font=self.font_emoji)
        text_width = bbox[2] - bbox[0]
        x = (128 - text_width) // 2
        draw.text((x, 0), emoji, font=self.font_emoji, fill=255)
        
        # 绘制手势名称
        bbox = draw.textbbox((0, 0), name, font=self.font_name)
        text_width = bbox[2] - bbox[0]
        x = (128 - text_width) // 2
        draw.text((x, 30), name, font=self.font_name, fill=255)
        
        # 绘制动作描述
        bbox = draw.textbbox((0, 0), action, font=self.font_action)
        text_width = bbox[2] - bbox[0]
        x = (128 - text_width) // 2
        draw.text((x, 48), action, font=self.font_action, fill=255)
        
        self.device.display(image)
    
    def clear(self):
        """清屏"""
        if self.connected:
            self.device.clear()


# ==================== 手势识别器 ====================
class GestureRecognizer:
    """MediaPipe 手势识别器"""
    
    def __init__(self):
        if not MEDIAPIPE_AVAILABLE:
            raise RuntimeError("MediaPipe 未安装")
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        print("[OK] MediaPipe Hands 初始化成功")
    
    def recognize(self, frame):
        """
        识别手势
        返回: (gesture_name, annotated_frame)
        """
        # 转换颜色空间
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        gesture = 'idle'
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 绘制手部关键点
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # 识别手势
                gesture = self._classify_gesture(hand_landmarks)
        
        return gesture, frame
    
    def _classify_gesture(self, landmarks):
        """根据关键点分类手势"""
        # 获取关键点坐标
        points = []
        for landmark in landmarks.landmark:
            points.append((landmark.x, landmark.y, landmark.z))
        
        # 手指关键点索引
        THUMB_TIP = 4
        INDEX_TIP = 8
        MIDDLE_TIP = 12
        RING_TIP = 16
        PINKY_TIP = 20
        
        WRIST = 0
        INDEX_MCP = 5
        MIDDLE_MCP = 9
        RING_MCP = 13
        PINKY_MCP = 17
        
        # 计算手指是否伸直 (指尖到手腕的距离 > 指根到手腕的距离)
        def is_finger_extended(tip_idx, mcp_idx):
            tip_to_wrist = np.linalg.norm(np.array(points[tip_idx]) - np.array(points[WRIST]))
            mcp_to_wrist = np.linalg.norm(np.array(points[mcp_idx]) - np.array(points[WRIST]))
            return tip_to_wrist > mcp_to_wrist * 1.3
        
        # 判断每个手指状态
        thumb_extended = points[THUMB_TIP][0] > points[THUMB_TIP - 2][0] if points[THUMB_TIP][0] > points[WRIST][0] else points[THUMB_TIP][0] < points[THUMB_TIP - 2][0]
        index_extended = is_finger_extended(INDEX_TIP, INDEX_MCP)
        middle_extended = is_finger_extended(MIDDLE_TIP, MIDDLE_MCP)
        ring_extended = is_finger_extended(RING_TIP, RING_MCP)
        pinky_extended = is_finger_extended(PINKY_TIP, PINKY_MCP)
        
        extended_fingers = sum([index_extended, middle_extended, ring_extended, pinky_extended])
        
        # 手势分类逻辑
        if extended_fingers == 0:
            return 'fist'  # 握拳
        elif extended_fingers == 1:
            if index_extended:
                # 判断食指指向方向
                dx = points[INDEX_TIP][0] - points[WRIST][0]
                dy = points[INDEX_TIP][1] - points[WRIST][1]
                if abs(dx) > abs(dy):
                    return 'point_right' if dx > 0 else 'point_left'
                else:
                    return 'point_down' if dy > 0 else 'point_up'
            elif thumb_extended:
                # 判断拇指方向
                if points[THUMB_TIP][1] < points[THUMB_TIP - 2][1]:
                    return 'thumbs_up'
                else:
                    return 'thumbs_down'
            return 'unknown'
        elif extended_fingers == 2:
            if index_extended and middle_extended:
                return 'peace'  # 剪刀手
            elif thumb_extended and index_extended:
                return 'ok'  # OK
            return 'unknown'
        elif extended_fingers >= 4:
            return 'palm'  # 手掌
        
        return 'unknown'
    
    def close(self):
        """释放资源"""
        self.hands.close()


# ==================== 手势平滑器 ====================
class GestureSmoother:
    """手势平滑器，减少抖动"""
    
    def __init__(self, window_size=GESTURE_WINDOW_SIZE, min_confidence=3):
        self.window_size = window_size
        self.min_confidence = min_confidence
        self.history = []
        self.stable_gesture = 'idle'
    
    def update(self, raw_gesture):
        """更新并返回稳定手势"""
        self.history.append(raw_gesture)
        if len(self.history) > self.window_size:
            self.history.pop(0)
        
        if len(self.history) < self.min_confidence:
            return None
        
        # 统计最常见的手势
        counter = Counter(self.history)
        most_common = counter.most_common(1)[0]
        gesture, count = most_common
        
        if count >= self.min_confidence and gesture != self.stable_gesture:
            self.stable_gesture = gesture
            return gesture
        
        return None
    
    def get_stable(self):
        """获取当前稳定手势"""
        return self.stable_gesture
    
    def reset(self):
        """重置历史"""
        self.history = []
        self.stable_gesture = 'idle'


# ==================== 主程序 ====================
class GestureRobotInteraction:
    """手势交互机器人主类"""
    
    def __init__(self, camera_index=0, headless=False):
        print("=" * 60)
        print("  🤖 智能手势交互小车")
        print("  MediaPipe 手势识别 + 舵机云台 + OLED 表情")
        print("=" * 60)
        print()
        
        # 检测是否有图形界面
        self.headless = headless or not os.environ.get('DISPLAY')
        if self.headless:
            print("[信息] 无图形界面模式 (headless)")
            print("[信息] OLED 和 舵机正常工作，不显示窗口")
        print()
        
        # 初始化各个模块
        self.camera = self._init_camera(camera_index)
        if not self.camera or not self.camera.isOpened():
            raise RuntimeError("无法打开摄像头")
        
        self.camera_index = camera_index
        self.gesture_recognizer = GestureRecognizer()
        self.oled = OLEDDisplay()
        self.servo = ServoController()
        self.smoother = GestureSmoother()
        
        self.running = False
        self.current_gesture = 'idle'
        self.frame = None
        self.lock = threading.Lock()
        
        print("[OK] 所有模块初始化完成")
        print()
        print("手势指令:")
        print("  ✋ 手掌张开 - 友好模式，环顾四周")
        print("  ✊ 握拳     - 警惕模式，锁定")
        print("  ✌️ 剪刀手   - 开心摇摆")
        print("  👍 点赞    - 确认，点头")
        print("  👎 倒赞    - 摇头")
        print("  ☝️ 指上     - 抬头")
        print("  👇 指下     - 低头")
        print("  👈 👉 指左右 - 左右转头")
        print()
        if not self.headless:
            print("按 Q 退出")
        else:
            print("按 Ctrl+C 退出")
        print()
    
    def _find_available_camera(self):
        """自动查找可用的摄像头"""
        # 首先尝试指定的索引
        for idx in [0, 1]:
            print(f"[信息] 尝试 /dev/video{idx}...")
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"[OK] 找到可用摄像头 /dev/video{idx}: {frame.shape}")
                    cap.release()
                    return idx
                cap.release()
        
        return None
    
    def _init_camera(self, camera_index):
        """初始化摄像头，带重试逻辑"""
        # 如果指定了索引，先尝试；否则自动查找
        if camera_index is not None:
            indices_to_try = [camera_index] + [i for i in [0, 1] if i != camera_index]
        else:
            indices_to_try = [0, 1]
        
        for idx in indices_to_try:
            print(f"[信息] 初始化摄像头 /dev/video{idx}...")
            
            for attempt in range(2):
                cap = cv2.VideoCapture(idx)
                
                if not cap.isOpened():
                    print(f"[警告] 第 {attempt+1} 次尝试失败，重试...")
                    time.sleep(1)
                    continue
                
                # 读取一帧测试（不设置参数，使用默认）
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"[OK] 摄像头初始化成功: {frame.shape}")
                    self.camera_index = idx  # 保存实际使用的索引
                    return cap
                
                cap.release()
                print(f"[警告] 第 {attempt+1} 次测试读取失败，重试...")
                time.sleep(1)
        
        return None
    
    def _restart_camera(self):
        """重启摄像头"""
        print("[信息] 尝试重启摄像头...")
        
        with self.lock:
            if self.camera:
                self.camera.release()
        
        time.sleep(1)
        
        self.camera = self._init_camera(self.camera_index)
        
        if self.camera and self.camera.isOpened():
            print("[OK] 摄像头重启成功")
            return True
        else:
            print("[错误] 摄像头重启失败")
            return False
    
    def _camera_loop(self):
        """摄像头采集循环 - 带帧率限制和自动恢复"""
        frame_interval = 1.0 / 10  # 限制 10 FPS，减轻 CPU 和 USB 带宽
        last_frame_time = 0
        frame_count = 0
        last_fps_time = time.time()
        consecutive_failures = 0
        max_failures = 10
        
        while self.running:
            current_time = time.time()
            
            # 帧率限制
            if current_time - last_frame_time < frame_interval:
                time.sleep(0.005)
                continue
            
            last_frame_time = current_time
            
            # 检查摄像头是否有效
            if not self.camera or not self.camera.isOpened():
                print("[错误] 摄像头已断开")
                if not self._restart_camera():
                    time.sleep(2)
                    continue
                consecutive_failures = 0
            
            ret, frame = self.camera.read()
            if not ret or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print(f"[错误] 连续 {max_failures} 次读取失败，尝试重启摄像头")
                    self._restart_camera()
                    consecutive_failures = 0
                else:
                    print(f"[警告] 摄像头读取失败 ({consecutive_failures}/{max_failures})")
                time.sleep(0.1)
                continue
            
            # 成功读取，重置失败计数
            consecutive_failures = 0
            
            # 镜像翻转
            frame = cv2.flip(frame, 1)
            
            # 识别手势（带超时保护）
            try:
                gesture, annotated_frame = self.gesture_recognizer.recognize(frame)
            except Exception as e:
                print(f"[错误] 手势识别失败: {e}")
                gesture = 'idle'
                annotated_frame = frame
            
            with self.lock:
                self.current_gesture = gesture
                self.frame = annotated_frame
            
            # FPS 统计
            frame_count += 1
            if current_time - last_fps_time >= 5.0:
                fps = frame_count / (current_time - last_fps_time)
                print(f"[性能] 摄像头 FPS: {fps:.1f}")
                frame_count = 0
                last_fps_time = current_time
                
                # 如果 FPS 太低，尝试重启摄像头
                if fps < 5.0:
                    print("[警告] 摄像头 FPS 过低，尝试重启")
                    self._restart_camera()
    
    def _update_display(self, gesture):
        """更新 OLED 显示和舵机动作"""
        if gesture not in ROBOT_EXPRESSIONS:
            gesture = 'unknown'
        
        expr = ROBOT_EXPRESSIONS[gesture]
        
        # 更新 OLED
        self.oled.show_expression(expr['emoji'], expr['name'], expr['action'])
        
        # 执行舵机动作
        self.servo.execute_action(expr['servo_action'])
        
        print(f"[识别] {expr['name']} {expr['emoji']} - {expr['action']}")
    
    def run(self):
        """主循环"""
        self.running = True
        
        # 启动摄像头线程
        camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        camera_thread.start()
        
        # 启动舵机平滑控制
        self.servo.start()
        
        # 初始化显示
        self._update_display('idle')
        
        last_gesture = 'idle'
        last_action_time = time.time()
        gesture_start_time = time.time()
        action_cooldown = 2.0  # 动作冷却时间（秒）
        last_health_check = time.time()
        loop_count = 0
        
        print("[运行中] 请在摄像头前展示手势...")
        print("[提示] 每次动作后有 2 秒冷却时间")
        
        try:
            while self.running:
                loop_start = time.time()
                loop_count += 1
                
                with self.lock:
                    gesture = self.current_gesture
                    frame = self.frame
                
                # 平滑手势
                stable_gesture = self.smoother.update(gesture)
                current_time = time.time()
                
                # 检测到手势变化或保持手势超过冷却时间
                should_trigger = False
                
                if stable_gesture:
                    if stable_gesture != last_gesture:
                        # 手势发生变化
                        should_trigger = True
                        gesture_start_time = current_time
                    elif stable_gesture != 'idle' and (current_time - last_action_time) > action_cooldown:
                        # 保持非待机手势超过冷却时间，重复触发
                        should_trigger = True
                        print(f"[重复触发] 保持 {stable_gesture} 手势 {action_cooldown} 秒")
                
                if should_trigger:
                    self._update_display(stable_gesture)
                    last_gesture = stable_gesture
                    last_action_time = current_time
                
                # 如果长时间没有检测到手势，重置为待机
                if last_gesture != 'idle' and gesture != last_gesture and (current_time - gesture_start_time) > 3.0:
                    print(f"[丢失手势] 回到待机状态")
                    self.smoother.reset()
                    last_gesture = 'idle'
                    self._update_display('idle')
                
                # 显示画面（仅在图形界面模式下）
                if not self.headless and frame is not None:
                    stable = self.smoother.get_stable()
                    cv2.putText(frame, f"Gesture: {gesture}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Stable: {stable}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    
                    # 显示冷却状态
                    cooldown_remaining = max(0, action_cooldown - (current_time - last_action_time))
                    if cooldown_remaining > 0:
                        cv2.putText(frame, f"Cooldown: {cooldown_remaining:.1f}s", (10, 90),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    cv2.imshow("Gesture Robot", frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                # 定期健康检查（每 10 秒）
                if current_time - last_health_check >= 10.0:
                    loop_time = current_time - last_health_check
                    loop_fps = loop_count / loop_time
                    print(f"[健康检查] 主循环 FPS: {loop_fps:.1f}, 循环次数: {loop_count}")
                    
                    # 如果主循环 FPS 太低，说明有问题
                    if loop_fps < 20:
                        print("[警告] 主循环性能下降，尝试清理资源...")
                        self.smoother.reset()  # 重置手势历史
                    
                    last_health_check = current_time
                    loop_count = 0
                
                # 计算本次循环耗时，动态调整睡眠时间
                loop_duration = time.time() - loop_start
                sleep_time = max(0, 0.03 - loop_duration)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n[信息] 收到中断信号")
        
        finally:
            self.shutdown()
    
    def shutdown(self):
        """关闭所有资源"""
        print("\n[信息] 正在关闭...")
        self.running = False
        
        self.camera.release()
        self.gesture_recognizer.close()
        self.oled.clear()
        self.servo.stop()
        
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        
        cv2.destroyAllWindows()
        print("[OK] 已安全退出")


# ==================== 入口 ====================
if __name__ == "__main__":
    import sys
    
    # 检查命令行参数
    headless = '--headless' in sys.argv or '-h' in sys.argv
    
    try:
        robot = GestureRobotInteraction(camera_index=0, headless=headless)
        robot.run()
    except Exception as e:
        print(f"[错误] {e}")
        import traceback
        traceback.print_exc()
