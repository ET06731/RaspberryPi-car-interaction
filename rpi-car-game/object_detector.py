#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
物品检测模块
基于OpenCV颜色识别的物品检测
"""

import cv2
import numpy as np


class ObjectDetector:
    """物品检测器"""

    # 预设物品颜色范围 (HSV格式)
    COLOR_RANGES = {
        "纸巾": {  # 白色/浅色
            "lower": np.array([0, 0, 180]),
            "upper": np.array([180, 50, 255])
        },
        "萝卜": {  # 橙色
            "lower": np.array([0, 100, 100]),
            "upper": np.array([25, 255, 255])
        },
        "苹果": {  # 红色
            "lower": np.array([0, 100, 100]),
            "upper": np.array([10, 255, 255])
        },
        "香蕉": {  # 黄色
            "lower": np.array([20, 100, 100]),
            "upper": np.array([35, 255, 255])
        },
        "绿色": {  # 绿色物品
            "lower": np.array([35, 50, 50]),
            "upper": np.array([85, 255, 255])
        },
        "蓝色": {  # 蓝色物品
            "lower": np.array([100, 50, 50]),
            "upper": np.array([130, 255, 255])
        },
    }

    # 物品位置映射 (测试模式下使用)
    TEST_OBJECTS = {
        "纸巾": (100, 300),
        "萝卜": (400, 300),
        "苹果": (700, 300),
    }

    def __init__(self, camera_index=0, use_camera=True):
        self.use_camera = use_camera
        self.cap = None

        if use_camera:
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                print("❌ 无法打开摄像头")
                self.use_camera = False

        # 默认参数
        self.min_area = 1000  # 最小检测区域
        self.frame_width = 640
        self.frame_height = 480

    def set_resolution(self, width=640, height=480):
        """设置摄像头分辨率"""
        if self.cap:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.frame_width = width
            self.frame_height = height

    def detect_object(self, object_name, frame=None):
        """
        检测指定物品的位置

        Args:
            object_name: 物品名称 (纸巾/萝卜/苹果/香蕉/绿色/蓝色)
            frame: 输入画面 (None表示从摄像头获取)

        Returns:
            dict: {
                "found": bool,
                "x": int,           # 物品中心x坐标
                "y": int,           # 物品中心y坐标
                "area": int,        # 检测区域面积
                "frame": numpy.ndarray  # 标注后的画面
            }
        """
        if frame is None and self.use_camera:
            ret, frame = self.cap.read()
            if not ret:
                return {"found": False, "x": 0, "y": 0, "area": 0, "frame": None}

        if frame is None:
            # 测试模式：返回模拟位置
            if object_name in self.TEST_OBJECTS:
                x, y = self.TEST_OBJECTS[object_name]
                return {
                    "found": True,
                    "x": x,
                    "y": y,
                    "area": 5000,
                    "frame": None
                }
            return {"found": False, "x": 0, "y": 0, "area": 0, "frame": None}

        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 获取颜色范围
        if object_name in self.COLOR_RANGES:
            lower = self.COLOR_RANGES[object_name]["lower"]
            upper = self.COLOR_RANGES[object_name]["upper"]
        else:
            # 默认白色
            lower = np.array([0, 0, 180])
            upper = np.array([180, 50, 255])

        # 颜色阈值
        mask = cv2.inRange(hsv, lower, upper)

        # 形态学处理
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 轮廓检测
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 找最大轮廓
        max_area = 0
        max_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area and area > self.min_area:
                max_area = area
                max_contour = contour

        # 绘制结果
        result_frame = frame.copy()

        if max_contour is not None:
            # 计算中心点
            M = cv2.moments(max_contour)
            if M["m00"] > 0:
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])

                # 绘制边界框
                x1, y1, w, h = cv2.boundingRect(max_contour)
                cv2.rectangle(result_frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)

                # 绘制中心点
                cv2.circle(result_frame, (x, y), 5, (0, 0, 255), -1)

                # 显示标签
                cv2.putText(result_frame, object_name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                return {
                    "found": True,
                    "x": x,
                    "y": y,
                    "area": max_area,
                    "frame": result_frame
                }

        # 未找到
        cv2.putText(result_frame, f"未检测到{object_name}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return {
            "found": False,
            "x": 0,
            "y": 0,
            "area": 0,
            "frame": result_frame
        }

    def scan_objects(self, frame=None):
        """
        扫描画面中所有可识别的物品

        Returns:
            list: 检测到的物品列表
        """
        detected = []

        if frame is None and self.use_camera:
            ret, frame = self.cap.read()
            if not ret:
                return []

        for object_name in self.COLOR_RANGES:
            result = self.detect_object(object_name, frame)
            if result["found"]:
                detected.append({
                    "name": object_name,
                    "x": result["x"],
                    "y": result["y"],
                    "area": result["area"]
                })

        return detected

    def get_frame(self):
        """获取当前帧"""
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None

    def release(self):
        """释放摄像头"""
        if self.cap:
            self.cap.release()
        print("📷 摄像头已释放")


if __name__ == "__main__":
    # 测试代码
    print("=== 物品检测器测试 ===")
    print("请选择测试模式:")
    print("1. 摄像头测试")
    print("2. 模拟测试")

    mode = input("输入选项: ").strip()

    detector = ObjectDetector(use_camera=(mode == "1"))

    if mode == "1":
        print("按 'q' 退出测试")
        while True:
            frame = detector.get_frame()
            if frame is not None:
                # 显示画面
                cv2.imshow("Object Detection", frame)

                # 按q退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    else:
        # 模拟测试
        print("\n模拟检测结果:")
        for obj in ["纸巾", "萝卜", "苹果"]:
            result = detector.detect_object(obj)
            status = "✅ 找到" if result["found"] else "❌ 未找到"
            print(f"  {obj}: {status}")

    detector.release()
    print("测试完成")
