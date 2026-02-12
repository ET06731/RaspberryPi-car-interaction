#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
寻物游戏逻辑模块
结合物品检测和小车控制，实现寻物游戏
"""

import time
import random
import cv2


class CarGame:
    """寻物游戏控制器"""

    # 支持的物品列表
    AVAILABLE_OBJECTS = ["纸巾", "萝卜", "苹果", "香蕉", "绿色物品", "蓝色物品"]

    def __init__(self, car_controller, object_detector, use_simulation=False):
        """
        初始化游戏

        Args:
            car_controller: 小车控制器实例
            object_detector: 物品检测器实例
            use_simulation: 是否使用模拟模式 (不连接真实硬件)
        """
        self.car = car_controller
        self.detector = object_detector
        self.use_simulation = use_simulation
        self.game_state = "idle"  # idle, searching, found, error
        self.target_object = None
        self.score = 0
        self.attempts = 0

        print("🎮 寻物游戏初始化完成")

    def start_game(self, target_object=None):
        """
        开始游戏

        Args:
            target_object: 目标物品 (None则随机选择)
        """
        if target_object is None:
            target_object = random.choice(self.AVAILABLE_OBJECTS)

        self.target_object = target_object
        self.game_state = "searching"

        print("\n" + "=" * 50)
        print(f"🎮 游戏开始!")
        print(f"📦 目标物品: {target_object}")
        print("=" * 50)

        return target_object

    def search_and_navigate(self, max_iterations=100):
        """
        搜索并导航到目标物品

        Args:
            max_iterations: 最大迭代次数

        Returns:
            bool: 是否找到目标
        """
        if self.game_state != "searching":
            print("❌ 游戏未开始")
            return False

        print(f"\n🔍 开始搜索: {self.target_object}")

        for iteration in range(max_iterations):
            # 获取当前帧
            frame = self.detector.get_frame()

            # 检测目标物品
            result = self.detector.detect_object(self.target_object, frame)

            if result["found"]:
                self.game_state = "found"
                print(f"\n✅ 找到 {self.target_object}!")
                print(f"📍 位置: ({result['x']}, {result['y']})")

                # 导航到物品
                self.navigate_to_object(result, frame)

                return True

            # 未找到，转圈搜索
            if iteration % 10 == 0:
                print(f"🔄 搜索中... 第{iteration + 1}次")
                self.car.spin_right(duration=0.5)

            # 显示检测结果 (如果有画面)
            if result["frame"] is not None:
                cv2.imshow("Game View", result["frame"])
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        print(f"\n❌ 未找到 {self.target_object}")
        self.game_state = "error"
        return False

    def navigate_to_object(self, detection_result, frame=None):
        """
        导航到检测到的物品

        Args:
            detection_result: 检测结果字典
            frame: 当前画面
        """
        if not detection_result["found"]:
            return

        print("\n🚗 导航到物品...")

        frame_width = self.detector.frame_width
        frame_center = frame_width // 2
        threshold = 50  # 居中阈值
        approach_threshold = 200  # 接近阈值

        iterations = 0
        max_approach = 50  # 最多接近50次

        while iterations < max_approach:
            # 重新检测
            result = self.detector.detect_object(self.target_object)
            if not result["found"]:
                print("❌ 丢失目标")
                break

            x = result["x"]
            y = result["y"]
            # 使用y坐标作为距离估计 (物体越近y越大)
            distance = y

            if distance < approach_threshold:
                # 足够接近
                print(f"✅ 已到达 {self.target_object}!")
                self.car.stop()
                break

            # 根据位置调整方向
            relative_x = x - frame_center

            if abs(relative_x) < threshold:
                # 居中，前进
                print("➡️ 前进")
                self.car.forward(duration=0.5)
            elif relative_x < 0:
                # 在左边
                print("⬅️ 左转")
                self.car.spin_left(duration=0.2)
            else:
                # 在右边
                print("➡️ 右转")
                self.car.spin_right(duration=0.2)

            iterations += 1
            time.sleep(0.1)

        self.car.stop()

    def scan_environment(self):
        """
        扫描环境，列出所有可见物品

        Returns:
            list: 检测到的物品列表
        """
        print("\n🔍 扫描环境...")

        detected = self.detector.scan_objects()

        if detected:
            print(f"\n📦 检测到 {len(detected)} 个物品:")
            for item in detected:
                print(f"  - {item['name']} at ({item['x']}, {item['y']})")
        else:
            print("❓ 未检测到已知物品")

        return detected

    def play_round(self):
        """玩一轮游戏"""
        # 选择随机目标
        target = self.start_game()

        # 搜索
        found = self.search_and_navigate()

        if found:
            self.score += 1

        self.attempts += 1

        print(f"\n📊 当前分数: {self.score}/{self.attempts}")

        return found

    def end_game(self):
        """结束游戏"""
        self.game_state = "idle"
        self.target_object = None

        print("\n" + "=" * 50)
        print("🎮 游戏结束!")
        print(f"📊 最终分数: {self.score}/{self.attempts}")
        print("=" * 50)

    def cleanup(self):
        """清理资源"""
        self.car.stop()
        self.detector.release()
        cv2.destroyAllWindows()
        print("🔌 资源已清理")


if __name__ == "__main__":
    # 测试代码
    from car_controller import CarController
    from object_detector import ObjectDetector

    print("=== 寻物游戏测试 ===")
    print("请选择模式:")
    print("1. 真实模式 (需要连接小车和摄像头)")
    print("2. 模拟模式 (仅测试逻辑)")

    mode = input("输入选项 (1/2): ").strip()

    if mode == "2":
        use_sim = True
    else:
        use_sim = False

    # 初始化
    if use_sim:
        car = None
    else:
        car = CarController()
        car.init()

    detector = ObjectDetector(use_camera=(mode == "1"))

    # 创建游戏
    game = CarGame(car, detector, use_simulation=use_sim)

    try:
        # 扫描环境
        game.scan_environment()

        # 玩一轮
        if input("\n是否开始游戏? (y/n): ").strip().lower() == 'y':
            game.play_round()

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        game.end_game()
        game.cleanup()
