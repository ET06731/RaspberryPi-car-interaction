#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
寻物游戏主入口
"""

from car_controller import CarController
from object_detector import ObjectDetector
from game import CarGame


def main():
    """主函数"""
    print("=" * 60)
    print("  🤖 树莓派小车寻物游戏")
    print("=" * 60)
    print()
    print("使用方法:")
    print("  1. 在小车上放置不同颜色的物品 (纸巾/萝卜/苹果等)")
    print("  2. 运行本程序")
    print("  3. 输入要寻找的物品名称")
    print("  4. 小车会自动寻找并移动到目标物品")
    print()
    print("支持的物品:")
    print("  - 纸巾 (白色)")
    print("  - 萝卜 (橙色)")
    print("  - 苹果 (红色)")
    print("  - 香蕉 (黄色)")
    print("  - 绿色物品")
    print("  - 蓝色物品")
    print()
    print("-" * 60)
    print()

    # 初始化硬件
    print("初始化硬件...")

    # 初始化小车
    try:
        car = CarController(speed=70)
        car.init()
        print("✅ 小车初始化成功")
    except Exception as e:
        print(f"❌ 小车初始化失败: {e}")
        print("将使用模拟模式")
        car = None

    # 初始化摄像头
    try:
        detector = ObjectDetector(camera_index=0, use_camera=True)
        detector.set_resolution(640, 480)
        print("✅ 摄像头初始化成功")
    except Exception as e:
        print(f"❌ 摄像头初始化失败: {e}")
        print("将使用模拟模式")
        detector = ObjectDetector(use_camera=False)

    # 创建游戏
    game = CarGame(car, detector, use_simulation=(car is None))

    try:
        # 扫描环境
        print("\n开始扫描环境...")
        detected = game.scan_environment()

        # 获取目标
        print("\n" + "-" * 60)
        target = input("请输入要寻找的物品名称: ").strip()

        if not target:
            print("❌ 未输入物品名称")
            return

        # 开始游戏
        game.start_game(target)
        found = game.search_and_navigate()

        if found:
            print("\n🎉 恭喜! 成功找到目标!")
        else:
            print("\n😢 未找到目标，请检查:")
            print("  - 物品颜色是否明显")
            print("  - 光线是否充足")
            print("  - 摄像头是否对准物品")

    except KeyboardInterrupt:
        print("\n\n用户中断游戏")
    finally:
        game.end_game()
        game.cleanup()


if __name__ == "__main__":
    main()
