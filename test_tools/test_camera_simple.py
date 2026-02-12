#!/usr/bin/env python3
import cv2
import time

print("=" * 50)
print("摄像头测试")
print("=" * 50)
print()

print("尝试打开摄像头...")

for idx in [0, 1]:
    print(f"\n尝试 /dev/video{idx}...")
    cap = cv2.VideoCapture(idx)
    
    if cap.isOpened():
        print(f"  ✓ 摄像头已打开")
        
        time.sleep(0.5)
        
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  ✓ 成功读取画面: {frame.shape}")
            
            cv2.imwrite(f"/home/yoi/test_camera_{idx}.jpg", frame)
            print(f"  ✓ 已保存测试图片: test_camera_{idx}.jpg")
            
            cap.release()
            print(f"\n✅ 摄像头 /dev/video{idx} 正常工作!")
            break
        else:
            print(f"  ✗ 无法读取画面")
            cap.release()
    else:
        print(f"  ✗ 无法打开")

print()
print("测试完成")
