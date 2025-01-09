"""
@author: 李文皓
@场景：眼在手外
@功能：用官方的YoloV8预训练模型进行杯子的检测（简化版，仅显示彩图和标记）
"""

import os
import time
import cv2
from ultralytics import YOLO
from kyle_robot_toolbox.camera import Gemini335

# 初始化摄像头
camera = Gemini335()

# 加载官方预训练YOLOv8模型
print("[INFO] 开始加载官方 YOLOv8 模型")
model = YOLO('yolov8n.pt')

# 创建显示窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while True:
    # 计时开始
    t_start = time.time()

    # 读取彩色图像
    img_bgr = camera.read_color_img()

    # 使用YOLOv8官方模型进行检测
    results = model.predict(source=img_bgr, imgsz=640, conf=0.5, iou=0.6, device=0, classes=[41], stream=False)

    # 获取检测结果
    canvas = img_bgr.copy()  # 在原图上绘制
    for result in results:
        boxes = result.boxes  # 获取检测框
        for box in boxes:
            # 获取类别ID和置信度
            cls_id = int(box.cls[0])
            conf = box.conf[0].item()
            
            # 检测 "杯子" 类别（COCO中'杯子'的类别ID为41）
            if cls_id != 41:
                continue
            
            # 获取边界框坐标
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # 绘制矩形框
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # 显示置信度
            cv2.putText(canvas, f"{conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 计算帧率
    t_end = time.time()
    fps = int(1 / (t_end - t_start))
    cv2.putText(canvas, f"FPS: {fps}", (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # 显示结果画面
    cv2.imshow("canvas", canvas)

    # 按下 'q' 键退出程序
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# 释放资源
cv2.destroyAllWindows()
camera.release()
