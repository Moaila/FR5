"""
@author: 李文皓
@场景：眼在手外
@功能：用自定义YoloV8模型进行杯子的检测（简化版，仅显示彩图和标记）
"""

import os
import time
import cv2
from kyle_robot_toolbox.camera import Gemini335
from kyle_robot_toolbox.yolov8 import YoloV8Detect

# 初始化摄像头
camera = Gemini335()

# 加载模型
print("[INFO] 开始加载 YOLOv8 模型")
model_path = "/home/newplace/FR5/baseyolo/runs/detect/train/weights/best.pt"
model = YoloV8Detect(model_path)

# 配置模型参数
model.IMAGE_SIZE = 1088  # 图像尺寸（必须是32的倍数）
model.CONFIDENCE = 0.5   # 置信度阈值
model.IOU = 0.6          # IOU阈值
print("[INFO] 完成 YOLOv8 模型加载")

# 创建显示窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while True:
    # 计时开始
    t_start = time.time()

    # 读取彩色图像
    img_bgr = camera.read_color_img()

    # YOLOv8目标检测
    canvas, class_id_list, xyxy_list, conf_list = model.detect(img_bgr, draw_label=False)

    # 遍历检测结果
    for i, xyxy in enumerate(xyxy_list):
        # 针对 "杯子" 类别（杯子的类别ID为15,可以看dataset.yaml文件）
        if class_id_list[i] != 15:
            continue

        # 绘制矩形框
        x1, y1, x2, y2 = map(int, xyxy)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 显示置信度
        confidence = conf_list[i]
        cv2.putText(canvas, f"{confidence:.2f}", (x1, y1 - 10),
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
