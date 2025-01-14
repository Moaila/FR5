"""
@author: 李文皓
@场景：眼在手上
@功能：用自定义YoloV8模型进行咖啡机按钮的检测（动态显示，并标记中心点）
"""

import os
import time
import cv2
import numpy as np
from kyle_robot_toolbox.camera import Gemini335
from kyle_robot_toolbox.yolov8 import YoloV8Detect

# 初始化摄像头
camera = Gemini335()

# 加载模型
print("[INFO] 开始加载 YOLOv8 模型")
model_path = "/home/newplace/FR5/coffeeyolo/runs/detect/train/weights/best.pt"
model = YoloV8Detect(model_path)

# 配置模型参数
model.IMAGE_SIZE = 1088  # 图像尺寸（必须是32的倍数）
model.CONFIDENCE = 0.5   # 置信度阈值
model.IOU = 0.6          # IOU阈值
print("[INFO] 完成 YOLOv8 模型加载")

# 创建显示窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)


while True:
    # 计时开始
    t_start = time.time()

    # 读取彩色图像
    img_bgr = camera.read_color_img()
    depth_img = camera.read_depth_img()
    depth_canvas_tmp = camera.depth_img2canvas(depth_img)

    #兼容尺寸差异
    dp_h, dp_w, _ = depth_canvas_tmp.shape
    depth_canvas = np.zeros_like(img_bgr)
    depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp

    # YOLOv8目标检测
    canvas, class_id_list, xyxy_list, conf_list = model.detect(img_bgr, draw_label=False)
    center_list = []

    # 遍历检测结果
    for i, xyxy in enumerate(xyxy_list):
        # 针对 "按钮" 类别（自训练模型中其ID为15）
        if class_id_list[i] != 15:  
            continue

         # 读取矩形框并计算中心点
        x1, y1, x2, y2 = xyxy
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # 绘制中心点
        cv2.circle(canvas, [cx, cy], 5, (255, 255, 0), -1)
        cv2.circle(depth_canvas, [cx, cy], 5, (255, 0, 255), -1)

        # 确保坐标在深度图范围内
        if cx >= dp_w or cy >= dp_h:
            continue

        # 读取深度值并转换为3D坐标
        depth_value = depth_img[cy, cx]
        if depth_value == 0:  # 无效深度值跳过
            continue

        cam_point3d = camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth_img)
        cam_x, cam_y, cam_z = cam_point3d

        # 显示三维坐标信息
        tag = f"{cam_x:.0f}, {cam_y:.0f}, {cam_z:.0f}"
        cv2.putText(canvas, tag, (cx - 80, cy - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 1, cv2.LINE_AA)
        
        center_list.append([cx, cy])

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
