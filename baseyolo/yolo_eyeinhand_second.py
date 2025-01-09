"""
@author: 李文皓
@场景：眼在手外
@功能：用官方YOLOv8模型进行杯子的检测与定位，显示方框、名称、置信度，并获取相机坐标系下的坐标
"""
import os
import time
import numpy as np
import cv2
from ultralytics import YOLO  # 使用官方ultralytics库
from kyle_robot_toolbox.camera import Gemini335

# 初始化相机
camera = Gemini335()

# 加载官方YOLOv8模型
print("[INFO] 开始加载官方 YOLOv8 模型")
model = YOLO('yolov8n.pt')  # 选择一个官方预训练模型
print("[INFO] 完成 YOLOv8 模型加载")

# 创建显示窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while True:
    # 计时启动
    t_start = time.time()

    # 读取图像
    img_bgr = camera.read_color_img()
    depth_img = camera.read_depth_img()
    depth_canvas_tmp = camera.depth_img2canvas(depth_img)

    # 兼容尺寸差异
    dp_h, dp_w, _ = depth_canvas_tmp.shape
    depth_canvas = np.zeros_like(img_bgr)
    depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp

    # YOLOv8目标检测
    results = model.predict(source=img_bgr, imgsz=640, conf=0.5, iou=0.6, device=0, classes=[41], stream=False)  # '41' 是COCO数据集中杯子的类别ID

    center_list = []
    for result in results:
        boxes = result.boxes  # 获取检测框
        for box in boxes:
            # 获取类别ID和置信度
            cls_id = int(box.cls[0])
            conf = box.conf[0].item()

            # 检测 "杯子" 类别
            if cls_id != 41:
                continue

            # 获取边界框
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # 绘制检测框
            cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绿色方框

            # 在检测框上方绘制类别名称和置信度
            label = f"Cup: {conf:.2f}"  # 类别名称和置信度
            cv2.putText(img_bgr, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 计算检测框中心点
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # 绘制中心点
            cv2.circle(img_bgr, (cx, cy), 5, (255, 255, 0), -1)
            cv2.circle(depth_canvas, (cx, cy), 5, (255, 0, 255), -1)

            # 确保坐标在深度图范围内
            if cx >= dp_w or cy >= dp_h:
                continue

            # 获取深度值并计算3D坐标
            depth_value = depth_img[cy, cx]
            if depth_value == 0:  # 如果深度无效，跳过
                continue

            # 利用深度图和相机内参计算3D坐标
            cam_point3d = camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth_img)
            cam_x, cam_y, cam_z = cam_point3d

            # 显示三维坐标信息
            tag = f"{cam_x:.0f}, {cam_y:.0f}, {cam_z:.0f}"
            cv2.putText(img_bgr, tag, (cx - 80, cy - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 1, cv2.LINE_AA)

            center_list.append([cx, cy])

    # 计算帧率
    t_end = time.time()
    fps = int(1 / (t_end - t_start))
    cv2.putText(img_bgr, f"FPS:{fps}", (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # 显示画面
    cv2.imshow("canvas", img_bgr)
    cv2.imshow("depth_canvas", depth_canvas)

    # 按 'q' 键退出
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
camera.release()
