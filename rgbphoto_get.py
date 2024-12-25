'''
@author: 李文皓
Gemini335 3D相机 - 测试彩图读取与动态更新与保存
'''
import numpy as np
import cv2 
import open3d as o3d
from matplotlib import pyplot as plt
import os
from datetime import datetime
from kyle_robot_toolbox.camera import Gemini335
import random

# 创建相机对象
camera = Gemini335()

# 设置图片保存路径
base_dir = "/home/newplace/FR5/yolo_dataset"
image_train_dir = os.path.join(base_dir, "/home/newplace/FR5/yolo_dataset/images/train")
image_val_dir = os.path.join(base_dir, "/home/newplace/FR5/yolo_dataset/images/val")

# 确保目录存在
os.makedirs(image_train_dir, exist_ok=True)
os.makedirs(image_val_dir, exist_ok=True)

train_ratio = 0.8


# 创建窗口
win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
cv2.namedWindow("color", flags=win_flag)

save_count = 0

print("按 'r' 保存当前帧，按 'q' 退出程序")

while True:
    # 采集彩图，色彩空间为BGR
    img_bgr = camera.read_color_img()
    cv2.imshow("Image", img_bgr)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # 按 'q' 键退出
        print("退出程序")
        break
    elif key == ord('r'):  # 按 'r' 键保存当前帧
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f"frame_{timestamp}.jpg"

        # 随机分配到训练或验证集
        if random.random() < train_ratio:
            save_path = os.path.join(image_train_dir, file_name)
            print(f"保存到训练集: {save_path}")
        else:
            save_path = os.path.join(image_val_dir, file_name)
            print(f"保存到验证集: {save_path}")

        # 保存图像
        cv2.imwrite(save_path, img_bgr)

        save_count += 1
        if save_count % 10 == 0:
            print(f"you have saved {save_count} images")

# 关闭摄像头
camera.release()

# 销毁所有的窗口
cv2.destroyAllWindows()
