"""
@author: 李文皓
@功能：全自动拍摄图片用于yolo训练
"""
import numpy as np
import cv2
import os
import random
from datetime import datetime
from kyle_robot_toolbox.camera import Gemini335
from fairino import Robot
import time

# 初始化相机
camera = Gemini335()

# 保存图片路径设置
base_dir = "/home/newplace/FR5/yolo_dataset"
image_train_dir = os.path.join(base_dir, "images/train")
image_val_dir = os.path.join(base_dir, "images/val")
os.makedirs(image_train_dir, exist_ok=True)
os.makedirs(image_val_dir, exist_ok=True)

# 训练集比例
train_ratio = 0.8

# 初始位置
home_position = [-80.134, -530.716, 297.034, 92.609, 1.282, 3.578] # 初始笛卡尔位置 (x, y, z, rx, ry, rz)

# 运动范围
motion_range = {
    "x": (-50, 50), "y": (-50, 50), "z": (-20, 20),  # 偏移范围 (mm)
    "rx": (-5, 5), "ry": (-5, 5), "rz": (-5, 5)  # 偏移范围 (deg)
}

# 点到点运动函数
def move_to_pose(robot, target_position):
    """
    控制机械臂运动到目标位姿
    """
    ret = robot.MoveCart(target_position, 0, 0)
    if ret != 0:
        print(f"机械臂运动失败，错误码: {ret}")
        raise RuntimeError(f"机械臂运动失败，错误码: {ret}")
    print(f"机械臂已移动到目标位姿: {target_position}")
    time.sleep(1.5)

# 拍摄并保存图片
def capture_and_save_image(camera, save_count):
    """
    捕捉图片并保存到随机的训练集或验证集
    """
    img_bgr = camera.read_color_img()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = f"frame_{timestamp}_{save_count}.jpg"

    if random.random() < train_ratio:
        save_path = os.path.join(image_train_dir, file_name)
        print(f"保存到训练集: {save_path}")
    else:
        save_path = os.path.join(image_val_dir, file_name)
        print(f"保存到验证集: {save_path}")

    cv2.imwrite(save_path, img_bgr)

# 自动运动与拍摄
def auto_motion_and_capture(robot, num_images):
    """
    自动控制机械臂运动与拍摄
    """
    # 回到初始位置
    print("机械臂移动到初始位置...")
    move_to_pose(robot, home_position)

    for i in range(num_images):
        # 生成随机目标位姿
        target_position = [
            home_position[0] + random.uniform(*motion_range["x"]),
            home_position[1] + random.uniform(*motion_range["y"]),
            home_position[2] + random.uniform(*motion_range["z"]),
            home_position[3] + random.uniform(*motion_range["rx"]),
            home_position[4] + random.uniform(*motion_range["ry"]),
            home_position[5] + random.uniform(*motion_range["rz"]),
        ]

        # 控制机械臂运动
        move_to_pose(robot, target_position)

        # 拍摄并保存图片
        capture_and_save_image(camera, i + 1)

        # 显示拍摄的图片
        img_bgr = camera.read_color_img()
        cv2.imshow("Image", img_bgr)
        cv2.waitKey(500)  # 停顿以便观察

# 主程序
if __name__ == "__main__":
    # 初始化机械臂
    robot = Robot.RPC('192.168.59.6')

    # 输入需要拍摄的图片张数
    num_images = int(input("请输入需要拍摄的图片张数: "))

    # 开始自动运动与拍摄
    try:
        auto_motion_and_capture(robot, num_images)
    except Exception as e:
        print(f"运行过程中出现错误: {e}")
    finally:
        # 释放资源
        camera.release()
        cv2.destroyAllWindows()
        print("程序结束，已释放资源")
