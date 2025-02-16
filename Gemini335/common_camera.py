"""
@作者：李文皓
@功能：分别读取两个相机设备的视频流并尝试同时显示
@注意事项：可能会有黄色的各种警告和报错，直接运行再说，能跑就是好代码
"""
import sys
import os
import time
import numpy as np
import cv2 
import open3d as o3d
from kyle_robot_toolbox.camera import Gemini335

# 添加动态链接库路径
pyorbbecsdk_path = "/home/newplace/FR5/pyorbbecsdk"
if pyorbbecsdk_path not in sys.path:
    sys.path.append(pyorbbecsdk_path)

# 设置LD_LIBRARY_PATH
os.environ["LD_LIBRARY_PATH"] = pyorbbecsdk_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")

# 导入pyorbbecsdk
try:
    from pyorbbecsdk import *
    print("pyorbbecsdk 导入成功！")
except ImportError as e:
    print(f"导入失败: {e}")
    sys.exit(1)

# 设置日志等级为ERROR
ctx = Context()
ctx.set_logger_level(OBLogLevel.ERROR)

# 查询设备列表
device_list = ctx.query_devices()
device_num = device_list.get_count()

if device_num == 0:
    print("[ERROR] 没有设备连接")
else:
    print(f"检测到 {device_num} 个设备")

# 获取设备序列号
serial_num1 = device_list.get_device_serial_number_by_index(0)
serial_num2 = device_list.get_device_serial_number_by_index(1)

print(f"第1个设备序列号为: {serial_num1}")
print(f"第2个设备序列号为: {serial_num2}")

# 创建两个相机对象，分别传入对应的序列号
camera1 = Gemini335(serial_num=serial_num1)
camera2 = Gemini335(serial_num=serial_num2)

# 创建窗口
win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
cv2.namedWindow("Camera 1", flags=win_flag)
cv2.namedWindow("Camera 2", flags=win_flag)

# 创建合并显示窗口
cv2.namedWindow("Combined View", flags=win_flag)

while True:
    # 从两个相机读取图像
    img1 = camera1.read_color_img()
    img2 = camera2.read_color_img()

    # 检查图像是否有效
    if img1 is not None and img2 is not None:
        # 调整图像大小以匹配（如果需要）
        height, width = img1.shape[:2]
        img2_resized = cv2.resize(img2, (width, height))

        # 水平拼接图像
        combined_img = np.hstack((img1, img2_resized))

        # 显示单独窗口
        cv2.imshow('Camera 1', img1)
        cv2.imshow('Camera 2', img2)

        # 显示合并视图
        cv2.imshow('Combined View', combined_img)

    # 处理键盘输入
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# 释放资源
camera1.release()
camera2.release()
cv2.destroyAllWindows()