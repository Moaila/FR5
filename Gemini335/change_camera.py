"""
@作者：李文皓
@功能：分别读取两个相机设备的视频流并尝试进行切换
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

current_camera = camera1  # 初始使用第一个相机

# 创建窗口
win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
cv2.namedWindow("color", flags=win_flag)

while True:
    # 从当前相机读取图像
    img_bgr = current_camera.read_color_img()
    if img_bgr is not None:
        cv2.imshow('color', img_bgr)
    
    key = cv2.waitKey(1)
    if key == ord('1'):
        current_camera = camera1
        print("切换到相机1")
    elif key == ord('2'):
        current_camera = camera2
        print("切换到相机2")
    elif key == ord('q'):
        break

# 释放两个相机资源
camera1.release()
camera2.release()
cv2.destroyAllWindows()

# 获取第一个设备, 并打开设备
# device1 = device_list.get_device_by_index(0)
# 根据设备序列号创建设备
# device1 = device_list.get_device_by_serial_number(serial_num1)

# 获取第二个设备, 并打开设备
# device2 = device_list.get_device_by_index(1)
# 根据设备序列号创建设备
# device2 = device_list.get_device_by_serial_number(serial_num2)

# 获取设备信息
# device1_info = device1.get_device_info()
# device2_info = device2.get_device_info()
# 可以直接将设备信息打印出来
# print(device1_info)
# print(device2_info)
# 获取设备UID
# device1_UID = device1_info.get_uid()
# print(f"设备UID: {device1_UID}")
# 获取设备UID
# device2_UID = device2_info.get_uid()
# print(f"设备UID: {device2_UID}")