# - 矩阵运算
import numpy as np
# - 图像处理
import cv2 
# - Open3D点云处理
import open3d as o3d
# - 绘图可视化
from matplotlib import pyplot as plt
# 从阿凯机器人工具箱导入Gemini335类
from kyle_robot_toolbox.camera import Gemini335

# 创建相机对象
camera = Gemini335()

img_bgr = camera.read_color_img()

# 图像移除畸变
img_bgr_undistor = camera.remove_distortion(img_bgr)

# 显示去除畸变的彩图 
plt.imshow(img_bgr_undistor[:, :, ::-1])

# 图像保存路径
img_path = "/home/tom/FR5/photo/photo1.png"
# 图像保存
ret = cv2.imwrite(img_path, img_bgr)
# 输出日志
if ret:
    print("图像保存成功")
else:
    print("检查图像保存路径是否存在，且路径不可以有中文。")


# 释放相机对象
camera.release()


