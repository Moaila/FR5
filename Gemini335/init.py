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

# 彩图获取与动态刷新
# 采集彩图，色彩空间BGR
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

# 深度图获取与动态更新
# 采集深度图, 单位为mm
# 数据格式为np.float32
depth_img = camera.read_depth_img() 
# 使用灰度图展示深度图
plt.imshow(depth_img,  cmap="gray")
# 保存图像
plt.savefig("data/read_depth_image/gray.png")
# 将深度图转换为彩图，进行可视化
# 可以指定最近距离跟最远距离, 单位mm
canvas = camera.depth_img2canvas(depth_img, min_distance=300, max_distance=600)
plt.imshow(canvas[:, :, ::-1])
# 保存图像
plt.savefig("data/read_depth_image/canvas.png")
# 深度图像保存路径
depth_img_path = "data/read_depth_image/demo.npy"
# 以二进制格式保存Numpy对象
np.save(depth_img_path, depth_img)
# 深度图像保存路径
depth_img_path = "data/read_depth_image/demo.npy"
# 图像保存
depth_img2 = np.load(depth_img_path)
if depth_img2 is None:
    print("深度图读取失败")
else:
    print("深度图读取成功")
# 将深度图转换为彩图，进行可视化
canvas = camera.depth_img2canvas(depth_img2)
plt.imshow(canvas[:, :, ::-1])

#像素坐标转换为三维空间坐标
# 使用read函数同时读取彩图与深度图
color_img, depth_img = camera.read()
# 生成画布
color_canvas = np.copy(color_img)
depth_canvas = camera.depth_img2canvas(depth_img)
# 定义像素坐标
px = 400
py = 200
radius = 10 # 像素半径
color = [0, 0, 255] # 颜色
thickness = 5 # 宽度
# 绘制圆圈
color_canvas = cv2.circle(color_canvas, [px, py], radius, color, thickness)
depth_canvas = cv2.circle(depth_canvas, [px, py], radius, color, thickness)
plt.figure(figsize=(12, 8))
plt.subplot(1, 2, 1)
plt.title("Color")
plt.imshow(color_canvas[:, :, ::-1])

plt.subplot(1, 2, 2)
plt.title("Depth")

plt.imshow(depth_canvas[:, :, ::-1])

plt.savefig("data/pixel2point3d/compare.png")
# 将像素坐标转换为RGB相机坐标系下的三维坐标
depth_value = depth_img[py, px]
print(f"深度值: {depth_value} mm")


# 释放相机对象
camera.release()


