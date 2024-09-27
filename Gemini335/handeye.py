import numpy as np
import cv2
import matplotlib.pyplot as plt
from kyle_robot_toolbox.camera import Gemini335

# 创建相机对象
camera = Gemini335()

# 彩图获取与动态刷新
img_bgr = camera.read_color_img()  # 从相机读取彩色图像
img_bgr_undistort = camera.remove_distortion(img_bgr)  # 去除图像畸变

# 图像显示
plt.imshow(img_bgr_undistort[:, :, ::-1])
plt.title("Undistorted Color Image")
plt.show()

# 将畸变校正后的彩图保存
img_path = "/home/tom/FR5/photo/undistorted_color_image.png"
cv2.imwrite(img_path, img_bgr_undistort)

# 对畸变校正后的图像进行灰度化处理
gray_img = cv2.cvtColor(img_bgr_undistort, cv2.COLOR_BGR2GRAY)

# 对图像进行模糊处理，减少噪声
blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

# 使用阈值处理，将灰度图转为二值图像
_, thresh_img = cv2.threshold(blurred_img, 127, 255, cv2.THRESH_BINARY)

# 查找图像中的轮廓
contours, _ = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 绘制轮廓
contour_image = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)

# 显示检测到的轮廓
plt.imshow(contour_image[:, :, ::-1])
plt.title("Contours of the Detected Pattern")
plt.show()

# 保存带有轮廓的图像
cv2.imwrite("/home/tom/FR5/photo/contours_detected.png", contour_image)

# 进一步步骤: 检测图案中的矩形
for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h
    
    # 判断是否为正方形或矩形
    if 0.9 < aspect_ratio < 1.1:  # 近似正方形
        print(f"Detected a square/rectangle at position: x={x}, y={y}, width={w}, height={h}")
        # 在原图中绘制矩形边框
        cv2.rectangle(contour_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

# 显示带矩形的图像
plt.imshow(contour_image[:, :, ::-1])
plt.title("Detected Rectangles in Pattern")
plt.show()

# 保存带矩形的图像
cv2.imwrite("/home/tom/FR5/photo/rectangles_detected.png", contour_image)

# 深度图获取与动态更新
depth_img = camera.read_depth_img()  # 从相机读取深度图像
plt.imshow(depth_img, cmap="gray")
plt.title("Depth Image")
plt.show()

# 保存深度图
np.save("/home/tom/FR5/photo/depth_image.npy", depth_img)

# 将深度图转换为伪彩图
canvas = camera.depth_img2canvas(depth_img, min_distance=300, max_distance=600)
plt.imshow(canvas[:, :, ::-1])
plt.title("Depth Image Canvas")
plt.show()

# 保存伪彩图
plt.savefig("/home/tom/FR5/photo/depth_image_canvas.png")

# 将像素坐标转换为RGB相机坐标系下的三维坐标
px, py = 400, 200  # 像素坐标
depth_value = depth_img[py, px]
print(f"像素坐标 ({px}, {py}) 对应的深度值: {depth_value} mm")

# 释放相机
camera.release()
