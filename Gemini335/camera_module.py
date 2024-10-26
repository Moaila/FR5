import numpy as np
import cv2
from kyle_robot_toolbox.camera import Gemini335

class CameraModule:
    def __init__(self):
        # 初始化Gemini335相机对象
        self.camera = Gemini335()
        print("相机已初始化")

    def capture_color_and_depth(self):
        # 获取彩图和深度图
        color_img, depth_img = self.camera.read()
        if color_img is None or depth_img is None:
            print("图像获取失败")
            return None, None
        return color_img, depth_img

    def undistort_image(self, img_bgr):
        # 去除图像的畸变
        img_bgr_undistorted = self.camera.remove_distortion(img_bgr)
        return img_bgr_undistorted

    def pixel_to_3d(self, x, y, depth_img):
        # 从深度图获取深度值
        depth_value = depth_img[y, x]
        if depth_value != 0:
            # 将像素坐标转换为相机坐标系下的三维坐标
            cam_point3d = self.camera.depth_pixel2cam_point3d(x, y, depth_value=depth_value)
            cam_x, cam_y, cam_z = cam_point3d
            print(f"相机坐标系下的三维坐标: [{cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f}] mm")
            return cam_x, cam_y, cam_z
        else:
            print("无效深度值")
            return None

    def draw_circle(self, img, x, y, radius=10, color=(0, 0, 255), thickness=5):
        # 在指定坐标处绘制圆圈
        img_with_circle = cv2.circle(img, (x, y), radius, color, thickness)
        return img_with_circle

    def show_images(self, color_img, depth_img, x, y):
        # 生成画布
        color_canvas = self.draw_circle(np.copy(color_img), x, y)
        depth_canvas = self.draw_circle(self.camera.depth_img2canvas(depth_img), x, y)
        
        # 显示彩图和深度图
        cv2.imshow('Color Image', color_canvas)
        cv2.imshow('Depth Image', depth_canvas)
        cv2.waitKey(1)

    def save_image(self, img, path):
        # 保存图像
        ret = cv2.imwrite(path, img)
        if ret:
            print("图像保存成功")
        else:
            print("图像保存失败，请检查路径")

    def release_camera(self):
        # 释放摄像头
        self.camera.release()
        print("相机已释放")
