'''
# 作者：李文皓
# 时间：2024/10/18 
# 此代码用于相机模块可以调用进行标定
'''

import numpy as np
import cv2
from kyle_robot_toolbox.camera import Gemini335

class CameraModule:
    def __init__(self):
        # 初始化相机对象
        self.camera = Gemini335()
        print("相机已初始化")

        # 默认像素坐标位置
        self.px, self.py = 0, 0

        # 创建窗口
        self.win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
        cv2.namedWindow("Color Image", flags=self.win_flag)
        cv2.namedWindow("Depth Image", flags=self.win_flag)

        # 绑定鼠标回调函数
        cv2.setMouseCallback("Color Image", self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        # 鼠标事件回调函数，更新像素坐标
        if event == cv2.EVENT_MOUSEMOVE:
            self.px, self.py = x, y

    def capture_color_and_depth(self):
        # 获取彩图和深度图
        color_img, depth_img = self.camera.read()
        if color_img is None or depth_img is None:
            print("图像获取失败")
            return None, None
        return color_img, depth_img

    def pixel_to_3d(self, depth_img):
        # 从深度图中获取深度值并计算三维坐标
        depth_value = depth_img[self.py, self.px]
        if depth_value != 0:
            cam_point3d = self.camera.depth_pixel2cam_point3d(self.px, self.py, depth_value=depth_value)
            cam_x, cam_y, cam_z = cam_point3d
            print(f"相机坐标系下的三维坐标: [{cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f}] mm")
            return cam_x, cam_y, cam_z
        else:
            print("深度值无效")
            return None

    def display_images_with_depth(self):
        # 主循环显示彩图和深度图，并实时显示鼠标位置的三维坐标
        while True:
            color_img, depth_img = self.capture_color_and_depth()
            if color_img is None or depth_img is None:
                continue

            # 生成带圆圈的画布
            color_canvas = np.copy(color_img)
            depth_canvas = self.camera.depth_img2canvas(depth_img)

            # 绘制圆圈和显示像素坐标
            color_canvas = cv2.circle(color_canvas, (self.px, self.py), radius=10, color=(0, 0, 255), thickness=5)
            depth_canvas = cv2.circle(depth_canvas, (self.px, self.py), radius=10, color=(0, 0, 255), thickness=5)
            cv2.putText(depth_canvas, f'PX: {self.px} PY: {self.py}', (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 计算并显示三维坐标
            cam_coords = self.pixel_to_3d(depth_img)
            if cam_coords:
                cam_x, cam_y, cam_z = cam_coords
                cv2.putText(depth_canvas, f'Camera: X {cam_x:.1f} Y {cam_y:.1f} Z {cam_z:.1f}', 
                            (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 显示图像
            cv2.imshow("Color Image", color_canvas)
            cv2.imshow("Depth Image", depth_canvas)

            # 按 'q' 键退出
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

    def release_camera(self):
        # 释放相机资源并关闭窗口
        self.camera.release()
        cv2.destroyAllWindows()
        print("相机已释放")
