'''
ArucoTag检测(2D相机版)

'''
import time
import numpy as np
import cv2
# 阿凯机器人工具箱
# - Gemini335类
from kyle_robot_toolbox.camera import Gemini335
# - ArucoTag
from kyle_robot_toolbox.opencv import ArucoTag

# 创建相机对象
camera = Gemini335()

# 图像预处理窗口
cv2.namedWindow('aruco',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

# 创建ArucoTag检测器
arucotag = ArucoTag(camera, \
    config_path="config/arucotag/arucotag.yaml")

while True:
    # 采集图像
    img_bgr = camera.read_color_img()
    if img_bgr is None:
        break
    # 图像移除畸变
    img_bgr = camera.remove_distortion(img_bgr)
    
    # 图像预处理
    # - 转换为灰度图
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # - 直方图均衡
    img_gray = cv2.equalizeHist(img_gray)
    # - 中值滤波
    img_gray = cv2.medianBlur(img_gray, 3)
    # - 转换为BGR图
    img_filter = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    
    # 检测ArucoTag
    has_aruco, canvas, aruco_ids, aruco_centers, corners, T_cam2aruco = \
        arucotag.aruco_pose_estimate(img_filter)
    # 显示图像
    cv2.imshow('aruco', canvas)
    # 按键退出
    key = cv2.waitKey(1)
    if key == ord('q'):
        # 如果按键为q 代表quit 退出程序
        break
cv2.destroyAllWindows()
camera.release()