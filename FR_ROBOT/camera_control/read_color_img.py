"""
测试用：
摄像头功能仅为读取彩色图像并显示
"""

import numpy as np
import cv2 
import open3d as o3d
from kyle_robot_toolbox.camera import Gemini335

camera = Gemini335()

win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
cv2.namedWindow("color", flags=win_flag)

while True:
	img_bgr = camera.read_color_img() 
	cv2.imshow('color', img_bgr)
	key = cv2.waitKey(1)
	if key == ord('q'):
		break

camera.release()
cv2.destroyAllWindows()
