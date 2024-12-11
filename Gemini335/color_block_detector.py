'''
色块检测 测试案例(Astra相机版)

'''
import time
import numpy as np
import cv2
# 阿凯机器人工具箱
# - Gemini335类
from kyle_robot_toolbox.camera import Gemini335
# - 色块检测
from kyle_robot_toolbox.opencv import detect_color_block

# 创建相机对象
camera = Gemini335()

# 载入HSV阈值
hsv_lowerb = np.load("config/hsv_range/red_hsv_lowerb.npy")
hsv_upperb = np.load("config/hsv_range/red_hsv_upperb.npy")

  
while True:
	# 开始时间
	t_start = time.time()

	# 读取图像
	img_bgr = camera.read_color_img()
	if img_bgr is None:
		print("图像获取失败")
		continue
	# 识别色块
	canvas = np.copy(img_bgr)
	rect_list, mask, canvas = detect_color_block(img_bgr, hsv_lowerb, hsv_upperb)

	# 统计帧率
	t_end = time.time()
	t_pass = t_end - t_start
	fps = int(1/t_pass)
	# 绘制帧率
	cv2.putText(canvas, text=f"FPS:{fps}",\
		org=(20, 20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
		fontScale=0.8, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
	
	cv2.imshow("canvas", canvas)
	key = cv2.waitKey(1)
	if key == ord('q'):
		break

cv2.destroyAllWindows()
camera.release()