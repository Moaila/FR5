'''
色块检测与定位 测试案例(Gemini335相机版)
----------------------------
作者: 阿凯爱玩机器人 | 微信: xingshunkai  | QQ: 244561792
B站: https://space.bilibili.com/40344504
淘宝店铺: https://shop140985627.taobao.com
购买链接: https://item.taobao.com/item.htm?id=677075846402
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

# 创建窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while True:
	# 开始时间
	t_start = time.time()
	# 读取图像
	# 读取彩图与深度图
	img_bgr, depth_img = camera.read()
	if img_bgr is None or depth_img is None:
		print("图像获取失败")
		continue
	# 将深度转换为可视化画布
	# 根据实际情况调整深度范围 [min_distance, max_distance]
	depth_canvas_tmp = camera.depth_img2canvas(depth_img, \
				min_distance=150, max_distance=300)
	# 为了兼容Gemini深度图与彩图尺寸不一致的情况
	# 要做一下特殊处理
	dp_h, dp_w, dp_ch = depth_canvas_tmp.shape
	depth_canvas = np.zeros_like(img_bgr)
	depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp
	# 识别色块
	canvas = np.copy(img_bgr)
	rect_list, mask, canvas = detect_color_block(img_bgr, hsv_lowerb, hsv_upperb)
	center_list = []

	for rect in rect_list:
		# 读取矩形框
		x, y, w, h = rect
		# 计算中心点
		cx = int(x + w/2)
		cy = int(y + h/2)
		# 绘制中心点
		cv2.circle(canvas, [cx, cy], 5, (255, 255, 0), -1)
		cv2.circle(depth_canvas, [cx, cy], 5, (255, 0, 255), -1)
		center = [cx, cy]
		center_list.append(center)
		
		px, py = center
		# 判断坐标是否在深度图的有效范围内
		if px >= dp_w or py >= dp_h:
			continue 
		# 读取深度值
		depth_value = depth_img[py, px]
		# 深度值无效
		if depth_value == 0:
			continue
		cam_point3d = camera.depth_pixel2cam_point3d(\
										px, py, depth_value=depth_value)
		# 计算三维坐标
		cam_x, cam_y, cam_z = cam_point3d
		# 在画面上绘制坐标
		tag = f"{cam_x:.0f}, {cam_y:.0f}, {cam_z:.0f}"
		cv2.putText(canvas, text=tag,\
			org=(px-50, py-60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
			fontScale=0.8, thickness=1, lineType=cv2.LINE_AA, color=(255, 0, 255))
	
	# 统计帧率
	t_end = time.time()
	t_pass = t_end - t_start
	fps = int(1/t_pass)
	# 绘制帧率
	cv2.putText(canvas, text=f"FPS:{fps}",\
		org=(20, 20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
		fontScale=0.8, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
	
	cv2.imshow("canvas", canvas)
	cv2.imshow("depth_canvas", depth_canvas)

	key = cv2.waitKey(1)
	if key == ord('q'):
		break

cv2.destroyAllWindows()
camera.release()