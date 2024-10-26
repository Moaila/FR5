'''
# 功能：利用彩色相机和深度相机，获取像素坐标对应的三维坐标
# 输入：彩色相机图像，深度相机图像
# 输出：像素坐标对应的三维坐标
'''
# - 矩阵运算
import numpy as np
# - 图像处理
import cv2 
# 自定义库
# 从阿凯机器人工具箱导入Gemini335类
from kyle_robot_toolbox.camera import Gemini335

# 创建相机对象
camera = Gemini335()

# 创建窗口
win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
cv2.namedWindow("color", flags=win_flag)
cv2.namedWindow("depth", flags=win_flag)

px = 0
py = 0
# 鼠标回调事件
def on_mouse(event,x,y,flags,param):
	global px, py
	if event == cv2.EVENT_MOUSEMOVE:
		# 鼠标移动事件
		px = x
		py = y
# 将on_mouse回调事件绑定到窗口image_win上
cv2.setMouseCallback("color", on_mouse)

while True:
	# 采集彩图
	color_img, depth_img = camera.read()
	if color_img is None or depth_img is None:
		print("图像获取失败")
		continue
	# 生成画布
	color_canvas = np.copy(color_img)
	depth_canvas = camera.depth_img2canvas(depth_img)
	# 绘制圆圈
	radius = 10 # 像素半径
	color = [0, 0, 255] # 颜色
	thickness = 5 # 宽度
	# 绘制圆圈
	color_canvas = cv2.circle(color_canvas, [px, py], radius, color, thickness)
	depth_canvas = cv2.circle(depth_canvas, [px, py], radius, color, thickness)
	# 像素坐标
	cv2.putText(depth_canvas, text=f'PX: {px} PY: {py}',\
			org=(20, 30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
			fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
 	# 计算三维坐标
	depth_value = depth_img[py, px]
	
	if depth_value != 0:
		print(f"深度值: {depth_value} mm")
    	# 有效深度
		cam_point3d = camera.depth_pixel2cam_point3d(\
									px, py, depth_value=depth_value)
		cam_x, cam_y, cam_z = cam_point3d
		print(f"彩色相机坐标系下的坐标: [{cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f}], 单位mm")
		# 在画布上绘制三维坐标
		cv2.putText(depth_canvas, text=f'Camera: X {cam_x:.1f} Y {cam_y:.1f} Z {cam_z:.1f}',\
			org=(20, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
			fontScale=1, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))
	else:
		print("深度值无效")
	
	# 显示图像
	cv2.imshow('color', color_canvas)
	cv2.imshow('depth', depth_canvas)
	
	key = cv2.waitKey(1)
	if key == ord('q'):
		# 如果按键为q 代表quit 退出程序
		break

# 关闭摄像头
camera.release()
# 销毁所有的窗口
cv2.destroyAllWindows()
