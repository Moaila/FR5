'''
ArucoTag 姿态矫正器

'''
import numpy as np
import cv2
import open3d as o3d
# 阿凯机器人工具箱
from kyle_robot_toolbox.open3d import *

def image_preprocessor(img_bgr, kerenal_size=3):
	'''图像预处理'''
	# 灰度图
	img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
	# 直方图均衡
	# img_gray = cv2.equalizeHist(img_gray)
	# 中值滤波
	# img_gray = cv2.medianBlur(img_gray, kerenal_size)
	# 转换为BGR空间
	img_filter = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
	return img_filter

def depth_image2canvas(camera, img_bgr, depth_img):
	'''深度图转换位画布'''
	# 将深度转换为可视化画布
	# 根据实际情况调整深度范围 [min_distance, max_distance]
	depth_canvas_tmp = camera.depth_img2canvas(depth_img, min_distance=300, \
										max_distance=400)
	# 为了兼容Gemini深度图与彩图尺寸不一致的情况
	# 要做一下特殊处理
	dp_h, dp_w, dp_ch = depth_canvas_tmp.shape
	depth_canvas = np.zeros_like(img_bgr)
	depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp
	return depth_canvas

def get_t_cam2aruco_by3d(camera, depth_img, aruco_ids, aruco_centers,  \
		canvas=None, depth_canvas=None):
	'''根据深度图得到ArucoTag的实际坐标'''
	# 过滤后的Aruco ID
	valid_aruco_mask = []
	# 获取深度图的尺寸
	dp_h, dp_w = depth_img.shape
	# 3D点列表
	t_cam2aruco_by3d = []
	aruco_num = len(aruco_ids)
	for i in range(aruco_num):
		# 获取Aruco ID与像素中心
		aruco_id = aruco_ids[i]
		px, py  = aruco_centers[i]
		# 判断坐标是否在深度图的有效范围内
		if px >= dp_w or py >= dp_h:
			valid_aruco_mask.append(False)
			continue 
		# 读取深度值
		depth_value = depth_img[py, px]
		# 深度值无效
		if depth_value == 0:
			valid_aruco_mask.append(False)
			continue
		
		# 添加过滤后的ArucoTag ID
		valid_aruco_mask.append(True)
		# 计算三维坐标
		cam_point3d = camera.depth_pixel2cam_point3d(\
										px, py, depth_value=depth_value)
		# 追加到列表中
		t_cam2aruco_by3d.append(cam_point3d)
		
		if canvas is not None:
			# 绘制中心点
			cv2.circle(canvas, [px, py], 5, (255, 255, 0), -1)
			# 在画面上绘制坐标
			cam_x, cam_y, cam_z = cam_point3d
			tag = f"X{cam_x:.0f}_Y{cam_y:.0f}_Z{cam_z:.0f}"
			cv2.putText(canvas, text=tag,\
				org=(px-50, py-30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
				fontScale=0.8, thickness=1, lineType=cv2.LINE_AA, color=(255, 0, 255))
		if depth_canvas is not None:
			cv2.circle(depth_canvas, [px, py], 5, (255, 0, 255), -1)
	# 格式转换
	valid_aruco_mask = np.bool_(valid_aruco_mask)
	t_cam2aruco_by3d = np.float64(t_cam2aruco_by3d)
	return valid_aruco_mask, t_cam2aruco_by3d

def adjust_T_cam2aruco(camera, img_filter, depth_img, \
		aruco_ids, aruco_corners, \
		T_cam2aruco_by2d, t_cam2aruco_by3d):
	'''矫正ArucoTag的位姿'''
	T_cam2aruco_by3d = []
	img_h, img_w, _ = img_filter.shape
	for aruco_idx in range(len(aruco_ids)):
		aruco_mask = np.zeros((img_h, img_w), dtype=np.uint8)
		aruco_mask = cv2.drawContours(aruco_mask,\
						[aruco_corners[aruco_idx].astype(np.int32)], \
						-1, color=255, thickness=-1)
		# 获取ArucoTag的点云
		aruco_pcd = camera.get_pcd(img_filter, depth_img, mask=aruco_mask)
		# 提取在2D图中得到的ArucoTag位姿
		T_cam2aruco1 = T_cam2aruco_by2d[aruco_idx]
		# 使用其中的旋转矩阵作为起始值
		T_temp = np.copy(T_cam2aruco1)
		T_temp[:3, 3] = t_cam2aruco_by3d[aruco_idx]
		# 矫正ArucoTag姿态
		T_cam2aruco2 = adjust_board_pose_by_pcd(T_temp, \
												aruco_pcd)
		T_cam2aruco_by3d.append(T_cam2aruco2)
	return T_cam2aruco_by3d