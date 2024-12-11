'''
ArucoTag可视化窗口(Astra相机版)
----------------------------
作者: 阿凯爱玩机器人 | 微信: xingshunkai  | QQ: 244561792
B站: https://space.bilibili.com/40344504
淘宝店铺: https://shop140985627.taobao.com
购买链接: https://item.taobao.com/item.htm?id=677075846402
'''
import copy
import numpy as np
import open3d as o3d
from kyle_robot_toolbox.open3d import *

class ArucoTagVisualizer():
	'''ArucoTag点云可视化窗口'''
	def __init__(self, camera, aruco_size=0.08,\
			box_depth = 0.01, window_name="ArucoTag"):
		# 相机
		self.camera = camera
		# ArucoTag尺寸
		self.aruco_size = aruco_size
		# 板材厚度
		self.box_depth = box_depth
		# 窗口名字
		self.window_name = window_name
		# 可视化窗口
		self.visualizer = o3d.visualization.Visualizer()
		# 绘制相机
		self.draw_camera()
		# 点云数据
		self.scene_pcd = o3d.geometry.PointCloud()
		# 是否初始化了场景点云
		self.is_init_scene_pcd = False
		# ArucoTag的Mesh
		self.aruco_geometry_list = []

	def draw_camera(self):
		'''绘制相机'''
		# 相机在世界坐标系下的位姿(相机外参)
		T_world2cam = np.eye(4)
		# 相机内参
		intrinsic = self.camera.intrinsic_new
		# 图像尺寸
		img_width = self.camera.img_width
		img_height = self.camera.img_height
		# 可视化相机平面距离相机坐标系原点距离
		# 单位m
		panel_distance = 0.1
		# 创建相机相关的Gemometry
		camera_geometries = geometry_camera(intrinsic, T_world2cam, \
				img_width, img_height, \
				panel_distance = panel_distance, \
				color=[0.8, 0.2, 0.2], \
				draw_panel=False)
		# 将Gemometry添加到Visualizer里面
		for geometry in camera_geometries:
			self.visualizer.add_geometry(geometry)
		return camera_geometries
	def create_window(self):
		'''创建窗口'''
		# 创建窗口
		self.visualizer.create_window(self.window_name, width=1280, height=720)
	
	def destroy_window(self):
		'''销毁窗口'''
		self.visualizer.destroy_window()

	def reset_scene_pcd(self):
		'''重置点云数据'''
		self.scene_pcd.clear()
	
	def update_scene_pcd(self, pcd, is_reset=True):
		'''更新点云数据'''
		if is_reset:
			self.reset_scene_pcd()
		# 添加新的场景点云
		self.scene_pcd += copy.deepcopy(pcd)
		if not self.is_init_scene_pcd:
			self.visualizer.add_geometry(self.scene_pcd, reset_bounding_box=True)
			self.is_init_scene_pcd = True
		# 可视化窗口更新
		self.visualizer.update_geometry(self.scene_pcd)
	
	def reset_aruco(self):
		'''清除已有的ArucoTag'''
		# 移除原有的Geometry
		for geometry in self.aruco_geometry_list:
			self.visualizer.remove_geometry(geometry)
		# 创建一个新的列表
		self.aruco_geometry_list = []

	def update_aruco(self, T_cam2aruco_list, is_reset=True):
		'''更新ArucoTag'''
		if is_reset:
			self.reset_aruco()
		
		for T_cam2aruco in T_cam2aruco_list:
			# 单位mm转m
			T_cam2board_m = np.copy(T_cam2aruco)
			T_cam2board_m[:3, 3] /= 1000.0
			# ArucoTag坐标系
			coord = geometry_coordinate(T_cam2board_m, size=self.aruco_size*1.5)
			# ArucoTag板
			# ArucoTag尺寸
			board_width, board_height = self.aruco_size, self.aruco_size 
			board = geometry_box(T_cam2board_m, board_width, board_height,\
								box_depth=self.box_depth, color=[0.0, 0.5, 0.0])
			self.aruco_geometry_list.append(coord)
			self.aruco_geometry_list.append(board)
		
		for geometry in self.aruco_geometry_list:
			self.visualizer.add_geometry(geometry, reset_bounding_box=False)

	def step(self):
		'''更新一步'''
		# 接收事件
		self.visualizer.poll_events()
		# 渲染器需要更新
		self.visualizer.update_renderer()