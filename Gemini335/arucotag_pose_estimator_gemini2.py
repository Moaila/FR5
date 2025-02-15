'''
ArucoTag检测+位姿估计(3D相机版)

'''
import time
import json
import numpy as np
import cv2
import rospy
from std_msgs.msg import UInt8MultiArray,Float32MultiArray,String
# 阿凯机器人工具箱
# - Gemini2类
from kyle_robot_toolbox.camera import Gemini2
from kyle_robot_toolbox.opencv import ArucoTag
from kyle_robot_toolbox.yolov8 import YoloV8Detect
from kyle_robot_toolbox.open3d import *
# ArucoTag可视化窗口
from arucotag_visualizer import ArucoTagVisualizer
# ArucoTag姿态矫正器
from arucotag_pose_adjust import *

# 创建相机对象
camera = Gemini2()

# 创建ArucoTag检测器
arucotag = ArucoTag(camera, \
	config_path="config/arucotag/arucotag.yaml")

# 创建ArucoTag可视化窗口
aruco_size = arucotag.config["aruco_size"]/1000.0
box_depth=0.01
visualizer = ArucoTagVisualizer(camera, \
	aruco_size=aruco_size, \
	box_depth=box_depth)
visualizer.create_window()

# 配置视角
json_path = "config/arucotag/render_option.json"
trajectory = json.load(open(json_path, "r", encoding="utf-8"))
view_point = trajectory["trajectory"][0]
def set_view_control():
	'''控制视野'''
	global view_point
	ctr = visualizer.visualizer.get_view_control()
	ctr.set_front(np.float64(view_point["front"]))
	ctr.set_lookat(np.float64(view_point["lookat"]))
	ctr.set_up(np.float64(view_point["up"]))
	ctr.set_zoom(np.float64(view_point["zoom"]))

is_draw_camera = False





def loop():
	global is_draw_camera
	global camera 
	# global mat_apart
	global arucotag
	global aruco_size
	global box_depth
	global visualizer
	global json_path
	global trajectory
	global view_point
	global cam_x
	global cam_y
	global cam_z
	# 主程序
	mat_apart = []# 二维数组拆分为一维数组，便于ros发送
	mat_apart_data = Float32MultiArray()
	T_cam2aruco_by3d_filter = []
	pub = rospy.Publisher('tag_pose', String, queue_size=10)
	pub_list = rospy.Publisher('tag_trans_mat', Float32MultiArray , queue_size=10)
	rospy.init_node('camera_publisher', anonymous=True)
	while True:
		cam_x = 0.0
		cam_y = 0.0
		cam_z = 0.0
		try:
			# cam_x = 0.0
			# cam_y = 0.0
			# cam_z = 0.0
			# 采集图像
			img_bgr, depth_img = camera.read()
			# 图像移除畸变
			img_bgr = camera.remove_distortion(img_bgr)
			# 图像预处理
			img_filter = image_preprocessor(img_bgr)
			# 根据深度图生成画布
			depth_canvas = camera.depth_img2canvas(depth_img, \
				min_distance=150, max_distance=300)
			# 彩图+深度图生成点云
			scene_pcd = camera.get_pcd(img_bgr, depth_img)

			# ArucoTag检测
			has_aruco, canvas, aruco_ids, aruco_centers,\
				aruco_corners, T_cam2aruco_by2d = \
				arucotag.aruco_pose_estimate(img_filter)
			
			# 更新可视化窗口
			# - 场景点云
			visualizer.update_scene_pcd(scene_pcd)

			if has_aruco:
				# 矫正ArucoTag的坐标
				print('+++++')
				# 注: 返回的t_cam2aruco_by3d是过滤后的
				valid_aruco_mask, t_cam2aruco_by3d_filter , cam_x, cam_y, cam_z= get_t_cam2aruco_by3d(\
						camera, depth_img, aruco_ids, aruco_centers,  \
						canvas=canvas, depth_canvas=depth_canvas)
				# 过滤有效的ID、中心、角点、空间变换
				# 矫正ArucoTag的姿态
				aruco_ids_filter = aruco_ids[valid_aruco_mask]
				aruco_centers_filter = aruco_centers[valid_aruco_mask]
				aruco_corners_filter = aruco_corners[valid_aruco_mask]
				T_cam2aruco_by2d_filter = T_cam2aruco_by2d[valid_aruco_mask]
				# 获取矫正后的ArucoTag姿态
				T_cam2aruco_by3d_filter = adjust_T_cam2aruco(camera, img_filter, depth_img, \
					aruco_ids_filter, aruco_corners_filter, \
					T_cam2aruco_by2d_filter, t_cam2aruco_by3d_filter)
				# print('姿态：',T_cam2aruco_by3d_filter)
				print('xyz:',cam_x, cam_y, cam_z)
				# 更新ArucoTag的可视化模型
				visualizer.update_aruco(T_cam2aruco_by3d_filter)
			else:
				cam_x = 0.0
				cam_y = 0.0
				cam_z = 0.0
				
				visualizer.reset_aruco()
			
			if not is_draw_camera:
				# 绘制相机
				visualizer.draw_camera()
				is_draw_camera = True
			# 控制视野
			set_view_control()
			# - 可视化器迭代
			visualizer.step()
			print('go here')
			# tag坐标打包处理并发送
			tag_pose = str(cam_x) + " " + str(cam_y) + " " + str(cam_z)
			print('tag_pose',tag_pose)
			if  T_cam2aruco_by3d_filter != []:
				tag_trans_mat = [row.tolist() for row in T_cam2aruco_by3d_filter[0]]
				for i in range(len(tag_trans_mat)):
					for j in range(len(tag_trans_mat[i])):
						mat_apart.append(tag_trans_mat[i][j])
				print(mat_apart,'\n')
				mat_apart_data.data = mat_apart
			pub.publish(tag_pose)
			pub_list.publish(mat_apart_data)
			cv2.imshow("depth", depth_canvas)
			cv2.imshow("canvas", canvas)
			mat_apart = []
			# 按键退出
			key = cv2.waitKey(1)
			if key == ord('q'):
				# 如果按键为q 代表quit 退出程序
				break
		except Exception as e:
			print(e)
			# 关闭窗口
			visualizer.destroy_window()
			# 释放相机
			camera.release()


if __name__ == '__main__':
    try:
        loop()
    except rospy.ROSInterruptException as e:
        print(e,"error\n")