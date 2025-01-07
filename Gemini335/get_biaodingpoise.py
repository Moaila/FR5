'''
ArucoTag检测+位姿估计(3D相机版)
----------------------------
作者: 阿凯爱玩机器人 | 微信: xingshunkai  | QQ: 244561792
B站: https://space.bilibili.com/40344504
淘宝店铺: https://shop140985627.taobao.com
购买链接: https://item.taobao.com/item.htm?id=677075846402
'''
import time
import json
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray, String
# 阿凯机器人工具箱
# - Gemini335类
from kyle_robot_toolbox.camera import Gemini335
from kyle_robot_toolbox.opencv import ArucoTag
from kyle_robot_toolbox.open3d import *
# ArucoTag可视化窗口
from arucotag_visualizer import ArucoTagVisualizer
# ArucoTag姿态矫正器
from arucotag_pose_adjust import *

# 创建相机对象
camera = Gemini335()

# 创建ArucoTag检测器
arucotag = ArucoTag(camera, 
    config_path="/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/arucotag.yaml")

# 创建ArucoTag可视化窗口
aruco_size = arucotag.config["aruco_size"]/1000.0
box_depth = 0.01
visualizer = ArucoTagVisualizer(camera, 
    aruco_size=aruco_size, 
    box_depth=box_depth)
visualizer.create_window()

# 配置视角
json_path = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/render_option.json"
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

# 初始化ROS节点和话题发布者
rospy.init_node('camera_publisher_335', anonymous=True)
pub = rospy.Publisher('tag_pose', String, queue_size=10)
pub_list = rospy.Publisher('tag_trans_mat', Float32MultiArray, queue_size=10)

# 主循环
while not rospy.is_shutdown():
    try:
        # 采集图像
        img_bgr, depth_img = camera.read()
        # 图像移除畸变
        img_bgr = camera.remove_distortion(img_bgr)
        # 图像预处理
        img_filter = image_preprocessor(img_bgr)
        # 根据深度图生成画布
        depth_canvas = camera.depth_img2canvas(depth_img, 
            min_distance=150, max_distance=300)
        # 彩图+深度图生成点云
        scene_pcd = camera.get_pcd(img_bgr, depth_img)

        # ArucoTag检测
        has_aruco, canvas, aruco_ids, aruco_centers, \
        aruco_corners, T_cam2aruco_by2d = \
            arucotag.aruco_pose_estimate(img_filter)

        # 更新可视化窗口：场景点云
        visualizer.update_scene_pcd(scene_pcd)

        cam_x, cam_y, cam_z = 0.0, 0.0, 0.0
        T_cam2aruco_by3d_filter = []

        if has_aruco:
            # 矫正ArucoTag坐标
            valid_aruco_mask, t_cam2aruco_by3d_filter = get_t_cam2aruco_by3d( 
                camera, depth_img, aruco_ids, aruco_centers, 
                canvas=canvas, depth_canvas=depth_canvas)

            # 过滤有效的数据
            aruco_ids_filter = aruco_ids[valid_aruco_mask]
            aruco_centers_filter = aruco_centers[valid_aruco_mask]
            aruco_corners_filter = aruco_corners[valid_aruco_mask]
            T_cam2aruco_by2d_filter = T_cam2aruco_by2d[valid_aruco_mask]

            # 姿态矫正
            T_cam2aruco_by3d_filter = adjust_T_cam2aruco(camera, img_filter, depth_img,
                aruco_ids_filter, aruco_corners_filter,
                T_cam2aruco_by2d_filter, t_cam2aruco_by3d_filter)

            # 从T_cam2aruco_by3d_filter中提取位置信息(假设为4x4变换矩阵)
            if len(T_cam2aruco_by3d_filter) > 0:
                trans_matrix = T_cam2aruco_by3d_filter[0]
                # 假设变换矩阵格式为4x4, 取[0:3,3]作为平移向量
                cam_x = trans_matrix[0,3]
                cam_y = trans_matrix[1,3]
                cam_z = trans_matrix[2,3]

            # 更新ArucoTag的可视化模型
            visualizer.update_aruco(T_cam2aruco_by3d_filter)
        else:
            # 没检测到ArucoTag时，复位显示
            visualizer.reset_aruco()

        if not is_draw_camera:
            # 绘制相机
            visualizer.draw_camera()
            is_draw_camera = True

        # 控制视野
        set_view_control()
        # 可视化器迭代
        visualizer.step()

        cv2.imshow("depth", depth_canvas)
        cv2.imshow("canvas", canvas)

        # 发布ROS话题: 标签姿态(x,y,z)
        tag_pose_str = str(cam_x) + " " + str(cam_y) + " " + str(cam_z)
        pub.publish(tag_pose_str)

        # 发布ROS话题: 变换矩阵
        mat_apart_data = Float32MultiArray()
        mat_apart = []
        # print('T_cam2aruco_by3d_filter:', T_cam2aruco_by3d_filter)
        if len(T_cam2aruco_by3d_filter) > 0:
            tag_trans_mat = [row.tolist() for row in T_cam2aruco_by3d_filter[0]]
            for i in range(len(tag_trans_mat)):
                for j in range(len(tag_trans_mat[i])):
                    mat_apart.append(tag_trans_mat[i][j])
            mat_apart_data.data = mat_apart
        pub_list.publish(mat_apart_data)

        # 按键退出
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    except Exception as e:
        print(e)
        # 关闭窗口
        visualizer.destroy_window()
        # 释放相机
        camera.release()
        break
