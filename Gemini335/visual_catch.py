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
from fairino import Robot


class Robot_System:
    def __init__(self):
        self.camera2base = [[-0.9983436760441877, 0.003310123068703671, -0.05743646566292077, -75.16659019011934], 
                    [0.00833952766183191, 0.9961254408213325, -0.08754746385180773, -771.9754021846082], 
                        [0.05692413179799392, -0.08788144988435317, -0.9945031392536017, 857.6738248978729], 
                        [0.0, 0.0, 0.0, 1.0]]
        self.obj_xyz_offset = [[-90, -60, 0], [+90, -60, 0]]
        self.catch_pre_xyz_offset = [[0, +120, 0], [0, +120, 0], [0, -120, 0], [0, -120, 0]]
        self.catch_direction = [[90.0,0.0,0.0], [90.0,0.0,0.0], [90.0,0.0,0.0], [90.0,0.0,0.0]]
        self.safe_catch_place_id = \
        {
            'shaobei':1,
            'shaoping':1,
            'liquid_add':3,
            'solid_add':3,
        }
        self.safe_place=[[-300.0, -350.0, 200.0, 90.0, 0.0, -90.0],
                    [0.0, -250.0, 400.0, 90.0, 0.0, 0.0],
                    [450.0, 50.0, 400.0, 90.0, 0.0, 90.0],
                    [100.0, 400.0, 400.0, 90.0, 0.0, 180.0]]

        self.now_place = 1

    def get_qrcode_xyz(self, arucotag_config_path, render_option_path, aruco_id_need):
        """
        获取指定Aruco标签的XYZ坐标。

        参数:
            arucotag_config_path (str): ArucoTag配置文件的路径。
            render_option_path (str): 渲染选项配置文件的路径。
            aruco_id_need (int): 需要检测的Aruco标签ID。

        返回:
            tuple: (x, y, z) 坐标，如果未检测到指定Aruco标签，则返回 (None, None, None)。
        """
        # 初始化相机
        arucotag_config_path = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/arucotag.yaml"
        render_option_path = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/render_option.json"
        camera = Gemini335()
        
        try:
            # 创建ArucoTag检测器
            arucotag = ArucoTag(camera, config_path=arucotag_config_path)
            
            # 读取渲染配置（如果需要）
            with open(render_option_path, "r", encoding="utf-8") as f:
                trajectory = json.load(f)
            view_point = trajectory["trajectory"][0]  # 可选，若不需要可移除此部分
            
            # 采集图像
            img_bgr, depth_img = camera.read()
            
            # 移除畸变
            img_bgr = camera.remove_distortion(img_bgr)
            
            # 图像预处理（假设存在该函数）
            img_filter = image_preprocessor(img_bgr)
            
            # ArucoTag检测
            has_aruco, _, aruco_ids, aruco_centers, aruco_corners, T_cam2aruco_by2d = arucotag.aruco_pose_estimate(img_filter)
            print(f"检测到{len(aruco_ids)}个ArucoTag。")
            print(f"Aruco IDs: {aruco_ids}")
            if not has_aruco:
                return (None, None, None)
            
            # 矫正ArucoTag坐标（假设存在该函数）
            valid_mask, t_cam2aruco_by3d = get_t_cam2aruco_by3d(
                camera, depth_img, aruco_ids, aruco_centers)
            
            # 过滤有效数据
            aruco_ids_filter = aruco_ids[valid_mask]
            aruco_centers_filter = aruco_centers[valid_mask]
            aruco_corners_filter = aruco_corners[valid_mask]
            T_cam2aruco_by2d_filter = T_cam2aruco_by2d[valid_mask]
            t_cam2aruco_by3d_filter = t_cam2aruco_by3d[valid_mask]
            
            # 姿态矫正
            T_cam2aruco_by3d_filter = adjust_T_cam2aruco(
                camera, img_filter, depth_img,
                aruco_ids_filter, aruco_corners_filter,
                T_cam2aruco_by2d_filter, t_cam2aruco_by3d_filter)
            # print(T_cam2aruco_by3d_filter)
            if len(T_cam2aruco_by3d_filter) == 0:
                return (None, None, None)
            
            # 查找指定的Aruco ID
            indices = np.where(aruco_ids_filter == aruco_id_need)[0]
            if len(indices) == 0:
                print(f"未检测到Aruco ID {aruco_id_need}。")
                return (None, None, None)
            
            # 假设每个Aruco ID唯一，取第一个匹配的
            index = indices[0]
            trans_matrix = T_cam2aruco_by3d_filter[index]
            xyz = trans_matrix[0, 3], trans_matrix[1, 3], trans_matrix[2, 3]

            base_xyz=self.camera_to_base(np.array([[xyz[0]],[xyz[1]],[xyz[2]],[1]]), self.camera2base)
            print(f"相机坐标系到机器人基坐标系的转换矩阵：\n{base_xyz[0][0], base_xyz[1][0], base_xyz[2][0]}")
            print(f"相机坐标系到机器人基坐标系的坐标：\n{base_xyz[0][0]+25.0, base_xyz[1][0]+200.0, base_xyz[2][0]+60.0}")

            mid_x=base_xyz[0][0]+25.0
            mid_y=base_xyz[1][0]+190.0
            mid_z=100.0

            return mid_x, mid_y, mid_z
        
        except Exception as e:
            print(f"Error: {e}")
            return (None, None, None)
        
        finally:
            # 释放相机资源
            camera.release()

    def camera_to_base(self, camera_trans_mat):
        """
        将相机坐标系转换为机器人基坐标系。

        参数:
            camera_trans_mat (np.ndarray): 相机坐标系的转换矩阵。
            self.camera2base (np.ndarray): 相机到机器人基坐标系的转换矩阵。

        返回:
            np.ndarray: 相机坐标系到机器人基坐标系的转换矩阵。
        """
        obj2base = np.dot(self.camera2base,camera_trans_mat)
        return obj2base

    def move_to_safe_catch(self, aim_place):
        if aim_place>self.now_place:
            for i in range(self.now_place+1, aim_place+1):
                desc_pos = self.safe_place[i]
                print(i)
                print(desc_pos)
                robot.MoveCart(desc_pos, 0, 0)
                time.sleep(2)
        else:
            for i in range(self.now_place-1, aim_place-1,-1):
                desc_pos = self.safe_place[i]
                print(i)
                print(desc_pos)
                robot.MoveCart(desc_pos, 0, 0)
                time.sleep(2)
        self.now_place = aim_place

    def visual_catch(self, tag_id, obj_id):

        #根据id确定安全位置, 移动到安全位置
        aim_place = self.safe_catch_place_id[tag_id]
        self.move_to_safe_catch(aim_place)

        #读取视觉
        mid_x, mid_y, mid_z = self.get_qrcode_xyz(tag_id)
        mid_z=100.0
        if mid_x==None:
            print('未检测到指定的Aruco标签或发生错误。')
            return 0

        #计算物体位置
        mid_x += self.obj_xyz_offset[obj_id][0]
        mid_y += self.obj_xyz_offset[obj_id][1]
        mid_z += self.obj_xyz_offset[obj_id][2]

        #移动到准备位置
        desc_pos_aim = [mid_x+self.catch_pre_xyz_offset[tag_id][0], mid_y+self.catch_pre_xyz_offset[tag_id][1], mid_z+self.catch_pre_xyz_offset[tag_id][2]] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0)
        time.sleep(3)

        #靠近，完成抓取
        desc_pos_aim = [mid_x, mid_y, mid_z] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0)
        time.sleep(3)

        robot.MoveGripper(1, 0, 50, 30, 10000, 1)
        time.sleep(3)

        #抬起
        desc_pos_aim = [mid_x, mid_y, mid_z+100] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0, vel=10)
        time.sleep(3)

        #移动到安全位置
        desc_pos = self.safe_place[aim_place]
        robot.MoveL(desc_pos, 0, 0)
        time.sleep(3)

    def visual_put(self, tag_id, obj_id):

        #根据id确定安全位置, 移动到安全位置
        aim_place = self.safe_catch_place_id[tag_id]
        self.move_to_safe_catch(aim_place)

        #读取视觉
        mid_x, mid_y, mid_z = self.get_qrcode_xyz(tag_id)
        mid_z=100.0
        if mid_x==None:
            print('未检测到指定的Aruco标签或发生错误。')
            return 0

        #计算物体位置
        mid_x += self.obj_xyz_offset[obj_id][0]
        mid_y += self.obj_xyz_offset[obj_id][1]
        mid_z += self.obj_xyz_offset[obj_id][2]

        #移动到准备位置上方
        desc_pos_aim = [mid_x, mid_y, mid_z+100] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0)
        time.sleep(3)

        #下降，完成放置
        desc_pos_aim = [mid_x, mid_y, mid_z+50] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0, vel=10)
        time.sleep(3)
        desc_pos_aim = [mid_x, mid_y, mid_z+5] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0, vel=10)
        time.sleep(3)

        robot.MoveGripper(1, 100, 50, 30, 10000, 1)
        time.sleep(3)

        #移动出去
        desc_pos_aim = [mid_x+self.catch_pre_xyz_offset[tag_id][0], mid_y+self.catch_pre_xyz_offset[tag_id][1], mid_z+self.catch_pre_xyz_offset[tag_id][2]] + self.catch_direction[tag_id]
        robot.MoveL(desc_pos_aim, 0, 0)
        time.sleep(3)

        #移动到安全位置
        desc_pos = self.safe_place[aim_place]
        robot.MoveL(desc_pos, 0, 0)
        time.sleep(3)

if __name__ == "__main__":
    robot = Robot.RPC('192.168.58.2')
    robot_system = Robot_System()
    robot.ActGripper(1,0)
    time.sleep(1)
    robot.ActGripper(1,1)
    time.sleep(1)
    robot.MoveGripper(1, 100, 50, 30, 10000, 1)
    time.sleep(3)
    desc_pos = robot_system.safe_place[3]
    robot_system.now_place = 3
    robot.MoveCart(desc_pos, 0, 0)
    time.sleep(2)
    # exit(0)
    input("按任意键...")
    # aruco_id_need = 1  # 需要检测的Aruco ID
    # object_id = 0
    # visual_catch(aruco_id_need, object_id)
    # object_id = 1
    # visual_put(aruco_id_need, object_id)
    robot_system.move_to_safe_catch(1)

