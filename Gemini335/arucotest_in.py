"""
@作者： 李文皓
@功能： 利用arucotag码测试眼在手上标定准度
"""
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

# cam2end = [[-0.9700547336446754, 0.2155199065413726, 0.11200439106551091, -10.436744467553321], 
#            [-0.24003934944040217, -0.7802677800973608, -0.5775493936124987, 111.81257773485395], 
#            [-0.03707997375654929, -0.5871399843532499, 0.808635711751515, 90.21707744059526], 
#            [0.0, 0.0, 0.0, 1.0]]
# cam2end = [[-0.9384727961446402, -0.3451753975450282, 0.011079522827835156, 4.970117486876816],
#             [0.3133581221842016, -0.8645761953311295, -0.39282908462579436, 34.66284541312341],
#               [0.14517402714553218, -0.3651875509876813, 0.9195447539113952, 63.6499542560027],
#                 [0.0, 0.0, 0.0, 1.0]]
cam2end = [[-0.994467572622588, 0.0901125937855804, 0.05398117675053677, -11.622704892320861], 
 [-0.10493108986323413, -0.8760152838651967, -0.4707299531734651, 75.8532493113124], 
 [0.0048696388214730865, -0.47378997760169406, 0.8806244056020331, 71.39665338729412], 
 [0.0, 0.0, 0.0, 1.0]]

def get_qrcode_xyz(arucotag_config_path, render_option_path, aruco_id_need):
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
        x, y, z = trans_matrix[0, 3], trans_matrix[1, 3], trans_matrix[2, 3]
        
        return (x, y, z)
    
    except Exception as e:
        print(f"Error: {e}")
        return (None, None, None)
    
    finally:
        # 释放相机资源
        camera.release()

def tf_get_obj_to_base2(end2base,camera2end,obj2camera):
    '''
    得到obj2base变换矩阵
    @输入：
        obj2camera：物体在相机坐标系下的变换矩阵
        camera2base：相机在机械臂底座下的变换矩阵
    @输出：
        obj2base：物体在机械臂底座下的变换矩阵
    '''
    obj2base = np.dot(np.dot(end2base,camera2end),obj2camera)
    return obj2base
def get_transform_mat(X,Y,Z,RX,RY,RZ):
    '''
    从机械臂末端6D数据得到end2base变换矩阵
    @输入：
        XYZ,RXRYRZ：机械臂末端6D数据
    @输出：
        end_to_base：机械臂end2base数据
    '''
    # 旋转角度
    rx = np.deg2rad(RX)
    ry = np.deg2rad(RY)
    rz = np.deg2rad(RZ)

    # 绕x轴旋转矩阵
    Rx = np.array([[1, 0, 0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx), np.cos(rx)]])

    # 绕y轴旋转矩阵
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                [0, 1, 0],
                [-np.sin(ry), 0, np.cos(ry)]])

    # 绕z轴旋转矩阵
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz), np.cos(rz), 0],
                [0, 0, 1]])

    # 旋转矩阵的乘积
    R = np.dot(np.dot(Rz, Ry),Rx)

    # 平移向量
    tx = X
    ty = Y
    tz = Z

    # 变换矩阵
    end_to_base = np.array([[R[0, 0], R[0, 1], R[0, 2], tx],
                [R[1, 0], R[1, 1], R[1, 2], ty],
                [R[2, 0], R[2, 1], R[2, 2], tz],
                [0, 0, 0, 1]])
    return end_to_base

if __name__ == "__main__":
    robot = Robot.RPC('192.168.59.6')
    arucotag_config = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/arucotag.yaml"
    render_option = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/render_option.json"
    aruco_id_need = 1  # 需要检测的Aruco ID
    xyz = get_qrcode_xyz(arucotag_config, render_option, aruco_id_need)
    if xyz != (None, None, None):
        print(f"指定Aruco ID {aruco_id_need}的坐标: x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}")
        _,fr5_A_end = robot.GetActualTCPPose(0)

        # 得到end2base矩阵
        end2base = get_transform_mat(fr5_A_end[0],fr5_A_end[1],fr5_A_end[2],fr5_A_end[3],fr5_A_end[4],fr5_A_end[5])
        tag_trans_mat = np.array([[xyz[0]],[xyz[1]],[xyz[2]],[1]])
        base_xyz=tf_get_obj_to_base2(end2base, cam2end, tag_trans_mat)
        # print(f"相机坐标系到机器人基坐标系的转换矩阵：\n{base_xyz[0][0],base_xyz[1][0],base_xyz[2][0]}")
        print(f"相机坐标系到机器人基坐标系的坐标：\n{base_xyz[0][0],base_xyz[1][0],base_xyz[2][0]}")