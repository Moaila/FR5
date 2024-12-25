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

# 眼在手外

# camera2base = [[-0.9983436760441877, 0.003310123068703671, -0.05743646566292077, -75.16659019011934], \
#                [0.00833952766183191, 0.9961254408213325, -0.08754746385180773, -771.9754021846082], \
#                 [0.05692413179799392, -0.08788144988435317, -0.9945031392536017, 857.6738248978729], \
#                 [0.0, 0.0, 0.0, 1.0]]

# 眼在手内
camera2base = [[0.9683207339192607, 0.08773797558436772, 0.23378837418148343, -811.8524677148089], \
               [-0.11459614017451836, 0.9879669781719643, 0.10386998940434929, 929.2761644138345], \
                [-0.22186185097750935, -0.12737070967107936, 0.9667233427401652, -1549.8493163129576], \
                [0.0, 0.0, 0.0, 1.0]]

# def get_qrcode_xyz(arucotag_config_path, render_option_path, aruco_id_need):
#     """
#     获取所有检测到的Aruco标签的XYZ坐标。

#     参数:
#         arucotag_config_path (str): ArucoTag配置文件的路径。
#         render_option_path (str): 渲染选项配置文件的路径。
#         aruco_id_need (int): 需要检测的Aruco标签ID。

#     返回:
#         tuple: (x, y, z) 坐标，如果未检测到指定Aruco标签，则返回 (None, None, None)。
#     """
#     # 初始化相机
#     camera = Gemini335()
#     try:
#         # 创建ArucoTag检测器
#         arucotag = ArucoTag(camera, config_path=arucotag_config_path)
        
#         # 读取渲染配置（如果需要）
#         with open(render_option_path, "r", encoding="utf-8") as f:
#             trajectory = json.load(f)
#         view_point = trajectory["trajectory"][0]  # 可选，若不需要可移除此部分
        
#         # 采集图像
#         img_bgr, depth_img = camera.read()
        
#         # 移除畸变
#         img_bgr = camera.remove_distortion(img_bgr)
        
#         # 图像预处理（假设存在该函数）
#         img_filter = image_preprocessor(img_bgr)
        
#         # ArucoTag检测
#         has_aruco, _, aruco_ids, aruco_centers, aruco_corners, T_cam2aruco_by2d = arucotag.aruco_pose_estimate(img_filter)
#         print(f"检测到{len(aruco_ids)}个ArucoTag。")
#         print(f"Aruco IDs: {aruco_ids}")
#         if not has_aruco:
#             return (None, None, None)
        
#         # 矫正ArucoTag坐标（假设存在该函数）
#         valid_mask, t_cam2aruco_by3d = get_t_cam2aruco_by3d(
#             camera, depth_img, aruco_ids, aruco_centers)
        
#         # 过滤有效数据
#         aruco_ids_filter = aruco_ids[valid_mask]
#         aruco_centers_filter = aruco_centers[valid_mask]
#         aruco_corners_filter = aruco_corners[valid_mask]
#         T_cam2aruco_by2d_filter = T_cam2aruco_by2d[valid_mask]
#         t_cam2aruco_by3d_filter = t_cam2aruco_by3d[valid_mask]
        
#         # 姿态矫正
#         T_cam2aruco_by3d_filter = adjust_T_cam2aruco(
#             camera, img_filter, depth_img,
#             aruco_ids_filter, aruco_corners_filter,
#             T_cam2aruco_by2d_filter, t_cam2aruco_by3d_filter)
#         # print(T_cam2aruco_by3d_filter)
#         if len(T_cam2aruco_by3d_filter) == 0:
#             return (None, None, None)
        
#         # 查找指定的Aruco ID
#         indices = np.where(aruco_ids_filter == aruco_id_need)[0]
#         if len(indices) == 0:
#             print(f"未检测到Aruco ID {aruco_id_need}。")
#             return (None, None, None)
        
#         # 假设每个Aruco ID唯一，取第一个匹配的
#         index = indices[0]
#         trans_matrix = T_cam2aruco_by3d_filter[index]
#         x, y, z = trans_matrix[0, 3], trans_matrix[1, 3], trans_matrix[2, 3]
        
#         return (x, y, z)
    
#     except Exception as e:
#         print(f"Error: {e}")
#         return (None, None, None)
    
#     finally:
#         # 释放相机资源
#         camera.release()

def camera_to_base(camera_trans_mat,camera2base):
    """
    将相机坐标系转换为机器人基坐标系。

    参数:
        camera_trans_mat (np.ndarray): 相机坐标系的转换矩阵。
        camera2base (np.ndarray): 相机到机器人基坐标系的转换矩阵。

    返回:
        np.ndarray: 相机坐标系到机器人基坐标系的转换矩阵。
    """
    obj2base = np.dot(camera2base,camera_trans_mat)
    return obj2base

def get_all_qrcode_xyz(arucotag_config_path, render_option_path):
    """
    获取所有检测到的 Aruco 标签的 ID 和 XYZ 坐标。

    参数:
        arucotag_config_path (str): ArucoTag 配置文件的路径。
        render_option_path (str): 渲染选项配置文件的路径。

    返回:
        dict: 包含所有检测到的 Aruco 标签的 ID 和对应的坐标，格式为 {id: (x, y, z)}。
              如果未检测到任何标签，则返回空字典 {}。
    """
    # 初始化相机
    camera = Gemini335()
    results = {}  # 用于存储检测结果

    try:
        # 创建 ArucoTag 检测器
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
        
        # ArucoTag 检测
        has_aruco, _, aruco_ids, aruco_centers, aruco_corners, T_cam2aruco_by2d = arucotag.aruco_pose_estimate(img_filter)
        print(f"检测到 {len(aruco_ids)} 个 ArucoTag。")
        print(f"Aruco IDs: {aruco_ids}")

        if not has_aruco:
            print("未检测到任何 Aruco 标签。")
            return results  # 返回空字典
        
        # 矫正 ArucoTag 坐标
        valid_mask, t_cam2aruco_by3d = get_t_cam2aruco_by3d(
            camera, depth_img, aruco_ids, aruco_centers
        )
        
        # 使用有效的 mask 过滤数据
        aruco_ids_filter = aruco_ids[valid_mask]
        t_cam2aruco_by3d_filter = t_cam2aruco_by3d[valid_mask]
        
        # 姿态矫正
        T_cam2aruco_by3d_filter = adjust_T_cam2aruco(
            camera, img_filter, depth_img,
            aruco_ids_filter, aruco_corners[valid_mask],
            T_cam2aruco_by2d[valid_mask], t_cam2aruco_by3d_filter
        )
        
        # 检查是否有有效的矫正结果
        if len(T_cam2aruco_by3d_filter) == 0:
            print("未检测到有效的 Aruco 标签。")
            return results  # 返回空字典

        # 遍历所有有效的标签
        for i, aruco_id in enumerate(aruco_ids_filter):
            trans_matrix = T_cam2aruco_by3d_filter[i]
            x, y, z = trans_matrix[0, 3], trans_matrix[1, 3], trans_matrix[2, 3]

            # 将结果保存到字典中
            results[int(aruco_id)] = (x, y, z)

        return results
    
    except Exception as e:
        print(f"Error: {e}")
        return results  # 返回空字典
    
    finally:
        # 释放相机资源
        camera.release()


if __name__ == "__main__":
    # robot = Robot.RPC('192.168.59.6')
    # robot.ActGripper(1,0)
    # time.sleep(1)
    # robot.ActGripper(1,1)
    # time.sleep(1)
    # robot.MoveGripper(1, 100, 50, 30, 10000, 1)
    # time.sleep(3)
    # desc_pos = [-90.0, -400.0, 100.0, 90.0, 0.0, 0.0]
    # robot.MoveCart(desc_pos, 0, 0)
    # # exit(0)
    # input("按任意键...")
    arucotag_config = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/arucotag.yaml"
    render_option = "/home/newplace/FR5/Gemini335/opencv-example/config/arucotag/render_option.json"
     # 调用检测函数
    results = get_all_qrcode_xyz(arucotag_config, render_option)

    if results:
        print("检测到以下 Aruco 标签及其坐标：")
        for aruco_id, coords in results.items():
            print(f"Aruco ID {aruco_id}: x={coords[0]:.3f}, y={coords[1]:.3f}, z={coords[2]:.3f}")
    else:
        print("未检测到任何 Aruco 标签。")
    # aruco_id_need = 1  # 需要检测的Aruco ID
    # xyz = get_qrcode_xyz(arucotag_config, render_option, aruco_id_need)
    # if xyz != (None, None, None):
    #     print(f"指定Aruco ID {aruco_id_need}的坐标: x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}")
    #     base_xyz=camera_to_base(np.array([[xyz[0]],[xyz[1]],[xyz[2]],[1]]),camera2base)
    #     print(f"相机坐标系到机器人基坐标系的转换矩阵：\n{base_xyz[0][0],base_xyz[1][0],base_xyz[2][0]}")
    #     print(f"相机坐标系到机器人基坐标系的坐标：\n{base_xyz[0][0]+25.0,base_xyz[1][0]+200.0,base_xyz[2][0]+60.0}")
    #     input("按任意键...")
    #     mid_x=base_xyz[0][0]+25.0
    #     mid_y=base_xyz[1][0]+190.0
    #     mid_z=100.0
    #     desc_pos_aim = [mid_x-90.0, mid_y+60.0, mid_z, 90.0,0.0,0.0]
    #     robot.MoveCart(desc_pos_aim, 0, 0)
    #     time.sleep(2)
    #     desc_pos_aim = [mid_x-90.0, mid_y-60.0, mid_z, 90.0,0.0,0.0]
    #     robot.MoveCart(desc_pos_aim, 0, 0)
    #     time.sleep(2)
    #     robot.MoveGripper(1, 0, 50, 30, 10000, 1)
    #     time.sleep(3)
    #     robot.MoveGripper(1, 100, 50, 30, 10000, 1)
    #     time.sleep(3)
    # else:
    #     print("未检测到指定的Aruco标签或发生错误。")