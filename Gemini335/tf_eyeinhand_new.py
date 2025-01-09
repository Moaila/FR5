#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
眼在手内标定程序，自动获取数据并完成Tsai-Lenz手眼标定。
'''

import rospy
import os
import json
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from fairino import Robot

# 定义保存的文件路径
output_file = "fr5_A_end_outputs.json"

# 初始化一个列表用于存储所有的 fr5_A_end 数据
all_fr5_A_end = []

# 初始化 ROS 话题接收变量
tag_trans_mat = []
robot = Robot.RPC('192.168.59.6')

def init():
    '''
    实例化机械臂对象
    '''
    global robot
    robot = Robot.RPC('192.168.59.6')
    time.sleep(0.5)

def save_to_file(matrix):
    '''
    保存变换矩阵到文件
    '''
    log_dir = "/home/newplace/FR5/demo_ws/src/frcobot_ros/fr5_moveit_config/log"
    file_name = "1.9_new_眼在手上标定结果.txt"
    
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    file_path = os.path.join(log_dir, file_name)
    
    # 如果文件已存在，则找到新的文件名
    index = 1
    while os.path.exists(file_path):
        index += 1
        file_path = os.path.join(log_dir, f"{index}.txt")
        print('重复文件名，保存至新文件。')

    # 转换为Python列表保存为JSON格式
    with open(file_path, 'w') as file:
        matrix_list = matrix.tolist()
        matrix_str = json.dumps(matrix_list, indent=4)
        file.write(matrix_str)
        print('文件已保存：', file_path)

def camera_callback2(rece_tag_trans_mat):
    '''
    回调函数：接收tag_to_camera的变换矩阵
    '''
    global tag_trans_mat
    if rece_tag_trans_mat.data:
        tag_trans_mat = rece_tag_trans_mat.data
        tag_trans_mat = list(tag_trans_mat)
        tag_trans_mat = [tag_trans_mat[i:i+4] for i in range(0, len(tag_trans_mat), 4)]
    else:
        rospy.logwarn("接收到的tag_trans_mat为空。")

def get_transform_mat(X, Y, Z, RX, RY, RZ):
    '''
    根据末端位姿生成变换矩阵
    '''
    rx, ry, rz = np.deg2rad([RX, RY, RZ])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return np.array([[R[0, 0], R[0, 1], R[0, 2], X],
                     [R[1, 0], R[1, 1], R[1, 2], Y],
                     [R[2, 0], R[2, 1], R[2, 2], Z],
                     [0, 0, 0, 1]])

def get_RT_from_transform_mat(transform_mat):
    '''
    从变换矩阵中提取旋转矩阵和平移向量
    '''
    rot_mat = transform_mat[:3, :3]
    translate_mat = transform_mat[:3, 3]
    return rot_mat, translate_mat

def get_transform_mat_from_RT(R, T):
    '''
    根据旋转矩阵和平移向量生成变换矩阵
    '''
    Z = [0.0, 0.0, 0.0, 1.0]
    return np.vstack((np.hstack((R, T.reshape(3, 1))), Z))

def solve_hand_eye_tsai_lenz(A_list, B_list):
    """
    使用 Tsai-Lenz 方法进行手眼标定，求解相机到机械臂末端的变换矩阵。

    参数：
    A_list: list, 机械臂末端姿态变换矩阵列表（4x4矩阵）
    B_list: list, 相机检测到的ArucoTag变换矩阵列表（4x4矩阵）

    返回：
    X: ndarray, 相机到机械臂末端的变换矩阵（4x4矩阵）
    """
    assert len(A_list) == len(B_list), "A_list 和 B_list 的长度必须相同！"

    # 初始化旋转和平移部分
    M = np.zeros((3, 3))  # 用于旋转部分的矩阵
    C = []  # 用于平移部分的系数矩阵
    d = []  # 用于平移部分的右端项

    for A, B in zip(A_list, B_list):
        # 提取 A 和 B 的旋转矩阵和平移向量
        R_A, t_A = A[:3, :3], A[:3, 3]
        R_B, t_B = B[:3, :3], B[:3, 3]

        # 构建旋转部分的方程
        M += R_A.T @ R_B - R_B.T @ R_A

        # 构建平移部分的方程
        C.append(np.eye(3) - R_A)
        d.append(t_B - R_B @ t_A)

    # 求解旋转矩阵
    U, _, Vt = np.linalg.svd(M)
    R_X = U @ Vt
    if np.linalg.det(R_X) < 0:
        R_X = -R_X  # 保证旋转矩阵的正定性

    # 将平移方程转换为矩阵形式
    C = np.vstack(C)
    d = np.hstack(d)
    t_X, _, _, _ = np.linalg.lstsq(C, d, rcond=None)

    # 构造最终的变换矩阵
    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X

    return X

def main():
    '''
    主函数：采集数据并完成Tsai-Lenz手眼标定
    '''
    R_tag2camera_list = [] 
    T_tag2camera_list = []
    R_end2base_list = []
    T_end2base_list = []

    rospy.init_node('tag_trans_mat_listener', anonymous=True)
    rospy.Subscriber('/tag_trans_mat', Float32MultiArray, camera_callback2)

    sample_times = int(input('------请输入采集次数------: '))
    input('------按下回车开始采集数据------')

    for i in range(sample_times):
        # 获取机械臂末端位姿
        try:
            status, fr5_A_end_pose = robot.GetActualTCPPose(0)
            if status != 0:
                rospy.logerr(f"获取机械臂位姿失败，状态码: {status}")
                continue
            fr5_A_end = fr5_A_end_pose  # 使用实际位姿数据
        except Exception as e:
            rospy.logerr(f"获取机械臂位姿时发生错误: {e}")
            continue

        print(f'采集 {i+1}/{sample_times} - fr5_A_end:', fr5_A_end)

        
        # 添加到历史记录
        all_fr5_A_end.append(fr5_A_end)
        with open(output_file, "w") as f:
            json.dump(all_fr5_A_end, f, indent=4)

        # 计算end2base矩阵
        try:
            end2base = get_transform_mat(fr5_A_end[0], fr5_A_end[1], fr5_A_end[2],
                                         fr5_A_end[3], fr5_A_end[4], fr5_A_end[5])
        except IndexError as e:
            rospy.logerr(f"生成end2base矩阵时发生错误: {e}")
            continue
        print('end2base:', end2base)
        R_end2base, T_end2base = get_RT_from_transform_mat(end2base)
        
        # 处理tag2camera变换矩阵
        if not tag_trans_mat:
            rospy.logerr("未接收到tag_trans_mat，跳过此次采集。")
            continue
        
        tag_trans_mat_np = np.array(tag_trans_mat)
        if tag_trans_mat_np.shape != (4, 4):
            rospy.logerr(f"接收到的tag_trans_mat形状不正确: {tag_trans_mat_np.shape}, 期望 (4,4)")
            continue
        
        R_tag2camera, T_tag2camera = get_RT_from_transform_mat(tag_trans_mat_np)

        # 保存旋转和平移矩阵
        R_end2base_list.append(R_end2base)
        T_end2base_list.append(T_end2base)
        R_tag2camera_list.append(R_tag2camera)
        T_tag2camera_list.append(T_tag2camera)

        # print(f"采集第 {i + 1} 组数据:")
        # print("R_end2base:", R_end2base)
        # print("T_end2base:", T_end2base)
        # print("R_tag2camera:", R_tag2camera)
        # print("T_tag2camera:", T_tag2camera)
        print('数据采集完成。准备进行下一次采集。')
        input('--------调整末端姿态后按回车继续--------')

    # 检查是否有足够的数据进行标定
    if len(R_end2base_list) < 3:
        rospy.logerr("数据采集次数不足，至少需要3组数据进行标定。")
        return
    
    A_list = [get_transform_mat_from_RT(R, T) for R, T in zip(R_end2base_list, T_end2base_list)]
    B_list = [get_transform_mat_from_RT(R, T) for R, T in zip(R_tag2camera_list, T_tag2camera_list)]


    # 调用Tsai-Lenz方法求解
    X = solve_hand_eye_tsai_lenz(A_list, B_list)

    print('手眼标定结果（相机到机械臂末端的变换矩阵）：')
    print(X)

    # 保存最终的变换矩阵
    save_to_file(X)

if __name__ == "__main__":
    try:
        init()
        main()
    except rospy.ROSInterruptException as e:
        print('ROS 中断:', e)
