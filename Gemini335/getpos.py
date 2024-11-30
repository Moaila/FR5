import numpy as np
from scipy.spatial.transform import Rotation as R
import re

# 读取数据
def read_data(file_path):
    camera_coords = []
    arm_poses = []
    base_coords = []
    pattern = re.compile(r'\{\((.*?)\),\((.*?)\),\((.*?)\)\}')
    
    with open(file_path, 'r') as file:
        for line_num, line in enumerate(file, 1):
            match = pattern.search(line.strip())
            if not match:
                print(f"Error: Line {line_num} does not match expected format: {line}")
                continue
            
            try:
                # 解析摄像机坐标
                cam_coords = tuple(map(float, match.group(1).split(',')))
                # 解析机械臂位姿（XYZ + RX, RY, RZ）
                arm_pose = tuple(map(float, match.group(2).split(',')))
                # 解析基座坐标
                base_coord = tuple(map(float, match.group(3).split(',')))
            except ValueError as e:
                print(f"Error: Unable to parse line {line_num}: {line}")
                print("Exception:", e)
                continue
            
            camera_coords.append(cam_coords)
            arm_poses.append(arm_pose)
            base_coords.append(base_coord)
    
    return np.array(camera_coords), np.array(arm_poses), np.array(base_coords)

# 计算位置转换矩阵
def compute_position_matrix(source_points, target_points):
    source_points = np.hstack((source_points, np.ones((source_points.shape[0], 1))))
    target_points = np.hstack((target_points, np.ones((target_points.shape[0], 1))))
    position_matrix, _, _, _ = np.linalg.lstsq(source_points, target_points, rcond=None)
    return position_matrix[:3, :]

# 将旋转角度(RX, RY, RZ)转换为旋转矩阵
def rotation_from_angles(rx, ry, rz):
    rotation = R.from_euler('xyz', [rx, ry, rz], degrees=True)
    return rotation.as_matrix()

# 应用位置和旋转矩阵
def transform_point(point, transformation_matrix):
    point_homogeneous = np.array([point[0], point[1], point[2], 1])
    transformed_point = transformation_matrix @ point_homogeneous
    return transformed_point[:3]

# 输出转换矩阵到文件
def save_matrices_to_file(camera_to_arm_matrix, arm_to_base_matrix, file_path):
    with open(file_path, 'w') as file:
        file.write("摄像机到机械臂的转换矩阵:\n")
        file.write(np.array2string(camera_to_arm_matrix, separator=', ') + "\n\n")
        file.write("机械臂到基座的转换矩阵:\n")
        file.write(np.array2string(arm_to_base_matrix, separator=', ') + "\n")

# 主程序
file_path = '/home/tom/FR5/new_conrol/log/reflect_danwei_mm.txt'
output_file = '/home/tom/FR5/new_conrol/log/newreflect.txt'
camera_coords, arm_poses, base_coords = read_data(file_path)

# 计算摄像机到机械臂的位置转换矩阵
camera_to_arm_position_matrix = compute_position_matrix(camera_coords, arm_poses[:, :3])

# 计算机械臂到基座的位置转换矩阵
arm_to_base_position_matrix = compute_position_matrix(arm_poses[:, :3], base_coords)

# 保存矩阵到文件
save_matrices_to_file(camera_to_arm_position_matrix, arm_to_base_position_matrix, output_file)

# 定义转换函数
def camera_to_arm_position(camera_point):
    return transform_point(camera_point, camera_to_arm_position_matrix)

def arm_to_base_position(arm_pose):
    arm_xyz = arm_pose[:3]
    rx, ry, rz = arm_pose[3], arm_pose[4], arm_pose[5]
    # 计算旋转矩阵
    rotation_matrix = rotation_from_angles(rx, ry, rz)
    
    # 创建机械臂到基座的完整位姿转换矩阵（4x4）
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix  # 设置旋转部分
    transformation_matrix[:3, 3] = arm_xyz  # 设置平移部分
    
    # 将机械臂末端坐标转换到基座坐标
    base_position = transformation_matrix @ np.array([*arm_xyz, 1])
    return base_position[:3]

# 测试部分
test_camera_point = [16.1, 35.5, 292.0]  # 示例摄像机坐标

# 计算机械臂位姿（位置部分）
arm_position = camera_to_arm_position(test_camera_point)
# 假设测试的机械臂姿态角为[133.165, 4.727, -3.144]（从第一行数据中取出）
test_arm_pose = np.hstack((arm_position, [133.165, 4.727, -3.144]))

# 计算基座坐标
base_position = arm_to_base_position(test_arm_pose)

print("机械臂位姿（位置）：", arm_position)
print("基座坐标：", base_position)
