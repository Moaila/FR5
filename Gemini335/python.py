import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

# 使用标定函数得到的 camera_to_end 变换矩阵
camera_to_end = np.array([
    [-0.98783223, -0.15434156, -0.01913546, 31.09967041],
    [0.15286409, -0.98621996, 0.06326743, 89.96770092],
    [-0.02863657, 0.05957249, 0.99781314, 101.35795142],
    [0, 0, 0, 1]
])

# 示例的 end_to_base 变换矩阵计算
def get_end_to_base_transform(X, Y, Z, RX, RY, RZ):
    """
    根据机械臂末端的 6D 数据计算 end_to_base 的变换矩阵。
    """
    # 旋转角度转为弧度
    rx, ry, rz = np.deg2rad([RX, RY, RZ])

    # 绕 X, Y, Z 轴的旋转矩阵
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    # 总旋转矩阵
    R = Rz @ Ry @ Rx

    # 平移向量
    T = np.array([[X], [Y], [Z]])

    # 构造 4x4 变换矩阵
    end_to_base = np.eye(4)
    end_to_base[:3, :3] = R
    end_to_base[:3, 3] = T.flatten()

    return end_to_base

# 计算物体在基座坐标系下的变换矩阵
def calculate_obj_to_base(end_to_base, camera_to_end, obj_to_camera):
    """
    根据 end_to_base、camera_to_end 和 obj_to_camera 计算 obj_to_base。
    """
    obj_to_base = end_to_base @ camera_to_end @ obj_to_camera
    return obj_to_base

# 测试代码
if __name__ == "__main__":
    # 示例的 obj_to_camera 变换矩阵，表示物体在相机坐标系下的位置和姿态
    obj_to_camera = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # 示例的机械臂末端位置和姿态数据
    X, Y, Z = -80.047, -443.528, 265.667  # 位置（单位：mm）
    RX, RY, RZ = 133.165, 4.727, -3.144  # 姿态（单位：度）

    # 计算 end_to_base 变换矩阵
    end_to_base = get_end_to_base_transform(X, Y, Z, RX, RY, RZ)
    print("End to Base Transform Matrix:\n", end_to_base)

    # 计算 obj_to_base 变换矩阵
    obj_to_base = calculate_obj_to_base(end_to_base, camera_to_end, obj_to_camera)
    print("Object to Base Transform Matrix:\n", obj_to_base)
