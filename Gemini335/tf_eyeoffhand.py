# #!/bin/python3
# #嘉航学长编写，适用于获取六自由度。夹取杯子的任务暂时不需要六自由度。
# # -*- coding: utf-8 -*-
# '''
# 眼在手外 末端需要与标定板保持相对静止
# 可以得到相机相对于底座的变换矩阵，并自动保存
# '''
# import rospy
# import os
# import json
# import time
# import cv2
# import numpy as np
# # from chemistry import ChemistryTest
# # from std_msgs.msg import Float32MultiArray

# # 使用说明：后面打包为roslaunch，使用时先开启arucotag6DOF，在启动tf即可

# # 最终目的，obj_to_base,需要obj_to_camera（视觉程序读）,camera_to_base（标定得到）
# # 其中，camera_to_base需要标定得到，通过end_to_base（机械臂读取）,tag_to_end（未知，但固定）,camera_to_tag（视觉程序读）

# # 将手眼标定方程（眼在手外）化为AX=XB形式：
# # end2base_1*base2end_2 * camera2base = camera2base * tag2camera_1*camera2tag_2
# # 标定利用opencv4中的calibrateHandEye()函数
# # 传入7个参数，前四个是输入，然后是两个输出，最后是标定方法（默认tsai）

# camera2base = [ 
#  [ 0.99885901,  0.02414404  ,0.04120379,99.50256158],
#  [ 0.02214817, -0.99859083 , 0.04822672,-542.01673407],
#  [ 0.04231012 ,-0.0472591 , -0.99798619,846.15889269],
#  [ 0.0,0.0,0.0,1.0]
# ]

# tag_trans_mat = []
# fr5_A = []
# fr5_B = []

# def init():
#     '''
#     初始化函数，包含双机械臂复位
#     '''
#     global fr5_A
#     global fr5_B
#     fr5_A = ChemistryTest(1)
#     fr5_B = ChemistryTest(2)
#     time.sleep(0.5)
#     fr5_A.dou_go_start(fr5_B)

# def save_to_file(matrix):
#     # 创建log文件夹（如果不存在）
#     log_dir = "/home/zjh/HN-1/demo_ws/src/frcobot_ros/fr5_moveit_config/log"
#     file_name = "1.txt"
#     if not os.path.exists(log_dir):
#         os.makedirs(log_dir)

#     # 确定文件路径和名称
#     file_path = os.path.join(log_dir, file_name)

#     # 如果文件已经存在，则找一个可用的文件名
#     index = 1
#     while os.path.exists(file_path):
#         index += 1
#         file_path = os.path.join(log_dir, f"{index}.txt")
#         print('chongfu')

#     # 将矩阵转换为Python列表
#     matrix_list = matrix.tolist()

#     # 保存矩阵列表到文件
#     with open(file_path, 'w') as file:
#         # 序列化矩阵列表为JSON字符串
#         matrix_str = json.dumps(matrix_list)
#         file.write(matrix_str)
#         print('file saved')

# def camera_callback2(rece_tag_trans_mat):
#     '''
#     回调函数，得到tag_to_camera的变换矩阵
#     由于ros功能限制，在此将二维数组压缩为一维数组接收，需要做对应解码处理
#     @输入：
#         rece_tag_trans_mat：ros发来的tag2camera信息
#     @输出：
#         None
#     '''
#     global tag_trans_mat
#     if rece_tag_trans_mat.data == []:
#         pass
#     else :
#         tag_trans_mat = rece_tag_trans_mat.data
#         tag_trans_mat = list(tag_trans_mat)
#         tag_trans_mat = [tag_trans_mat[i:i+4] for i in range(0, len(tag_trans_mat), 4)]
#         # print(tag_trans_mat,'\n')

# def get_transform_mat(X,Y,Z,RX,RY,RZ):
#     '''
#     从机械臂末端6D数据得到end2base变换矩阵
#     @输入：
#         XYZ,RXRYRZ：机械臂末端6D数据
#     @输出：
#         end_to_base：机械臂end2base数据
#     '''
#     # 旋转角度
#     rx = np.deg2rad(RX)
#     ry = np.deg2rad(RY)
#     rz = np.deg2rad(RZ)

#     # 绕x轴旋转矩阵
#     Rx = np.array([[1, 0, 0],
#                 [0, np.cos(rx), -np.sin(rx)],
#                 [0, np.sin(rx), np.cos(rx)]])

#     # 绕y轴旋转矩阵
#     Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
#                 [0, 1, 0],
#                 [-np.sin(ry), 0, np.cos(ry)]])

#     # 绕z轴旋转矩阵
#     Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
#                 [np.sin(rz), np.cos(rz), 0],
#                 [0, 0, 1]])

#     # 旋转矩阵的乘积
#     R = np.dot(np.dot(Rz, Ry),Rx)

#     # 平移向量
#     tx = X
#     ty = Y
#     tz = Z

#     # 变换矩阵
#     end_to_base = np.array([[R[0, 0], R[0, 1], R[0, 2], tx],
#                 [R[1, 0], R[1, 1], R[1, 2], ty],
#                 [R[2, 0], R[2, 1], R[2, 2], tz],
#                 [0, 0, 0, 1]])
#     return end_to_base
    
# def tf_get_obj_to_base(obj2camera,camera2base):
#     '''
#     得到obj2base变换矩阵
#     @输入：
#         obj2camera：物体在相机坐标系下的变换矩阵
#         camera2base：相机在机械臂底座下的变换矩阵
#     @输出：
#         obj2base：物体在机械臂底座下的变换矩阵
#     '''
#     obj2base = np.dot(camera2base,obj2camera)
#     return obj2base
    
# def get_RT_from_transform_mat(transform_mat):
#     '''
#     给定变换矩阵，给出旋转矩阵与平移向量
#     @输入：
#         transform_mat：待拆解的变换矩阵
#     @输出：
#         rot_mat：旋转矩阵
#         translate_mat:平移向量
#     '''
#     rot_mat = transform_mat[:3,:3]
#     translate_mat = transform_mat[:3,3]
#     return rot_mat,translate_mat

# def get_transform_mat_from_RT(R,T):
#     '''
#     给定旋转矩阵和平移向量，给出变换矩阵
#     @输入：
#         R：旋转矩阵
#         T：平移向量
#     @输出：
#         M：变换矩阵
#     '''
#     Z = [0.0,0.0,0.0,1.0]
#     M = np.vstack((np.hstack((R, T)), Z))
#     return M

# def main():
#     R_base2end_list = []
#     T_base2end_list = []
#     R_tag2camera_list = [] 
#     T_tag2camera_list = []
#     R_camera2base = []
#     T_camera2base = []
#     R_camera2base = np.array(R_camera2base)
#     T_camera2base = np.array(T_camera2base)
#     global tag_trans_mat
#     rospy.Subscriber('/tag_trans_mat',Float32MultiArray,camera_callback2)
#     sample_times = input('------请输入采集次数------')
#     input('------等待按下回车开始采集数据------')
#     for i in range(int(sample_times)):
#         fr5_B_end = fr5_B.robot.GetActualToolFlangePose(0)
#         fr5_B_end = fr5_B_end[-6:]# 得到机械臂末端xyz，rxryrz

#         # 得到end2base矩阵
#         end2base = get_transform_mat(fr5_B_end[0],fr5_B_end[1],fr5_B_end[2],fr5_B_end[3],fr5_B_end[4],fr5_B_end[5])
#         # print('end2base:',end2base)

#         # 得到并处理base2end矩阵
#         base2end = np.linalg.inv([end2base])
#         base2end = base2end[0]
#         # print('base2end:',base2end)

#         # 得到base2end的旋转矩阵与平移向量
#         R_base2end , T_base2end = get_RT_from_transform_mat(base2end)

#         # 得到tag2camera的旋转矩阵与平移向量
#         print('tag_trans_mat',tag_trans_mat)
#         tag_trans_mat = np.array(tag_trans_mat)
#         R_tag2camera , T_tag2camera = get_RT_from_transform_mat(tag_trans_mat)

#         # 把上述四个矩阵制成列表
#         R_base2end_list.append(R_base2end)
#         T_base2end_list.append(T_base2end)
#         R_tag2camera_list.append(R_tag2camera)
#         T_tag2camera_list.append(T_tag2camera)

#         input('--------等待调整末端姿态并重新记录--------')

#     # 创建一个字典，用于存储矩阵和对应的文字说明
#     matrix_dict = {
#         "R_base2end_list": R_base2end_list,
#         "T_base2end_list": T_base2end_list,
#         "R_tag2camera_list": R_tag2camera_list,
#         "T_tag2camera_list": T_tag2camera_list
#     }

#     R_camera2base,T_camera2base = cv2.calibrateHandEye(R_base2end_list,T_base2end_list,R_tag2camera_list,T_tag2camera_list,method=cv2.CALIB_HAND_EYE_TSAI)

#     print('R_camera2base',R_camera2base)
#     print('T_camera2base',T_camera2base)
#     # 保存变换矩阵
#     save_to_file(get_transform_mat_from_RT(R_camera2base,T_camera2base))
#     # # 打印矩阵和文字说明
#     # for key, matrix in matrix_dict.items():
#     #     print(key + ":")
#     #     if isinstance(matrix, np.ndarray):
#     #         matrix = matrix.tolist()
#     #     print(matrix)
#     #     print("\n")

# def test():
#     global tag_trans_mat
#     rospy.init_node('fr5_main', anonymous=True)
#     rospy.Subscriber('/tag_trans_mat',Float32MultiArray,camera_callback2)
#     while True:
#         input('等待按下回车进行一次计算：')
#         # print('tag2camera', tag_trans_mat)
#         # print('cameara2base', camera2base)
#         obj2base = tf_get_obj_to_base(tag_trans_mat,camera2base)
#         print('结果为：',obj2base)

# def newtest():
#     R = [[ 0.99885901 , 0.02414404 , 0.04120379],
#         [ 0.02214817 ,-0.99859083 , 0.04822672],
#         [ 0.04231012 ,-0.0472591  ,-0.99798619]]
#     T = [[99.50256158],
#          [ -542.01673407],
#          [ 846.15889269]]
#     print(get_transform_mat_from_RT(R,T))
#     save_to_file(get_transform_mat_from_RT(R,T))

# if __name__ == "__main__":
#     try:
#         init()
#         main()
#         # newtest()
#     except rospy.ROSInterruptException as e:
#         print(e,"\n")