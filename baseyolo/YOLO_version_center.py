"""
@author: 李文皓
@function: 整个做咖啡流程中的所有视觉信号获取,请注意，这时的数据未加上偏置，需要在机械臂端专门加入偏置来进行调整
"""
import os
import time
import cv2
import numpy as np
import rospy
from std_msgs.msg import String, Bool, Float32MultiArray, Int32
from kyle_robot_toolbox.camera import Gemini335
from kyle_robot_toolbox.yolov8 import YoloV8Detect
from fairino import Robot
import sys

# 添加动态链接库路径
pyorbbecsdk_path = "/home/newplace/FR5/pyorbbecsdk"
if pyorbbecsdk_path not in sys.path:
    sys.path.append(pyorbbecsdk_path)

# 设置LD_LIBRARY_PATH
os.environ["LD_LIBRARY_PATH"] = pyorbbecsdk_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")

# 导入pyorbbecsdk
try:
    from pyorbbecsdk import *
    print("pyorbbecsdk 导入成功！")
except ImportError as e:
    print(f"导入失败: {e}")
    sys.exit(1)

class CoffeeVisionSystem:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('integrated_vision_system')
        
        # 相机初始化
        self.init_cameras()
        self.current_camera = self.camera1  # 默认使用第一个相机
        self.current_mode = 'eye_to_hand'  # 初始模式为眼在手外
        
        # 机械臂初始化
        self.robot = Robot.RPC('192.168.59.6')
        
        # 模型配置
        self.models = {
            'cup': self.init_model(
                "/home/newplace/FR5/baseyolo/runs/detect/train/weights/best.pt",
                target_class=15,
                image_size=1088,
                confidence=0.5,
                iou=0.6
            ),
            'button': self.init_model(
                "/home/newplace/FR5/coffeeyolo/runs/detect/train/weights/best.pt",
                target_class=15,
                image_size=1088,
                confidence=0.5,
                iou=0.6
            ),
            'red_button': self.init_model(
                "/home/newplace/FR5/redbuttonyolo/runs/detect/train/weights/best.pt",
                target_class=15,
                image_size=1088,
                confidence=0.5,
                iou=0.6
            )
        }

        # 坐标转换矩阵
        self.calibration = {
            'eye_to_hand': np.array([
                [-0.9983436760441877, 0.003310123068703671, -0.05743646566292077, -75.16659019011934],
                [0.00833952766183191, 0.9961254408213325, -0.08754746385180773, -771.9754021846082],
                [0.05692413179799392, -0.08788144988435317, -0.9945031392536017, 857.6738248978729],
                [0.0, 0.0, 0.0, 1.0]
            ]),
            'eye_in_hand': np.array([
                [-0.9769009491021905, 0.1488913742200175, 0.15328370535094146, -7.005541914341826],
                [-0.19755886673025527, -0.9026958602452877, -0.38224426493052494, 65.07411248380654],
                [0.08145569237012701, -0.4036973403168453, 0.9112592537810569, 55.61473264653973],
                [0.0, 0.0, 0.0, 1.0]
            ])
        }

        # ROS发布器
        self.pub_cup_pose = rospy.Publisher('/vision/cup_pose', String, queue_size=10)
        self.pub_button_pose = rospy.Publisher('/vision/button_pose', String, queue_size=10)
        self.pub_red_button_status = rospy.Publisher('/vision/red_button_status', Bool, queue_size=10)

        # ROS订阅器
        self.stage_complete = 0  # 阶段完成标志
        rospy.Subscriber('/arm/stage_status', Int32, self.stage_callback)

        # 可视化窗口
        cv2.namedWindow('Main Display', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth Display', cv2.WINDOW_NORMAL)

    def init_cameras(self):
        """初始化相机设备"""
        ctx = Context()
        device_list = ctx.query_devices()
        if device_list.get_count() < 2:
            raise RuntimeError("需要两个相机设备")
        
        # 获取设备序列号
        serial_num1 = device_list.get_device_serial_number_by_index(0)
        serial_num2 = device_list.get_device_serial_number_by_index(1)
        """
# @author: 李文皓
# @function: 测试YOLO_version.py代码的正确性
# """
# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import Int32, String

# def cup_pose_callback(msg):
#     """杯子坐标回调函数"""
#     print(f"接收到杯子坐标: {msg.data}")

# def button_pose_callback(msg):
#     """按钮坐标回调函数"""
#     print(f"接收到按钮坐标: {msg.data}")

# def stage_controller():
#     # 初始化ROS节点
#     rospy.init_node('stage_controller', anonymous=True)
    
#     # 创建发布器
#     pub = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)
    
#     # 创建订阅器
#     rospy.Subscriber('/vision/cup_pose', String, cup_pose_callback)
#     rospy.Subscriber('/vision/button_pose', String, button_pose_callback)
    
#     # 初始阶段状态
#     stage_status = 0
#     pub.publish(stage_status)
    
#     print("阶段控制器已启动")
#     print("当前阶段: 0 (眼在手外)")
#     print("按 '1' 切换到阶段1 (眼在手上)")
#     print("按 'q' 退出程序")

#     try:
#         while not rospy.is_shutdown():
#             # 获取用户输入
#             key = input()
            
#             if key == '1':
#                 if stage_status == 0:
#                     stage_status = 1
#                     pub.publish(stage_status)
#                     print("\n切换到阶段1 (眼在手上)")
#                     print("正在发送信号: 1")
#                 else:
#                     print("\n已经是阶段1，无需切换")
                    
#             elif key == 'q':
#                 print("\n退出程序")
#                 break
                
#             else:
#                 print(f"\n无效输入: {key}")
#                 print("请输入 '1' 或 'q'")

#     except KeyboardInterrupt:
#         print("\n程序被中断")

# if __name__ == '__main__':
#     try:
#         stage_controller()
#     except rospy.ROSInterruptException:
#         pass
        # 创建相机实例
        self.camera1 = Gemini335(serial_num=serial_num1)
        self.camera2 = Gemini335(serial_num=serial_num2)
        
        print(f"相机1（序列号：{serial_num1}）初始化完成")
        print(f"相机2（序列号：{serial_num2}）初始化完成")

    def stage_callback(self, msg):
        """阶段完成回调函数"""
        if msg.data == 1 and self.stage_complete == 0:
            self.switch_camera()
            self.stage_complete = 1
            rospy.loginfo("第一阶段完成，已切换相机")

    def switch_camera(self):
        """切换相机"""
        if self.current_camera == self.camera1:
            self.current_camera = self.camera2
            self.current_mode = 'eye_in_hand'  # 切换到眼在手上模式
            rospy.loginfo("切换到相机2（眼在手上）")
        else:
            self.current_camera = self.camera1
            self.current_mode = 'eye_to_hand'  # 切换到眼在手外模式
            rospy.loginfo("切换到相机1（眼在手外）")

    def init_model(self, path, **kwargs):
        """初始化YOLOv8模型"""
        print(f"[INFO] 加载模型: {os.path.basename(path)}")
        model = YoloV8Detect(path)
        for key, value in kwargs.items():
            setattr(model, key.upper(), value)
        return model

    def coordinate_transform(self, point3d, matrix):
        """通用坐标变换"""
        homogeneous = np.append(point3d, 1)
        transformed = np.dot(matrix, homogeneous)
        return transformed[:3]

    def get_robot_pose(self):
        """获取机械臂当前位姿"""
        status, pose = self.robot.GetActualTCPPose(0)
        if status == 0 and len(pose) == 6:
            return pose
        return None

    def process_eye_to_hand(self, img, depth):
        """眼在手外处理流程（杯子检测）"""
        # 获取检测结果
        detection_result = self.models['cup'].detect(img)
        if not isinstance(detection_result, tuple) or len(detection_result) != 4:
            return img
            
        canvas, class_ids, xyxy_list, conf_list = detection_result
        
        # 处理每个检测到的杯子
        for i, xyxy in enumerate(xyxy_list):
            if class_ids[i] != 15:  # 过滤非杯子类别
                continue
                
            # 计算中心点坐标
            cx, cy = self.get_center(xyxy)
            
            # 从深度图获取3D坐标（使用当前相机）
            point3d = self.current_camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth)
            if point3d is None:
                continue
                
            # 坐标系转换
            arm_coord = self.coordinate_transform(
                point3d, 
                self.calibration['eye_to_hand']
            )
            
            # 发布杯子坐标
            self.pub_cup_pose.publish(f"{arm_coord[0]:.2f},{arm_coord[1]:.2f},{arm_coord[2]:.2f}")
            
            # 绘制标注
            self.draw_annotation(canvas, cx, cy, point3d, arm_coord)
            
            # 在深度图上绘制检测点
            depth_canvas = self.current_camera.depth_img2canvas(depth)
            cv2.circle(depth_canvas, (cx, cy), 5, (0, 255, 0), -1)
            cv2.imshow('Depth Display', depth_canvas)
            
        return canvas

    def process_eye_in_hand(self, img, depth):
        """眼在手上处理流程（按钮检测）"""
        # 普通按钮检测
        btn_result = self.models['button'].detect(img)
        if not isinstance(btn_result, tuple):
            return img
            
        canvas, class_ids, xyxy_list, conf_list = btn_result
        
        # 红色按钮检测
        red_result = self.models['red_button'].detect(img)
        if isinstance(red_result, tuple):
            _, red_class_ids, _, _ = red_result
            self.red_button_detected = any(id == 15 for id in red_class_ids)
            self.pub_red_button_status.publish(self.red_button_detected)

        # 处理每个检测到的按钮
        for i, xyxy in enumerate(xyxy_list):
            if class_ids[i] != 15:  # 过滤非按钮类别
                continue
                
            # 计算中心点坐标
            cx, cy = self.get_center(xyxy)
            
            # 从深度图获取3D坐标
            point3d = self.current_camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth)
            if point3d is None:
                continue
                
            # 动态坐标转换
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                continue
                
            # 构建变换矩阵
            T = self.build_transformation_matrix(robot_pose)
            base_coord = self.coordinate_transform(point3d, T)
            
            # 发布按钮坐标
            self.pub_button_pose.publish(f"{base_coord[0]:.2f},{base_coord[1]:.2f},{base_coord[2]:.2f}")
            
            # 绘制标注
            self.draw_annotation(canvas, cx, cy, point3d, base_coord)
            
            # 在深度图上绘制检测点
            depth_canvas = self.current_camera.depth_img2canvas(depth)
            cv2.circle(depth_canvas, (cx, cy), 5, (0, 0, 255), -1)
            cv2.imshow('Depth Display', depth_canvas)
            
        return canvas

    def build_transformation_matrix(self, pose):
        """根据机械臂位姿构建完整变换矩阵"""
        # 从位姿数据中提取旋转和平移
        translation = np.array(pose[:3])
        rotation_vector = np.array(pose[3:])
        
        # 转换旋转向量为旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        
        # 构建末端到基座的变换矩阵
        T_end2base = np.eye(4)
        T_end2base[:3, :3] = rotation_matrix
        T_end2base[:3, 3] = translation
        
        # 组合相机到末端的标定矩阵
        return np.dot(T_end2base, self.calibration['eye_in_hand'])

    def get_center(self, xyxy):
        """计算检测框中心"""
        return int((xyxy[0]+xyxy[2])/2), int((xyxy[1]+xyxy[3])/2)

    def draw_annotation(self, canvas, cx, cy, cam_coord, target_coord):
        """可视化标注"""
        cv2.putText(canvas, 
                   f"Cam: {cam_coord[0]:.0f},{cam_coord[1]:.0f},{cam_coord[2]:.0f}",
                   (cx-120, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 1)
        cv2.putText(canvas,
                   f"Target: {target_coord[0]:.0f},{target_coord[1]:.0f},{target_coord[2]:.0f}",
                   (cx-120, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 1)

    def run(self):
        try:
            while not rospy.is_shutdown():
                start_time = time.time()
                
                # 获取图像数据
                color_img = self.current_camera.read_color_img()
                depth_img = self.current_camera.read_depth_img()
                
                # 空图像检查
                if color_img is None or depth_img is None:
                    rospy.logwarn("获取图像失败，跳过本次处理")
                    continue
                
                # 根据模式处理图像
                if self.current_mode == 'eye_to_hand':
                    processed = self.process_eye_to_hand(color_img, depth_img)
                else:
                    processed = self.process_eye_in_hand(color_img, depth_img)
                
                # 显示处理结果
                if processed is not None:
                    # 添加状态信息
                    fps = 1/(time.time()-start_time)
                    status_text = f"{self.current_mode} | FPS: {fps:.1f}"
                    if self.current_mode == 'eye_in_hand':
                        status_text += f" | Red: {self.red_button_detected}"
                    cv2.putText(processed, status_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    cv2.imshow('Main Display', processed)
                
                # 处理键盘输入
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

        finally:
            # 释放资源
            self.camera1.release()
            self.camera2.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    vision_system = CoffeeVisionSystem()
    vision_system.run()