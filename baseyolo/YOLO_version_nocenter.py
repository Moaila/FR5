"""
@author: 李文皓
@function:整个做咖啡流程中的所有视觉信号获取不打中心点
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
import open3d as o3d

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

        self.current_mode = 'eye_to_hand'
        
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
                "/home/newplace/FR5/coffeeyolo/runs/detect/train2/weights/best.pt",
                target_class=15,
                image_size=1088,
                confidence=0.5,
                iou=0.6
            ),
            'red_button': self.init_model(
                "/home/newplace/FR5/redbuttonyolo/runs/detect/train2/weights/best.pt",
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
            'eye_in_hand': np.array([[-0.9769009491021905, 0.1488913742200175, 0.15328370535094146, -7.005541914341826], 
                                 [-0.19755886673025527, -0.9026958602452877, -0.38224426493052494, 65.07411248380654], 
                                 [0.08145569237012701, -0.4036973403168453, 0.9112592537810569, 55.61473264653973], 
                                 [0.0, 0.0, 0.0, 1.0]])
        }

        # ROS发布器
        self.pub_cup = rospy.Publisher('/vision/cup_pose', String, queue_size=10)
        self.pub_button = rospy.Publisher('/vision/button_pose', String, queue_size=10)
        self.pub_red_button = rospy.Publisher('/vision/red_button_detected', Bool, queue_size=10)
        self.pub_trans_mat = rospy.Publisher('/vision/trans_matrix', Float32MultiArray, queue_size=10)

        # ROS订阅器
        self.stage_complete = 0  # 阶段完成标志
        rospy.Subscriber('/arm/stage_status', Int32, self.stage_callback)

        # 可视化窗口
        cv2.namedWindow('Main Display', cv2.WINDOW_NORMAL)

    def init_cameras(self):
        """初始化相机设备"""
        ctx = Context()
        device_list = ctx.query_devices()
        if device_list.get_count() < 2:
            raise RuntimeError("需要两个相机设备")
        
        # 获取设备序列号
        serial_num1 = device_list.get_device_serial_number_by_index(0)
        serial_num2 = device_list.get_device_serial_number_by_index(1)
        
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

    def mode_callback(self, msg):
        """视觉模式切换回调"""
        new_mode = msg.data
        if new_mode in ['eye_to_hand', 'eye_in_hand']:
            self.current_mode = new_mode
            rospy.loginfo(f"视觉模式切换至: {new_mode}")

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
        """眼在手外处理流程"""
        # 杯子检测
        canvas, _, xyxy_list, class_list = self.models['cup'].detect(img)
        for i, xyxy in enumerate(xyxy_list):
            if class_list[i] != 15:
                continue

            # 坐标转换逻辑
            cx, cy = self.get_center(xyxy)
            point3d = self.camera.depth_pixel2cam_point3d(cx, cy, depth)
            
            # 坐标系转换
            arm_coord = self.coordinate_transform(point3d, self.calibration['eye_to_hand'])
            
            # 发布坐标
            self.pub_cup.publish(f"{arm_coord[0]:.2f},{arm_coord[1]:.2f},{arm_coord[2]:.2f}")
            
            # 可视化标注
            self.draw_annotation(canvas, cx, cy, point3d, arm_coord)
            
        return canvas

    def process_eye_in_hand(self, img, depth):
        """眼在手上处理流程"""
        # 普通按钮检测
        canvas, _, xyxy_list, class_list = self.models['button'].detect(img)
        red_detected = False

        # 红色按钮检测
        red_canvas, red_xyxy, _, _ = self.models['red_button'].detect(img)
        if len(red_xyxy) > 0:
            red_detected = True
            self.pub_red_button.publish(True)

        for i, xyxy in enumerate(xyxy_list):
            if class_list[i] != 15:
                continue

            # 坐标转换逻辑
            cx, cy = self.get_center(xyxy)
            point3d = self.camera.depth_pixel2cam_point3d(cx, cy, depth)
            
            # 动态坐标转换
            robot_pose = self.get_robot_pose()
            if robot_pose:
                # 构建当前位姿变换矩阵
                T = self.build_transformation_matrix(robot_pose)
                base_coord = self.coordinate_transform(point3d, T)
                
                # 发布坐标
                self.pub_button.publish(f"{base_coord[0]:.2f},{base_coord[1]:.2f},{base_coord[2]:.2f}")
                
                # 可视化标注
                self.draw_annotation(canvas, cx, cy, point3d, base_coord)

        return cv2.addWeighted(canvas, 0.7, red_canvas, 0.3, 0)

    def build_transformation_matrix(self, pose):
        """根据机器人位姿构建变换矩阵"""
        rotation_matrix, _ = cv2.Rodrigues(np.array(pose[3:]))
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = pose[:3]
        return np.dot(T, self.calibration['eye_in_hand'])

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
                
                # 从当前相机读取图像
                color_img = self.current_camera.read_color_img()
                depth_img = self.current_camera.read_depth_img()
                
                # 根据当前模式处理图像
                if self.current_mode == 'eye_to_hand':
                    processed = self.process_eye_to_hand(color_img, depth_img)
                else:
                    processed = self.process_eye_in_hand(color_img, depth_img)
                
                # 显示性能信息
                fps = 1/(time.time()-start_time)
                cv2.putText(processed, f"FPS: {fps:.1f} | Mode: {self.current_mode}",
                           (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                
                # 显示结果
                cv2.imshow('Main Display', processed)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.camera1.release()
            self.camera2.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    vision_system = CoffeeVisionSystem()
    vision_system.run()