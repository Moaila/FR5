"""
@author: 李文皓
@场景：眼在手上
@功能：用YOLOv8模型检测目标（如杯子），从相机坐标系转换到机械臂基座坐标系，并发布ROS消息。
C: 表示相机坐标系下的三维坐标
B: 表示机械臂基座坐标系下的三维坐标
"""
import os
import time
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray, String
from ultralytics import YOLO  # 使用官方ultralytics库
from kyle_robot_toolbox.camera import Gemini335
from fairino import Robot  # 机械臂接口

# 初始化相机
camera = Gemini335()

# 初始化机械臂
robot = Robot.RPC('192.168.59.6')

# 加载YOLOv8模型
print("[INFO] 开始加载YOLOv8模型")
model = YOLO('yolov8n.pt')  # 使用预训练模型
print("[INFO] 完成YOLOv8模型加载")

# 相机到机械臂末端的转换矩阵（由眼在手上标定得到）
camera_to_end_matrix = np.array([[-0.9769009491021905, 0.1488913742200175, 0.15328370535094146, -7.005541914341826], 
                                 [-0.19755886673025527, -0.9026958602452877, -0.38224426493052494, 65.07411248380654], 
                                 [0.08145569237012701, -0.4036973403168453, 0.9112592537810569, 55.61473264653973], 
                                 [0.0, 0.0, 0.0, 1.0]])

# 初始化ROS节点和话题发布
rospy.init_node('camera_publisher_335', anonymous=True)
pub_pose = rospy.Publisher('tag_pose', String, queue_size=10)

# 定义函数：计算物体在基座坐标系下的变换矩阵
def tf_get_obj_to_base2(end2base, camera2end, obj2camera):
    """
    计算目标（物体）在基座坐标系下的齐次变换矩阵
    @输入：
        end2base: 末端到基座的变换矩阵 (4x4)
        camera2end: 相机到末端的变换矩阵 (4x4)
        obj2camera: 物体在相机坐标系下的齐次坐标
    @输出：
        obj2base: 物体在基座坐标系下的齐次坐标
    """
    obj2base = np.dot(np.dot(end2base, camera2end), obj2camera)
    return obj2base

# 创建显示窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while not rospy.is_shutdown():
    try:
        # 计时启动
        t_start = time.time()

        # 读取图像
        img_bgr = camera.read_color_img()
        depth_img = camera.read_depth_img()
        depth_canvas_tmp = camera.depth_img2canvas(depth_img)

        # 确保深度图尺寸与彩色图对齐
        dp_h, dp_w, _ = depth_canvas_tmp.shape
        depth_canvas = np.zeros_like(img_bgr)
        depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp

        # YOLOv8目标检测
        results = model.predict(source=img_bgr, imgsz=640, conf=0.5, iou=0.6, device=0, classes=[41], stream=False)  # '41' 是COCO中的杯子类别

        center_list = []
        for result in results:
            boxes = result.boxes  # 检测框
            for box in boxes:
                # 获取类别ID和置信度
                cls_id = int(box.cls[0])
                conf = box.conf[0].item()

                # 检测杯子类别
                if cls_id != 41:
                    continue

                # 获取边界框并计算中心点
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # 绘制检测框和中心点
                cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(img_bgr, (cx, cy), 5, (255, 255, 0), -1)

                # 检查深度图范围
                if cx >= dp_w or cy >= dp_h:
                    continue

                # 获取深度值并计算相机坐标
                depth_value = depth_img[cy, cx]
                if depth_value == 0:  # 深度值无效，跳过
                    continue

                # 相机坐标系下的三维坐标
                # cam_point3d = camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth_img)
                # cam_x, cam_y, cam_z = cam_point3d
                # 相机坐标系下的三维坐标
                cam_point3d = camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth_img)
                cam_x, cam_y, cam_z = cam_point3d
                obj2camera = np.array([cam_x, cam_y, cam_z, 1]).reshape(4, 1)

                # 实时获取机械臂末端相对于基座的位姿
                status, end_pose = robot.GetActualTCPPose(0)
                if status != 0:
                    rospy.logerr("无法获取机械臂末端位姿！跳过此点。")
                    continue

                # 检查末端位姿数据是否完整
                if len(end_pose) != 6:
                    rospy.logerr(f"获取的机械臂末端位姿数据格式不正确，数据长度为 {len(end_pose)}，预期为 6。")
                    continue

                try:
                    # 提取平移向量和旋转向量
                    translation = np.array(end_pose[:3])  # 平移部分 (x, y, z)
                    rotation_vector = np.array(end_pose[3:])  # 旋转部分 (rx, ry, rz)

                    # 将旋转向量转换为旋转矩阵
                    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

                    # 计算末端到基座的齐次变换矩阵
                    T_end2base = np.eye(4)
                    T_end2base[:3, :3] = rotation_matrix  # 旋转矩阵
                    T_end2base[:3, 3] = translation  # 平移向量

                    # 计算物体在基座坐标系下的位置
                    obj2base = tf_get_obj_to_base2(T_end2base, camera_to_end_matrix, obj2camera)
                    base_x, base_y, base_z = obj2base[:3, 0]

                    # 显示相机和基座坐标
                    tag_camera = f"C:{cam_x:.0f},{cam_y:.0f},{cam_z:.0f}"
                    tag_base = f"B:{base_x:.0f},{base_y:.0f},{base_z:.0f}"
                    cv2.putText(img_bgr, tag_camera, (cx - 80, cy - 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 1, cv2.LINE_AA)
                    cv2.putText(img_bgr, tag_base, (cx - 80, cy - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1, cv2.LINE_AA)

                    # 发布机械臂基座坐标
                    pose_msg = f"{base_x:.2f},{base_y:.2f},{base_z:.2f}"
                    pub_pose.publish(pose_msg)

                    # 打印基座坐标
                    print(f"Base Coordinates: x={base_x:.2f}, y={base_y:.2f}, z={base_z:.2f}")

                    # 添加到中心点列表
                    center_list.append([cx, cy])

                except Exception as e:
                    rospy.logerr(f"坐标转换或矩阵计算失败: {e}")
                    continue

        # 计算FPS
        t_end = time.time()
        fps = int(1 / (t_end - t_start))
        cv2.putText(img_bgr, f"FPS:{fps}", (20, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 显示画面
        cv2.imshow("canvas", img_bgr)
        cv2.imshow("depth_canvas", depth_canvas)

        # 按 'q' 键退出
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    except Exception as e:
        rospy.logerr(f"主循环发生错误: {e}")
        continue

cv2.destroyAllWindows()
camera.release()

