"""
@author: 李文皓
@场景：眼在手上
@功能：用自定义YoloV8模型进行咖啡机按钮的检测，并发布目标位置到机械臂基座坐标系下的ROS话题,同时检测出来有没有操作成功出现的红色按钮
"""
import os
import time
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from kyle_robot_toolbox.camera import Gemini335
from kyle_robot_toolbox.yolov8 import YoloV8Detect
from fairino import Robot

# 初始化摄像头
camera = Gemini335()

# 初始化机械臂
robot = Robot.RPC('192.168.59.6')

# 加载模型（按钮检测）
print("[INFO] 开始加载 YOLOv8 模型")
model_path = "/home/newplace/FR5/coffeeyolo/runs/detect/train2/weights/best.pt"
model = YoloV8Detect(model_path)

# 配置模型参数
model.IMAGE_SIZE = 1088  # 图像尺寸（必须是32的倍数）
model.CONFIDENCE = 0.5   # 置信度阈值
model.IOU = 0.6          # IOU阈值
print("[INFO] 完成 YOLOv8 模型加载")

# 加载红色按钮检测模型
print("[INFO] 开始加载红色按钮 YOLOv8 模型")
red_button_model_path = "/home/newplace/FR5/redbuttonyolo/runs/detect/train2/weights/best.pt"
red_button_model = YoloV8Detect(red_button_model_path)

# 配置红色按钮检测模型参数
red_button_model.IMAGE_SIZE = 1088
red_button_model.CONFIDENCE = 0.5
red_button_model.IOU = 0.6
print("[INFO] 完成红色按钮 YOLOv8 模型加载")

# ROS节点初始化
rospy.init_node('yolov8_coffee_detector', anonymous=True)
pose_pub = rospy.Publisher('tag_pose', String, queue_size=10)
status_pub = rospy.Publisher('button_status', String, queue_size=10)

# 相机到机械臂末端的转换矩阵
camera_to_end_matrix = np.array([
    [-0.9769009491021905, 0.1488913742200175, 0.15328370535094146, -7.005541914341826],
    [-0.19755886673025527, -0.9026958602452877, -0.38224426493052494, 65.07411248380654],
    [0.08145569237012701, -0.4036973403168453, 0.9112592537810569, 55.61473264653973],
    [0.0, 0.0, 0.0, 1.0]
])

# 定义函数：计算物体在基座坐标系下的齐次变换矩阵
def tf_get_obj_to_base2(end2base, camera2end, obj2camera):
    obj2base = np.dot(np.dot(end2base, camera2end), obj2camera)
    return obj2base

# 定义函数：检测红色按钮并发布状态
def detect_red_button(image):
    """
    检测红色按钮的存在并发布状态
    :param image: 输入彩色图像
    :return: None
    """
    _, class_id_list, _, _ = red_button_model.detect(image, draw_label=False)
    red_button_detected = any(class_id == 0 for class_id in class_id_list)
    status = "1" if red_button_detected else "0"
    status_pub.publish(status)

# 创建显示窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while not rospy.is_shutdown():
    try:
        t_start = time.time()

        # 读取彩色图像和深度图像
        img_bgr = camera.read_color_img()
        depth_img = camera.read_depth_img()
        depth_canvas_tmp = camera.depth_img2canvas(depth_img)

        # 兼容尺寸差异
        dp_h, dp_w, _ = depth_canvas_tmp.shape
        depth_canvas = np.zeros_like(img_bgr)
        depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp

        # YOLOv8目标检测
        canvas, class_id_list, xyxy_list, conf_list = model.detect(img_bgr, draw_label=False)

        for i, xyxy in enumerate(xyxy_list):
            if class_id_list[i] != 15:
                continue

            x1, y1, x2, y2 = xyxy
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cv2.circle(canvas, [cx, cy], 5, (255, 255, 0), -1)
            cv2.circle(depth_canvas, [cx, cy], 5, (255, 0, 255), -1)

            if cx >= dp_w or cy >= dp_h:
                continue

            depth_value = depth_img[cy, cx]
            if depth_value == 0:
                continue

            cam_point3d = camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth_img)
            cam_x, cam_y, cam_z = cam_point3d
            obj2camera = np.array([cam_x, cam_y, cam_z, 1]).reshape(4, 1)

            status, end_pose = robot.GetActualTCPPose(0)
            if status != 0 or len(end_pose) != 6:
                continue

            translation = np.array(end_pose[:3])
            rotation_vector = np.array(end_pose[3:])
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            T_end2base = np.eye(4)
            T_end2base[:3, :3] = rotation_matrix
            T_end2base[:3, 3] = translation

            obj2base = tf_get_obj_to_base2(T_end2base, camera_to_end_matrix, obj2camera)
            base_x, base_y, base_z = obj2base[:3, 0]

            pose_msg = f"{base_x:.2f},{base_y:.2f},{base_z:.2f}"
            pose_pub.publish(pose_msg)

        # 检测红色按钮状态
        detect_red_button(img_bgr)

        # 计算FPS
        t_end = time.time()
        fps = int(1 / (t_end - t_start))
        cv2.putText(canvas, f"FPS: {fps}", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("canvas", canvas)
        cv2.imshow("depth_canvas", depth_canvas)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    except Exception as e:
        rospy.logerr(f"主循环发生错误: {e}")
        continue

cv2.destroyAllWindows()
camera.release()
