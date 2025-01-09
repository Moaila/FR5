"""
@author: 李文皓
@场景：眼在手外
@功能：用自定义YoloV8模型进行杯子的检测与定位，获取到的坐标是相对相机坐标系与机械臂坐标系下的坐标
C: 表示相机坐标系下的三维坐标
A: 表示机械臂坐标系下的三维坐标
"""
import os
import time
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray, String
from kyle_robot_toolbox.camera import Gemini335
from kyle_robot_toolbox.yolov8 import YoloV8Detect

# 初始化相机和模型
camera = Gemini335()
print("[INFO] 开始YoloV8模型加载")
model_path = "/home/newplace/FR5/baseyolo/runs/detect/train/weights/best.pt"
model = YoloV8Detect(model_path)

# 图像尺寸选取32的倍数，否则会报错
model.IMAGE_SIZE = 1088  # 图像尺寸
model.CONFIDENCE = 0.5  # 置信度
model.IOU = 0.6         # IOU阈值
print("[INFO] 完成YoloV8模型加载")

# 标定矩阵（眼在手外的结果）
camera_to_arm_matrix = np.array([
    [-0.9983436760441877, 0.003310123068703671, -0.05743646566292077, -75.16659019011934],
    [0.00833952766183191, 0.9961254408213325, -0.08754746385180773, -771.9754021846082],
    [0.05692413179799392, -0.08788144988435317, -0.9945031392536017, 857.6738248978729],
    [0.0, 0.0, 0.0, 1.0]
])

rospy.init_node('camera_publisher_335', anonymous=True)
pub_pose = rospy.Publisher('tag_pose', String, queue_size=10)
pub_trans_mat = rospy.Publisher('tag_trans_mat', Float32MultiArray, queue_size=10)


# 创建窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

while True:
    # 计时启动
    t_start = time.time()

    # 读取图像
    img_bgr = camera.read_color_img()
    depth_img = camera.read_depth_img()
    depth_canvas_tmp = camera.depth_img2canvas(depth_img)

    # 兼容尺寸差异
    dp_h, dp_w, _ = depth_canvas_tmp.shape
    depth_canvas = np.zeros_like(img_bgr)
    depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp

    # YOLOv8检测
    canvas, class_id_list, xyxy_list, conf_list = model.detect(img_bgr, draw_label=False)
    center_list = []

    for i, xyxy in enumerate(xyxy_list):
        # 针对 "杯子" 类别（自训练模型中其ID为15）
        if class_id_list[i] != 15:  
            continue

        # 读取矩形框并计算中心点
        x1, y1, x2, y2 = xyxy
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # 绘制中心点
        cv2.circle(canvas, [cx, cy], 5, (255, 255, 0), -1)
        cv2.circle(depth_canvas, [cx, cy], 5, (255, 0, 255), -1)

        # 确保坐标在深度图范围内
        if cx >= dp_w or cy >= dp_h:
            continue

        # 读取深度值并转换为3D坐标
        depth_value = depth_img[cy, cx]
        if depth_value == 0:  # 无效深度值跳过
            continue

        # 相机坐标系下的三维坐标
        cam_point3d = camera.depth_pixel2cam_point3d(cx, cy, depth_image=depth_img)
        cam_x, cam_y, cam_z = cam_point3d

        # 将相机坐标转换到机械臂坐标系
        cam_point_homogeneous = np.array([cam_x, cam_y, cam_z, 1])  # 齐次坐标
        arm_point_homogeneous = np.dot(camera_to_arm_matrix, cam_point_homogeneous)
        arm_x, arm_y, arm_z = arm_point_homogeneous[:3]  # 提取机械臂坐标

        # 显示相机坐标和机械臂坐标信息
        tag_camera = f"C:{cam_x:.0f},{cam_y:.0f},{cam_z:.0f}"
        tag_arm = f"A:{arm_x:.0f},{arm_y:.0f},{arm_z:.0f}"
        cv2.putText(canvas, tag_camera, (cx - 80, cy - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, tag_arm, (cx - 80, cy - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1, cv2.LINE_AA)

        # 发布机械臂坐标到 'tag_pose' 话题
        pose_msg = f"{arm_x:.2f},{arm_y:.2f},{arm_z:.2f}"
        pub_pose.publish(pose_msg)

        # 发布转换矩阵到 'tag_trans_mat' 话题
        trans_mat_msg = Float32MultiArray()
        trans_mat_msg.data = camera_to_arm_matrix.flatten()
        pub_trans_mat.publish(trans_mat_msg)

        # 打印机械臂坐标
        print(f"Mechanical Arm Coordinates: x={arm_x:.2f}, y={arm_y:.2f}, z={arm_z:.2f}")

        # 添加到中心点列表
        center_list.append([cx, cy])

    # 计算FPS
    t_end = time.time()
    fps = int(1 / (t_end - t_start))
    cv2.putText(canvas, f"FPS:{fps}", (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # 显示画面
    cv2.imshow("canvas", canvas)
    cv2.imshow("depth_canvas", depth_canvas)

    # 退出程序
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
camera.release()
