import pygame
import time
import numpy as np
import cv2
from fairino import Robot
from kyle_robot_toolbox.camera import Gemini335

# 初始化机器人
robot = Robot.RPC('192.168.59.6')

# 初始化相机
camera = Gemini335()

# 定义最大线速度和最大角速度 
max_linear_speed = 100  
max_angular_speed = 20
dt = 0.05  # 时间间隔 50ms

# 启动伺服模式
ret = robot.ServoMoveStart()
if ret != 0:
    print("伺服启动失败，错误码：", ret)
    exit()

# 初始化 pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("没有检测到手柄，请连接手柄再试。")
    exit()

# 获取手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 创建窗口显示相机图像
win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
cv2.namedWindow("color", flags=win_flag)

gripper_open = True

try:
    while True:
        pygame.event.pump()

        # 读取相机图像并显示
        color_img = camera.read_color_img()
        if color_img is not None:
            cv2.imshow('color', color_img)

        # 按'q'键退出图像显示
        if cv2.waitKey(1) == ord('q'):
            break

        # 读取手柄输入
        left_x = joystick.get_axis(0)  # 左摇杆X轴
        left_y = joystick.get_axis(1)  # 左摇杆Y轴
        right_x = joystick.get_axis(2)  # 右摇杆X轴
        right_y = joystick.get_axis(3)  # 右摇杆Y轴
        rt_val = (joystick.get_axis(5) + 1) / 2  # RT按钮
        lt_val = (joystick.get_axis(4) + 1) / 2  # LT按钮

        # 获取当前位姿
        ret_code, pose = robot.GetActualTCPPose()
        if ret_code != 0:
            print("获取机械臂位置失败，错误码：", ret_code)
            break
        x, y, z, rx, ry, rz = pose

        # 计算移动速度
        velocity_x = max_linear_speed * left_x  
        velocity_y = max_linear_speed * left_y  
        velocity_z = max_linear_speed * (rt_val - lt_val)  
        angular_velocity_rz = max_angular_speed * right_x  
        angular_velocity_rx = max_angular_speed * right_y  

        # 根据速度计算新的目标位置和姿态
        new_x = x + velocity_x * dt
        new_y = y + velocity_y * dt
        new_z = z + velocity_z * dt
        new_rx = rx + angular_velocity_rx * dt
        new_rz = rz + angular_velocity_rz * dt

        # 使用伺服模式更新位姿
        pos_gain = [1.0] * 6
        ret = robot.ServoCart(0, [new_x, new_y, new_z, new_rx, ry, new_rz], pos_gain)
        if ret != 0:
            print("伺服更新失败，错误码：", ret)
            break

        # 夹爪控制
        button_x = joystick.get_button(3)  # X按钮控制开夹爪
        button_y = joystick.get_button(4)  # Y按钮控制关夹爪
        button_a = joystick.get_button(0)  # A按钮控制左旋
        button_b = joystick.get_button(1)  # B按钮控制右旋

        # 夹爪打开和关闭
        if button_x and not gripper_open:
            print("打开夹爪")
            robot.MoveGripper(1, 100, 50, 10, 10000, 1)
            gripper_open = True
        elif button_y and gripper_open:
            print("关闭夹爪")
            robot.MoveGripper(1, 0, 50, 10, 10000, 1)
            gripper_open = False

        # 夹爪旋转控制
        if button_a:
            # 按下A按钮，夹爪左旋
            print("夹爪左旋")
            angular_velocity_j6 = max_angular_speed
            current_joint_pos = robot.GetActualJointPosDegree()[1]
            new_j6 = current_joint_pos[5] + angular_velocity_j6 * dt
            robot.ServoJ([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], 
                          current_joint_pos[3], current_joint_pos[4], new_j6])

        elif button_b:
            # 按下B按钮，夹爪右旋
            print("夹爪右旋")
            angular_velocity_j6 = -max_angular_speed
            current_joint_pos = robot.GetActualJointPosDegree()[1]
            new_j6 = current_joint_pos[5] + angular_velocity_j6 * dt
            robot.ServoJ([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], 
                          current_joint_pos[3], current_joint_pos[4], new_j6])

        time.sleep(dt)

except KeyboardInterrupt:
    print("控制结束。")

finally:
    # 停止伺服运动
    robot.ServoMoveEnd()
    camera.release()
    cv2.destroyAllWindows()
    pygame.quit()
