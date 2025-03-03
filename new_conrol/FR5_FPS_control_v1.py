"""
@Author:  李文皓，王嘉璇
@Date: 2025/3/3
@Description: 键鼠控制重构版本
"""
import threading
import time
import numpy as np
import cv2
import open3d as o3d
import pygame
from kyle_robot_toolbox.camera import Gemini335
from fairino import Robot
import sys
# 摄像头处理函数
def camera_thread():
    # 创建相机对象
    camera = Gemini335()
    # 创建窗口
    win_flag = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED
    cv2.namedWindow("color", flags=win_flag)
    while True:
        # 采集彩图, 色彩空间BGR
        img_bgr = camera.read_color_img() 
        # 显示图像
        cv2.imshow('color', img_bgr)
        key = cv2.waitKey(1)
        if key == ord('q'):
            # 如果按键为q 代表quit 退出程序
            break
    # 关闭摄像头
    camera.release()
    # 销毁所有的窗口
    cv2.destroyAllWindows()
# 速度限制函数
def limit_speed(v, max_speed):
    if v > max_speed:
        return max_speed
    elif v < -max_speed:
        return -max_speed
    else:
        return v
# 机械臂处理函数
def robot_arm_thread():
    error_codes = {
        -7: ("上传文件不存在", "检查文件名称是否正确"),
        -6: ("保存文件路径不存在", "检查文件路径是否正确"),
        -5: ("LUA文件不存在", "检查lua文件名称是否正确"),
        -4: ("xmlrpc接口执行失败", "请联系后台工程"),
        -3: ("xmlrpc通讯失败", "请检查网络连接及服务器IP地址是否正确"),
        -2: ("与控制器通讯异常", "检查与控制器硬件连接"),
        -1: ("其他错误", "联系售后工程师查看控制器日志"),
        0: ("调用成功", ""),
        1: ("接口参数个数不一致", "检查接口参数个数"),
        3: ("接口参数值异常", "检查参数类型或范围"),
        8: ("轨迹文件打开失败", "检查TPD轨迹文件是否存在或轨迹名是否正确"),
        9: ("TPD文件名发送失败", "检查TPD轨迹名是否正确"),
        10: ("TPD文件内容发送失败", "检查TPD文件内容是否正确")
    }
    # 初始化机器人
    robot = Robot.RPC('192.168.59.6')
    ret = robot.ServoMoveStart()
    if ret != 0:
        error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
        print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        return
    # 初始化 pygame
    pygame.init()
    # 设置窗口大小
    screen = pygame.display.set_mode((2000, 2000))
    pygame.display.set_caption("机器人控制")
    # 设置字体
    font = pygame.font.Font(None, 36)
    max_linear_speed = 100
    max_angular_speed = 30
    h = 0.5
    dt = 0.02
    # 夹爪初始化
    robot.SetGripperConfig(4, 0, 0, 1)
    time.sleep(0.5)
    robot.ActGripper(1, 1)
    time.sleep(2)
    gripper_open = True
    gripper_position = 100  # 初始夹爪开合度，范围 [0, 100]
    gripper_speed = 100  # 夹爪开合速度
    gripper_force = 50  # 夹爪的力
    gripper_step = 20  # 每次按键的步长调整量
    debounce_time = 0.3  # 去抖动时间，300毫秒
    screen_center = (screen.get_width() // 2, screen.get_height() // 2)
    mouse_x, mouse_y = screen_center
    try:
        while True:
            pygame.event.pump()
            # 获取当前位姿
            current_pose = robot.GetActualTCPPose()
            ret_code, pose = current_pose
            if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
                x, y, z, rx, ry, rz = pose
            else:
                print("返回值不是预期的6个元素，实际为:", current_pose)
                break
            # 空间矢量
            vector_n = np.array([np.sin(np.radians(rz)), -np.cos(np.radians(rz)), np.cos(np.radians(rx))]) # 末端法向量
            vector_r = np.array([x, y, z]) # 夹爪空间坐标
            # 读取鼠标输入
            mouse_x, mouse_y = pygame.mouse.get_pos()
            delta_x = mouse_x - screen_center[0]
            delta_y = mouse_y - screen_center[1]
            m_x = limit_speed(delta_x, max_angular_speed)
            m_y = limit_speed(delta_y, max_angular_speed)
            # 读取键盘输入
            # 结束控制
            key_p = pygame.key.get_pressed()[pygame.K_p]
            # 控制夹爪旋转
            key_z = pygame.key.get_pressed()[pygame.K_z]
            key_c = pygame.key.get_pressed()[pygame.K_c]
            # 控制夹爪开合
            key_f = pygame.key.get_pressed()[pygame.K_f]
            key_g = pygame.key.get_pressed()[pygame.K_g]
            # 控制机械臂前后左右上下移动
            key_w = pygame.key.get_pressed()[pygame.K_w]
            key_s = pygame.key.get_pressed()[pygame.K_s]
            key_a = pygame.key.get_pressed()[pygame.K_a]
            key_d = pygame.key.get_pressed()[pygame.K_d]
            key_q = pygame.key.get_pressed()[pygame.K_q]#z_up
            key_e = pygame.key.get_pressed()[pygame.K_e]#z_down
            # 另外一套控制方式，功能重复，可以暂时不使用
            key_up = pygame.key.get_pressed()[pygame.K_UP]
            key_down = pygame.key.get_pressed()[pygame.K_DOWN]
            key_left = pygame.key.get_pressed()[pygame.K_LEFT]
            key_right = pygame.key.get_pressed()[pygame.K_RIGHT]
            #quit
            if key_p:
                break
            if key_f:
                gripper_position = min(gripper_position + gripper_step, 100)
                robot.MoveGripper(1, gripper_position, gripper_speed, gripper_force, 30000, 1)
                print(f"夹爪打开到: {gripper_position}")
                time.sleep(debounce_time)

            if key_g:
                gripper_position = max(gripper_position - gripper_step, 0)
                robot.MoveGripper(1, gripper_position, gripper_speed, gripper_force, 30000, 1)
                print(f"夹爪关闭到: {gripper_position}")
                time.sleep(debounce_time)
            # 计算r速度
            v_ws = (key_w - key_s) * max_linear_speed * vector_n # 前后
            vector_l = np.array([np.cos(np.radians(rz)), np.sin(np.radians(rz)), - np.sin(np.radians(ry))])
            v_ad = (key_a - key_d) * max_linear_speed * vector_l # 左右 
            v_qe = (key_q - key_e) * max_linear_speed * np.array([0, 0, 1]) # 上下
            v_ud = (key_up - key_down) * max_linear_speed * np.array([0, -1, 0]) # y_前后
            v_lr = (key_left - key_right) * max_linear_speed * np.array([1, 0, 0]) # x_左右
            v_r = v_ws + v_ad + v_qe + v_ud + v_lr
            v_r_norm = np.linalg.norm(v_r)
            if v_r_norm > max_linear_speed:
                v_r = v_r / v_r_norm * max_linear_speed # 速度限制
            # 计算xyz
            x = x + limit_speed(v_r[0], max_linear_speed) * dt
            y = y + limit_speed(v_r[1], max_linear_speed) * dt
            z = z + limit_speed(v_r[2], max_linear_speed) * dt
            # 计算下一时刻的rz，rx
            rx = rx + m_y * dt
            rz = rz + m_x * dt
            ry = ry + (key_c - key_z) * dt * max_angular_speed
            # 夹抓控制
            # 更新机器人位姿
            pos_gain = [1.0] * 6
            ret = robot.ServoCart(0, [x, y, z, rx, ry, rz], pos_gain)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                break
            # 显示机器人位姿
            pose_text = font.render(f"位姿: x={x:.2f}, y={y:.2f}, z={z:.2f}, rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}", True, (255, 255, 255))
            screen.blit(pose_text, (20, 60))
            # 更新屏幕
            screen.fill((0, 0, 0))
            pygame.mouse.set_pos(screen_center)
            pygame.display.flip()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("控制结束。")
    finally:
        # robot.ServoMoveEnd()
        pygame.quit()
# 启动线程
camera_thread = threading.Thread(target=camera_thread)
robot_arm_thread = threading.Thread(target=robot_arm_thread)
camera_thread.start()
robot_arm_thread.start()
camera_thread.join()
robot_arm_thread.join()