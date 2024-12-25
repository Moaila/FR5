import time
import pygame
from fairino import Robot
import threading

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
        10: ("TPD文件内容发送失败", "检查TPD轨迹内容是否正确")
    }

    # 初始化机器人
    robot = Robot.RPC('192.168.59.6')

    # 注意58.6的机械臂不需要下面的伺服启动代码和末尾的结束代码
    ret = robot.ServoMoveStart()
    if ret != 0:
        error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
        print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        return

    # 初始化 pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("没有检测到手柄，请连接手柄再试。")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    max_linear_speed = 100  
    max_angular_speed = 20
    dt = 0.05

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

    try:
        while True:
            pygame.event.pump()
            # 读取手柄输入
            left_x = joystick.get_axis(0)
            left_y = joystick.get_axis(1)
            right_x = joystick.get_axis(2)
            right_y = joystick.get_axis(3)
            rt_val = (joystick.get_axis(5) + 1) / 2
            lt_val = (joystick.get_axis(4) + 1) / 2

            # 获取当前位姿
            current_pose = robot.GetActualTCPPose()
            ret_code, pose = current_pose

            if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
                x, y, z, rx, ry, rz = pose
            else:
                print("返回值不是预期的6个元素，实际为:", current_pose)
                break

            # 计算移动速度
            velocity_x = max_linear_speed * left_x
            velocity_y = max_linear_speed * left_y
            velocity_z = max_linear_speed * (rt_val - lt_val)
            angular_velocity_rz = max_angular_speed * right_x
            angular_velocity_rx = max_angular_speed * right_y

            # 计算新的目标位置和姿态
            new_x = x + velocity_x * dt
            new_y = y + velocity_y * dt
            new_z = z + velocity_z * dt
            new_rx = rx + angular_velocity_rx * dt
            new_rz = rz + angular_velocity_rz * dt

            pos_gain = [1.0] * 6
            ret = robot.ServoCart(0, [new_x, new_y, new_z, new_rx, ry, new_rz], pos_gain)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                break

            # 夹爪控制
            button_x = joystick.get_button(3)
            button_y = joystick.get_button(4)
            button_a = joystick.get_button(0)
            button_b = joystick.get_button(1)

            if button_x:
                gripper_position = min(gripper_position + gripper_step, 100)  # 限制最大值为 100
                robot.MoveGripper(1, gripper_position, gripper_speed, gripper_force, 30000, 1)  # 非阻塞模式
                print(f"夹爪打开到: {gripper_position}")
                time.sleep(debounce_time)  # 去抖动延迟

            if button_y:
                gripper_position = max(gripper_position - gripper_step, 0)  # 限制最小值为 0
                robot.MoveGripper(1, gripper_position, gripper_speed, gripper_force, 30000, 1)  # 非阻塞模式
                print(f"夹爪关闭到: {gripper_position}")
                time.sleep(debounce_time)  # 去抖动延迟

            # 夹爪旋转控制
            if button_a:
                print("夹爪左旋")
                angular_velocity_j6 = max_angular_speed
                current_joint_pos = robot.GetActualJointPosDegree()[1]
                new_j6 = current_joint_pos[5] + angular_velocity_j6 * dt
                robot.ServoJ([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], 
                             current_joint_pos[3], current_joint_pos[4], new_j6])

            elif button_b:
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
        robot.ServoMoveEnd()
        pygame.quit()


# 启动线程
robot_arm_thread = threading.Thread(target=robot_arm_thread)

robot_arm_thread.start()
robot_arm_thread.join()
