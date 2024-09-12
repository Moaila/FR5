import pygame
from fairino import Robot
import time

# 初始化机器人
robot = Robot.RPC('192.168.59.6')

# 启动伺服模式
ret = robot.ServoMoveStart()
if ret != 0:
    print("伺服启动失败，错误码：", ret)
    exit()

# 初始化 pygame
pygame.init()
pygame.joystick.init()

# 获取手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 定义最大线速度 (单位: mm/s) 和最大角速度 (单位: rad/s)
max_linear_speed = 100  # 最大线速度 100 mm/s
max_angular_speed = 0.1  # 最大角速度 0.1 rad/s

# 定义时间间隔
dt = 0.05  # 时间间隔 50ms

try:
    while True:
        pygame.event.pump()

        # 读取手柄输入
        left_x = joystick.get_axis(0)  # 左摇杆X轴 (控制平移)
        left_y = joystick.get_axis(1)  # 左摇杆Y轴 (控制平移)
        right_x = joystick.get_axis(2)  # 右摇杆X轴 (控制旋转)
        rt_val = (joystick.get_axis(5) + 1) / 2  # RT按钮 (上升)
        lt_val = (joystick.get_axis(4) + 1) / 2  # LT按钮 (下降)

        # 获取当前位姿
        current_pose = robot.GetActualTCPPose()

        # current_pose 是一个包含两个元素的元组，第一个是返回码，第二个是位姿
        ret_code, pose = current_pose

        # 检查返回值是否为预期的
        if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
            x, y, z, rx, ry, rz = pose
        else:
            print("返回值不是预期的6个元素，实际为:", current_pose)
            break

        # 计算移动速度
        velocity_x = max_linear_speed * left_x  # X方向速度
        velocity_y = max_linear_speed * left_y  # Y方向速度
        velocity_z = max_linear_speed * (rt_val - lt_val)  # Z方向速度 (RT上升, LT下降)
        
        # 计算旋转速度
        angular_velocity_rz = max_angular_speed * right_x  # 绕Z轴旋转角速度

        # 根据速度计算新的目标位置和姿态 (根据时间间隔 dt 来计算位移)
        new_x = x + velocity_x * dt
        new_y = y + velocity_y * dt
        new_z = z + velocity_z * dt
        new_rz = rz + angular_velocity_rz * dt

        # 使用伺服模式更新位姿
        pos_gain = [1.0] * 6  # 位姿增量增益
        ret = robot.ServoCart(0, [new_x, new_y, new_z, rx, ry, new_rz], pos_gain)
        if ret != 0:
            print("伺服更新失败，错误码：", ret)
            break

        # 休眠一小段时间，模拟实时控制
        time.sleep(dt)

except KeyboardInterrupt:
    print("控制结束。")

finally:
    # 停止伺服运动
    robot.ServoMoveEnd()
    pygame.quit()
