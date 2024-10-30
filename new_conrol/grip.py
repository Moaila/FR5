import pygame
from fairino import Robot
import time

# 初始化机器人
robot = Robot.RPC('192.168.59.6')

# 夹爪初始化配置
ret = robot.SetGripperConfig(4, 0)
if ret != 0:
    print(f"配置夹爪失败，错误码: {ret}")
    exit()

ret = robot.ActGripper(1, 1)  # 激活夹爪
if ret != 0:
    print(f"激活夹爪失败，错误码: {ret}")
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

# 定义夹爪的开合距离初始值
gripper_position = 100  # 初始开合度，范围 [0, 100]
gripper_speed = 100  # 夹爪的开合速度，0-100
gripper_force = 50  # 夹爪的力，0-100
gripper_step = 20  # 每次按键的步长调整量

# 去抖动的延迟时间
debounce_time = 0.3  # 300毫秒

# 持续检测手柄输入
try:
    while True:
        pygame.event.pump()

        # 获取 X 和 Y 按钮的状态
        button_x = joystick.get_button(3) 
        button_y = joystick.get_button(4)  

        # 如果按住 X 按钮，逐步打开夹爪
        if button_x:
            gripper_position = min(gripper_position + gripper_step, 100)  # 限制最大值为 100
            robot.MoveGripper(1, gripper_position, gripper_speed, gripper_force, 30000, 1)  # 使用非阻塞模式
            print(f"夹爪打开到: {gripper_position}")
            time.sleep(debounce_time)  # 增加去抖动延迟

        # 如果按住 Y 按钮，逐步关闭夹爪
        elif button_y:
            gripper_position = max(gripper_position - gripper_step, 0)  # 限制最小值为 0
            robot.MoveGripper(1, gripper_position, gripper_speed, gripper_force, 30000, 1)  # 使用非阻塞模式
            print(f"夹爪关闭到: {gripper_position}")
            time.sleep(debounce_time)  # 增加去抖动延迟

except KeyboardInterrupt:
    print("控制结束。")

finally:
    # 清理资源
    pygame.quit()
