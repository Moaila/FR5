import pygame
from fairino import Robot
import time
from utils.logger import setup_logger
from utils.error_codes import error_codes

class JoystickController:
    def __init__(self, robot_ip):
        self.logger = setup_logger("JoystickController", "robot_arm_app.log")
        
        # 初始化机器人
        self.robot = Robot.RPC(robot_ip)

        # 启动伺服模式
        ret = self.robot.ServoMoveStart()
        if ret != 0:
            error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
            self.logger.error(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
            raise Exception("伺服启动失败")

        # 初始化 pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.logger.error("没有检测到手柄，请连接手柄再试。")
            raise Exception("没有检测到手柄")
        
        # 获取手柄
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # 定义最大线速度 和 最大角速度 
        self.max_linear_speed = 100  
        self.max_angular_speed = 20  #可以尝试增大

        # 定义时间间隔
        self.dt = 0.05  # 时间间隔 50ms

        # 夹爪初始化
        self.robot.SetGripperConfig(4, 0, 0, 1)
        time.sleep(0.5)
        self.robot.ActGripper(1, 1)
        time.sleep(2)
        self.gripper_open = True

    def control_robot(self):
        try:
            while True:
                pygame.event.pump()

                # 读取手柄输入
                left_x = self.joystick.get_axis(0)
                left_y = self.joystick.get_axis(1)
                right_x = self.joystick.get_axis(2)
                right_y = self.joystick.get_axis(3)
                rt_val = (self.joystick.get_axis(5) + 1) / 2
                lt_val = (self.joystick.get_axis(4) + 1) / 2

                # 获取当前位姿
                current_pose = self.robot.GetActualTCPPose()
                ret_code, pose = current_pose

                if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
                    x, y, z, rx, ry, rz = pose
                else:
                    self.logger.error(f"返回值不是预期的6个元素，实际为: {current_pose}")
                    break

                # 计算移动速度
                velocity_x = self.max_linear_speed * left_x
                velocity_y = self.max_linear_speed * left_y
                velocity_z = self.max_linear_speed * (rt_val - lt_val)

                # 计算旋转速度
                angular_velocity_rz = self.max_angular_speed * right_x
                angular_velocity_rx = self.max_angular_speed * right_y

                # 根据速度计算新的目标位置和姿态
                new_x = x + velocity_x * self.dt
                new_y = y + velocity_y * self.dt
                new_z = z + velocity_z * self.dt
                new_rx = rx + angular_velocity_rx * self.dt
                new_rz = rz + angular_velocity_rz * self.dt

                # 使用伺服模式更新位姿
                pos_gain = [1.0] * 6
                ret = self.robot.ServoCart(0, [new_x, new_y, new_z, new_rx, ry, new_rz], pos_gain)
                if ret != 0:
                    error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                    self.logger.error(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                    break

                # 夹爪控制
                button_x = self.joystick.get_button(3)
                button_y = self.joystick.get_button(4)

                if button_x and not self.gripper_open:
                    self.logger.info("打开夹爪")
                    self.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
                    self.gripper_open = True
                elif button_y and self.gripper_open:
                    self.logger.info("关闭夹爪")
                    self.robot.MoveGripper(1, 0, 50, 10, 10000, 1)
                    self.gripper_open = False

                time.sleep(self.dt)

        except KeyboardInterrupt:
            self.logger.info("控制结束")
        finally:
            # 停止伺服运动
            self.robot.ServoMoveEnd()
            pygame.quit()

