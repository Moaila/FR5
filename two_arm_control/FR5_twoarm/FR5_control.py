"""
通过该程序进行双机械臂协同控制
"""
import threading
import time
import pygame
from fairino import Robot
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

def handle_input(joystick, left_arm, right_arm):
    pygame.event.pump()
    # 读取左摇杆数据
    left_x_val = joystick.get_axis(0)  # 左摇杆的X轴
    left_y_val = joystick.get_axis(1)  # 左摇杆的Y轴

    # 读取右摇杆数据
    right_x_val = joystick.get_axis(2)  # 右摇杆的X轴
    right_y_val = joystick.get_axis(3)  # 右摇杆的Y轴

    # 读取LT和RT（扳机）
    lt_val = (joystick.get_axis(4) + 1) / 2  # LT按钮, 值范围从-1到1，转换到0到1
    rt_val = (joystick.get_axis(5) + 1) / 2  # RT按钮, 值范围从-1到1，转换到0到1

    # 读取按钮状态（A, B, X, Y）
    a_button = joystick.get_button(0)  # A按钮
    b_button = joystick.get_button(1)  # B按钮
    x_button = joystick.get_button(3)  # X按钮
    y_button = joystick.get_button(4)  # Y按钮

    # 读取LB和RB按钮
    lb_button = joystick.get_button(6)  # LB按钮
    rb_button = joystick.get_button(7)  # RB按钮

    # 读取方向键（D-Pad）
    hat = joystick.get_hat(0)  # 获取方向键状态（返回一个(x, y)元组）
    dpad_left = hat == (-1, 0)  # 左方向键按下
    dpad_right = hat == (1, 0)  # 右方向键按下
    dpad_up = hat == (0, 1)  # 上方向键按下
    dpad_down = hat == (0, -1)  # 下方向键按下

    # 左右摇杆会控制机械臂在xy平面内自由移动
    left_arm.move_xy(left_x_val, left_y_val)
    right_arm.move_xy(right_x_val,right_y_val)

    # 判断LT，RT控制左机械臂下降和右机械臂下降
    if lt_val > 0.1:
        left_arm.move_z(-lt_val)
    if rt_val > 0.1:
        right_arm.move_z(-rt_val)
    # 通过左右摇杆按下分别控制左右摇杆上升
    if joystick.get_button(13):
            left_arm.move_z(1)
    if joystick.get_button(14):
            right_arm.move_z(1)
    # A控制左机械臂夹爪闭合，X控制右机械臂夹爪闭合，B控制左机械臂夹爪打开，Y控制右机械臂夹爪打开
    if a_button:
        left_arm.gripper_control(open=False)
    if x_button:
        right_arm.gripper_control(open=False)
    if b_button:
        left_arm.gripper_control(open=False)
    if y_button:
        right_arm.gripper_control(open=False)
    if lb_button:
        if dpad_left:
            left_arm.arm_gripper_left(open=True)
        if dpad_right:
            left_arm.arm_gripper_right(open=True)
        if dpad_up:
            left_arm.arm_gripper_up(open=True)
        if dpad_down:    
            left_arm.arm_gripper_down(open=True)
    if rb_button:
        if dpad_left:
            right_arm.arm_gripper_left(open=True)
        if dpad_right:
            right_arm.arm_gripper_right(open=True)
        if dpad_up:
            right_arm.arm_gripper_up(open=True)
        if dpad_down:    
            right_arm.arm_gripper_down(open=True)

# 左机械臂控制器
class LeftArmController:
    def __init__(self, ip):
        self.robot = Robot.RPC(ip)
        self.max_linear_speed = 100
        self.max_angular_speed = 20
        self.dt = 0.05
        self.gripper_max_position = 100 # 夹爪初始开合程度，范围[0,100]
        self.gripper_speed = 100 # 夹爪开合速度
        self.gripper_force = 50 # 夹爪的力
        self.gripper_step = 20 # 每次按键的步长调整
        self.debounce_time = 0.3 # 去抖动时间

    # def move_xy(self, x, y):
    #     current_pose = self.robot.GetActualTCPPose()
    #     ret_code, pose = current_pose
    #     if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
    #             x, y, z, rx, ry, rz = pose
    #     else:
    #         print("返回值不是预期的6个元素，实际为:", current_pose)
    #     velocity_x = self.max_linear_speed * x
    #     velocity_y = self.max_linear_speed * y
    #     new_x = x + velocity_x * self.dt
    #     new_y = y + velocity_y * self.dt
    #     pos_gain = [1.0] * 6
    #     ret = self.robot.ServoCart(0, [new_x, new_y, current_pose[2], current_pose[3], current_pose[4], current_pose[5]], pos_gain)
    #     if ret != 0:
    #             error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
    #             print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                
    def move_xy(self, x, y):
        current_pose = self.robot.GetActualTCPPose()
        ret_code, pose = current_pose  # 解包 ret_code 和 pose
        if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
            x, y, z, rx, ry, rz = pose  # 解包 pose 中的元素
        else:
            print("返回值不是预期的6个元素，实际为:", current_pose)
            return  # 如果返回值不符合预期，直接返回

        velocity_x = self.max_linear_speed * x
        velocity_y = self.max_linear_speed * y
        new_x = x + velocity_x * self.dt
        new_y = y + velocity_y * self.dt
        pos_gain = [1.0] * 6
        ret = self.robot.ServoCart(0, [new_x, new_y, z, rx, ry, rz], pos_gain)  # 使用解包后的 z, rx, ry, rz
        if ret != 0:
            error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
            print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")            
    def move_z(self, z):
        current_pose = self.robot.GetActualTCPPose()[1]
        new_z = current_pose[2] + z * self.max_linear_speed * self.dt
        pos_gain = [1.0] * 6
        self.robot.ServoCart(0, [current_pose[0], current_pose[1], new_z, current_pose[3], current_pose[4], current_pose[5]], pos_gain)

    def gripper_control(self, open=True):
        if open:
            gripper_position = min(gripper_position + self.gripper_step, 100)  # 限制最大值为 100
            self.robot.MoveGripper(1, gripper_position, self.gripper_speed, self.gripper_force, 30000, 1)  # 非阻塞模式
            print(f"夹爪打开到: {gripper_position}")
            time.sleep(self.debounce_time)  # 去抖动延迟
        else:
            gripper_position = max(gripper_position - self.gripper_step, 0)  # 限制最小值为 0
            self.robot.MoveGripper(1, gripper_position, self.gripper_speed, self.gripper_force, 30000, 1)  # 非阻塞模式
            print(f"夹爪关闭到: {gripper_position}")
            time.sleep(self.debounce_time)  # 去抖动延迟
    
    def arm_gripper_left(self, open=True):
        print("夹爪左旋")
        angular_velocity_j6 = self.max_angular_speed
        current_joint_pos = self.robot.GetActualJointPosDegree()[1]
        new_j6 = current_joint_pos[5] + angular_velocity_j6 * self.dt
        self.robot.ServoJ([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], 
                        current_joint_pos[3], current_joint_pos[4], new_j6])
    def arm_gripper_right(self, open=True):
        print("夹爪右旋")
        angular_velocity_j6 = -self.max_angular_speed
        current_joint_pos = self.robot.GetActualJointPosDegree()[1]
        new_j6 = current_joint_pos[5] + angular_velocity_j6 * self.dt
        self.robot.ServoJ([current_joint_pos[0], current_joint_pos[1], current_joint_pos[2], 
                        current_joint_pos[3], current_joint_pos[4], new_j6])
    def arm_gripper_up(self, open=True):
        if open:
            current_pose = self.robot.GetActualTCPPose()
            ret_code, pose = current_pose

            if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
                x, y, z, rx, ry, rz = pose
            else:
                print("返回值不是预期的6个元素，实际为:", current_pose)
            angular_velocity_rz = self.max_angular_speed * 0.1
            angular_velocity_rx = self.max_angular_speed * 0.1
            new_rx = rx + angular_velocity_rx * self.dt
            new_rz = rz + angular_velocity_rz * self.dt
            pos_gain = [1.0] * 6
            ret = self.robot.ServoCart(0, [x, y, z, new_rx, ry, new_rz], pos_gain)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")

    def arm_gripper_down(self, open=True):
        if open:
            current_pose = self.robot.GetActualTCPPose()
            ret_code, pose = current_pose

            if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
                x, y, z, rx, ry, rz = pose
            else:
                print("返回值不是预期的6个元素，实际为:", current_pose)
            angular_velocity_rz = self.max_angular_speed * 0.1
            angular_velocity_rx = self.max_angular_speed * 0.1
            new_rx = rx - angular_velocity_rx * self.dt
            new_rz = rz - angular_velocity_rz * self.dt
            pos_gain = [1.0] * 6
            ret = self.robot.ServoCart(0, [x, y, z, new_rx, ry, new_rz], pos_gain)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")

# 右机械臂控制器
class RightArmController(LeftArmController):
    def __init__(self, ip):
        super().__init__(ip)

# 各类初始化
def init():
    # 初始化机械臂
    left_arm = LeftArmController('192.168.58.6')
    right_arm = RightArmController('192.168.59.6')
    # 手柄初始化
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("没有检测到手柄，请连接手柄再试。")
        return
    # 创建一个手柄设备
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"手柄名称: {joystick.get_name()}")
    print(f"手柄按钮数: {joystick.get_numbuttons()}")

    # 启动手柄输入处理线程
    input_thread = threading.Thread(target=handle_input, args=(joystick, left_arm, right_arm))
    input_thread.start()

# 主程序
def main():
    init()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序终止。")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()