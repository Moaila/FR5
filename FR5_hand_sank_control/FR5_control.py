from fairino import Robot

robot = Robot()

ret,version  = robot.GetSDKVersion()    #查询SDK版本号
if ret ==0:
    print("SDK版本号为", version )
else:
    print("查询失败，错误码为",ret)

# 获取机械臂当前的末端位姿（初始位置）
current_pose = robot.get_tcp_pose()

# 每次点击的移动增量
move_increment = 0.01

# 处理手柄输入并调用机械臂的控制函数
def handle_joystick_input(input_value):
    # 对输入进行处理
    if isinstance(input_value, tuple):  # 方向键输入（前后左右）
        handle_dpad_input(input_value)
    elif isinstance(input_value, int):  # 按钮输入
        handle_button_input(input_value)

# 处理方向键输入（前后左右的移动）
def handle_dpad_input(dpad_value):
    global current_pose


    if dpad_value == (-1, 0):
        print("向左移动")
        # 控制机械臂向左移动
        current_pose[0] -= move_increment
        
    elif dpad_value == (1, 0):
        print("向右移动")
        # 控制机械臂向右移动
        current_pose[0] += move_increment

    elif dpad_value == (0, 1):
        print("向前移动")
        # 控制机械臂向前移动
        current_pose[1] += move_increment

    elif dpad_value == (0, -1):
        print("向后移动")
        # 控制机械臂向后移动
        current_pose[1] -= move_increment

    move_robot_to_pose(current_pose)

# 处理按键输入（旋转、上升、下降等操作）
def handle_button_input(button_index):
    if button_index == 3:  # X 按键
        print("左旋转")
        # 控制机械臂关节左旋转
        current_pose[5] -= move_increment

    elif button_index == 1:  # B 按键
        print("右旋转")
        # 控制机械臂6号关节右旋转
        current_pose[5] += move_increment
        
    elif button_index == 4:  # Y 按键
        print("上升")
        # 控制机械臂上升
        current_pose[2] += move_increment
        
    elif button_index == 0:  # A 按键
        print("下降")
        # 控制机械臂下降
        current_pose[2] -= move_increment
        
def move_robot_to_pose(pose):
    joint_angles = robot.inverse_kinematics(pose)  # 计算关节角度
    if joint_angles:  # 如果逆运动学求解成功
        robot.move_joints(joint_angles)  # 移动到新的关节角度
    else:
        print("逆运动学计算失败，无法移动到指定位置")
