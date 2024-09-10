from fairino import Robot
import hand_sank

# 处理手柄输入并调用机械臂的控制函数
def handle_joystick_input(input_value):
    # 对输入进行处理
    if isinstance(input_value, tuple):  # 方向键输入（前后左右）
        handle_dpad_input(input_value)
    elif isinstance(input_value, int):  # 按钮输入
        handle_button_input(input_value)

# 处理方向键输入（前后左右的移动）
def handle_dpad_input(dpad_value):
    if dpad_value == (-1, 0):
        print("向左移动")
        # 控制机械臂向左移动
        
    elif dpad_value == (1, 0):
        print("向右移动")
        # 控制机械臂向右移动

    elif dpad_value == (0, 1):
        print("向前移动")
        # 控制机械臂向前移动
    elif dpad_value == (0, -1):
        print("向后移动")
        # 控制机械臂向后移动

# 处理按键输入（旋转、上升、下降等操作）
def handle_button_input(button_index):
    if button_index == 3:  # X 按键
        print("左旋转")
        # 控制机械臂6号关节左旋转

    elif button_index == 1:  # B 按键
        print("右旋转")
        # 控制机械臂6号关节右旋转
        
    elif button_index == 4:  # Y 按键
        print("上升")
        # 控制机械臂上升
        
    elif button_index == 0:  # A 按键
        print("下降")
        # 控制机械臂下降
        