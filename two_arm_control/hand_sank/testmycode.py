import pygame

# 初始化 pygame
pygame.init()
pygame.joystick.init()

# 检查是否有手柄连接
if pygame.joystick.get_count() == 0:
    print("没有检测到手柄，请连接手柄再试。")
    exit()

# 获取手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 输出手柄信息
print(f"手柄名称: {joystick.get_name()}")
print(f"手柄按钮数: {joystick.get_numbuttons()}")

# 读取手柄输入并保存数据
def read_controller_input():
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

    # 打印所有读取的数据
    print(f"左摇杆 X: {left_x_val}, Y: {left_y_val}")
    print(f"右摇杆 X: {right_x_val}, Y: {right_y_val}")
    print(f"LT: {lt_val}, RT: {rt_val}")
    print(f"A按钮: {a_button}, B按钮: {b_button}, X按钮: {x_button}, Y按钮: {y_button}")
    print(f"LB按钮: {lb_button}, RB按钮: {rb_button}")
    print(f"方向键: 上={dpad_up}, 下={dpad_down}, 左={dpad_left}, 右={dpad_right}")

# 主循环读取输入
try:
    while True:
        pygame.event.pump()  # 处理事件队列
        read_controller_input()  # 读取并保存输入
        pygame.time.wait(50)  # 每50毫秒读取一次输入

except KeyboardInterrupt:
    print("程序终止。")
finally:
    pygame.quit()
