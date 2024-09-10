import pygame
import sys

# 初始化pygame的手柄模块
pygame.init()
pygame.joystick.init()

# 检测是否有手柄接入
if pygame.joystick.get_count() == 0:
    print("bro，是不是没插手柄，赶紧插上阿，记得要用GameSir-T4n Lite")
    sys.exit()

# 初始化第一个手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"检测到手柄: {joystick.get_name()}")

# 按键映射列表
button_map = {
    3: "左旋转 (X)",  # 按键 3 -> X -> 左旋转
    1: "右旋转 (B)",  # 按键 1 -> B -> 右旋转
    4: "上升 (Y)",    # 按键 4 -> Y -> 上升
    0: "下降 (A)",    # 按键 0 -> A -> 下降
}

# 方向键（D-Pad）映射
direction_map = {
    (-1, 0): "左",    # D-Pad 左
    (1, 0): "右",     # D-Pad 右
    (0, 1): "前",     # D-Pad 上（前）
    (0, -1): "后"     # D-Pad 下（后）
}

# 事件循环，持续检测按键输入
while True:
    pygame.event.pump()  # 更新事件

    # 遍历每一个按键并检测其状态
    for button_index in range(joystick.get_numbuttons()):
        if joystick.get_button(button_index):
            if button_index in button_map:
                print(f"按下了 {button_map[button_index]} 按键")

    # 检测方向键（帽子开关/方向键）
    for hat_index in range(joystick.get_numhats()):
        hat_value = joystick.get_hat(hat_index)
        if hat_value in direction_map:
            print(f"方向键操作: {direction_map[hat_value]}")
    
    pygame.time.wait(100)  # 延迟100ms避免重复输出太快



# import pygame
# import sys

# # 初始化pygame的手柄模块
# pygame.init()
# pygame.joystick.init()

# # 检测是否有手柄接入
# if pygame.joystick.get_count() == 0:
#     print("bro，是不是没插手柄，赶紧插上阿，不然没你好果子吃")
#     sys.exit()

# # 初始化第一个手柄
# joystick = pygame.joystick.Joystick(0)
# joystick.init()

# print(f"检测到手柄: {joystick.get_name()}")

# # 事件循环，持续检测按键输入
# while True:
#     pygame.event.pump()  # 更新事件

#     # 遍历每一个按键并检测其状态
#     for button_index in range(joystick.get_numbuttons()):
#         if joystick.get_button(button_index):
#             print(f"按下了按钮索引: {button_index}")

#     # # 遍历轴（axes）读取轴的状态（如LT、RT和摇杆）
#     # for axis_index in range(joystick.get_numaxes()):
#     #     axis_value = joystick.get_axis(axis_index)
#     #     if abs(axis_value) > 0.1:  # 设定阈值，排除小范围波动
#     #         print(f"轴索引 {axis_index} 的值: {axis_value}")
#         # 检测方向键（帽子开关）
#     # for hat_index in range(joystick.get_numhats()):
#     #     hat_value = joystick.get_hat(hat_index)
#     #     if hat_value != (0, 0):  # 当方向键被按下时
#     #         print(f"方向键状态: {hat_value}")

#     pygame.time.wait(100)  # 增加少量延迟以避免重复输出过快
