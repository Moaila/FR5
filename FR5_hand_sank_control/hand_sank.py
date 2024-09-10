import pygame
import sys

# 初始化pygame的手柄模块
pygame.init()
pygame.joystick.init()

# 检测是否有手柄接入
if pygame.joystick.get_count() == 0:
    print("没有检测到手柄，请检查连接")
    sys.exit()

# 初始化第一个手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"检测到手柄: {joystick.get_name()}")

# 按键映射列表（具体手柄的按钮索引可能需要调整）
# 您可以通过调试确定每个按键的索引
button_map = {
    0: "Y",   # 按键 0 -> Y
    1: "B",   # 按键 1 -> B
    2: "A",   # 按键 2 -> A
    3: "X",   # 按键 3 -> X
    4: "左方向键",  # 按键 4 -> 左方向键
    5: "右方向键",  # 按键 5 -> 右方向键
    6: "上方向键",  # 按键 6 -> 上方向键
    7: "下方向键",  # 按键 7 -> 下方向键
}

# 事件循环，持续检测按键输入
while True:
    pygame.event.pump()  # 更新事件
    
    # 遍历每一个按键并检测其状态
    for button_index in range(joystick.get_numbuttons()):
        if joystick.get_button(button_index):
            if button_index in button_map:
                print(f"按下了 {button_map[button_index]} 按键")
    
    # 退出条件：可以按Ctrl+C手动退出终端中的程序
