"""
测试手柄各个按键
"""
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

# 输出手柄的信息
print(f"手柄名称: {joystick.get_name()}")
print(f"手柄按钮数: {joystick.get_numbuttons()}")

# 按钮 ID 列表
button_ids = list(range(joystick.get_numbuttons()))

try:
    while True:
        pygame.event.pump()  # 处理事件队列

        # 遍历所有按钮，检测按下状态
        for button_id in button_ids:
            if joystick.get_button(button_id):
                print(f"按钮 {button_id} 被按下")

        # 休眠一小段时间以减少 CPU 占用
        pygame.time.wait(100)

except KeyboardInterrupt:
    print("检测结束。")

finally:
    pygame.quit()

# A-0， B-1，X-3，Y-4，左摇杆按下为13，右摇杆按下为14，RT-9，LT-8，LB-6，LT-7，右边小按钮是11，左边小按钮是10。