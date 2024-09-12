import pygame
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# 初始化 pygame
pygame.init()

# 手柄初始化
pygame.joystick.init()

# 检查是否有手柄连接
if pygame.joystick.get_count() == 0:
    print("没有检测到手柄，请连接手柄再试。")
    exit()

# 获取手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 设置数据存储结构
history_length = 100  # 只显示最近100次数据
left_x_axis = deque(maxlen=history_length)
left_y_axis = deque(maxlen=history_length)
right_x_axis = deque(maxlen=history_length)
right_y_axis = deque(maxlen=history_length)
lt_axis = deque(maxlen=history_length)
rt_axis = deque(maxlen=history_length)

# 初始数据填充
for _ in range(history_length):
    left_x_axis.append(0)
    left_y_axis.append(0)
    right_x_axis.append(0)
    right_y_axis.append(0)
    lt_axis.append(0)
    rt_axis.append(0)

# 创建实时更新图表
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

ax1.set_ylim([-1.2, 1.2])
ax1.set_title('左摇杆位置（X轴和Y轴）')
line1, = ax1.plot(left_x_axis, label='左摇杆 X轴', color='b')
line2, = ax1.plot(left_y_axis, label='左摇杆 Y轴', color='r')
ax1.legend()

ax2.set_ylim([-1.2, 1.2])
ax2.set_title('右摇杆位置（X轴和Y轴）')
line3, = ax2.plot(right_x_axis, label='右摇杆 X轴', color='g')
line4, = ax2.plot(right_y_axis, label='右摇杆 Y轴', color='m')
ax2.legend()

ax3.set_ylim([-0.2, 1.2])
ax3.set_title('LT和RT按键状态')
line5, = ax3.plot(lt_axis, label='LT', color='y')
line6, = ax3.plot(rt_axis, label='RT', color='c')
ax3.legend()

plt.tight_layout()

try:
    while True:
        pygame.event.pump()  # 处理事件队列

        # 读取左摇杆数据（GameSir T4n Lite的左摇杆一般是轴0/1）
        left_x_val = joystick.get_axis(0)  # 左摇杆的X轴
        left_y_val = joystick.get_axis(1)  # 左摇杆的Y轴

        # 读取右摇杆数据（GameSir T4n Lite的右摇杆一般是轴2/3）
        right_x_val = joystick.get_axis(2)  # 右摇杆的X轴
        right_y_val = joystick.get_axis(3)  # 右摇杆的Y轴

        # 读取LT和RT（一般是轴4和轴5）
        rt_val = (joystick.get_axis(5) + 1) / 2  # RT按钮, 值范围从-1到1，转换到0到1
        lt_val = (joystick.get_axis(4) + 1) / 2  # LT按钮, 同上

        # 更新数据队列
        left_x_axis.append(left_x_val)
        left_y_axis.append(left_y_val)
        right_x_axis.append(right_x_val)
        right_y_axis.append(right_y_val)
        lt_axis.append(lt_val)
        rt_axis.append(rt_val)

        # 更新图表数据
        line1.set_ydata(left_x_axis)
        line2.set_ydata(left_y_axis)
        line3.set_ydata(right_x_axis)
        line4.set_ydata(right_y_axis)
        line5.set_ydata(lt_axis)
        line6.set_ydata(rt_axis)

        # 重绘图表
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        ax3.relim()
        ax3.autoscale_view()

        plt.draw()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("程序终止。")

finally:
    pygame.quit()
