import pygame
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

pygame.init()

pygame.joystick.init()

if pygame.joystick.get_count() ==0:
    print("没有监测到手柄，请重试")
    exit()

#手柄获取
joystick = pygame.joystick.Joystick(0)
joystick.init()

history_length = 100  # 我们只显示最近100次数据
x_axis = deque(maxlen=history_length)
y_axis = deque(maxlen=history_length)
lt_axis = deque(maxlen=history_length)
rt_axis = deque(maxlen=history_length)

# 初始数据填充
for _ in range(history_length):
    x_axis.append(0)
    y_axis.append(0)
    lt_axis.append(0)
    rt_axis.append(0)

# 创建实时更新图表
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

ax1.set_ylim([-1.2, 1.2])
ax1.set_title('摇杆位置（X轴和Y轴）')
line1, = ax1.plot(x_axis, label='X轴', color='b')
line2, = ax1.plot(y_axis, label='Y轴', color='r')
ax1.legend()

ax2.set_ylim([-0.2, 1.2])
ax2.set_title('LT和RT按键状态')
line3, = ax2.plot(lt_axis, label='LT', color='g')
line4, = ax2.plot(rt_axis, label='RT', color='m')
ax2.legend()

plt.tight_layout()

try:
    while True:
        pygame.event.pump()  # 处理事件队列

        # 读取摇杆数据（GameSir T4n Lite的摇杆一般是轴0/1和轴2/3）
        x_val = joystick.get_axis(0)  # 左摇杆的X轴
        y_val = joystick.get_axis(1)  # 左摇杆的Y轴
        rt_val = (joystick.get_axis(5) + 1) / 2  # RT按钮, 值范围从-1到1，转换到0到1
        lt_val = (joystick.get_axis(4) + 1) / 2  # LT按钮, 同上

        # 更新数据队列
        x_axis.append(x_val)
        y_axis.append(y_val)
        lt_axis.append(lt_val)
        rt_axis.append(rt_val)

        # 更新图表数据
        line1.set_ydata(x_axis)
        line2.set_ydata(y_axis)
        line3.set_ydata(lt_axis)
        line4.set_ydata(rt_axis)

        # 重绘图表
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()

        plt.draw()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("程序终止。")

finally:
    pygame.quit()