import tkinter as tk
from tkinter import messagebox
from fairino import Robot
import pygame
import threading
import subprocess
import os
import signal

class RobotArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("机械臂控制应用")
        self.root.geometry("600x400")  # 增加窗口大小

        # 创建标签来显示机械臂和手柄状态
        self.robot_status_label = tk.Label(self.root, text="机械臂状态: 未连接", fg="red")
        self.robot_status_label.pack(pady=10)

        self.joystick_status_label = tk.Label(self.root, text="手柄状态: 未连接", fg="red")
        self.joystick_status_label.pack(pady=10)

        # 创建连接按钮
        self.connect_button = tk.Button(self.root, text="尝试连接机械臂和手柄", command=self.connect_devices)
        self.connect_button.pack(pady=20)

        # 创建控制机械臂按钮，但初始状态为隐藏
        self.control_button = tk.Button(self.root, text="使用手柄控制机械臂", command=self.control_robot)
        self.control_button.pack_forget()  # 隐藏按钮

        # 创建停止控制机械臂按钮，但初始状态为隐藏
        self.stop_button = tk.Button(self.root, text="停止使用手柄", command=self.stop_robot_control)
        self.stop_button.pack_forget()  # 隐藏按钮

        # 初始化 pygame
        pygame.init()
        pygame.joystick.init()

        # 使用 after 方法定时检测手柄状态
        self.poll_joystick()
        # 使用线程来检测机械臂状态，降低检测频率
        self.poll_robot()

        # 用于保存外部脚本的进程对象
        self.robot_process = None

    def connect_devices(self):
        # 手动触发机械臂连接检测
        threading.Thread(target=self.check_robot_connection).start()

    def poll_robot(self):
        # 每5秒检测一次机械臂状态
        threading.Thread(target=self.check_robot_connection).start()
        self.root.after(5000, self.poll_robot)

    def check_robot_connection(self):
        # 检测机械臂连接状态的操作放在后台线程中
        try:
            self.robot = Robot.RPC('192.168.59.6')
            ret, version_info = self.robot.GetSDKVersion()
            if ret != 0:
                raise Exception(f"查询SDK版本失败，错误码: {ret}")
            self.update_robot_status("机械臂状态: 已连接", "green")
        except Exception as e:
            self.update_robot_status("机械臂状态: 未连接", "red")
        
        # 检查是否应该显示控制按钮
        self.update_control_button()

    def poll_joystick(self):
        # 重新初始化 pygame
        pygame.joystick.quit()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.update_joystick_status(f"手柄状态: 已连接 - {self.joystick.get_name()}", "green")
        else:
            self.update_joystick_status("手柄状态: 未连接", "red")

        # 每2秒检测一次
        self.root.after(2000, self.poll_joystick)

        # 检查是否应该显示控制按钮
        self.update_control_button()

    def update_robot_status(self, status_text, color):
        self.root.after(0, lambda: self.robot_status_label.config(text=status_text, fg=color))

    def update_joystick_status(self, status_text, color):
        self.joystick_status_label.config(text=status_text, fg=color)

    def update_control_button(self):
        # 检查机械臂和手柄是否都已连接
        if self.robot_status_label.cget("text") == "机械臂状态: 已连接" and \
           self.joystick_status_label.cget("text").startswith("手柄状态: 已连接"):
            self.connect_button.pack_forget()  # 隐藏连接按钮
            self.control_button.pack(pady=20)  # 显示控制按钮
        else:
            self.control_button.pack_forget()  # 隐藏控制按钮
            self.connect_button.pack(pady=20)  # 显示连接按钮

    def control_robot(self):
        # 调用外部 Python 脚本来控制机械臂
        try:
            # 使用完整路径
            self.robot_process = subprocess.Popen(['python', '/home/tom/FR5/new_conrol/FR5_control_robotarm.py'])
            self.control_button.pack_forget()  # 隐藏控制按钮
            self.stop_button.pack(pady=20)  # 显示停止按钮
        except Exception as e:
            messagebox.showerror("错误", f"启动控制程序失败: {e}")

    def stop_robot_control(self):
        # 停止外部 Python 脚本
        if self.robot_process:
            try:
                os.kill(self.robot_process.pid, signal.SIGTERM)
                self.robot_process = None
                self.stop_button.pack_forget()  # 隐藏停止按钮
                self.control_button.pack(pady=20)  # 显示控制按钮
            except Exception as e:
                messagebox.showerror("错误", f"停止控制程序失败: {e}")

# 创建主窗口
root = tk.Tk()
app = RobotArmApp(root)
root.mainloop()
