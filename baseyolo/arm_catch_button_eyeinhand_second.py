"""
@author: 李文皓
@场景：眼在手上
@操作说明：使用时先启动roscore, 然后运行yolov8_coffee_third.py，再运行本程序
"""
import numpy as np
import rospy
from std_msgs.msg import String
from fairino import Robot
import time

# 全局变量，用于存储从 ROS 话题接收到的目标位置和红色按钮信号
target_position = None
red_button_signal = None

# 用于保存最后一次接收到的位置
position_history = [] 

# 相机基本偏置量
x_offset = -15  # x轴方向偏置量
y_offset = 125  # y轴方向偏置量
z_offset = 285.0  # z轴方向偏置量，控制抓取时的高度

# 各个按钮位置偏置,x,y,z
button_offsets = {
    1:('button',(0,0,0)),
    2:('Cappuccino', (-65,-35,60)),
}

# 回调函数：接收目标位置
def target_position_callback(msg):
    global target_position, position_history
    try:
        target_position = [float(coord) for coord in msg.data.split(",")]
        position_history.append(target_position)
        if len(position_history) > 1:
            position_history.pop(0)
    except ValueError:
        rospy.logerr("接收到的目标位置格式不正确：%s", msg.data)
        target_position = None

# 回调函数：接收红色按钮信号
def red_button_signal_callback(msg):
    global red_button_signal
    red_button_signal = int(msg.data)  # 接收信号并转换为整数（1 或 0）

# 机械臂抓取函数（闭环控制）
def robot_grab():
    global target_position, position_history, red_button_signal, x_offset, y_offset, z_offset

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

    robot = Robot.RPC('192.168.59.6')
    ret = robot.ServoMoveStart()
    if ret != 0:
        error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
        print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        return 

    robot.SetGripperConfig(4, 0, 0, 1)
    time.sleep(1.5)
    robot.ActGripper(1, 1)
    time.sleep(1.5)
    robot.MoveGripper(1, 0, 50, 30, 10000, 1)
    time.sleep(3)

    home_position = [-90.0, -400.0, 285.0, 90.0, 0.0, 0.0]
    robot.MoveCart(home_position, 0, 0)

    try:
        print("请选择您想要点击的按钮：")
        for key, value in button_offsets.items():
            print(f"{key}: {value[0]}")
        button_choice = int(input("请输入按钮编号："))

        if button_choice not in button_offsets:
            print("无效的选择，退出程序。")
            return

        selected_button_name, button_offset = button_offsets[button_choice]
        print(f"您选择了按钮：{selected_button_name}")

        while not rospy.is_shutdown():
            # 判断是否有有效的目标位置
            if target_position:
                target_x = target_position[0] + x_offset + button_offset[0]
                target_y = target_position[1] + y_offset + button_offset[1]
                target_z = z_offset + button_offset[2]
            elif position_history:
                last_position = position_history[-1]
                target_x = last_position[0] + x_offset + button_offset[0]
                target_y = last_position[1] + y_offset + button_offset[1]
                target_z = z_offset + button_offset[2]
                print("目标丢失，使用最后保存的位置！")
            else:
                print("等待目标位置...")
                time.sleep(0.1)
                continue

            print(f"最终目标位置: x={target_x}, y={target_y}, z={target_z}")
            input("按任意键开始移动...")

            # 移动到目标位置
            ret = robot.MoveCart([target_x, target_y, target_z, 90.0, 0.0, 0.0], 0, 0)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服运动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                return

            print("点击完成，等待检测红色按钮信号...")
            time.sleep(2)  # 等待信号更新

            if red_button_signal == 1:
                print("红色按钮检测成功，任务完成！")
                break
            else:
                print("未检测到红色按钮，重新点击目标...")
                time.sleep(1)  # 等待再次点击前的短暂延迟

    except KeyboardInterrupt:
        print("任务中断。")

    finally:
        robot.MoveCart(home_position, 0, 0)
        robot.ServoMoveEnd()
        print("伺服模式已停止。")

# 主函数
if __name__ == "__main__":
    rospy.init_node("robot_grab_node", anonymous=True)
    rospy.Subscriber("tag_pose", String, target_position_callback)
    rospy.Subscriber("button_status", String, red_button_signal_callback)

    print("等待目标位置和红色按钮信号...")
    time.sleep(2)

    robot_grab()
