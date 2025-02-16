"""
@author: 李文皓
@function：串联整个流程，实现视觉+传统方法制作咖啡全流程
"""
import numpy as np
import rospy
from std_msgs.msg import String, Int32
from fairino import Robot
import time

# 全局变量，用于存储从 ROS 话题接收到的目标位置
target_position = None

# 用于保存最后一次接收到的位置
position_history = []

# 目标位置回调函数
def target_position_callback(msg):
    global target_position
    try:
        # 将目标位置从字符串解析为浮点数
        target_position = [float(coord) for coord in msg.data.split(",")]
        # 记录位置到历史记录
        position_history.append(target_position)
        # 限制历史记录数组的长度
        if len(position_history) > 1:
            position_history.pop(0)
    except ValueError:
        rospy.logerr("接收到的目标位置格式不正确：%s", msg.data)
        target_position = None

# 第一阶段：利用眼在手外获取到杯子位置然后进行抓取然后到达指定位置
def robot_grab():
    # 手动偏置量
    x1_offset = 55  # x轴方向偏置量
    y1_offset = 180   # y轴方向偏置量
    z1_offset = 100.0   # z轴方向偏置量
    global target_position, position_history,robot

    # 错误码映射表
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

    # 创建 ROS 发布器，用于发送相机切换信号
    pub_stage_status = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)

    # 启动伺服模式
    ret = robot.ServoMoveStart()
    if ret != 0:
        error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
        print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        return

    robot.SetGripperConfig(4, 0, 0, 1)
    time.sleep(2)
    robot.ActGripper(1,1)
    time.sleep(2)
    robot.MoveGripper(1, 100, 50, 30, 10000, 1)
    time.sleep(4)
    
    print("机械臂复位中...")
    desc_pos = [-90.0, -400.0, 300.0, 90.0, 0.0, 0.0]
    robot.MoveCart(desc_pos, 0, 0)
    input("按任意键...")

    try:
        while not rospy.is_shutdown():
            # 判断是否有有效的目标位置
            if target_position:
                # 计算最终目标位置（接收到的位置 + 手动偏置量）
                target_x1 = target_position[0] + x1_offset
                target_y1 = target_position[1] + y1_offset
                target_z1 = z1_offset
            elif position_history:
                # 如果没有新位置，使用最后保存的有效位置
                last_position = position_history[-1]
                target_x1 = last_position[0] + x1_offset
                target_y1 = last_position[1] + y1_offset
                target_z1 = z1_offset
                print("目标丢失，使用最后保存的位置！")
            else:
                # 如果没有历史记录，等待目标出现
                print("等待目标位置...")
                time.sleep(0.1)
                continue

            print(f"接收到的目标位置: {target_position if target_position else '无 (使用历史记录)'}")
            print(f"手动偏置量: x={x1_offset}, y={y1_offset}, z={z1_offset}")
            print(f"最终目标位置: x={target_x1}, y={target_y1}, z={target_z1}")

            
            input("按任意键...")

            # time.sleep(2)

            # 向下移动到抓取位置
            ret = robot.MoveCart([target_x1, target_y1, target_z1, 90.0, 0.0, 0.0], 0, 0)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服运动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                return
            
            # input("按任意键...")
            time.sleep(2)

            # 执行抓取动作
            print("开始抓取...")
            robot.MoveGripper(1, 40, 50, 90, 10000, 1)  # 关闭夹爪
            time.sleep(3)

            input("按任意键...")

            # 抬升到下一阶段位置
            next_position = [-90.0, -400.0, 285.0, 90.0, 0.0, 0.0]
            ret = robot.MoveCart(next_position, 0, 0)
            if ret != 0:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                print(f"伺服运动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
                return

            print("抓取完成并抬升到目标位置！")

            pub_stage_status.publish(1)  # 发送信号 1，表示第一阶段完成
            print("已发送相机切换信号，等待相机切换完成...")
            time.sleep(5)  # 等待相机切换完成

            break  # 抓取完成后退出循环
        else:
            print("等待目标位置...")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("抓取任务中断。")
    finally:
        # 停止伺服模式
        robot.ServoMoveEnd()
        print("伺服模式已停止。")

# 主函数
if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("robot_grab_node", anonymous=True)
    # 订阅目标位置话题
    rospy.Subscriber("/vision/cup_pose", String, target_position_callback)
    print("等待目标位置话题数据...")
    time.sleep(2)

    # 初始化机器人
    robot = Robot.RPC('192.168.59.6')  

    # 启动抓取流程
    robot_grab()