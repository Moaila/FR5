import time
import numpy as np
from fairino import Robot

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

# 初始化机器人
robot = Robot.RPC('192.168.59.6')
ret = robot.ServoMoveStart()
if ret != 0:
    error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
    print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
    exit()

# 定义移动参数保护安全
max_linear_speed = 50
max_angular_speed = 1

# 定义时间间隔
dt = 0.05

# 定义移动步长
linear_step = 10
angular_step = np.deg2rad(10)

# 初始位姿
current_pose = robot.GetActualTCPPose()
ret_code, pose = current_pose

if ret_code == 0 and isinstance(pose, list) and len(pose) == 6:
    x, y, z, rx, ry, rz = pose
else:
    print("无法获取初始位姿，请检查机械臂状态。")
    robot.ServoMoveEnd()
    exit()

print(f"当前机械臂初始位姿: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")

# 主循环
try:
    while True:
        print("\n请输入移动方向：")
        print("1 - 向左移动")
        print("2 - 向右移动")
        print("3 - 向下移动")
        print("4 - 向上移动")
        print("5 - 旋转末端（顺时针或逆时针随机旋转10度）")
        direction = input("方向 (1-5): ").strip()

        if direction not in ["1", "2", "3", "4", "5"]:
            print("无效输入，请输入数字 1-5。")
            continue

        # 根据输入方向计算新的目标位置或姿态
        if direction == "1":  # 向左移动
            x -= linear_step
        elif direction == "2":  # 向右移动
            x += linear_step
        elif direction == "3":  # 向下移动
            z -= linear_step
        elif direction == "4":  # 向上移动
            z += linear_step
        elif direction == "5":  # 旋转末端
            random_choice = np.random.choice([-1, 1])  # 随机顺时针或逆时针
            rz += random_choice * angular_step

        # 使用伺服模式更新位姿
        pos_gain = [1.0] * 6  # 位姿增量增益
        ret = robot.ServoCart(0, [x, y, z, rx, ry, rz], pos_gain)
        if ret != 0:
            error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
            print(f"伺服更新失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
            break

        print(f"机械臂已移动到新位置: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")

        # 可加入其他操作，比如拍照或数据记录
        # robot.capture_image()

        # 休眠一小段时间，模拟实时控制
        time.sleep(dt)

except KeyboardInterrupt:
    print("控制结束。")

finally:
    # 停止伺服运动
    robot.ServoMoveEnd()
    print("伺服模式已关闭。")


