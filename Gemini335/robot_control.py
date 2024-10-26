from fairino import Robot
import time
import camera_module

# 初始化机器人连接
robot = Robot.RPC('192.168.58.2')

# 启动伺服运动
robot.ServoMoveStart()

# 预先定义的关键位姿列表（假设为关节角度）
# 每个位置可以是关节角度的列表，也可以是笛卡尔坐标的列表
preset_positions = [
    [0, -45, 90, 0, 45, 0],   # 位姿1：关节角度
    [10, -40, 85, 0, 50, 10], # 位姿2：关节角度
    [-10, -50, 95, 0, 40, -10], # 位姿3：关节角度
    # 添加更多位姿
]

# 循环遍历每个预设位姿
for joint_pos in preset_positions:
    # 移动到该位姿
    error = robot.ServoJ(joint_pos, vel=20)  # 使用较慢速度，确保移动稳定
    if error != 0:
        print(f"伺服运动错误码：{error}")
        break

    # 稍微等待机械臂稳定
    time.sleep(0.5)

    # 采集图像（调用摄像头模块）
    color_img, depth_img = camera_module.capture_color_and_depth()
    
    # 如果需要，可以在此保存图像或提取特征点
    # 示例：保存图像到本地或进行标定点提取
    # cam_module.save_image(color_img, f"position_{preset_positions.index(joint_pos)}.png")

    # 打印当前位姿以确保记录
    print(f"完成位置 {preset_positions.index(joint_pos) + 1} 的图像采集")

# 结束伺服运动
robot.ServoMoveEnd()
