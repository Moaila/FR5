from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

error,joint_pos = robot.GetActualJointPosDegree()
print("机器人当前关节位置",joint_pos)
joint_pos = [joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]]
error_joint = 0
count =100
error = robot.ServoMoveStart()  #伺服运动开始
print("伺服运动开始错误码",error)
while(count):
    error = robot.ServoJ(joint_pos)   #关节空间伺服模式运动
    if error!=0:
        error_joint =error
    joint_pos[0] = joint_pos[0] + 0.1  #每次1轴运动0.1度，运动100次
    count = count - 1
    time.sleep(0.008)
print("关节空间伺服模式运动错误码",error_joint)
error = robot.ServoMoveEnd()  #伺服运动结束
print("伺服运动结束错误码",error)
mode = 2  #[0]-绝对运动(基坐标系)，[1]-增量运动(基坐标系)，[2]-增量运动(工具坐标系)
n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   #笛卡尔空间位姿增量
error,desc_pos = robot.GetActualTCPPose()
print("机器人当前笛卡尔位置",desc_pos)
count = 100
error_cart =0
error = robot.ServoMoveStart()  #伺服运动开始
print("伺服运动开始错误码",error)
while(count):
    error = robot.ServoCart(mode, n_pos, vel=40)   #笛卡尔空间伺服模式运动
    if error!=0:
        error_cart =error
    count = count - 1
    time.sleep(0.008)
print("笛卡尔空间伺服模式运动错误码", error_cart)
error = robot.ServoMoveEnd()  #伺服运动开始
print("伺服运动结束错误码",error)

print("关节扭矩控制错误码",error_torques)
