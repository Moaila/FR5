from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
# P1=[-321.821, 125.694, 282.556, 174.106, -15.599, 152.669]
name = 'tpd2023'   #轨迹名
blend = 1   #是否平滑，1-平滑，0-不平滑
ovl = 100.0   #速度缩放
ret = robot.LoadTPD(name)  #轨迹预加载
print("轨迹预加载错误码",ret)
ret,P1 = robot.GetTPDStartPose(name)   #获取轨迹起始位姿
print ("获取轨迹起始位姿错误码",ret,"起始位姿",P1)
ret = robot.MoveL(P1,0,0)       #运动到起始点
print("运动到起始点错误码",ret)
time.sleep(10)
ret = robot.MoveTPD(name, blend, ovl)  #轨迹复现
print("轨迹复现错误码",ret)
