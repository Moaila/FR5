from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

name = "/fruser/traj/trajHelix_aima_1.txt"   #轨迹名
blend = 1   #是否平滑，1-平滑，0-不平滑
ovl = 50.0   #速度缩放
ft =[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
ret = robot.LoadTrajectoryJ(name,ovl)  #轨迹预加载
print("轨迹预加载错误码",ret)
ret,P1 = robot.GetTrajectoryStartPose(name)   #获取轨迹起始位姿
print ("获取轨迹起始位姿错误码",ret,"起始位姿",P1)
ret = robot.MoveL(P1,1,0)       #运动到起始点
print("运动到起始点错误码",ret)
ret = robot.GetTrajectoryPointNum()       #获取轨迹点编号
print("获取轨迹点编号错误码",ret)
time.sleep(10)
ret = robot.MoveTrajectoryJ()  #轨迹复现
print("轨迹复现错误码",ret)
time.sleep(10)
ret = robot.SetTrajectoryJSpeed(ovl)  #设置轨迹运行中的速度
print("设置轨迹运行中的速度错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceTorque(ft)  #设置轨迹运行中的力和扭矩
print("设置轨迹运行中的力和扭矩错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceFx(0) #设置轨迹运行中的沿x方向的力
print("设置轨迹运行中的沿x方向的力错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceFy(0) #设置轨迹运行中的沿y方向的力
print("设置轨迹运行中的沿y方向的力错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceFz(0) #设置轨迹运行中的沿z方向的力
print("设置轨迹运行中的沿z方向的力错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJTorqueTx(0) #设置轨迹运行中的绕x轴的扭矩
print("设置轨迹运行中的绕x轴的扭矩错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJTorqueTy(0) #设置轨迹运行中的绕y轴的扭矩
print("设置轨迹运行中的绕y轴的扭矩错误码",ret)
time.sleep(1)
ret = robot.SetTrajectoryJTorqueTz(0) #设置轨迹运行中的绕z轴的扭矩
print("设置轨迹运行中的绕z轴的扭矩错误码",ret)
time.sleep(1)