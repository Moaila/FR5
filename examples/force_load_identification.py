from fairino import Robot

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

#负载辨识滤波初始化
error = robot.LoadIdentifyDynFilterInit()
print("LoadIdentifyDynFilterInit:",error)
#负载辨识变量初始化
error = robot.LoadIdentifyDynVarInit()
print("LoadIdentifyDynVarInit:",error)

joint_torque= [0,0,0,0,0,0]
joint_pos= [0,0,0,0,0,0]
gain=[0,0.05,0,0,0,0,0,0.02,0,0,0,0]
t =10
error,joint_pos=robot.GetActualJointPosDegree(1)
joint_pos[1] = joint_pos[1]+10
error,joint_torque=robot.GetJointTorques(1)
joint_torque[1] = joint_torque[1]+2
#负载辨识主程序
error = robot.LoadIdentifyMain(joint_torque, joint_pos, t)
print("LoadIdentifyMain:",error)
#获取负载辨识结果
error = robot.LoadIdentifyGetResult(gain)
print("LoadIdentifyGetResult:",error)

