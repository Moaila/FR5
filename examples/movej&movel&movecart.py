from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

desc_pos1 = [36.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos2 = [136.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos3 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
joint_pos4 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
joint_pos5 = [-43.24, -70.476, 93.688, -114.079, -62, -80]
joint_pos6 = [-83.24, -96.416, 43.188, -74.079, -80, -10]

desc_pos7 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos8 = [236.794,-575.119, 165.379, -176.938, 2.535, -179.829]
desc_pos9 = [236.794,-475.119, 265.379, -176.938, 2.535, -179.829]
tool = 0 #工具坐标系编号
user = 0 #工件坐标系编号

ret = robot.MoveL(desc_pos1, tool, user)   #笛卡尔空间直线运动
print("笛卡尔空间直线运动点1:错误码", ret)

robot.MoveL(desc_pos2, tool, user, vel=30, acc=100)
print("笛卡尔空间直线运动点2:错误码", ret)

robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
print("笛卡尔空间直线运动点3:错误码", ret)

ret = robot.MoveJ(joint_pos4, tool, user, vel=30)   #关节空间运动
print("关节空间运动点4:错误码", ret)

ret = robot.MoveJ(joint_pos5, tool, user)
print("关节空间运动点5:错误码", ret)

robot.MoveJ(joint_pos6, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
print("关节空间运动点6:错误码", ret)

robot.MoveCart(desc_pos7, tool, user)
print("笛卡尔空间点到点运动点7:错误码", ret)

robot.MoveCart(desc_pos8, tool, user, vel=30)
print("笛卡尔空间点到点运动点8:错误码", ret)

robot.MoveCart(desc_pos9, tool, user,)
print("笛卡尔空间点到点运动点9:错误码", ret)