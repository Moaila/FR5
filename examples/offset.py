from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

desc_pos1 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
desc_pos2 = [-187.519, 310.248, 297, -157.278, -31.188, 107.199]
desc_pos3 = [-127.519, 256.248, 312, -147.278, -51.588, 107.199]
joint_pos1 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
desc_pos4 = [-140.519, 219.248, 300, -137.278, -11.188, 127.199]
desc_pos5 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
desc_pos6 = [-207.519, 229.248, 347, -157.278, -31.188, 107.199]

tool = 0 #工具坐标系编号
user = 0 #工件坐标系编号
flag = 1  #0-基坐标系下/工件坐标系下偏移，2-工具坐标系下偏移
offset_pos = [10,20,30,0,0,0]  #位姿偏移量

ret = robot.MoveL(desc_pos1, tool, user, joint_pos=joint_pos1)   #笛卡尔空间直线运动
print("笛卡尔空间直线运动点1:错误码", ret)
ret = robot.StopMotion()  #终止运动
print("终止运动:错误码", ret)

robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
print("笛卡尔空间直线运动点2:错误码", ret)

ret = robot.PointsOffsetEnable(flag,offset_pos)
print("点位整体偏移开始:错误码", ret)

robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
print("笛卡尔空间直线运动点3:错误码", ret)

robot.MoveL(desc_pos4, tool, user, vel=30, acc=100)
print("笛卡尔空间直线运动点4:错误码", ret)

robot.MoveL(desc_pos5, tool, user)
print("笛卡尔空间直线运动点5:错误码", ret)
ret = robot.PointsOffsetDisable()
print("点位整体偏移结束:错误码", ret)