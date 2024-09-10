from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

tool = 0 #工具坐标系编号
user = 0 #工件坐标系编号
lastFlag= 0 # 是否为最后一个点，0-否，1-是

joint_pos1 = [116.489,-85.278,111.501,-112.486,-85.561,24.693]
joint_pos2 = [86.489,-65.278,101.501,-112.486,-85.561,24.693]
joint_pos3 = [116.489,-45.278,91.501,-82.486,-85.561,24.693]

desc_pos4 = [236.794,-375.119, 65.379, -176.938, 2.535, -179.829]
desc_pos5 = [236.794,-275.119, 165.379, -176.938, 2.535, -179.829]
desc_pos6 = [286.794,-375.119, 265.379, -176.938, 2.535, -179.829]

ret = robot.SplineStart() #样条运动开始
print("样条运动开始:错误码", ret)
ret = robot.SplinePTP(joint_pos1, tool, user)   #样条运动PTP
print("样条运动PTP运动点1:错误码", ret)
ret = robot.SplinePTP(joint_pos2, tool, user)   #样条运动PTP
print("样条运动PTP运动点2:错误码", ret)
ret = robot.SplinePTP(joint_pos3, tool, user)   #样条运动PTP
print("样条运动PTP运动点3:错误码", ret)
ret = robot.SplineEnd() #样条运动结束
print("样条运动结束:错误码", ret)


ret = robot.NewSplineStart(1) #新样条运动开始
print("新样条运动开始:错误码", ret)
ret = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag)#新样条指令点
print("新样条指令点4:错误码", ret)
ret = robot.NewSplinePoint(desc_pos5, tool, user, lastFlag, vel=30)#新样条指令点
print("新样条指令点5:错误码", ret)
lastFlag = 1
ret = robot.NewSplinePoint(desc_pos6, tool, user, lastFlag, vel=30)#新样条指令点
print("新样条指令点6:错误码", ret)
ret = robot.NewSplineEnd() #新样条运动结束
print("新样条运动结束:错误码", ret)