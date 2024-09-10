from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#传送带跟踪抓取
robot.MoveL([-333.597, 60.354, 404.341, -179.143, -0.778, 91.275],0,0)
error =robot.ConveyorIODetect(1000)
print("传送带工件IO检测错误码",error)
error =robot.ConveyorGetTrackData(1)
print("获取物体当前位置错误码",error)
error =robot.ConveyorTrackStart(1)
print("传动带跟踪开始错误码",error)
error =robot.ConveyorTrackMoveL("cvrCatchPoint",0,0,vel = 60.0)
print("直线运动错误码",error)
error =robot.MoveGripper(1,55,20,20,30000,0)
print("夹爪控制错误码",error)
error =robot.ConveyorTrackMoveL("cvrRaisePoint",0,0,vel = 60.0)
print("直线运动错误码",error)
error = robot.ConveyorTrackEnd()
print("传动带跟踪结束错误码错误码",error)
error = robot.MoveL([-333.625, -229.039, 404.340, -179.141, -0.778, 91.276], 0, 0,vel =30)
error = robot.MoveL([-333.564, 332.204, 342.217, -179.145, -0.780, 91.268], 0, 0,vel =30)
error = robot.MoveGripper(1,100,10,21,30000,0)
print("夹爪控制错误码",error)


