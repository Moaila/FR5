from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#传送带运动，1-运动，0-停止
status = 1
robot.ConveyorStartEnd(status)
#点记录
ret = robot.ConveyorPointIORecord()
print("记录IO检测点",ret)
ret = robot.ConveyorPointARecord()
print("记录A点",ret)
ret = robot.ConveyorRefPointRecord()
print("记录参考点",ret)
ret = robot.ConveyorPointBRecord()
print("记录B点",ret)



