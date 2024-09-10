from fairino import Robot

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

ret = robot.GetSoftwareVersion()
print("GetSoftwareVersion()：", ret)
ret = robot.GetSlaveHardVersion()
print("GetSlaveHardVersion()：", ret)
ret = robot.GetSlaveFirmVersion()
print("GetSlaveFirmVersion()：", ret[0])
print("GetSlaveFirmVersion()：driver1", ret[1])
print("GetSlaveFirmVersion()：driver2", ret[2])
print("GetSlaveFirmVersion()：driver3", ret[3])
print("GetSlaveFirmVersion()：driver4", ret[4])
print("GetSlaveFirmVersion()：driver5", ret[5])
print("GetSlaveFirmVersion()：driver6", ret[6])
print("GetSlaveFirmVersion()：end", ret[7])
