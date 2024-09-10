from fairino import Robot

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
error = robot.PointTableUpLoad("/home/fr/RD06/SDK1/test/point_table_a.db")
print("PointTableUpLoad错误码:",error)
