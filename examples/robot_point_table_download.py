from fairino import Robot

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
error = robot.PointTableDownLoad("point_table_a.db","/home/fr/RD06/SDK1/test/download/")
print("PointTableDownLoad错误码:",error)

