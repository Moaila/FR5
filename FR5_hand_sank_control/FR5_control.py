from fairino import Robot

# 与机器人控制器建立连接，连接成功返回一个机器人对象

robot = Robot.RPC('192.168.58.2')

ret,ip = robot.GetControllerIP()    #查询控制器IP

if ret ==0:

    print("控制器IP为", ip)

else:

    print("查询失败,请检查连接",ret)