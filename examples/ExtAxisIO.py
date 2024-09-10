from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#设置扩展DO
error = robot.SetAuxDO(1,True,False,True)
print("GetAuxAI",error)
#设置扩展AO
error = robot.SetAuxAO(1,60,True)
print("SetAuxAO",error)
#设置扩展DI输入滤波时间
error = robot.SetAuxDIFilterTime(10)
print("SetAuxDIFilterTime",error)
#设置扩展AI输入滤波时间
error = robot.SetAuxAIFilterTime(0,10)
print("SetAuxAIFilterTime",error)
#等待扩展DI输入
error = robot.WaitAuxDI(0,False,100,False)
print("WaitAuxDI",error)
#等待扩展AI输入
error = robot.WaitAuxAI(0,0,100,500,False)
print("WaitAuxAI",error)
#获取扩展AI值
error = robot.GetAuxAI(0,False)
print("GetAuxAI",error)
#获取扩展DI值
error = robot.GetAuxDI(0,True)
print("GetAuxDI",error)
