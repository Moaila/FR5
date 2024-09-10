from time import sleep
from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
ret = robot = Robot.RPC('192.168.58.2')

ret =robot.SetExDevProtocol(4098)
print("SetExDevProtocol",ret)

ret =robot.GetExDevProtocol()
print("GetExDevProtocol",ret)

ret =robot.AuxServoSetParam(1,1,1,1,131072,15.45)#设置485扩展轴参数
print("AuxServoSetParam",ret)
sleep(1)

ret =robot.AuxServoGetParam(1)#获取485扩展轴配置参数
print("AuxServoGetParam",ret)
sleep(1)

ret =robot.AuxServoGetStatus(1)#查询状态
print("AuxServoGetStatus",ret)
sleep(1)

ret =robot.AuxServoClearError(1)#清除错误
print("AuxServoClearError",ret)
sleep(1)