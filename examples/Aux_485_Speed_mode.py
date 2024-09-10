from time import sleep
from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
ret = robot = Robot.RPC('192.168.58.2')
ret =robot.AuxServoEnable(1,0)#修改控制模式前需去使能
print("AuxServoEnable(0)",ret)
sleep(3)

ret = robot.AuxServoSetControlMode(1, 1)  # 设置为速度模式
print("AuxServoSetControlMode",ret)
sleep(3)

ret =robot.AuxServoEnable(1,1)#修改控制模式后需使能
print("AuxServoEnable(1)",ret)
sleep(3)

ret =robot.AuxServoHoming(1,1,10,10)#回零
print("AuxServoHoming",ret)
sleep(5)

ret =robot.AuxServoGetStatus(1)#查询状态
print("AuxServoGetStatus",ret)
sleep(1)

ret = robot.AuxServoSetTargetSpeed(1, 30)  # 速度模式运动，速度30
print("AuxServoSetTargetSpeed", ret)
sleep(10)

ret = robot.AuxServoGetStatus(1)  # 查询状态
print("AuxServoGetStatus", ret)
sleep(1)

ret = robot.AuxServoSetTargetSpeed(1, 60)  # 速度模式运动，速度60
print("AuxServoSetTargetSpeed", ret)
sleep(10)
ret = robot.AuxServoGetStatus(1)  # 查询状态
print("AuxServoGetStatus", ret)
sleep(1)

ret = robot.AuxServoSetTargetSpeed(1, 0)  # 结束速度模式运动前应当把速度设为0
print("AuxServoSetTargetSpeed", ret)
sleep(3)
ret = robot.AuxServoGetStatus(1)  # 查询状态
print("AuxServoGetStatus", ret)
sleep(1)