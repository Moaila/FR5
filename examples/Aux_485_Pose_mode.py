from time import sleep
from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
ret = robot = Robot.RPC('192.168.58.2')
ret =robot.AuxServoEnable(1,0)#修改控制模式前需去使能
print("AuxServoEnable(0)",ret)
sleep(3)

ret =robot.AuxServoSetControlMode(1,0)#设置为位置模式
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
i=1
while(i<5):
    ret =robot.AuxServoSetTargetPos(1,300*i,30)#位置模式运动，速度30
    print("AuxServoSetTargetPos",ret)
    sleep(11)
    ret =robot.AuxServoGetStatus(1)#查询状态
    print("AuxServoGetStatus",ret)
    sleep(1)
    i=i+1

