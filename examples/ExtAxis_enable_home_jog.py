from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#UDP扩展轴去使能
error = robot.ExtAxisServoOn(1,0)
print("ExtAxisServoOn return:",error)
#UDP扩展轴使能
error = robot.ExtAxisServoOn(1,1)
print("ExtAxisServoOn return:",error)
#UDP扩展轴回零
error = robot.ExtAxisSetHoming(1,0,40,40)
print("ExtAxisSetHoming return:",error)
time.sleep(1)
#UDP扩展轴点动开始
error = robot.ExtAxisStartJog(1,1,20,20,20)
print("ExtAxisStartJog return:",error)
time.sleep(1)
#UDP扩展轴点动停止
error = robot.ExtAxisStopJog(1)
print("ExtAxisStopJog return:",error)