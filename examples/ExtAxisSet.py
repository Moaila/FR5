from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#UDP扩展轴通讯参数配置
error = robot.ExtDevSetUDPComParam('192.168.58.88',2021,2,50,5,50,1,2,5)
print("ExtDevSetUDPComParam return:",error)
#UDP扩展轴通讯参数配置
error = robot.ExtDevGetUDPComParam()
print("ExtDevGetUDPComParam return:",error)

#加载UDP通信
error = robot.ExtDevLoadUDPDriver()
print("ExtDevLoadUDPDriver return:",error)
#卸载UDP通信
error = robot.ExtDevUnloadUDPDriver()
print("ExtDevUnloadUDPDriver return:",error)
#加载UDP通信
error = robot.ExtDevLoadUDPDriver()
print("ExtDevLoadUDPDriver return:",error)

#UDP扩展轴通信异常断开后恢复连接
error = robot.ExtDevUDPClientComReset()
print("ExtDevUDPClientComReset return:",error)
#UDP扩展轴通信异常断开后关闭通讯
error = robot.ExtDevUDPClientComClose()
print("ExtDevUDPClientComClose return:",error)

#设置扩展机器人相对扩展轴位置
error = robot.SetRobotPosToAxis(1)
print("SetRobotPosToAxis return:",error)
#设置扩展轴系统DH参数配置
error = robot.SetAxisDHParaConfig(4,128.5,206.4,0,0,0,0,0,0,)
print("SetAxisDHParaConfig return:",error)
#UDP扩展轴参数配置
error = robot.ExtAxisParamConfig(1,1,0,1000,-1000,1000,1000,1.905,262144,
                                 200,1,1,0)
print("ExtAxisParamConfig return:",error)



































