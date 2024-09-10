from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

company = 17    #传感器厂商，17-坤维科技
device = 0      #传感器设备号

error = robot.FT_SetConfig(company, device)   #配置力传感器
print("配置力传感器错误码",error)
config = robot.FT_GetConfig() #获取力传感器配置信息，厂商编号下发比反馈大1
print('获取力传感器配置信息',config)
time.sleep(1)
error = robot.FT_Activate(0)  #传感器复位
print("传感器复位错误码",error)
time.sleep(1)
error = robot.FT_Activate(1)  #传感器激活
print("传感器激活错误码",error)
time.sleep(1)
error = robot.SetLoadWeight(0.0)    #末端负载设置为零
print("末端负载设置为零错误码",error)
time.sleep(1)
error = robot.SetLoadCoord(0.0,0.0,0.0)  #末端负载质心设置为零
print("末端质心设置为零错误码",error)
time.sleep(1)
error = robot.FT_SetZero(0)   #传感器去除零点
print("传感器去除零点错误码",error)
time.sleep(1)
error = robot.FT_GetForceTorqueOrigin()   #查询传感器原始数据
print("查询传感器原始数据",error)
error = robot.FT_SetZero(1)   #传感器零点矫正,注意此时末端不能安装工具，只有力传感器
print("传感器零点矫正",error)
time.sleep(1)
error = robot.FT_GetForceTorqueRCS()  #查询传感器坐标系下数据
print("查询传感器坐标系下数据",error)

