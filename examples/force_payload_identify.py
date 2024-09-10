from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

#负载辨识，此时末端安装要辨识的工具，工具安装在力传感器下方,末端竖直向下
error = robot.FT_SetRCS(0)    #设置参考坐标系为工具坐标系，0-工具坐标系，1-基坐标系
print('设置参考坐标系错误码',error)
time.sleep(1)
tool_id = 10  #传感器坐标系编号
tool_coord = [0.0,0.0,35.0,0.0,0.0,0.0]   # 传感器相对末端法兰位姿
tool_type = 1  # 0-工具，1-传感器
tool_install = 0 # 0-安装末端，1-机器人外部
error = robot.SetToolCoord(tool_id,tool_coord,tool_type,tool_install)     #设置传感器坐标系，传感器相对末端法兰位姿
print('设置传感器坐标系错误码',error)
time.sleep(1)
error = robot.FT_PdIdenRecord(tool_id)   #记录辨识数据
print('记录负载重量错误码',error)
time.sleep(1)
error = robot.FT_PdIdenRecord()  #计算负载重量，单位kg
print('计算负载重量错误码',error)

#负载质心辨识，机器人需要示教三个不同的姿态，然后记录辨识数据，最后计算负载质心
robot.Mode(1)
ret = robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
time.sleep(5)
ret = robot.DragTeachSwitch(0)
time.sleep(1)
error = robot.FT_PdCogIdenRecord(tool_id,1)
print('负载质心1错误码',error)#记录辨识数据
ret = robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
time.sleep(5)
ret = robot.DragTeachSwitch(0)
time.sleep(1)
error = robot.FT_PdCogIdenRecord(tool_id,2)
print('负载质心2错误码',error)
ret = robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
time.sleep(5)
ret = robot.DragTeachSwitch(0)
time.sleep(1)
error = robot.FT_PdCogIdenRecord(tool_id,3)
print('负载质心3错误码',error)
time.sleep(1)
error = robot.FT_PdCogIdenCompute()
print('负载质心计算错误码',error)


