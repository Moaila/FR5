from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

error = robot.SetSpeed(20)
print("设置全局速度错误码:",error)

error_value = 0
for i in range(1,21):
    error = robot.SetSysVarValue(i,10)
    if error != 0:
        error_value =error
print("设置系统变量1-21，为10,错误码",error_value)
robot.WaitMs(1000)
for i in range(1,21):
    sys_var = robot.GetSysVarValue(i)
    print("系统变量编号:",i,"值",sys_var)

t_coord = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
robot.SetToolList(0,[0,0,0,0,0,0],0,0)#设置参考点前应当将工具和工件号坐标系切换至0
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1, 7):
    robot.DragTeachSwitch(1)
    error = robot.SetToolPoint(i)  # 实际应当控制机器人按照要求移动到合适位置后再发送指令
    print("六点法设置工具坐标系，记录点", i, "错误码", error)
    time.sleep(1)
    robot.DragTeachSwitch(0)
    time.sleep(1)
error, t_coord = robot.ComputeTool()
print("六点法计算工具坐标系错误码", error, "工具TCP", t_coord)

robot.SetToolList(0,[0,0,0,0,0,0],0,0)#设置参考点前应当将工具和工件号坐标系切换至0
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1, 5):
    robot.DragTeachSwitch(1)  # 切入拖动示教模式
    time.sleep(1)
    error = robot.SetTcp4RefPoint(i)  # 应当控制机器人按照要求移动到合适位置后再发送指令
    print("四点法设置工具坐标系，记录点", i, "错误码", error)
    robot.DragTeachSwitch(0)
    time.sleep(1)
error, t_coord = robot.ComputeTcp4()
print("四点法设置工具坐标系错误码", error, "工具TCP", t_coord)

error = robot.SetToolCoord(10,t_coord,0,0)
print("设置工具坐标系错误码",error)
error = robot.SetToolList(10,t_coord,0,0)
print("设置工具坐标系列表错误码",error)

etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
etool = [21.0,22.0,23.0,24.0,25.0,26.0]
robot.SetToolList(0,[0,0,0,0,0,0],0,0)#设置参考点前应当将工具和工件号坐标系切换至0
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1,4):
    error = robot.SetExTCPPoint(i) #应当控制机器人按照要求移动到合适位置后再发送指令
    print("三点法设置外部工具坐标系，记录点",i,"错误码",error)
    time.sleep(1)
error,etcp = robot.ComputeExTCF()
print("三点法设置外部工具坐标系错误码",error,"外部工具TCP",etcp)
error = robot.SetExToolCoord(10,etcp,etool)
print("设置外部工具坐标系错误码",error)
error = robot.SetExToolList(10,etcp,etool)
print("设置外部工具坐标系列表错误码",error)

w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
robot.SetToolList(0,[0,0,0,0,0,0],0,0)#设置参考点前应当将工具和工件号坐标系切换至0
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1,4):
    error = robot.SetWObjCoordPoint(i) #实际应当控制机器人按照要求移动到合适位置后再发送指令
    print("三点法设置工件坐标系，记录点",i,"错误码",error)
    time.sleep(1)
error, w_coord = robot.ComputeWObjCoord(1)
print("三点法计算工件坐标系错误码",error,"工件坐标系", w_coord)

error = robot.SetWObjCoord(11,w_coord)
print("设置工件坐标系错误码",error)
error = robot.SetWObjList(11,w_coord)
print("设置工件坐标系列表错误码",error)

error = robot.SetLoadWeight(0)#！！！负载重量设置应于实际相符(错误负载重量设置可能会导致拖动模式下机器人失控)
print("设置负载重量错误码",error)
error = robot.SetLoadCoord(3.0,4.0,5.0)
print("设置负载质心错误码",error)#！！！负载质心设置应于实际相符(错误负载质心设置可能会导致拖动模式下机器人失控)

error = robot.SetRobotInstallPos(0) #！！！安装方式设置应与实际一致 0-正装，1-侧装，2-倒装 (错误安装方式设置会导致拖动模式下机器人失控）
print("设置机器人安装方式错误码",error)
error = robot.SetRobotInstallAngle(0.0,0.0) #！！！安装角度设置应与实际一致 (错误安装角度设置会导致拖动模式下机器人失控）
print("设置机器人安装角度错误码",error)
