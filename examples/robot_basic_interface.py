from fairino import Robot
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
print("与机器人控制器建立连接",robot)

ret,version  = robot.GetSDKVersion()    #查询SDK版本号
if ret ==0:
    print("SDK版本号为", version )
else:
    print("查询失败，错误码为",ret)

ret,ip = robot.GetControllerIP()    #查询控制器IP
if ret ==0:
    print("控制器IP为", ip)
else:
    print("查询失败，错误码为",ret)

#机器人进入或退出拖动示教模式
ret = robot.Mode(1) #机器人切入手动模式
print("机器人切入手动模式", ret)
time.sleep(1)
ret = robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
print("机器人切入拖动示教模式", ret)
time.sleep(1)
ret,state = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
if ret == 0:
    print("当前拖动示教模式状态：", state)
else:
    print("查询失败，错误码为：",ret)
time.sleep(3)
ret = robot.DragTeachSwitch(0)  #机器人切入非拖动示教模式，必须在手动模式下才能切入非拖动示教模式
print("机器人切入非拖动示教模式", ret)
time.sleep(1)
ret,state = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
if ret == 0:
    print("当前拖动示教模式状态：", state)
else:
    print("查询失败，错误码为：",ret)
time.sleep(3)

#机器人上使能或下使能
ret = robot.RobotEnable(0.5)   #机器人下使能
print("机器人下使能", ret)
time.sleep(3)
ret = robot.RobotEnable(1)   #机器人上使能，机器人上电后默认自动上使能
print("机器人上使能", ret)



#机器人手自动模式切换
ret = robot.Mode(0)   #机器人切入自动运行模式
print("机器人切入自动运行模式", ret)
time.sleep(1)
ret = robot.Mode(1)   #机器人切入手动模式
print("机器人切入手动模式", ret)
