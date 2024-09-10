from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
desc_pos1=[-333.683,-228.968,404.329,-179.138,-0.781,91.261]
desc_pos2=[-333.683,-100.8,404.329,-179.138,-0.781,91.261]
zlength1 =10
zlength2 =15
zangle1 =10
zangle2 =15

#测试外设指令
ret = robot.SetGripperConfig(4,0)  #配置夹爪
print("配置夹爪错误码", ret)
time.sleep(1)
config = robot.GetGripperConfig()     #获取夹爪配置
print("获取夹爪配置",config)
error = robot.ActGripper(1,0)  #激活夹爪
print("激活夹爪错误码",error)
time.sleep(1)
error = robot.ActGripper(1,1)#激活夹爪
print("激活夹爪错误码",error)
time.sleep(2)
error = robot.MoveGripper(1,100,48,46,30000,0) #控制夹爪
print("控制夹爪错误码",error)
time.sleep(3)
error = robot.MoveGripper(1,0,50,0,30000,0) #控制夹爪
print("控制夹爪错误码",error)
time.sleep(5)
error = robot.GetGripperMotionDone() #获取夹爪运动状态
print("获取夹爪运动状态错误码",error)

error = robot.ComputePrePick(desc_pos1, zlength1, zangle1) #计算预抓取点-视觉
print("计算预抓取点",error)
error = robot.ComputePrePick(desc_pos2, zlength2, zangle2) #计算撤退点-视觉
print("计算撤退点",error)