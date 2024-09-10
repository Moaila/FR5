from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

type = 1  # 数据类型，1-关节位置
name = 'tpd2023'  # 轨迹名
period = 4  #采样周期，2ms或4ms或8ms
di = 0 # di输入配置
do = 0 # do输出配置
ret = robot.SetTPDParam(name, period, di_choose=di)    #配置TPD参数
print("配置TPD参数错误码", ret)

robot.Mode(1)  # 机器人切入手动模式

time.sleep(1)  
robot.DragTeachSwitch(1)  #机器人切入拖动示教模式
ret = robot.GetActualTCPPose()
print("获取当前工具位姿", ret)
time.sleep(1)
ret = robot.SetTPDStart(name, period, do_choose=do)   # 开始记录示教轨迹
print("开始记录示教轨迹错误码", ret)
time.sleep(15)
ret = robot.SetWebTPDStop()  # 停止记录示教轨迹
print("停止记录示教轨迹错误码", ret)
robot.DragTeachSwitch(0)  #机器人切入非拖动示教模式

# robot.SetTPDDelete('tpd2023')   # 删除TPD轨迹
