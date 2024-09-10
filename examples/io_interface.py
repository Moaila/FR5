from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

# 设置控制箱DO
for i in range(0,16):
    error = robot.SetDO(i,1)      #打开控制箱DO
error = robot.WaitMs(1000)
print("WaitMs错误码:",error)
for i in range(0,16):
    robot.SetDO(i,0)      #关闭控制箱DO
robot.WaitMs(1000)

# 设置工具DO
error_tooldo = 0
for i in range(0,2):
    error = robot.SetToolDO(i,1)    #打开工具DO
robot.WaitMs(1000)
for i in range(0,2):
    error = robot.SetToolDO(i,0)    #关闭工具DO

# 设置控制箱AO
error = robot.SetAO(0,100.0)
print("设置AO0错误码:", error)
error = robot.SetAO(1,100.0)
print("设置AO1错误码:", error)

# 设置工具AO
error = robot.SetToolAO(0,100.0)
print("设置ToolAO0错误码:", error)
robot.WaitMs(1000)
error = robot.SetToolAO(0,0.0)
print("设置ToolAO0错误码:", error)

# 获取控制箱DI
error = robot.GetDI(0,0)
print("获取DI0",error)

# 获取工具DI
tool_di = robot.GetToolDI(1,0)
print("获取ToolDI",tool_di)

# 获取控制箱AI
error = robot.GetAI(0)
print("获取控制箱AI0",error)

# 获取工具AI
error = robot.GetToolAI(0)
print("获取ToolAI0",error)


max_waittime = 2000
#等待控制箱DI
error = robot.WaitDI(0,1,max_waittime,0)
print("WaitDI错误码",error)
#等待控制箱多路DI
error = robot.WaitMultiDI(1,3,1,max_waittime,0)
print("WaitMultiDI错误码",error)
#等待工具DI
error = robot.WaitToolDI(1,1,max_waittime,0)
print("WaitToolDI错误码",error)


#等待控制箱AI
error = robot.WaitAI(0,0,50,max_waittime,1)         #忽略超时提示程序继续执行
print("WaitAI错误码",error)
#等待工具AI
error = robot.WaitToolAI(0,0,50,max_waittime,0)
print("WaitToolAI错误码",error)



