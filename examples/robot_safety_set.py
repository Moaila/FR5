from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
level = [1.0,2.0,3.0,4.0,5.0,6.0]
error = robot.SetAnticollision(0,level,1)
print("设置碰撞等级错误码:",error)
level = [50.0,20.0,30.0,40.0,50.0,60.0]
error = robot.SetAnticollision(1,level,1)
print("设置碰撞等级错误码:",error)
error = robot.SetCollisionStrategy(1)
print("设置碰撞后策略错误码:",error)

p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
error = robot.SetLimitPositive(p_limit)
print("设置正限位错误码:",error)
error = robot.SetLimitNegative(n_limit)
print("设置负限位错误码:",error)

error = robot.ResetAllError()
print("错误状态清除:",error)

error = robot.FrictionCompensationOnOff(1)
print("关节摩擦力补偿开关错误码:",error)
lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
wcoeff = [0.4,0.4,0.4,0.4,0.4,0.4]
ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
error = robot.SetFrictionValue_level(lcoeff)
print("设置关节摩擦力补偿系数-正装错误码:",error)
error =robot.SetFrictionValue_wall(wcoeff)
print("设置关节摩擦力补偿系数-侧装错误码:",error)
error =robot.SetFrictionValue_ceiling(ccoeff)
print("设置关节摩擦力补偿系数-倒装错误码:",error)
error =robot.SetFrictionValue_freedom(fcoeff)
print("设置关节摩擦力补偿系数-自由装错误码:",error)


robot.SetFrictionValue_level(lcoeff)
