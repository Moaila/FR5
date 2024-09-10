from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#碰撞守护
actFlag = 1   #开启标志，0-关闭碰撞守护，1-开启碰撞守护
sensor_num = 1  #力传感器编号
is_select = [1,1,1,1,1,1]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,0.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
max_threshold = [10.0,10.0,10.0,10.0,10.0,10.0]  #最大阈值
min_threshold = [5.0,5.0,5.0,5.0,5.0,5.0]   #最小阈值
P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    #开启碰撞守护
print("开启碰撞守护错误码",error)
error = robot.MoveL(P1,1,0)         #笛卡尔空间直线运动
print("笛卡尔空间直线运动错误码",error)
error = robot.MoveL(P2,1,0)
print("笛卡尔空间直线运动错误码",error)
error = robot.MoveL(P3,1,0)
print("笛卡尔空间直线运动错误码",error)
actFlag = 0  
error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    #关闭碰撞守护
print("关闭碰撞守护错误码",error)