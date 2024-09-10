from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

P = [36.794,-675.119, 65.379, -176.938, 2.535, -179.829]
#恒力参数
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]
gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #力PID参数，力矩PID参数
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 100.0  #最大调整距离
max_ang = 5.0  #最大调整角度
#直线插入参数
rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
force_goal = 10.0  #力或力矩阈值（0~100），单位N或Nm
disMax = 100.0 #最大插入距离，单位mm
linorn = 1 #插入方向，1-正方向，2-负方向
#默认参数 lin_v：直线速度，单位 mm/s 默认1
#默认参数 lin_a：直线加速度，单位 mm/s^2，暂不使用 默认0
error = robot.MoveL(P,1,0) #笛卡尔空间直线运动至初始点
print("笛卡尔空间直线运动至初始点",error)
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开启错误码",error)
error = robot.FT_LinInsertion(rcs,force_goal,disMax,linorn)
print("直线插入错误码",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制关闭错误码",error)