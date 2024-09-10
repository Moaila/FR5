from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

P = [36.794,-675.119, 65.379, -176.938, 2.535, -179.829]
#恒力参数
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #力PID参数，力矩PID参数
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 100.0  #最大调整距离
max_ang = 5.0  #最大调整角度
#旋转插入参数
rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
forceInsertion = 2.0 #力或力矩阈值（0~100），单位N或Nm
orn = 1 #力的方向，1-fz,2-mz
#默认参数 angVelRot：旋转角速度，单位 °/s  默认 3
#默认参数 angleMax：最大旋转角度，单位 ° 默认 5
#默认参数 angAccmax：最大旋转加速度，单位 °/s^2，暂不使用 默认0
#默认参数 rotorn：旋转方向，1-顺时针，2-逆时针 默认1

error = robot.MoveL(P,1,0) #笛卡尔空间直线运动至初始点
print("笛卡尔空间直线运动至初始点",error)
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开启错误码",error)
error = robot.FT_RotInsertion(rcs,1,orn)
print("旋转插入错误码",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制关闭错误码",error)
