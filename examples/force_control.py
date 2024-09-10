from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

#恒力控制
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]
gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #力PID参数，力矩PID参数
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 100.0  #最大调整距离
max_ang = 0.0  #最大调整角度
J1=[70.395, -46.976, 90.712, -133.442, -87.076, -27.138]
P2=[-123.978, -674.129, 44.308, -178.921, 2.734, -172.449]
P3=[123.978, -674.129, 42.308, -178.921, 2.734, -172.449]
error = robot.MoveJ(J1,1,0)
print("关节空间运动指令错误码",error)
error = robot.MoveL(P2,1,0)
print("笛卡尔空间直线运动指令错误码",error)
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开启错误码",error)
error = robot.MoveL(P3,1,0)   #笛卡尔空间直线运动
print("笛卡尔空间直线运动指令错误码",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制结束错误码",error)