from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

J1=[75.005,-46.434,90.687,-133.708,-90.315,-27.139]
P2=[-77.24,-679.599,38.328,179.373,-0.028,-167.849]
P3=[77.24,-679.599,38.328,179.373,-0.028,-167.849]
#恒力控制参数
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [1,1,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [-10.0,-10.0,-10.0,0.0,0.0,0.0]
gain = [0.0005,0.0,0.0,0.0,0.0,0.0] #力PID参数，力矩PID参数
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 1000.0  #最大调整距离
max_ang = 0.0  #最大调整角度
error = robot.MoveJ(J1,1,0)
print("关节空间运动到点1错误码",error)
#柔顺控制
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开始错误码",error)
p = 0.00005  #位置调节系数或柔顺系数
force = 30.0 #柔顺开启力阈值，单位N
error = robot.FT_ComplianceStart(p,force)
print("柔顺控制开始错误码",error)
error = robot.MoveL(P2,1,0,vel =10)   #笛卡尔空间直线运动
print("笛卡尔空间直线运动到点2错误码", error)
error = robot.MoveL(P3,1,0,vel =10)
print("笛卡尔空间直线运动到点3错误码", error)
time.sleep(1)
error = robot.FT_ComplianceStop()
print("柔顺控制结束错误码",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制关闭错误码",error)