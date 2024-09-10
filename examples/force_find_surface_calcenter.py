from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

#恒力控制
status = 1  #恒力控制开启标志，0-关，1-开
sensor_num = 1 #力传感器编号
is_select = [1,0,0,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
force_torque = [-2.0,0.0,0.0,0.0,0.0,0.0]
gain = [0.0002,0.0,0.0,0.0,0.0,0.0]  #力PID参数，力矩PID参数
adj_sign = 0  #自适应启停状态，0-关闭，1-开启
ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
max_dis = 15.0  #最大调整距离
max_ang = 0.0  #最大调整角度

#表面定位参数
rcs = 0 #参考坐标系，0-工具坐标系，1-基坐标系
direction = 1 #移动方向，1-正方向，2-负方向
axis = 1 #移动轴，1-X,2-Y,3-Z
lin_v = 3.0  #探索直线速度，单位mm/s
lin_a = 0.0  #探索直线加速度，单位mm/s^2
disMax = 50.0 #最大探索距离，单位mm
force_goal = 2.0 #动作终止力阈值，单位N
P1=[-77.24,-679.599,58.328,179.373,-0.028,-167.849]
robot.MoveCart(P1,1,0)       #关节空间点到点运动
#x方向寻找中心
#第1个表面
error = robot.FT_CalCenterStart()
print("计算中间平面开始错误码",error)
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开始错误码",error)
error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
print("寻找X+表面错误码",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制结束错误码",error)
time.sleep(2)
error = robot.MoveCart(P1,1,0)       #关节空间点到点运动
print("关节空间点到点运动错误码",error)
time.sleep(5)
#第2个表面
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开始错误码",error)
direction = 2 #移动方向，1-正方向，2-负方向
error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
print("寻找X—表面错误码",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制结束错误码",error)
#计算x方向中心位置
error,xcenter = robot.FT_CalCenterEnd()
print("计算X方向中间平面结束错误码",xcenter)
error = robot.MoveCart(xcenter,1,0)
print("关节空间点到点运动错误码",error)
time.sleep(1)

#y方向寻找中心
#第1个表面
error =robot.FT_CalCenterStart()
print("计算中间平面开始错误码",error)
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开始错误码",error)
direction = 1 #移动方向，1-正方向，2-负方向
axis = 2 #移动轴，1-X,2-Y,3-Z
disMax = 150.0 #最大探索距离，单位mm
lin_v = 6.0  #探索直线速度，单位mm/s
error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
print("寻找表面Y+错误码",error)
status = 0
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制结束错误码",error)
error =robot.MoveCart(P1,1,0)       #关节空间点到点运动
print("关节空间点到点运动错误码",error)
robot.WaitMs(1000)
#第2个表面
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制开始错误码",error)
direction = 2 #移动方向，1-正方向，2-负方向
error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
print("寻找表面Y-错误码",error)
status = 0
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
print("恒力控制结束错误码",error)
#计算y方向中心位置
error,ycenter=robot.FT_CalCenterEnd()
print("计算中间平面Y方向结束错误码",ycenter)
error =robot.MoveCart(ycenter,1,0)
print("关节空间点到点运动错误码",error)