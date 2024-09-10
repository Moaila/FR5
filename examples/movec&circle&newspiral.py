from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

desc_pos1 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_posc1 = [266.794,-455.119, 65.379, -176.938, 2.535, -179.829] #MoveC过渡点
desc_posc2 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829]  #MoveC目标点

desc_pos2 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_posc3 = [256.794,-435.119, 65.379, -176.938, 2.535, -179.829]   #Circle路径点
desc_posc4 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829]  #Circle目标点

desc_pos3 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos_spiral= [236.794,-475.119, -65.379, -176.938, 2.535, -179.829]#Spiral目标点

#螺旋线参数[circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction]
# circle_num:螺旋圈数，circle_angle:螺旋倾角，rad_init:螺旋初始半径，rad_add:半径增量，
# rotaxis_add:转轴方向增量，rot_direction:旋转方向，0-顺时针，1-逆时针
param = [5.0,10,30,10,5,0]

tool = 0#工具坐标系编号
user = 0 #工件坐标系编号


ret = robot.MoveL(desc_pos1, tool, user, vel=30, acc=100)
print("笛卡尔空间直线运动:错误码", ret)

ret = robot.MoveC(desc_posc1, tool, user, desc_posc2,tool, user)  #笛卡尔空间圆弧运动
print("笛卡尔空间圆弧运动:错误码", ret)

robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
print("笛卡尔空间直线运动:错误码", ret)

ret = robot.Circle(desc_posc3, tool, user, desc_posc4, tool, user, vel_t=40, offset_flag=1, offset_pos=[5,10,15,0,0,1])  #笛卡尔空间圆弧运动
print("笛卡尔空间圆弧运动:错误码", ret)

ret = robot.NewSpiral(desc_pos_spiral, tool, user, param,vel=40 )  #笛卡尔空间螺旋线运动
print("笛卡尔空间螺旋线运动:错误码", ret)