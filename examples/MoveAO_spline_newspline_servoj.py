from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
tool =0
user =0
#控制箱运动AO开始
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]+10
#样条运动开始
error = robot.SplineStart()
print("SplineStart",error)
error = robot.SplinePTP(joint_pos,tool,user)   #样条运动PTP
print("SplinePTP",error)
joint_pos[0] = joint_pos[0]-10
error = robot.SplinePTP(joint_pos, tool, user)   #样条运动PTP
print("SplinePTP",error)
error = robot.SplineEnd() #样条运动结束
print("SplineEnd",error)
#控制箱AO飞拍停止
error = robot.MoveAOStop()
print("MoveAOStop",error)


#末端运动AO开始
error = robot.MoveToolAOStart(0,100,100,1)
print("MoveToolAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]+10
#样条运动开始
error = robot.SplineStart()
print("SplineStart",error)
error = robot.SplinePTP(joint_pos,tool,tool)   #样条运动PTP
print("SplinePTP",error)
joint_pos[0] = joint_pos[0]-10
error = robot.SplinePTP(joint_pos, tool, user)   #样条运动PTP
print("SplinePTP",error)
error = robot.SplineEnd() #样条运动结束
print("SplineEnd",error)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)



#控制箱运动AO开始
error = robot.MoveAOStart(0,100,100,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos1 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos2 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos3 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos4 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos1[1] = desc_pos1[1]-50
desc_pos1[2] = desc_pos1[2]-50
desc_pos2[1] = desc_pos2[1]-100
desc_pos2[2] = desc_pos2[2]+50
desc_pos3[1] = desc_pos3[1]-150
desc_pos3[2] = desc_pos3[2]-50
desc_pos4[1] = desc_pos4[1]-200
desc_pos4[2] = desc_pos4[2]+50
error = robot.MoveL(desc_pos1,1,1)
print("MoveL",error)
#新样条运动开始
lastFlag = 0
error = robot.NewSplineStart(1)
print("NewSplineStart",error)
print("desc_pos",desc_pos)
error = robot.NewSplinePoint(desc_pos1, tool, user, lastFlag)#新样条指令点
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos2, tool, user, lastFlag, vel=30)#新样条指令点
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos3, tool, user, lastFlag, vel=30)#新样条指令点
print("NewSplinePoint",error)
lastFlag = 1
error = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag, vel=30)#新样条指令点
print("NewSplinePoint",error)
error = robot.NewSplineEnd() #新样条运动结束
print("NewSplineEnd",error)
#控制箱运动AO停止
error = robot.MoveAOStop()
print("MoveAOStop",error)


#末端运动AO开始
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos1 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos2 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos3 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos4 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos1[1] = desc_pos1[1]+50
desc_pos1[2] = desc_pos1[2]-50
desc_pos2[1] = desc_pos2[1]+100
desc_pos2[2] = desc_pos2[2]+50
desc_pos3[1] = desc_pos3[1]+150
desc_pos3[2] = desc_pos3[2]-50
desc_pos4[1] = desc_pos4[1]+200
desc_pos4[2] = desc_pos4[2]+50
error = robot.MoveL(desc_pos1,1,1)
print("MoveL",error)
#新样条运动开始
lastFlag = 0
error = robot.NewSplineStart(1)
print("NewSplineStart",error)
print("desc_pos",desc_pos)
error = robot.NewSplinePoint(desc_pos1, tool, user, lastFlag)#新样条指令点
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos2, tool, user, lastFlag, vel=30)#新样条指令点
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos3, tool, user, lastFlag, vel=30)#新样条指令点
print("NewSplinePoint",error)
lastFlag = 1
error = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag, vel=30)#新样条指令点
print("NewSplinePoint",error)
error = robot.NewSplineEnd() #新样条运动结束
print("NewSplineEnd",error)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)


#控制箱运动AO开始
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",joint_pos)
error_joint = 0
count =100
error = robot.ServoMoveStart()  #伺服运动开始
print("ServoMoveStart",error)
while(count):
    error = robot.ServoJ(joint_pos)   #关节空间伺服模式运动
    if error!=0:
        error_joint =error
    joint_pos[0] = joint_pos[0] + 0.1  #每次1轴运动0.1度，运动100次
    count = count - 1
    time.sleep(0.008)
print("ServoJ",error_joint)
error = robot.ServoMoveEnd()  #伺服运动结束
print("ServoMoveEnd",error)
#控制箱运动AO停止
error = robot.MoveAOStop()
print("MoveAOStop",error)

mode = 2  #[0]-绝对运动(基坐标系)，[1]-增量运动(基坐标系)，[2]-增量运动(工具坐标系)
n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   #笛卡尔空间位姿增量
error,desc_pos = robot.GetActualTCPPose()
print("机器人当前笛卡尔位置",desc_pos)
count = 100
error_cart =0
error = robot.ServoMoveStart()  #伺服运动开始
print("伺服运动开始错误码",error)
while(count):
    error = robot.ServoCart(mode, n_pos, vel=40)   #笛卡尔空间伺服模式运动
    if error!=0:
        error_cart =error
    count = count - 1
    time.sleep(0.008)
print("笛卡尔空间伺服模式运动错误码", error_cart)
error = robot.ServoMoveEnd()  #伺服运动结束
print("伺服运动结束错误码",error)
time.sleep(3)

#末端运动AO开始
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
mode = 2  #[0]-绝对运动(基坐标系)，[1]-增量运动(基坐标系)，[2]-增量运动(工具坐标系)
n_pos = [0.0,0.0,-0.5,0.0,0.0,0.0]   #笛卡尔空间位姿增量
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",desc_pos)
count = 100
error_cart =0
error = robot.ServoMoveStart()  #伺服运动结束
print("ServoMoveStart",error)
while(count):
    error = robot.ServoCart(mode, n_pos, vel=40)   #笛卡尔空间伺服模式运动
    if error!=0:
        error_cart =error
    count = count - 1
    time.sleep(0.008)
print("ServoCart", error_cart)
error = robot.ServoMoveEnd()  #伺服运动开始
print("ServoMoveEnd",error)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)


