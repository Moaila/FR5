from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

#控制箱运动AO开始
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]+10
#机器人关节运动
error = robot.MoveJ(joint_pos,0,0)
print("MoveJ",error)
time.sleep(3)
#控制箱运动AO停止
error = robot.MoveAOStop()
print("MoveAOStop",error)


#末端运动AO开始
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]-10
#机器人关节运动
error = robot.MoveJ(joint_pos,0,0)
print("MoveJ",error)
time.sleep(3)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)


#控制箱运动AO开始
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos[2] = desc_pos[2]+50
#笛卡尔空间直线运动
error = robot.MoveL(desc_pos,0,0)
print("MoveL",error)
time.sleep(3)
#控制箱运动AO停止
error = robot.MoveAOStop()
print("MoveAOStop",error)


#末端运动AO开始
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos[2] = desc_pos[2]-50
#笛卡尔空间直线运动
error = robot.MoveL(desc_pos,0,0)
print("MoveL",error)
time.sleep(3)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)



#控制箱运动AO开始
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]+20
desc_pos_mid[1] = desc_pos_mid[1]-20
desc_pos_end[0]=desc_pos_end[0]+40
desc_pos_end[1]=desc_pos_end[1]
#笛卡尔空间圆弧运动
error = robot.MoveC(desc_pos_mid,0,0,desc_pos_end,0,0)
print("MoveC",error)
#控制箱运动AO停止
error = robot.MoveAOStop()
print("MoveAOStop",error)

time.sleep((1))
#末端运动AO开始
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]-20
desc_pos_mid[1] = desc_pos_mid[1]+20
desc_pos_end[0]=desc_pos_end[0]-40
desc_pos_end[1]=desc_pos_end[1]
#笛卡尔空间圆弧运动
error = robot.MoveC(desc_pos_mid,0,0,desc_pos_end,0,0)
print("MoveC",error)
time.sleep(3)
time.sleep(3)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)

#控制箱运动AO开始
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]-20
desc_pos_mid[1] = desc_pos_mid[1]+20
desc_pos_end[0]=desc_pos_end[0]-40
desc_pos_end[1]=desc_pos_end[1]
#笛卡尔空间整圆运动
error = robot.Circle(desc_pos_mid,0,0,desc_pos_end,1,1)
print("Circle",error)
time.sleep(3)
#控制箱运动AO停止
error = robot.MoveAOStop()
print("MoveAOStop",error)


#末端运动AO开始
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]+20
desc_pos_mid[1] = desc_pos_mid[1]-20
desc_pos_end[0]=desc_pos_end[0]+40
desc_pos_end[1]=desc_pos_end[1]
#笛卡尔空间整圆运动
error = robot.Circle(desc_pos_mid,0,0,desc_pos_end,1,1)
print("Circle",error)
time.sleep(3)
time.sleep(3)
#末端运动AO停止
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)


