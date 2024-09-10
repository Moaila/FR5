from fairino import Robot
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#1.标定并应用机器人工具坐标系，您可以使用四点法或六点法进行工具坐标系的标定和应用，涉及工具坐标系标定的接口如下：
point_num=1
id=1
coord=[100,200,300,0,0,0,]
type=0
install=0
#1.设置工具坐标系
# robot.SetToolPoint(point_num)  #设置工具参考点-六点法
# robot.ComputeTool() #计算工具坐标系
# robot.SetTcp4RefPoint()   #设置工具参考点-四点法
# robot.ComputeTcp4()   #计算工具坐标系-四点法
# robot.SetToolCoord(id, coord,type,install)  #设置应用工具坐标系
# robot.SetToolList(id, coord,type,install)   #设置应用工具坐标系列表

#2.设置UDP通信参数，并加载UDP通信
robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
robot.ExtDevLoadUDPDriver();

#3.设置扩展轴参数，包括扩展轴类型、扩展轴驱动器参数、扩展轴DH参数
robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)#单轴变位机及DH参数
robot.SetRobotPosToAxis(1);  #扩展轴安装位置
robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)#伺服驱动器参数，本示例为单轴变位机，因此只需要设置一个驱动器参数，若您选择包含多个轴的扩展轴类型，需要每一个轴设置驱动器参数

#4.设置所选的轴使能、回零
robot.ExtAxisServoOn(1, 0);
robot.ExtAxisSetHoming(1, 0, 20, 3);

#5.进行扩展轴坐标系标定及应用(注意：变位机和直线滑轨的标定接口不同，以下时变位机的标定接口)
pos =[0,0,0,0,0,0] #输入您的标定点坐标
robot.SetRefPointInExAxisEnd(pos)
robot.PositionorSetRefPoint(1)#您需要通过四个不同位置的点来标定扩展轴，因此需要调用此接口4次才能完成标定
error,coord = robot.PositionorComputeECoordSys()#计算扩展轴标定结果
robot.ExtAxisActiveECoordSys(1, 1, coord, 1); #将标定结果应用到扩展轴坐标系

method=1
#6.在扩展轴上标定工件坐标系，您需要用到以下接口
# robot.SetWObjCoordPoint( point_num)
# error,coord=robot.ComputeWObjCoord( method)
# robot.SetWObjCoord(id,coord)
# robot.SetWObjList(id, coord)

#7.记录您的同步关节运动起始点
startdescPose = [0,0,0,0,0,0]#输入您的坐标
startjointPos = [0,0,0,0,0,0]#输入您的坐标
startexaxisPos = [0,0,0,0,]#输入您的坐标

#8.记录您的同步关节运动终点坐标
enddescPose = [0,0,0,0,0,0]#输入您的坐标
endjointPos = [0,0,0,0,0,0]#输入您的坐标
endexaxisPos = [0,0,0,0,]#输入您的坐标


#9.编写同步运动程序
#运动到起始点，假设应用的工具坐标系、工件坐标系都是1
robot.ExtAxisMove(startexaxisPos, 20);
robot.MoveJ(startjointPos,  1, 1, desc_pos=startdescPose,exaxis_pos=startexaxisPos);

#开始同步运动
robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, endexaxisPos);