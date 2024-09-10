import time
from fairino import Robot

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#设置扩展轴坐标系参考点-四点法
error = robot.ExtAxisSetRefPoint(1)
print("ExtAxisComputeECoordSys(1) return:",error)
time.sleep(5)
error = robot.ExtAxisSetRefPoint(2)
print("ExtAxisComputeECoordSys(2) return:",error)
time.sleep(5)
error = robot.ExtAxisSetRefPoint(3)
print("ExtAxisComputeECoordSys(3) return:",error)
time.sleep(5)
error = robot.ExtAxisSetRefPoint(4)
print("ExtAxisComputeECoordSys(4) return:",error)
#计算扩展轴坐标系-四点法
error,coord = robot.ExtAxisComputeECoordSys()
print("ExtAxisComputeECoordSys() return:",error,coord)
#应用扩展轴坐标系
error = robot.ExtAxisActiveECoordSys(1,1,coord,1)
print("ExtAxisActiveECoordSys() return:",error)

error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
#设置标定参考点在变位机末端坐标系下位姿
error = robot.SetRefPointInExAxisEnd(desc_pos)
print("SetRefPointInExAxisEnd(1) return:",error)

#变位机坐标系参考点设置-四点法
error = robot.PositionorSetRefPoint(1)
print("PositionorSetRefPoint(1) return:",error)
time.sleep(15)
error = robot.ExtAxisStartJog(1,1,20,20,5)
print("ExtAxisStartJog return:",error)
error = robot.ExtAxisStartJog(2,1,20,20,5)
print("ExtAxisStartJog return:",error)
time.sleep(10)
error = robot.PositionorSetRefPoint(2)
print("PositionorSetRefPoint(2) return:",error)
time.sleep(5)
error = robot.ExtAxisStartJog(1,1,20,20,5)
print("ExtAxisStartJog return:",error)
error = robot.ExtAxisStartJog(2,1,20,20,5)
print("ExtAxisStartJog return:",error)
time.sleep(10)
error = robot.PositionorSetRefPoint(3)
print("PositionorSetRefPoint(3) return:",error)
time.sleep(5)
error = robot.ExtAxisStartJog(1,1,20,20,5)
print("ExtAxisStartJog return:",error)
error = robot.ExtAxisStartJog(2,1,20,20,5)
print("ExtAxisStartJog return:",error)
time.sleep(10)
error = robot.PositionorSetRefPoint(4)
print("PositionorSetRefPoint(4) return:",error)
#变位机坐标系计算-四点法
error,coord = robot.PositionorComputeECoordSys()
print("PositionorComputeECoordSys() return:",error,coord)


