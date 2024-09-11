import Robot
import time

class fr5robot:
    def __init__(self, index=1):
        if index == 1:
            self.robot = Robot.RPC('192.168.59.6')
        if index == 2:
            self.robot = Robot.RPC('192.168.58.6')
        self.robot.SetGripperConfig(4, 0, 0, 1)
        time.sleep(0.5)
        self.robot.ActGripper(1, 1)
        time.sleep(2)
        print("夹爪初始化完成")

    def MoveL(self, x=0, y=0, z=0, movespeed=100.0):
        pos_now = self.robot.GetActualToolFlangePose(0)[1]
        pos_now[0] += x
        pos_now[1] += y
        pos_now[2] += z
        self.robot.MoveL(pos_now, 0, 0, blendR=-1.0)
