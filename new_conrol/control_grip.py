from fairino import Robot
import time
robot = Robot.RPC('192.168.59.6')
robot.SetGripperConfig(4, 0, 0, 1)
time.sleep(2)
robot.ActGripper(1,1)
time.sleep(2)
robot.MoveGripper(1, 100, 50, 30, 10000, 1)
time.sleep(4)
print("end")