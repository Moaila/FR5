from fairino import Robot

if __name__ == "__main__":
    robot = Robot.RPC('192.168.59.6')
    ret = robot.GetActualTCPPose()

    print("获取当前工具位姿", ret)
    ret1 = robot.GetActualToolFlangePose()

    print("获取当前末端法兰位姿", ret1)