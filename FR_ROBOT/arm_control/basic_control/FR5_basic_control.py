from fairino import Robot


class FR5Robot:
    def __init__(self, ip_address: str):
        """
        Initialize the robot connection.
        :param ip_address: IP address of the robot controller.
        """
        self.robot = Robot.RPC(ip_address)

    # ----------- Basic Motion Functions -----------
    def move_joint(self, joint_positions, tool=0, user=0, velocity=20.0, blend_time=-1.0):
        """
        Perform a joint space motion.
        :param joint_positions: List of 6 joint angles in degrees.
        :param tool: Tool index.
        :param user: User (workpiece) coordinate index.
        :param velocity: Speed percentage [0-100].
        :param blend_time: Blend time in ms, use -1 for blocking.
        """
        ret = self.robot.MoveJ(joint_positions, tool, user, vel=velocity, blendT=blend_time)
        print(f"MoveJ Result: {ret}")
        return ret

    def move_cartesian(self, pose, tool=0, user=0, velocity=20.0, blend_time=-1.0):
        """
        Perform a Cartesian space motion.
        :param pose: List [x, y, z, rx, ry, rz] in mm and degrees.
        :param tool: Tool index.
        :param user: User (workpiece) coordinate index.
        :param velocity: Speed percentage [0-100].
        :param blend_time: Blend time in ms, use -1 for blocking.
        """
        ret = self.robot.MoveL(pose, tool, user, vel=velocity, blendT=blend_time)
        print(f"MoveL Result: {ret}")
        return ret

    def jog(self, ref, axis, direction, max_distance, velocity=20.0, acceleration=100.0):
        """
        Perform a jogging operation.
        :param ref: Reference coordinate system (0: Joint, 2: Base, 4: Tool, 8: Workpiece).
        :param axis: Axis index (1-6).
        :param direction: Direction (0: Negative, 1: Positive).
        :param max_distance: Maximum distance/angle for the jog (mm or degrees).
        :param velocity: Speed percentage [0-100].
        :param acceleration: Acceleration percentage [0-100].
        """
        ret = self.robot.StartJOG(ref, axis, direction, max_distance, vel=velocity, acc=acceleration)
        print(f"StartJOG Result: {ret}")
        return ret

    def stop_jog(self, ref):
        """
        Stop jogging with deceleration.
        :param ref: Reference coordinate system (1: Joint, 3: Base, 5: Tool, 9: Workpiece).
        """
        ret = self.robot.StopJOG(ref)
        print(f"StopJOG Result: {ret}")
        return ret

    def stop_immediate(self):
        """
        Stop all motion immediately.
        """
        ret = self.robot.ImmStopJOG()
        print(f"ImmStopJOG Result: {ret}")
        return ret

    # ----------- State Query Functions -----------
    def get_joint_positions(self):
        """
        Get current joint positions in degrees.
        :return: List of 6 joint positions or error code.
        """
        error, positions = self.robot.GetActualJointPosDegree()
        if error == 0:
            print(f"Current Joint Positions: {positions}")
        else:
            print(f"Error getting joint positions: {error}")
        return error, positions

    def get_end_effector_pose(self):
        """
        Get current end-effector pose [x, y, z, rx, ry, rz].
        :return: Pose or error code.
        """
        error, pose = self.robot.GetActualTCPPose()
        if error == 0:
            print(f"Current End-Effector Pose: {pose}")
        else:
            print(f"Error getting end-effector pose: {error}")
        return error, pose

    def get_flange_pose(self):
        """
        Get the current flange pose [x, y, z, rx, ry, rz].
        :return: Pose or error code.
        """
        error, pose = self.robot.GetActualToolFlangePose()
        if error == 0:
            print(f"Current Flange Pose: {pose}")
        else:
            print(f"Error getting flange pose: {error}")
        return error, pose

    # ----------- Utilities -----------
    def start_servo(self):
        """
        Start servo mode.
        """
        ret = self.robot.ServoMoveStart()
        print(f"ServoMoveStart Result: {ret}")
        return ret

    def end_servo(self):
        """
        End servo mode.
        """
        ret = self.robot.ServoMoveEnd()
        print(f"ServoMoveEnd Result: {ret}")
        return ret

    def move_servo_joint(self, joint_positions):
        """
        Perform servo joint movement.
        :param joint_positions: Target joint positions in degrees.
        """
        ret = self.robot.ServoJ(joint_positions, axisPos=[0, 0, 0, 0])
        print(f"ServoJ Result: {ret}")
        return ret

    def move_servo_cartesian(self, mode, pose):
        """
        Perform servo Cartesian movement.
        :param mode: 0 for absolute, 1 for relative (base), 2 for relative (tool).
        :param pose: Target pose or pose increment [x, y, z, rx, ry, rz].
        """
        ret = self.robot.ServoCart(mode, pose, pos_gain=[1.0] * 6)
        print(f"ServoCart Result: {ret}")
        return ret