from fairino import Robot
from utils.logger import setup_logger
from utils.error_codes import error_codes


class RobotController:
    def __init__(self, robot_ip):
        self.logger = setup_logger("RobotController", "robot_arm_app.log")
        self.robot = Robot.RPC(robot_ip)

    def reset_position(self):
        """复位机械臂到设定的初始位姿（6个关节角度全0）"""
        self.logger.info("开始复位机械臂到初始位姿")

        # 定义初始位姿（6个关节角度全为0）
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 初始关节位置
        acc = 50  # 加速度
        vel = 20  # 速度

        try:
            # 使用 MoveJoint 来移动到设定的关节位置
            ret = self.robot.MoveJoint(home_position, vel)
            if ret == 0:
                self.logger.info("机械臂成功复位到初始位置")
            else:
                error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
                self.logger.error(f"机械臂复位失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        except Exception as e:
            self.logger.error(f"复位机械臂时发生错误: {e}")
            raise e
