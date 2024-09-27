from PyQt5 import QtCore, QtGui, QtWidgets
from joystick_controller import JoystickController
from camera_controller import CameraController
from robot_controller import RobotController
from main_window import *
class MyRobotApp(Ui_MainWindow):
    def __init__(self, MainWindow):
        super().setupUi(MainWindow)
        # 初始化控制器
        self.joystick_controller = JoystickController()
        self.camera_controller = CameraController()
        self.robot_controller = RobotController()
        
        # 绑定按钮事件
        self.pushButton.clicked.connect(self.start_joystick)  # 使用手柄
        self.pushButton_2.clicked.connect(self.stop_joystick)  # 停止手柄
        self.pushButton_3.clicked.connect(self.reset_robot_position)  # 回到初始位置
        self.pushButton_4.clicked.connect(self.start_camera)  # 开启摄像头
        self.pushButton_5.clicked.connect(self.stop_camera)  # 关闭摄像头
        self.pushButton_6.clicked.connect(self.connect_to_robot)  # 连接机械臂

    def start_joystick(self):
        # 启动手柄控制
        self.joystick_controller.start()
        self.label_2.setText("手柄控制已启动")

    def stop_joystick(self):
        # 停止手柄控制
        self.joystick_controller.stop()
        self.label_2.setText("手柄控制已停止")

    def reset_robot_position(self):
        # 复位机械臂到初始位置
        self.robot_controller.reset_position()
        self.label_2.setText("机械臂已复位")

    def start_camera(self):
        # 开启摄像头
        self.camera_controller.start()
        self.label_2.setText("摄像头已开启")

    def stop_camera(self):
        # 关闭摄像头
        self.camera_controller.stop()
        self.label_2.setText("摄像头已关闭")

    def connect_to_robot(self):
        # 通过输入的IP地址连接机械臂
        ip_address = self.lineEdit.text()
        if self.robot_controller.connect(ip_address):
            self.label_2.setText(f"成功连接到机械臂 {ip_address}")
        else:
            self.label_2.setText("连接失败，请检查IP地址")

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = MyRobotApp(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
