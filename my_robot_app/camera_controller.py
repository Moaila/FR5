import cv2
from kyle_robot_toolbox.camera import Gemini335
from utils.logger import setup_logger

class CameraController:
    def __init__(self):
        """初始化摄像头"""
        self.logger = setup_logger("CameraController", "robot_arm_app.log")
        self.camera = None
        self.is_running = False

    def start_camera(self):
        """启动摄像头并显示图像"""
        if not self.is_running:
            self.camera = Gemini335()
            self.logger.info("摄像头启动成功")
            cv2.namedWindow("color", flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
            self.is_running = True

            while self.is_running:
                # 采集彩图，色彩空间为BGR
                img_bgr = self.camera.read_color_img()
                
                # 显示图像
                cv2.imshow('color', img_bgr)
                key = cv2.waitKey(1)
                
                # 在显示窗口按 'q' 可以退出
                if key == ord('q'):
                    self.logger.info("按 'q' 键退出摄像头显示")
                    break
        else:
            self.logger.warning("摄像头已经在运行")

    def stop_camera(self):
        """关闭摄像头并销毁窗口"""
        if self.is_running:
            self.camera.release()  # 关闭摄像头
            cv2.destroyAllWindows()  # 销毁所有窗口
            self.logger.info("摄像头已关闭并销毁窗口")
            self.is_running = False
        else:
            self.logger.warning("摄像头未在运行")

