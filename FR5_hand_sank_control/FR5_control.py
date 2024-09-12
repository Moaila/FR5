from fairino import Robot
from fr5_init_new import fr5robot  
import time

robot = fr5robot()  


move_increment = 10


def handle_joystick_input(input_value):

    if isinstance(input_value, tuple): 
        handle_dpad_input(input_value)
    elif isinstance(input_value, int):  
        handle_button_input(input_value)


def handle_dpad_input(dpad_value):
    if dpad_value == (-1, 0):
        print("Moving left")
        robot.MoveL(x=-move_increment)  

    elif dpad_value == (1, 0):
        print("Moving right")
        robot.MoveL(x=move_increment) 

    elif dpad_value == (0, 1):
        print("Moving forward")
        robot.MoveL(y=move_increment)  

    elif dpad_value == (0, -1):
        print("Moving backward")
        robot.MoveL(y=-move_increment) 

def handle_button_input(button_index):
    j

    if button_index == 3:  # 左旋转
        print("Rotating left (joint 5)")
        Robot.StartJOG(0, 5, -1, max_dis=10.0, vel=20.0, acc=100.0)  # 左旋转10度
        time.sleep(0.5)  # 设置一个延迟来确保旋转执行
        Robot.StopJOG(5)  # 停止关节5的点动运动

    elif button_index == 1:  # 右旋转
        print("Rotating right (joint 5)")
        Robot.StartJOG(0, 5, 1, max_dis=10.0, vel=20.0, acc=100.0)  # 右旋转10度
        time.sleep(0.5)  # 设置一个延迟来确保旋转执行
        Robot.StopJOG(5)  # 停止关节5的点动运动

    elif button_index == 4:  
        print("Moving up")
        robot.MoveL(z=move_increment)  

    elif button_index == 0:  
        print("Moving down")
        robot.MoveL(z=-move_increment)  
