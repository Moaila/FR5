from fr5_init import fr5robot  


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
    if button_index == 3:  
        print("Rotating left")
        robot.MoveL(z=-move_increment)  

    elif button_index == 1:  
        print("Rotating right")
        robot.MoveL(z=move_increment)  

    elif button_index == 4:  
        print("Moving up")
        robot.MoveL(z=move_increment)  

    elif button_index == 0:  
        print("Moving down")
        robot.MoveL(z=-move_increment)  
