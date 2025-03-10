"""
@author: 李文皓
@场景： 眼在手外+眼在手上 强化学习加视觉结合
@操作说明：使用时先启动roscore,然后运行yolov8_cup_detector_third.py，再运行本程序

"""
import numpy as np
import rospy
import math
from std_msgs.msg import String, Int32, Bool
from fairino import Robot
import time
from stable_baselines3 import PPO
from scipy.spatial.transform import Rotation as R # 用于旋转矩阵的转换
import sys
import select

# 全局变量，用于存储从 ROS 话题接收到的目标位置
target_position = None
button_position = None
ifredbutton = None

# 保存想喝的饮品的位置
drink_position = None

# 用于保存最后一次接收到的位置
position_history = []

# 用于记录杯子放在什么地方了
cup_position = None

# 手动偏置量
x_offset = 55  # x轴方向偏置量
y_offset = 40   # y轴方向偏置量
z_offset = 76   # z轴方向偏置量
button_offsets = {
    1:('button',(0,0,0)),
    2:('Cappuccino', (-65,-35,60)),
}
# 眼在手上相机基本偏置量
x1_offset = -15  # x轴方向偏置量
y1_offset = 125  # y轴方向偏置量
z1_offset = 285.0  

# 错误码映射表
error_codes = {
    -7: ("上传文件不存在", "检查文件名称是否正确"),
    -6: ("保存文件路径不存在", "检查文件路径是否正确"),
    -5: ("LUA文件不存在", "检查lua文件名称是否正确"),
    -4: ("xmlrpc接口执行失败", "请联系后台工程"),
    -3: ("xmlrpc通讯失败", "请检查网络连接及服务器IP地址是否正确"),
    -2: ("与控制器通讯异常", "检查与控制器硬件连接"),
    -1: ("其他错误", "联系售后工程师查看控制器日志"),
    0: ("调用成功", ""),
    1: ("接口参数个数不一致", "检查接口参数个数"),
    3: ("接口参数值异常", "检查参数类型或范围"),
    8: ("轨迹文件打开失败", "检查TPD轨迹文件是否存在或轨迹名是否正确"),
    9: ("TPD文件名发送失败", "检查TPD轨迹名是否正确"),
    10: ("TPD文件内容发送失败", "检查TPD文件内容是否正确")
}

# 目标位置回调函数
def target_position_callback(msg):
    global target_position
    try:
        # 将目标位置从字符串解析为浮点数
        target_position = [float(coord) for coord in msg.data.split(",")]
        # 记录位置到历史记录
        position_history.append(target_position)
        # 限制历史记录数组的长度
        if len(position_history) > 1:
            position_history.pop(0)
    except ValueError:
        rospy.logerr("接收到的目标位置格式不正确：%s", msg.data)
        target_position = None

def button_position_callback(msg):
    """按钮位置回调函数"""
    global button_position
    try:
        # 解析数据格式："x,y,z"
        coords = msg.data.strip().split(',')
        if len(coords) != 3:
            rospy.logerr(f"无效的按钮坐标数据: {msg.data}")
            return
        button_position = [float(coords[0]), float(coords[1]), float(coords[2])]
        # 只保留最新位置
        position_history.append(button_position.copy())
        if len(position_history) > 3:
            position_history.pop(0)
    except Exception as e:
        rospy.logerr(f"按钮位置解析失败: {str(e)}, 原始数据: {msg.data}")

def red_button_callback(msg):
    """红色标志回调函数u"""
    global ifredbutton
    ifredbutton = int(msg.data)

def button_positon_get():
    global button_position, position_history, robot, ifredbutton, drink_position
    x3_offset = -15  # x轴方向偏置量
    y3_offset = 125   # y轴方向偏置量
    z3_offset = 285.0   # z轴方向偏置量

    # 偏置表需要专门测量调整，但是可以选择相信视觉不做调整
    button_offsets = {
        1:('button',(0,0,0)),
        2:('Cappuccino', (-60,-432,45)),
        3:('Espress',(60,-432,45)),
    }

    try:
        print("请选择您想要点击的按钮：")
        for key, value in button_offsets.items():
            print(f"{key}: {value[0]}")
        button_choice = int(input("请输入按钮编号："))

        if button_choice not in button_offsets:
            print("无效的选择，退出程序。")
            return
        

        selected_button_name, button_offset = button_offsets[button_choice]
        print(f"您选择了按钮：{selected_button_name}")
        # 判断是否有有效的目标位置
        print("接收到的按钮坐标为：",button_position)

        if button_position:
            target_x = button_position[0] + x3_offset + button_offset[0]
            target_y = button_position[1] + y3_offset + button_offset[1]
            target_z = z3_offset + button_offset[2]
        elif position_history:
            last_position = position_history[-1]
            target_x = last_position[0] + x3_offset + button_offset[0]
            target_y = last_position[1] + y3_offset + button_offset[1]
            target_z = z3_offset + button_offset[2]
            print("目标丢失，使用最后保存的位置！")
        else:
            print("等待目标位置...")
            time.sleep(0.1)

        print(f"最终目标位置: x={target_x}, y={target_y}, z={target_z}")
        input("按任意键...")

        drink_position = [target_x, target_y, target_z]
            
    except KeyboardInterrupt:
        print("任务中断。")


#强化学习获得观测值
def get_obs(J1,target_position,robot,angle_noise = [0,0,0,0,0,0],pose_noise = [0,0,0,0,0,0]):
    joint = J1
    # joint = [joint[i] + angle_noise[i] for i in range(6)]
    robot_pose = robot.GetActualTCPPose(0)
    #if robot_pose type is int
    while type(robot_pose) == int:
        robot_pose = robot.GetActualTCPPose(0)
        print("robot_error",robot_pose)
        time.sleep(0.5)
    robot_pose = [robot_pose[1][i] for i in range(6)]
    # robot_pose = robot.GetForwardKin(joint)[1:7]
    # joint[5] = joint[5]-90
    rotation = R.from_euler('xyz',[robot_pose[3],robot_pose[4],robot_pose[5]],degrees=True)
    relative_position = np.array([0,0,0.13])
    rotation_relative_position = rotation.apply(relative_position)
    gripper_centre_pos = [robot_pose[0]/1000,robot_pose[1]/1000,robot_pose[2]/1000] + rotation_relative_position
    gripper_centre_pos[0:2] = -gripper_centre_pos[0:2]
    obs_joint_angles = ((np.array([joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]],dtype=np.float32)/180)+1)/2
    obs_gripper_orientation = (np.array([-robot_pose[3],-robot_pose[4],robot_pose[5]],dtype=np.float32)+180)/360
    obs_gripper_centre_pos = np.array([(gripper_centre_pos[0]+0.5)/1,
                                        (gripper_centre_pos[1]+0.5)/2,
                                        (gripper_centre_pos[2]+0.5)/1],dtype=np.float32)
    obs_target_position = np.array([(target_position[0]+1)/2,
                                    target_position[1]/1,
                                    target_position[2]/0.5],dtype=np.float32)
    obs = np.hstack((obs_gripper_centre_pos,obs_joint_angles,obs_gripper_orientation,obs_target_position),dtype=np.float32).flatten().reshape(1,15)# obs_gripper_orientation,
    return obs,gripper_centre_pos

#强化学习控制机械臂运动
def RL_Move(stage,target,robot):
    _,J1 = robot.GetActualJointPosDegree()
    if _ != 0:
        print("机械臂获取关节角度失败")
        print("error_code:",_)
        return
    print("J1:",J1)
    # 加载模型
    model1 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model1_pick_2.zip")
    model2 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model2_place.zip")
    model3 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model3_button.zip")
    model4 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model4_grasp.zip")
    model5 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model5_send.zip")
    model = [model1,model1, model2,model3,model3,model4,model4,model5]

    best_dis = 100
    print("当前步骤为：",stage+1)
    print("杯子位置为：",target)
    # robot.ServoMoveStart()
    # 主控制循环
    acc = 0.0
    vel = 0.0
    t = 0.016
    lookahead_time = 0.0
    P = 0.0
    count = 100
    # pose_noise = add_noise(range1=20,gaussian=True)
    pose_noise = 0
    for i in range(100):
        # 从机械臂获取当前状态
        # 检测是否有按键输入
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.readline().strip()
            if key == 'q':
                print("检测到按键输入 'q'，程序退出。")
                break
        # J1=robot.GetActualJointPosDegree()[1]
        obs,pos = get_obs(J1,target,robot=robot)
        # print("pos:",pos)

        # 使用模型预测动作
        action, _states = model[stage].predict(obs, deterministic=True)
        # print("action:"+str(action))
        action = action[0:6]

        # 将动作应用于机械臂
        step_arrays = action/5
        J1 = np.array(J1)
        for a in range(5):
            J1 = J1 + step_arrays
            J = J1.tolist()# 
            robot.ServoJ(J, acc, vel, t, lookahead_time, P)
            # time.sleep(0.002)
        # time.sleep(0.5)

        distance = math.sqrt((target[0]-pos[0])**2+(target[1]-pos[1])**2+(target[2]-pos[2])**2)
        print("distance:",distance)
        print("pos",pos)
        if distance <= 0.02:
            if distance < best_dis:
                best_dis = distance
        if best_dis < 1:
            if distance > best_dis or distance < 0.01:
                print("到达目标位置，开始夹取")
                # 关闭夹爪
                Reply = input("按enter复位")
                break
        if pos[2] < 0.05:
            if stage == 3:
                if pos[2] < 0.04:
                    print("到达下限位")
                    exit(-1)
                else:
                    continue
            else:
                print("到达下限位")
                exit(-1)
            time.sleep(10000)
            break


# 机械臂抓取函数
def robot_grab():
    global target_position, button_position, position_history, x_offset, y_offset, z_offset, error_codes, robot, drink_position, ifredbutton
     # 创建 ROS 发布器，用于发送相机切换信号
    pub_stage_status = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)
    # 初始化伺服运动
    ret = robot.ServoMoveStart()
    if ret != 0:
        error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
        print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        return

    #夹爪初始化
    robot.SetGripperConfig(4, 0, 0, 1)
    time.sleep(2)
    robot.ActGripper(1, 1)
    time.sleep(2)
    robot.MoveGripper(1, 100, 50, 1, 10000, 1)
    time.sleep(3)
    print("----------")

    # 初始化机械臂
    J1init = [30.0, -137.0, 128.0, 9.0, 30.0, 0.0]
    p1init = robot.GetForwardKin(J1init)[1:7][0]
    # p1 = [p1init[1],p1init[2],p1init[3],p1init[4],p1init[5],p1init[6]]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveJ(J1init,0,0,p1init,10.0,0.0,100.0,eP1,-1.0,0,dP1)
    time.sleep(5)
    input("按任意键...")

    try:
        while not rospy.is_shutdown():
            # 判断是否有有效的目标位置
            if target_position:
                # 计算最终目标位置（接收到的位置 + 手动偏置量）
                target_x = target_position[0] + x_offset
                target_y = target_position[1] + y_offset
                target_z = z_offset
            elif position_history:
                # 如果没有新位置，使用最后保存的有效位置
                last_position = position_history[-1]
                target_x = last_position[0] + x_offset
                target_y = last_position[1] + y_offset
                target_z = z_offset
                print("目标丢失，使用最后保存的位置！")
            else:
                # 如果没有历史记录，等待目标出现
                print("等待目标位置...")
                time.sleep(0.1)
                continue

            print(f"接收到的目标位置: {target_position if target_position else '无 (使用历史记录)'}")
            print(f"手动偏置量: x={x_offset}, y={y_offset}, z={z_offset}")
            print(f"最终目标位置: x={target_x}, y={target_y}, z={target_z}")

            
            input("按任意键...")

            # time.sleep(2)
            gripper_state = [100,85,100,0,0,100,85,100]
            # 向下移动到抓取位置
            #预夹取
            RL_Move(0,[-target_x/1000,-target_y/1000-0.04,target_z/1000+0.02],robot)
            time.sleep(0.5)
            robot.MoveGripper(1, gripper_state[0]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)
            RL_Move(1,[-target_x/1000,-target_y/1000,target_z/1000],robot)
            robot.MoveGripper(1, gripper_state[1]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)
            pub_stage_status.publish(1)  # 发送信号 1，表示第一阶段完成
            print("已发送相机切换信号，等待相机切换完成...")
            time.sleep(1)  # 等待相机切换完成
            print("ok")
            cup_position = [0.3,0.484,0.08]
            coffee_position = [0.1,0.76,0.12]
            catcher_pos = [0.0,0.0,0.0]
            button_height = 0.28
            obstacle_wide = 0.04
            button_length = 0.01
            position1 = [cup_position[0],cup_position[1],cup_position[2]] #杯子初始位置
            position2 = [coffee_position[0]+0.02,coffee_position[1],coffee_position[2]] #咖啡机位置
            position3 = [coffee_position[0],coffee_position[1]-0.1,0.30] #咖啡机按钮预置位置
            position4 = [coffee_position[0]+0.047,coffee_position[1]+0.005,0.30] #咖啡机按钮位置
            position5 = [coffee_position[0],coffee_position[1]-0.15,coffee_position[2] + 0.08] #夹杯子预置位置
            position6 = [coffee_position[0],coffee_position[1]+0.015,coffee_position[2]] #咖啡机位置
            position7 = [cup_position[0],cup_position[1],cup_position[2]] #杯子预计放置位置
            position = [position1,position2,position3,position4,position5,position6,position7]
            #步骤2把杯子放到咖啡机上
            #预夹取
            RL_Move(2,position[1],robot)
            robot.MoveGripper(1, gripper_state[2]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)
            #步骤3将机械臂移动到观察按钮的位置
            RL_Move(3,position[2],robot)
            #步骤4点击按钮
            
            # 选择按钮并将选择的按钮存放在 drink_position里面
            button_positon_get()
            robot.MoveGripper(1, gripper_state[3]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)  

            RL_Move(4,[-drink_position[0]/1000,-drink_position[1]/1000,drink_position[2]/1000],robot)


            # 步骤5将夹爪移动到等待位置
            RL_Move(5,position[4],robot)
            robot.MoveGripper(1, gripper_state[5]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)
            # 步骤6夹杯子
            RL_Move(6,position[5],robot)
            robot.MoveGripper(1, gripper_state[6]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)    
            # 步骤7送杯子      
            RL_Move(7,position[6],robot)
            robot.MoveGripper(1, gripper_state[7]
                            , 50, 10, 10000, 1)
            time.sleep(0.5)
            break
        else:
            print("等待目标位置...")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("抓取任务中断。")

    finally:
        J1init = [30.0, -137.0, 128.0, 9.0, 30.0, 0.0]
        p1init = robot.GetForwardKin(J1init)[1:7][0]
        eP1=[0.000,0.000,0.000,0.000]
        dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
        robot.MoveJ(J1init,0,0,p1init,10.0,0.0,100.0,eP1,-1.0,0,dP1)
        # 停止伺服模式
        robot.ServoMoveEnd()
        print("伺服模式已停止。")

# 主函数
if __name__ == "__main__":
    rospy.init_node("robot_grab_node", anonymous=True)
    # 订阅目标位置话题
    rospy.Subscriber("/vision/cup_pose", String, target_position_callback)
    rospy.Subscriber('/vision/button_pose', String, button_position_callback)
    rospy.Subscriber('/vision/red_button_status', Bool, red_button_callback)
    print("等待目标位置话题数据...")
    time.sleep(2)

    robot = Robot.RPC('192.168.59.6')  

    # 启动抓取流程
    robot_grab()
