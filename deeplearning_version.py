"""
@author: 李文皓
@场景： 眼在手外+眼在手上 强化学习加视觉结合
@操作说明：使用时先启动roscore,然后运行yolov8_cup_detector_third.py，再运行本程序

"""
import numpy as np
import rospy
import math
from std_msgs.msg import String, Int32
from fairino import Robot
import time
from stable_baselines3 import PPO
from scipy.spatial.transform import Rotation as R # 用于旋转矩阵的转换

# 全局变量，用于存储从 ROS 话题接收到的目标位置
target_position = None

# 用于保存最后一次接收到的位置
position_history = []

# 手动偏置量
x_offset = 55  # x轴方向偏置量
y_offset = 40   # y轴方向偏置量
z_offset = 80   # z轴方向偏置量
button_offsets = {
    1:('button',(0,0,0)),
    2:('Cappuccino', (-65,-35,60)),
}
# 眼在手上相机基本偏置量
x1_offset = -15  # x轴方向偏置量
y1_offset = 125  # y轴方向偏置量
z1_offset = 285.0  # z轴方向偏置量，控制抓取时的高度

def select_button():
    """显示按钮菜单并获取用户选择"""
    print("\n请选择要操作的咖啡按钮：")
    for key in button_offsets:
        print(f"{key}: {button_offsets[key][0]}")
    while True:
        try:
            choice = int(input("请输入按钮编号 (1-2): "))
            if choice in button_offsets:
                return choice
            print("无效输入，请重新选择！")
        except ValueError:
            print("请输入有效数字！")

def get_button_position(base_position, button_choice):
    """根据基础位置和按钮选择计算最终位置"""
    if button_choice not in button_offsets:
        raise ValueError("无效的按钮选择")
    
    # 获取基础位置的笛卡尔坐标
    base_x, base_y, base_z = base_position[0], base_position[1], base_position[2]
    
    # 获取按钮偏移量（单位：mm）
    _, offset = button_offsets[button_choice]
    offset_x, offset_y, offset_z = offset
    
    target_x = base_x + x1_offset + offset_x
    target_y = base_y + y1_offset + offset_y
    target_z = z1_offset + offset_z
    
    return [target_x, target_y, target_z]

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
    print("robot_pose",robot_pose)
    robot_pose = [robot_pose[1][i] for i in range(6)]
    # robot_pose = robot.GetForwardKin(joint)[1:7]
    # joint[5] = joint[5]-90
    print("robot_pose:"+str(robot_pose))
    rotation = R.from_euler('xyz',[robot_pose[3],robot_pose[4],robot_pose[5]],degrees=True)
    relative_position = np.array([0,0,0.12])
    rotation_relative_position = rotation.apply(relative_position)
    gripper_centre_pos = [robot_pose[0]/1000,robot_pose[1]/1000,robot_pose[2]/1000] + rotation_relative_position
    gripper_centre_pos[0:2] = -gripper_centre_pos[0:2]
    print("gripper_centre_pos:"+str(gripper_centre_pos)) # 机械臂末端中心位置
    
    
    obs_joint_angles = ((np.array([joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]],dtype=np.float32)/180)+1)/2
    obs_gripper_orientation = (np.array([-robot_pose[3],-robot_pose[4],robot_pose[5]],dtype=np.float32)+180)/360
    print("obs_gripper_orientation:"+str(obs_gripper_orientation))
    obs_gripper_centre_pos = np.array([(gripper_centre_pos[0]+0.5)/1,
                                        (gripper_centre_pos[1]+0.5)/2,
                                        (gripper_centre_pos[2]+0.5)/1],dtype=np.float32)
    obs_target_position = np.array([(target_position[0]+1)/2,
                                    target_position[1]/1,
                                    target_position[2]/0.5],dtype=np.float32)
    obs = np.hstack((obs_gripper_centre_pos,obs_joint_angles,obs_gripper_orientation,obs_target_position),dtype=np.float32).flatten().reshape(1,15)# obs_gripper_orientation,
    print("obs:"+str(obs))
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
    model1 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model1_pick.zip")
    model2 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model2_place.zip")
    model3 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model3_button.zip")
    model4 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model4_grasp.zip")
    model5 = PPO.load("/home/newplace/FR5/RL_weights_hyh/model5_send.zip")
    model = [model1,model2,model3,model3,model4,model4,model5]
    gripper_state = [90,100,0,0,100,90,100]
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
        # J1=robot.GetActualJointPosDegree()[1]
        obs,pos = get_obs(J1,target,robot=robot)
        # print("pos:",pos)

        # 使用模型预测动作
        action, _states = model[stage].predict(obs, deterministic=True)
        print("action:"+str(action))
        action = action[0:6]

        # 将动作应用于机械臂
        step_arrays = action/10
        J1 = np.array(J1)
        for a in range(10):
            J1 = J1 + step_arrays
            J = J1.tolist()# 
            robot.ServoJ(J, acc, vel, t, lookahead_time, P)
            # time.sleep(0.002)
        print("成功执行第"+str(i)+"步")
        # time.sleep(0.5)

        distance = math.sqrt((target[0]-pos[0])**2+(target[1]-pos[1])**2+(target[2]-pos[2])**2)
        print("distance:",distance)
        if distance <= 0.015:
            if distance < best_dis:
                best_dis = distance
        if best_dis < 1:
            if distance > best_dis or distance < 0.01:
                print("到达目标位置，开始夹取")
                # 关闭夹爪
                time.sleep(0.5)
                robot.MoveGripper(1, gripper_state[stage]
                            , 50, 10, 10000, 1)
                time.sleep(0.5)
                Reply = input("按enter复位")
                break
        if pos[2] < 0.07:
            print("到达下限位")
            exit(-1)
            time.sleep(10000)
            break
        
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

# 按钮位置回调函数
def button_pose_callback(msg):
    global button_position
    try:
        button_position = [float(coord) for coord in msg.data.split(",")]
        print(f"接收到按钮坐标: {button_position}")
    except ValueError:
        rospy.logerr("接收到的按钮位置格式不正确：%s", msg.data)
        button_position = None

def check_button_success():
    """检查红色按钮状态"""
    global red_button_signal
    timeout = 10  # 超时时间（秒）
    start_time = time.time()
    
    print("等待按钮状态反馈...")
    while time.time() - start_time < timeout:
        if red_button_signal == 1:
            return True
        time.sleep(0.5)
    return False

# 机械臂抓取函数
def robot_grab():
    global target_position, button_position, position_history, x_offset, y_offset, z_offset

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
     # 创建 ROS 发布器，用于发送相机切换信号
    pub_stage_status = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)

    # 初始化机器人
    robot = Robot.RPC('192.168.59.6')  
    ret = robot.ServoMoveStart()
    if ret != 0:
        error_description, solution = error_codes.get(ret, ("未知错误", "请查看日志"))
        print(f"伺服启动失败，错误码：{ret}，错误描述：{error_description}，处理建议：{solution}")
        return

    #夹爪初始化
    robot.SetGripperConfig(4, 0, 0, 1)
    time.sleep(0.5)
    robot.ActGripper(1, 1)
    time.sleep(0.5)
    robot.MoveGripper(1, 100, 50, 1, 10000, 1)
    time.sleep(0.5)
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

            # 向下移动到抓取位置
            
            RL_Move(0,[-target_x/1000,-target_y/1000,target_z/1000],robot)

            pub_stage_status.publish(1)  # 发送信号 1，表示第一阶段完成
            print("已发送相机切换信号，等待相机切换完成...")
            time.sleep(1)  # 等待相机切换完成
            print("ok")
            cup_position = [0.3,0.484,0.08]
            coffee_position = [0.07,0.75,0.12]
            catcher_pos = [0.0,0.0,0.0]
            button_height = 0.28
            obstacle_wide = 0.04
            button_length = 0.01
            position1 = [cup_position[0],cup_position[1],cup_position[2]] #杯子初始位置
            position2 = [coffee_position[0],coffee_position[1],coffee_position[2]] #咖啡机位置
            position3 = [coffee_position[0],coffee_position[1]-0.1,0.33] #咖啡机按钮预置位置
            position4 = [coffee_position[0]+0.047,coffee_position[1]+0.005,0.30] #咖啡机按钮位置
            position5 = [coffee_position[0],coffee_position[1]-0.15,coffee_position[2] + 0.08] #夹杯子预置位置
            position6 = [coffee_position[0],coffee_position[1]+0.01,coffee_position[2]] #咖啡机位置
            position7 = [cup_position[0],cup_position[1] - 0.1,cup_position[2]] #杯子预计放置位置
            position = [position1,position2,position3,position4,position5,position6,position7]
            #步骤2把杯子放到咖啡机上
            RL_Move(1,position[1],robot)
            #步骤3将机械臂移动到观察按钮的位置
            RL_Move(2,position[2],robot)
            #步骤4点击按钮
            while button_position is None:
                print("等待按钮坐标...")
                time.sleep(0.1)
            
            print(f"\n当前检测到的按钮基础位置：{button_position}")
            
            # 用户选择按钮
            button_choice = select_button()
            button_name, _ = button_offsets[button_choice]
            print(f"\n已选择 {button_name} 按钮，开始操作...")
            
            # 计算最终按钮位置
            target_pos = get_button_position(button_position, button_choice)
            print(f"计算后的目标位置：X={target_pos[0]:.3f}m, Y={target_pos[1]:.3f}m, Z={target_pos[2]:.3f}m")
            input("按任意键...")

            RL_Move(3,position[3],robot)
            RL_Move(4,position[4],robot)
            RL_Move(5,position[5],robot)
            RL_Move(6,position[6],robot)
                
            
        else:
            print("等待目标位置...")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("抓取任务中断。")

    finally:
        # 停止伺服模式
        robot.ServoMoveEnd()
        print("伺服模式已停止。")

# 主函数
if __name__ == "__main__":
    rospy.init_node("robot_grab_node", anonymous=True)
    # 订阅目标位置话题
    rospy.Subscriber("/vision/cup_pose", String, target_position_callback)
    rospy.Subscriber('/vision/button_pose', String, button_pose_callback)
    print("等待目标位置话题数据...")
    time.sleep(2)

    # 启动抓取流程
    robot_grab()
