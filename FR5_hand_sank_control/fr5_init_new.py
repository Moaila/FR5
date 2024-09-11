'''
法奥新版SDK，适配fr5机械臂，python=3.10
update date 2024-04-11
'''

import sys
import copy
import time
import Robot
import numpy as np
import rospy
from std_msgs.msg import Int32
import threading

class fr5robot:
    def __init__(self, index=1):
        # Initialize the ROS node

        # 初始化变量
        if index == 1:
            self.robot = Robot.RPC('192.168.59.6')
        if index == 2:
            self.robot = Robot.RPC('192.168.58.6')
        self.index = index
        # 夹爪初始化 大寰
        self.robot.SetGripperConfig(4, 0, 0, 1)
        time.sleep(0.5)
        # self.robot.ActGripper(1, 0)
        # time.sleep(0.5)
        self.robot.ActGripper(1, 1)
        time.sleep(2)
        self.robot.MoveGripper(1, 100, 50, 10, 10000, 1)
        time.sleep(0.5)

        print("夹爪初始化完成")
        rospy.init_node('fr5_main', anonymous=True)
        self.pub_gripper1 = rospy.Publisher('pub_gripper1', Int32, queue_size=0)
        self.pub_gripper2 = rospy.Publisher('pub_gripper2', Int32, queue_size=0)

    def MoveGripper(self,index,pos,speed,force,maxtime,block):
        '''
        index:夹爪编号；
        pos:位置百分比，范围[0~100]；
        speed:速度百分比，范围[0~100];
        force:力矩百分比，范围[0~100]；
        maxtime:最大等待时间，范围[0~30000]，单位[ms]；
        block:0-阻塞，1-非阻塞。
        '''
        if self.index == 1:
            self.robot.MoveGripper(index,pos,speed,force,maxtime,block)
            gripper_pos = Int32()
            gripper_pos.data = pos
            self.pub_gripper2.publish(gripper_pos)
        elif self.index == 2:
            self.robot.MoveGripper(index,pos,speed,force,maxtime,block)
            gripper_pos = Int32()
            gripper_pos.data = pos
            self.pub_gripper1.publish(gripper_pos)
        else :
            exit()

    def Go_to_start_zone(self,v = 30.0, open = 1):
        '''
            机械臂复位
        '''
        self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], v, 200.0)
        if open:
            self.MoveGripper(1, 100, 50, 10, 10000, 1)

    def  dou_go_start(self, fr5_B, v=50.0):
        '''
            两个机械臂同时复位
        '''
        self.Go_to_start_zone(v)
        time.sleep(1)
        fr5_B.Go_to_start_zone(v)

    def MoveL(self, x=0.000, y=0.000, z=0.000, movespeed=100.0):
        '''
            机械臂直线运动
        '''
        eP1 = [0.000, 0.000, 0.000, 0.000]
        dP1 = [x, y, z, 0.000, 0.000, 0.000]
        pos_now = self.robot.GetActualToolFlangePose(0)
        # print('pos_now:',pos_now)

        while( type(pos_now) != tuple):
            pos_now = self.robot.GetActualToolFlangePose(0)
            time.sleep(0.5)
        pos_now = pos_now[1]

        pos_now[0] = pos_now[0] + x
        pos_now[1] = pos_now[1] + y
        pos_now[2] = pos_now[2] + z
        ret = self.robot.MoveL(
            pos_now,
            0,
            0,
            blendR = -1.0,
        )
        print('cuowuma',ret)

    def point_safe_move(self, start_catch_position, v=60.0, height=250.0, last_v = 0):  
        '''
            机械臂安全运动到指定位置
        '''

        end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
        # print(type(end_height_from_sdk) )
        # 如果查询失败的话，重新查询
        while( type(end_height_from_sdk) != tuple):
            end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
            print('failed to get end_height_from_sdk1')
            time.sleep(0.5)
        print('end_height_from_sdk:',end_height_from_sdk)
        if len(end_height_from_sdk) == 2:
            end_height = end_height_from_sdk[1]
        if end_height[2] < height:
            # pass
            self.MoveL(0.0, 0.0, (max(height, start_catch_position[2]) - end_height[2] ), v)
        time.sleep(1)
        middle_pos = copy.deepcopy(start_catch_position)

        end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
        # 如果查询失败的话，重新查询
        while( type(end_height_from_sdk) != tuple):
            end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
            print('failed to get end_height_from_sdk2')
            time.sleep(0.5)
        print('end_height_from_sdk:',end_height_from_sdk)
        if len(end_height_from_sdk) == 2:
            end_height = end_height_from_sdk[1]
        if (
            end_height[2]  < height
            and start_catch_position[2] < height
        ):
            middle_pos[2] = height
        else:
            middle_pos[2] = max(end_height[2] , start_catch_position[2])
        self.robot.MoveCart(middle_pos, 0, 0)

        time.sleep(1)
        end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
        # 如果查询失败的话，重新查询
        while( type(end_height_from_sdk) != tuple):
            end_height_from_sdk = self.robot.GetActualToolFlangePose(0)
            print('failed to get end_height_from_sdk3')
            time.sleep(0.5)
        print('end_height_from_sdk:',end_height_from_sdk)
        if len(end_height_from_sdk) == 2:
            end_height = end_height_from_sdk[1]
        if last_v == 0:
            self.MoveL(
                0.0, 0.0, (start_catch_position[2] - end_height[2] ), v
        )
        else:
            self.MoveL(
                0.0, 0.0, (start_catch_position[2] - end_height[2] ), last_v
        )

        # 参数为容器半径，容器上平面离夹爪中心高度，角度增量及方向，指令周期，最大旋转角度
    
    def pour(self, r, h, i=-2, max_angel=90, rate = 100.0, v = 70.0, upright = 1, shake = 1):
        rate /= 100
        types = {
            "100ml": {"diameter": 10, "height": 20},
            "200ml": {"diameter": 15, "height": 25},
            "250ml": {"diameter": 12, "height": 22},
        }
        # 伺服运动参数预置
        t=0.002
        eP0 = [0.000, 0.000, 0.000, 0.000]
        dP0 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
        gain = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # 位姿增量比例系数，仅在增量运动下生效，范围[0~1]
        
        P1 = self.robot.GetActualTCPPose(0)
        J1 = self.robot.GetActualJointPosDegree(0)
        while( type(P1) != tuple):
            P1 = self.robot.GetActualToolFlangePose(0)
            print('when executing pouring,failed to get P1 from sdk')
            time.sleep(0.5)
        P1 = P1[1]
        while( type(J1) != tuple):
            J1 = self.robot.GetActualJointPosDegree(0)
            print('when executing pouring,failed to get J1 from sdk')
            time.sleep(0.5)
        J1 = J1[1]

        # 计算旋转参数
        R = np.sqrt(r**2 + h**2)
        phi = np.arctan(h / r)
        l = np.pi * R / 180  # 弧长（x理论增量）
        # 工具坐标笛卡尔增量
        n_pos = [
            float(2.2 * l * np.sin(phi) * np.sign(i)) * rate,
            float(2.2 * l * np.cos(phi)) * rate,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        joint_pos_difference = 0  # 确保进入倾倒循环
        
        # 在末端关节伺服旋转时，执行空间伺服运动以确保仪器出料口位置稳定
        # 可通过修改单步时延t的大小来调整旋转速度
        while np.abs(joint_pos_difference) < max_angel:
            self.robot.ServoCart(2, n_pos, gain, 0.0, 0.0, t, 0.0, 0.0)  # 工具笛卡尔坐标增量移动

            joint_pos = self.robot.GetActualJointPosDegree(0)
            while( type(joint_pos) != tuple):
                joint_pos = self.robot.GetActualJointPosDegree(0)
                print('when executing pouring,failed to get end_height_from_sdk2')
                time.sleep(0.5)
            joint_pos = joint_pos[1]
            joint_pos[5] = joint_pos[5] + i * rate

            self.robot.ServoJ(joint_pos, 0.0, 0.0, t, 0.0, 0.0)  # 关节角增量移动

            time.sleep(t)

            # 更新差值
            joint_pos_difference = joint_pos[5] - J1[5]

        joint_pos = self.robot.GetActualJointPosDegree(0)
        while( type(joint_pos) != tuple):
            joint_pos = self.robot.GetActualJointPosDegree(0)
            print('when executing pouring,failed to get end_height_from_sdk2')
            time.sleep(0.5)
        joint_pos = joint_pos[1]
        pos_record = self.robot.GetActualTCPPose(0)
        while( type(pos_record) != tuple):
            pos_record = self.robot.GetActualTCPPose(0)
            print('when executing pouring,failed to get pos record')
            time.sleep(0.5)
        pos_record = pos_record[1]
        max_angel = joint_pos[5] + 6.0
        min_angel = joint_pos[5] - 6.0
        shakes = 0
        if shake == 1:
            while shakes < 200:
                self.robot.ServoJ(joint_pos, 0.0, 0.0, t, 0.0, 0.0)
                if joint_pos[5] > max_angel:
                    i = -1
                if joint_pos[5] < min_angel:
                    i = 1
                joint_pos[5] += i

                time.sleep(0.002)
                shakes += 1
            self.robot.MoveCart(pos_record, 0, 0, 0.0, 0.0, v, -1.0, -1)
            
        # 回到倾倒起点，有些场景下可能需要，暂时保留
        # self.MoveL(0.0, 0.0, (300 - self.robot.GetActualTCPPose(0)[3]), 50.0)
        # time.sleep(0.5)
        
        # 是否需要上抬以规避倾倒仪器回归水平位时的碰撞
        if upright == 0:
            return
        else:
            P1 = self.robot.GetActualTCPPose(0)[1]
            P1[2] += 60.0
            if np.abs(P1[5]) > 80.0 and np.abs(P1[5]) < 170.0:
                P1 = P1[0:3]
                P1 += [90.0, 0.0, -90.0]
            else:
                P1 = P1[0:3]
                P1 += [90.0, 0.0, 0.0]
            self.robot.MoveCart(P1, 0, 0, 0.0, 0.0, v, -1.0, -1)
    
