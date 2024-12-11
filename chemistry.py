# #coding:UTF-8
# import sys
# import os
# sys.path.append('../')
# # 获取parent_folder文件夹的路径
# parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# # 将parent_folder文件夹添加到sys.path列表中
# sys.path.append(parent_path)
# from fr5_init_cp import fr5robot
# from fairino import Robot
# import time
# import copy
# import math
# import numpy as np
# import threading
# # import force_sensor

# # 709靠门一侧机械臂记为A，另一台记为B
# # 当机械臂A朝ym方向时候 ：x正方向偏差2mm,需要在x负方向补偿2mm
# ym_to_x_bias = -2.0
# rA_ym_to_x_bias = -8.0

# # 夹爪安装位置：水平朝y负方向时RXRYRZ = [90.0 ,0.0 ,0.0]

# # 起始区和结束区末端姿态
# start_zone_pos = [400.0, -650.0, 250.0, 90.0, 0.0, 0.0]
# end_zone_pos = [400.0, -400.0, 250.0, 90.0, 0.0, 0.0]

# # 机械臂A末端到实际夹爪末端偏移量
# a_bias = 220.0
# b_bias = 150.0
# arm1_ex_start_pos = [0.0, -400.0, 300.0, 90.0, 0.0, 0.0]

# reaction_pos = {
#             1 : [-200.0, -700.0, 100.0],
#             2 : [0.0, -700.0, 100.0],
#         }



# reaction_done = threading.Event()
# wash_done = threading.Event()
# def utest():
#     force_sensor.zero_calibration()

# class ChemistryTest(fr5robot):
#     def __init__(self,index = 1):
#         super().__init__(index)
#         self.robot.SetSpeed(50)
#         # fr5_B.robot.SetSpeed(20)
        
#     def reset(self,index):
#         '''
#         复位:
#         index:选择机械臂
#         '''
#         if index == 1:
#             self.Go_to_start_zone()
#         elif index == 2:
#             rA = fr5robot(2)
#             rA.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0])

#     def GripperReset(self,index,mode):
#         '''
#         夹爪复位
#         index:选择机械臂
#         mode:0-关闭 1-打开
#         '''
#         if index == 1 and mode == 0:
#             self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#         elif index == 1 and mode == 1:
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         elif index == 2 and mode == 0:
#             rA = fr5robot(2)
#             rA.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#         elif index == 2 and mode == 1:
#             rA = fr5robot(2)
#             rA.MoveGripper(1, 100, 50, 10, 10000, 1)    # open

#     def Stop(self):
#         '''
#         紧急停止
#         index:选择机械臂
#         '''
#         # rA = fr5robot(2)
#         rA = Robot.RPC("192.168.59.6")
#         for _ in range(3):
#             self.robot.StopMotion()
#             rA.StopMotion()
#             time.sleep(0.1)

        

#     def Go_to_start_zone(self,v = 30.0, open = 1):
#         self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], v, 200.0)
#         if open:
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)
            
#     def Go_to_end_zone(self):
#         self.point_safe_move(end_zone_pos, 30.0, 250.0)

#     def F101_catch_02(self,start_catch_position,catch_direction,sel_num,times,is_force_sensor = True):
#         '''
#         指定坐标抓取指定物体
#         预期效果： 机械臂回到起始区————运动到指定位置抓取物体————抬起展示抓取效果————松开夹爪回到结束区
#             start_catch_position: 目标物体绝对xy坐标
#             catch_direction: ym——y轴负方向  xm——x负方向
#             sel_num:抓取的对: 1---试管 2---烧杯 3---量筒 4---反应瓶
#             times:次数
#             is_force_sensor:末端是否有力传感器
#         '''
#         # 输入坐标
#         # start_catch_position = input("--------------请输入目标物体绝对xy坐标---------------\n")
#         # 数据处理str转float
#         if is_force_sensor == True:
#             real_bias = a_bias
#         else:
#             real_bias = b_bias 
#         print("目标物体绝对xy坐标:",start_catch_position)
#         start_catch_position = start_catch_position.split(" ")
#         start_catch_position = [float(num) for num in start_catch_position]
#         # 输入方向
#         # catch_direction = input("--------请输入夹爪夹取方向（ym——y轴负方向  xm——x负方向）--------\n")
#         rxryrz = []
#         if catch_direction == "ym":
#             print("夹爪夹取方向是：y轴负方向")
#             # 数据处理y
#             start_catch_position[1] = start_catch_position[1] + real_bias
#             start_catch_position[0] = start_catch_position[0] + ym_to_x_bias
#             rxryrz = [90.0, 0.0, 0.0]
#             dir = "ym"
#         elif catch_direction == "xm":
#             print("夹爪夹取方向是：x轴负方向")
#             # 数据处理x
#             start_catch_position[0] = start_catch_position[0] + real_bias
#             rxryrz = [90.0, 0.0, -90.0]
#             dir = "xm"
#         else:
#             print("------error!-------")
#             exit()
#         # 合法检测
#         if len(start_catch_position) != 2:
#             print("xy坐标错误")
#             exit()
#         else:
#             print("----------------------")
#         # 选择抓取对象
#         # 注意 抓反应瓶时候，末端离瓶子中心点位180mm 改变瓶子放置
#         while True:
#             # sel_num = input(
#             #     "--------请选择要抓取的对象：输入1---试管 输入2---烧杯 输入3---量筒 输入4---反应瓶--------\n"
#             # )
#             if int(sel_num) == 1:
#                 print("抓取的对象：试管")
#                 start_catch_position += [130.0]
#                 start_catch_position += rxryrz
#                 break
#             elif int(sel_num) == 2:
#                 print("抓取的对象：烧杯")
#                 start_catch_position += [60.0]
#                 start_catch_position += rxryrz
#                 break
#             elif int(sel_num) == 3:
#                 print("抓取的对象：量筒")
#                 start_catch_position += [60.0]
#                 start_catch_position += rxryrz
#                 break
#             elif int(sel_num) == 4:
#                 print("抓取的对象：反应瓶")
#                 start_catch_position += [100.0]
#                 start_catch_position += rxryrz
#                 break
#             else:
#                 print("--------输入错误！请重新输入！---------")
#                 continue
#         self.Go_to_start_zone()
#         if(int(sel_num)==4):
#                 self.MoveGripper(1, 50, 50, 10, 10000, 1)  # close
#                 time.sleep(3)
                
#         self.Safe_move(start_catch_position, dir)  # for display
#         time.sleep(2)
#         # 拿起+放下
#         for i in range(int(times)):
#             if(int(sel_num)==4):
#                 self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#                 time.sleep(3)
#                 self.MoveL(0.0,0.0,20.0,20.0)
#                 time.sleep(1)
#                 self.MoveL(0.0,0.0,-20.0,20.0)
#                 time.sleep(1)
#                 self.MoveGripper(1, 50, 50, 10, 10000, 1)  # open
#                 time.sleep(3)

#             # elif(int(sel_num)==4):
#             else:
#                 self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#                 time.sleep(3)
#                 self.MoveL(0.0,0.0,20.0,20.0)
#                 time.sleep(1)
#                 self.MoveL(0.0,0.0,-20.0,20.0)
#                 time.sleep(1)
#                 self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#                 time.sleep(3)
                
#         self.Go_to_start_zone()
#         print("动作完成")

#     def F101_move_01(self,start_catch_position,catch_direction,sel_num,times):
#         '''
#         10次移动
#             start_catch_position: 目标物体绝对xy坐标
#             catch_direction: ym——y轴负方向  xm——x负方向
#             sel_num:抓取的对: 2---烧杯 3---量筒
#         '''
#         # 输入坐标
#         # start_catch_position = input("--------------请输入目标物体绝对xy坐标---------------\n")
#         # 数据处理str转float
#         start_catch_position = start_catch_position.split(" ")
#         start_catch_position = [float(num) for num in start_catch_position]
#         # 输入方向
#         # catch_direction = input( "--------请输入夹爪夹取方向（ym——y轴负方向  xm——x负方向  xp——x正方向）--------" )
#         rxryrz = []
#         if catch_direction == "ym":
#             # 数据处理y
#             start_catch_position[1] = start_catch_position[1] + a_bias
#             start_catch_position[0] = start_catch_position[0] + ym_to_x_bias
#             rxryrz = [90.0, 0.0, 0.0]
#             dir = "ym"
#         elif catch_direction == "xm":
#             # 数据处理x
#             start_catch_position[0] = start_catch_position[0] + a_bias
#             rxryrz = [90.0, 0.0, -90.0]
#             dir = "xm"
#         # elif catch_direction == "xp":
#         #     # 数据处理y
#         #     start_catch_position[0] = start_catch_position[0] - a_bias
#         #     rxryrz = [90.0, 0.0, 90.0]
#         else:
#             print("------error!-------")
#             exit()
#         # 合法检测
#         if len(start_catch_position) != 2:
#             print("xy坐标错误")
#             exit()
#         else:
#             print("----------------------")
#         # 选择抓取对象
#         while True:
#             # sel_num = input("--------请选择要移动的对象：输入2---烧杯 输入3---量筒--------\n")
#             # if int(sel_num) == 1:
#             #     start_catch_position += [130.0]
#             #     start_catch_position += rxryrz
#             #     break
#             if int(sel_num) == 1:
#                 start_catch_position += [70.0]
#                 start_catch_position += rxryrz
#                 break
#             elif int(sel_num) == 2:
#                 start_catch_position += [70.0]
#                 start_catch_position += rxryrz
#                 break
#             else:
#                 print("--------输入错误！请重新输入！---------")
#                 continue
#         self.Go_to_start_zone()
#         time.sleep(1)
#         self.Safe_move(start_catch_position, dir)  # for display
#         time.sleep(3)
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#         print("---移动10次---\n")
#         time.sleep(2)
#         # 抬起
#         start_catch_position[2] = start_catch_position[2] + 10
#         self.robot.MoveCart(start_catch_position, 0, 0, 0.0, 0.0, 20.0, -1.0, -1)
#         time.sleep(2)
#         for i in range(int(times)):
#             # x正方向平移
#             start_catch_position[0] = start_catch_position[0] + 100
#             self.robot.MoveCart(start_catch_position, 0, 0, 0.0, 0.0, 20.0, -1.0, -1)
#             time.sleep(3)
#             # x负方向平移
#             start_catch_position[0] = start_catch_position[0] - 100
#             self.robot.MoveCart(start_catch_position, 0, 0, 0.0, 0.0, 20.0, -1.0, -1)
#             time.sleep(3)
#         # 落下
#         start_catch_position[2] = start_catch_position[2] - 10
#         self.robot.MoveCart(start_catch_position, 0, 0, 0.0, 0.0, 20.0, -1.0, -1)
#         time.sleep(2)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         time.sleep(1)
#         self.Go_to_start_zone()

#     def F101_put_01(self,start_catch_position,catch_direction,end_catch_position,end_catch_direction,sel_num,times):
#         '''
#         10次移动抓取位置 建议x[-500~-800]y[-500~500]​
#             start_catch_position: 目标物体绝对xy坐标
#             catch_direction: 抓取时夹爪方向 ym——y轴负方向  xm——x负方向
#             end_catch_position: 目的放置位置的xy坐标
#             end_catch_direction: 放置时的夹爪方向 ym——y轴负方向  xm——x负方向
#             sel_num:抓取的对: 2---烧杯 3---量筒
#         '''
#         # 输入坐标
#         # start_catch_position = input("--------------请输入目标物体绝对xy坐标---------------\n")
#         # 数据处理str转float
#         start_catch_position = start_catch_position.split(" ")
#         start_catch_position = [float(num) for num in start_catch_position]

#         # 输入方向
#         # catch_direction = input("--------请输入夹爪夹取方向（ym——y轴负方向  xm——x负方向）--------")
#         start_rxryrz = []
#         if catch_direction == "ym":
#             # 数据处理y
#             start_catch_position[1] = start_catch_position[1] + a_bias
#             start_rxryrz = [90.0, 0.0, 0.0]
#             dir1 = "ym"
#         elif catch_direction == "xm":
#             # 数据处理x
#             start_catch_position[0] = start_catch_position[0] + a_bias
#             start_rxryrz = [90.0, 0.0, -90.0]
#             dir1 = "xm"
#         # elif catch_direction == "xp":
#         #     # 数据处理y
#         #     start_catch_position[0] = start_catch_position[0] - a_bias
#         #     start_rxryrz = [90.0, 0.0, 90.0]
#         else:
#             print("------error!-------")
#             exit()
#         # 合法检测
#         if len(start_catch_position) != 2:
#             print("xy坐标错误")
#             exit()
#         else:
#             print("----------------------")

#         # 输入坐标
#         # end_catch_position = input("--------------请输入目的位置绝对xy坐标---------------\n")
#         # 数据处理str转float
#         end_catch_position = end_catch_position.split(" ")
#         end_catch_position = [float(num) for num in end_catch_position]
#         # 输入方向
#         # end_catch_direction = input("--------请输入夹爪夹取方向（ym——y轴负方向  xm——x负方向  ）--------")
#         end_rxryrz = []
#         if end_catch_direction == "ym":
#             # 数据处理y
#             end_catch_position[1] = end_catch_position[1] + a_bias
#             end_rxryrz = [90.0, 0.0, 0.0]
#             dir2 = "ym"
#         elif end_catch_direction == "xm":
#             # 数据处理x
#             end_catch_position[0] = end_catch_position[0] + a_bias
#             end_rxryrz = [90.0, 0.0, -90.0]
#             dir2 = "xm"
#         # elif end_catch_direction == "xp":
#         #     # 数据处理y
#         #     end_catch_position[0] = end_catch_position[0] - a_bias
#         #     end_rxryrz = [90.0, 0.0, 90.0]
#         else:
#             print("------error!-------")
#             exit()
#         # 合法检测
#         if len(end_catch_position) != 2:
#             print("xy坐标错误")
#             exit()
#         else:
#             print("----------------------")

#         # 选择抓取对象
#         while True:
#             if int(sel_num) == 1:
#                 start_catch_position += [65.0]
#                 start_catch_position += start_rxryrz
#                 end_catch_position += [65.0]
#                 end_catch_position += end_rxryrz
#                 break
#             elif int(sel_num) == 3:
#                 start_catch_position += [70.0]
#                 start_catch_position += start_rxryrz
#                 end_catch_position += [70.0]
#                 end_catch_position += end_rxryrz
#                 break
#             else:
#                 print("--------输入错误！请重新输入！---------")
#                 continue
#         self.Go_to_start_zone()
#         time.sleep(1)
#         for i in range(int(times)):
#             self.Safe_move(start_catch_position, dir1)  # for display
#             time.sleep(2)
#             self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#             time.sleep(2)
#             self.MoveL(0.0,0.0,200.0-start_catch_position[2],50.0)# 抬升至200.0
#             time.sleep(1)
#             high = end_catch_position[2]
#             end_catch_position[2] = 200.0
#             self.robot.MoveCart(end_catch_position, 0, 0, 0.0, 0.0, 30.0, -1.0, -1)#xy定位，z是200
#             time.sleep(1)
#             end_catch_position[2] = high
#             self.robot.MoveCart(end_catch_position, 0, 0, 0.0, 0.0, 30.0, -1.0, -1)#z到指定位置
#             time.sleep(1)
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#             self.Go_to_start_zone()
#             time.sleep(3)

#     def safe_catch(self, catch=0):
#         # 输入坐标
#         start_catch_position = input("--------------请输入目标物体绝对xy坐标---------------\n")
#         # 数据处理str转float
#         start_catch_position = start_catch_position.split(" ")
#         start_catch_position = [float(num) for num in start_catch_position]
#         # 输入方向
#         catch_direction = input(
#             "--------请输入夹爪夹取方向（ym——y轴负方向  xm——x负方向  xp——x正方向）--------\n"
#         )
#         rxryrz1 = []
#         rxryrz2 = []
#         if catch_direction == "ym":
#             # 数据处理y
#             start_catch_position[1] = (
#                 start_catch_position[1] + a_bias + 100
#             )
#             start_catch_position[0] = start_catch_position[0] + ym_to_x_bias  # x方向补齐偏差
#             rxryrz1 = [90.0, 0.0, 0.0]
#             rxryrz2 = [90.0, 0.0, 0.0]
#         elif catch_direction == "xm":
#             # 数据处理x
#             start_catch_position[0] = start_catch_position[0] + a_bias
#             # 此时y方向上观测 无偏差
#             rxryrz1 = [90.0, 0.0, -90.0]
#             rxryrz2 = [90.0, 0.0, -90.0]
#         elif catch_direction == "xp":
#             # 数据处理y
#             start_catch_position[0] = start_catch_position[0] - a_bias
#             rxryrz1 = [90.0, 0.0, 90.0]
#             rxryrz2 = [90.0, 0.0, 90.0]
#         else:
#             print("------error!-------")
#             exit()
#         # 合法检测
#         if len(start_catch_position) != 2:
#             print("xy坐标错误")
#             exit()
#         else:
#             print("----------------------")

#         start_catch_position2 = copy.deepcopy(start_catch_position)
#         start_catch_position3 = copy.deepcopy(start_catch_position)
#         # 选择抓取对象
#         while True:
#             sel_num = input(
#                 "--------请选择要抓取的对象：输入1---试管 输入2---烧杯 输入3---量筒 输入4---反应瓶 输入5---大烧杯--------\n"
#             )
#             if int(sel_num) == 1:
#                 start_catch_position += [130.0]
#                 start_catch_position += rxryrz1
#                 start_catch_position2 += [130.0]
#                 start_catch_position2 += rxryrz2
#                 break
#             elif int(sel_num) == 2:
#                 start_catch_position += [65.0]
#                 start_catch_position += rxryrz1
#                 start_catch_position2 += [65.0]
#                 start_catch_position2 += rxryrz2
#                 break
#             elif int(sel_num) == 3:
#                 start_catch_position += [60.0]
#                 start_catch_position += rxryrz1
#                 start_catch_position2 += [60.0]
#                 start_catch_position2 += rxryrz2
#                 break
#             elif int(sel_num) == 4:
#                 start_catch_position += [110.0]
#                 start_catch_position += rxryrz1
#                 start_catch_position2 += [110.0]
#                 start_catch_position2 += rxryrz2
#                 break
#             elif int(sel_num) == 5:
#                 start_catch_position += [60.0]
#                 start_catch_position += rxryrz1
#                 start_catch_position2 += [60.0]
#                 start_catch_position2 += rxryrz2
#                 break
#             else:
#                 print("--------输入错误！请重新输入！---------")
#                 continue

#         self.point_safe_move(start_catch_position)  # for display
#         time.sleep(0.5)
#         start_catch_position[1] = start_catch_position[1] - 100
#         self.robot.MoveCart(start_catch_position, 0, 0, 0.0, 0.0, 50.0, -1.0, -1)
#         time.sleep(1)
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close

#     def F101_pour_02(self,v=30.0,preset = 1,x=0.0, y=0.0 ,max_angle = 110):
#         '''
#             end_catch_position: 倾倒位置绝对xy坐标
#         '''
#         oriren=0
#         index=0
#         print("------------倾倒实验------------\n")
#         print("请将100ml烧杯放置在200 -700处")
#         self.Go_to_start_zone()
#         # self.safe_catch(1)
#         self.point_safe_move([200.0, -450.0, 65.0, 90.0, 0.0, 0.0], v)
#         self.MoveL(0.0, -100.0, 0.0, v)
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close

#         start_pos = self.robot.GetActualTCPPose(0)[1:7]
#         if x == 0.0 and y == 0.0 and index == 0:
#             if preset:
#                 end_catch_position = [0.0, -700.0]
#             else:
#                 end_catch_position = input("--------------请输入倾倒目标位置绝对xy坐标---------------\n")
#                 end_catch_position = end_catch_position.split(" ")
#                 end_catch_position = [float(num) for num in end_catch_position]

#             # ins = input(
#             #     "--------------请输入倾倒仪器种类与倾倒方向---------------\n\
#             #           /         1---烧杯 2---量筒 3---试管\n\
#             #                     0---倾倒方向为逆时针 1---倾倒方向为顺时针\n"
#             # )
#             ins = "1 0"
#             index = int(ins.split(" ")[0])
#             oriren = int(ins.split(" ")[1])

#             if len(end_catch_position) != 2:
#                 print("xy坐标错误")
#                 exit()
#             elif int(index) not in [1, 2, 3]:
#                 print("index error")
#                 exit()
#             else:
#                 print("----------------------")
#         else:
#             end_catch_position = [float(x), float(y)]
#             index = 1

#         # 序号：[半径，高度]
#         Item_type = {1: [39, 30.5], 2: [10, 160], 3: [10, 75]}
#         # radius = Item_type[index][0]
#         bias = Item_type[index][0]
#         height = Item_type[index][1]
#         pre_angle = 0.0

#         # phi = np.arctan2(radius, height)
#         # omega = np.pi/2 - phi - np.deg2rad(np.abs(pre_angle))
#         # bias = float(np.sqrt(radius**2 + height**2) * np.cos(omega))
#         # print(bias)

#         # 倾倒方向
#         if oriren:
#             bias = -bias
#             pre_angle = -pre_angle

#         end_catch_position[0] = end_catch_position[0] + bias + 20
#         end_catch_position[1] = end_catch_position[1] + a_bias
#         end_catch_position += [350.0, 90.0, pre_angle, 0.0]

#         print(end_catch_position)
#         self.point_safe_move(end_catch_position, v)
#         print("到达目的倾倒位置，开始倾倒")
#         # 执行倾倒
#         self.pour(np.abs(bias), height, float(2 * np.sign(bias)), max_angle - np.abs(pre_angle), 100)

#         # 返回起点
#         self.point_safe_move(start_pos, v)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         self.Go_to_start_zone()

#     def F101_pour_03(self,rA, v=70.0,preset = 1,x=0.0, y=0.0 ,max_angle = 110):
#         up_height = 250.0
#         fr5 = self
#         # rA = ChemistryTest(2)
#         set_done = threading.Event()
#         pour_done = threading.Event()
        
#         def up_bottle(self,v=30.0,x=0.0, y=0.0 ,max_angle = 110):
#             oriren=0
#             index=0
#             print("------------倾倒实验------------\n")
#             print("请将100ml烧杯放置在200 -700处")
#             self.Go_to_start_zone(v)
#             # self.safe_catch(1)
#             self.point_safe_move([200.0, -600.0 + b_bias - 10.0, 65.0, 90.0, 0.0, 0.0], v)
#             self.MoveL(0.0, -100.0, 0.0, v)
#             self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close

#             start_pos = self.robot.GetActualTCPPose(0)[1:7]
#             if x == 0.0 and y == 0.0 and index == 0:
#                 end_catch_position = [0.0, -700.0]
                
#                 ins = "1 0"
#                 index = int(ins.split(" ")[0])
#                 oriren = int(ins.split(" ")[1])

#                 if len(end_catch_position) != 2:
#                     print("xy坐标错误")
#                     exit()
#                 elif int(index) not in [1, 2, 3]:
#                     print("index error")
#                     exit()
#                 else:
#                     print("----------------------")
#             else:
#                 end_catch_position = [float(x), float(y)]
#                 index = 1

#             # 序号：[半径，高度]
#             Item_type = {1: [39, 30.5], 2: [10, 160], 3: [10, 75]}
#             # radius = Item_type[index][0]
#             bias = Item_type[index][0]
#             height = Item_type[index][1]
#             pre_angle = 0.0

#             end_catch_position[0] = end_catch_position[0] + bias +20
#             end_catch_position[1] = end_catch_position[1] + b_bias
#             end_catch_position += [up_height, 90.0, pre_angle, 0.0]

#             print(end_catch_position)
#             self.point_safe_move(end_catch_position, v)
#             print("到达目的倾倒位置，开始倾倒")
#             # 等待置位
#             set_done.wait()
#             # 执行倾倒
#             self.pour(np.abs(bias), height, float(2 * np.sign(bias)), max_angle - np.abs(pre_angle), 100, up_height)
#             pour_done.set()
#             # 返回起点
#             self.point_safe_move(start_pos, v)
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#             self.Go_to_start_zone(v)

#         def under_bottle(self,v = 30.0,x=0.0, y=-700.0):
#             height_dis = 100.0
#             self.Go_to_start_zone(v)
#             self.point_safe_move([200.0, -350.0 , 65.0, 90.0, 0.0, 0.0], v)
#             self.MoveL(0.0, -100.0, 0.0, v)
#             start_pos = self.robot.GetActualTCPPose(0)[1:7]
#             self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
            
#             if up_height - height_dis > 0:
#                 self.point_safe_move([0.0 - x, -1300.0 - y + 150.0, up_height - height_dis, 90.0, 0.0, 0.0], v, 100.0)
#             set_done.set()
            
#             pour_done.wait()
#             self.point_safe_move(start_pos, v, 100.0)
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)  # close
#             self.Go_to_start_zone(v)
            
#         if preset:
#             thread_up = threading.Thread(target=up_bottle,args=(fr5,v))
#             thread_down = threading.Thread(target=under_bottle,args=(rA,v))
#         else:
#             thread_up = threading.Thread(target=up_bottle,args=(fr5,v,x,y,max_angle))
#             thread_down = threading.Thread(target=under_bottle,args=(rA,v,x,y))

#         thread_up.start()
#         thread_down.start()

#         thread_up.join()
#         thread_down.join()

#     def F101_mix_01(self, v=50.0, mix_v=80.0,t = 5):
#         print("------------搅拌实验------------\n")
#         print("请将200ml烧杯放置在-100 -700处")
#         self.Go_to_start_zone()
#         # rA = fr5robot(2)
#         # time.sleep(1) # 防止夹爪超时
#         # rA.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
        
#         # 运动至初始位置
#         # rA.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], v)

#         pre_pos = [280.0, -570.0, 590.0, 89.9, 0.0, 89.9]
#         # 辅助机械臂就位
#         # rA.robot.MoveCart(pre_pos, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         # time.sleep(0.5)
#         # rA.MoveL(28.0, 0.0, 0.0, v)

#         #### 夹按钮测试
#         # while True:
#         #     rf.MoveGripper(1, 0, 30, 10, 10000, 1)
#         #     time.sleep(1)
            
#         #     joint_pos = rA.robot.GetActualJointPosDegree(0)[1:7]
#         #     max_angel = joint_pos[5] + mix_v

#         #     while joint_pos[5] < max_angel:
#         #         rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #         joint_pos[5] = joint_pos[5] + 1
#         #         time.sleep(0.008)

#         #     time.sleep(1)

#         #     while joint_pos[5] > 0:
#         #         rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #         joint_pos[5] = joint_pos[5] - 1
#         #         time.sleep(0.008)
                
#         #     rf.MoveGripper(1, 100, 30, 10, 10000, 1)

#         ####
#         bias = a_bias

#         self.point_safe_move([-100.0, -700.0 + 100.0 + a_bias - 10.0, 65.0, 90.0, 0.0, 0.0])
#         self.MoveL(0.0, -100.0, 0.0, 30.0)
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#         catch_pos = self.robot.GetActualTCPPose(0)[1:7]

#         mix_pos = [-510.0, -742.00, 220.00, 90.0, 0.0, 0.0]

#         mix_pos[1] += bias
#         route1 = copy.deepcopy(mix_pos)
#         route1[0] += 100.0
#         route1[2] -= 85.0
#         self.MoveL(0.0, 0.0, (300 - self.robot.GetActualTCPPose(0)[3]), v)
#         self.point_safe_move(route1)

#         route2 = copy.deepcopy(mix_pos)
#         route2[2] -= 85.0
#         self.robot.MoveCart(route2, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         time.sleep(1)
#         self.MoveL(0.0, 0.0, (mix_pos[2] - self.robot.GetActualTCPPose(0)[3]), v)
#         time.sleep(1)
#         ####
        
#         # 搅拌开始
#         # 抓住旋钮
#         # t = int(input("搅拌即将开始，请确认搅拌安全，并请输入搅拌时间："))
#         # rA.MoveGripper(1, 0, 50, 0, 30000, 0)
#         # # input("请确认已经抓住旋钮，按回车继续")
#         # joint_pos = rA.robot.GetActualJointPosDegree(0)[1:7]
#         # max_angel = joint_pos[5] + mix_v

#         # while joint_pos[5] < max_angel + 30:
#         #     rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #     joint_pos[5] = joint_pos[5] + 1
#         #     time.sleep(0.008)

#         # while joint_pos[5] > max_angel:
#         #     rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #     joint_pos[5] = joint_pos[5] - 1
#         #     time.sleep(0.008)

#         # time.sleep(t)

#         # while joint_pos[5] > 0:
#         #     rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #     joint_pos[5] = joint_pos[5] - 1
#         #     time.sleep(0.008)

#         # rA.MoveGripper(1, 100, 50, 0, 30000, 0)
#         # time.sleep(1)
#         # rA.MoveL(-20.0, 0.0, 0.0, v)
#         # # 搅拌结束

#         input("enter to continue")

#         self.MoveL(0.0, 0.0, (route2[2] - self.robot.GetActualTCPPose(0)[3]), v)
#         self.robot.MoveCart(route1, 0, 0, 0.0, 0.0, 30.0, -1.0, -1)

#         self.point_safe_move(catch_pos, v)

#         self.MoveGripper(1, 100, 50, 0, 30000, 0)
#         self.MoveL(0.0, 50.0, 0.0, v)
#         self.Go_to_start_zone()

#         # 运动至初始位置
#         # rA.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], v)

#     def Test(self):
#         #self.Gripper1_move(1, 0, 50, 100, 10000, 1)
#         rA = fr5robot(2)
#         time.sleep(1) # 防止夹爪超时
#         self.MoveGripper(1, 100, 50, 100, 10000, 1)
#         rA.MoveGripper(1, 100, 50, 100, 10000, 1)

#     def grab_rubber(self,fr5_B):
#         # B抓走二者
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         # time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 0, 50, 100, 10000, 1)  # close
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,140.0,20.0)
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -300.0 + rA_ym_to_x_bias, 0.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)

#     def put_rubber(self,fr5_B):
#         # B放回二者
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 500.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveL(0.0,0.0,-290.0,20.0)
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 100, 50, 100, 10000, 1)  # open
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,100.0,40.0)
#         time.sleep(1)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)


#     def New_Gua_B_1(self,fr5_B):
#         # -----------------------------------------------------以下为右机械臂的操作-----------------------------------------------------#
#         '''
#             以下为右机械臂的操作：
#             机械臂归位、移动到砂芯漏斗处，抓住漏斗，拿起，横放到指定位置等待刮取
#         '''    
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 0, 50, 100, 10000, 1)  # close
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,140.0,30.0)
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 0.0 + rA_ym_to_x_bias, -340.0, 350.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 0.0 + rA_ym_to_x_bias, -440.0, 350.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,244.643,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)

#     def New_Gua_A_1(self):
#         self.Go_to_start_zone()
#         time.sleep(1)
#         self.Safe_move( [300.0 , -400.0 + a_bias, 400.0, 90.0, 0.0, 0.0], "ym", )
#         time.sleep(1)
#         self.robot.MoveCart( [ 300.0 , -400.0 +  a_bias, 400.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 295.0 , -500.0, 370.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 295.0 , -500.0, 320.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#         time.sleep(3)
#         self.MoveL(0.0,0.0,100.0,30.0)
#         time.sleep(1)
#         self.MoveL(0.0,200.0,0.0,50.0)
#         time.sleep(1)
#         self.robot.MoveCart( [ 300.0 , -300.0 +  a_bias, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 100.0 , -500.0 +  a_bias, 250.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 100.0 , -650.0 +  a_bias, 250.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(1)
#         # self.robot.MoveCart( [ 110.0 , -758.0 +  a_bias, 100.4, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         # time.sleep(1)
#         # self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         # time.sleep(1)

#     def New_Gua_A_goback(self):
#         '''
#             以下为左机械臂的操作：
#             机械臂回位
#         '''   
#         self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], 70.0, 200.0)
#         time.sleep(1)
#         self.Safe_move( [300.0 , -400.0 + a_bias, 420.0, 90.0, 0.0, 0.0], "ym", )
#         time.sleep(1)
#         self.robot.MoveCart( [ 300.0 , -400.0 +  a_bias, 420.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 295.0 , -500.0, 370.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.MoveL(0.0,0.0,-50.0,5.0)
#         time.sleep(1)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         time.sleep(3)
#         self.MoveL(0.0,0.0,100.0,35.0)
#         time.sleep(1)
#         self.Go_to_start_zone()
    
#     def New_Gua_B_goback(self,fr5_B):
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)# rA归位
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 500.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveL(0.0,0.0,-290.0,15.0)
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 100, 50, 100, 10000, 1)  # open
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,100.0,25.0)
#         time.sleep(1)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)

#     def test(self,fr5_B):
#         '''
#             以下为左机械臂的操作：
#             机械臂缓慢伸入，同时根据力传感器反馈的数据随时进行停止，当末端z方向力达到1N时，停止前伸
#         ''' 
#         force_max = 1.0
#         is_to_limit1 = True # 是否达到第一段刮取力度极限的标志  
#         is_to_limit2 = True # 是否达到第二段刮取力度极限的标志
#         test_time1 = 0
#         test_time2 = 0
#         delta_y1 = 1.0
#         delta_y2 = 0.05
#         input('-----------输入以开始-------------')
#         self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(1)
#         # 第一段
#         force_sensor.zero_calibration()
#         input('--------')
#         while is_to_limit1:
#             force_sensor.read_force_sensor()
#             if float(force_sensor.real_force_data[2]) > 10:
#                 force_sensor.zero_calibration() # 数据漂的严重，重新校准
#             if float(force_sensor.real_force_data[2]) < 2*(-1.0) :
#                 print("受到外力干扰！")
#                 is_to_limit1 = False
#                 input('-----请处理-----')
#             self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias - test_time1 * delta_y1 , 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 5.0, -1.0, -1, )
#             # self.MoveL(0.0,delta_y1*(-1.0),0.0,1.0)
#             test_time1 = test_time1 + 1
#             if test_time1 > 100:
#                 is_to_limit1 = False

#         self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 5.0, -1.0, -1, )
#         time.sleep(2)
#         # 第二段
#         force_sensor.zero_calibration()
#         while is_to_limit2 :
#             force_sensor.read_force_sensor()
#             if float(force_sensor.real_force_data[2]) > 10:
#                 force_sensor.zero_calibration() # 数据漂的严重，重新校准
#             if float(force_sensor.real_force_data[2]) < force_max*(-1.0) :
#                 print("超过材料受力上限！")
#                 is_to_limit2 = False
#             self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias - test_time2 * delta_y2 , 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             test_time2 = test_time2 + 1
#             # if test_time2 > 30 :
#             #     is_to_limit2 = False
#         print("到达指定位置！")
#         gua_position = self.robot.GetActualToolFlangePose(0)
#         gua_position = gua_position[-6:]
#         print("当前刮取位置位姿：",gua_position)


#     def New_Gua(self,gua_times,fr5_B,force_max = 1.0):
#         force_sensor.talker()
#         def New_Gua_B_1():
#         # -----------------------------------------------------以下为右机械臂的操作-----------------------------------------------------#
#             '''
#                 以下为右机械臂的操作：
#                 机械臂归位、移动到砂芯漏斗处，抓住漏斗，拿起，横放到指定位置等待刮取
#             '''    
#             fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#             fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.MoveGripper(1, 0, 50, 100, 10000, 1)  # close
#             time.sleep(3)
#             fr5_B.MoveL(0.0,0.0,140.0,30.0)
#             time.sleep(1)
#             fr5_B.robot.MoveCart( [ 0.0 + rA_ym_to_x_bias, -340.0, 350.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.robot.MoveCart( [ 0.0 + rA_ym_to_x_bias, -440.0, 350.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.robot.MoveCart( [ 33.123,-459.412,244.643,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#         def New_Gua_A_1():
#             self.Go_to_start_zone()
#             time.sleep(1)
#             self.Safe_move( [300.0 , -400.0 + a_bias, 400.0, 90.0, 0.0, 0.0], "ym", )
#             time.sleep(1)
#             self.robot.MoveCart( [ 300.0 , -400.0 +  a_bias, 400.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 295.0 , -500.0+2.0, 370.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 295.0 , -500.0+2.0, 320.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#             time.sleep(3)
#             self.MoveL(0.0,0.0,100.0,30.0)
#             time.sleep(1)
#             self.MoveL(0.0,200.0,0.0,50.0)
#             time.sleep(1)
#             self.robot.MoveCart( [ 300.0 , -300.0 +  a_bias, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 100.0 , -500.0 +  a_bias, 250.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 100.0 , -650.0 +  a_bias, 250.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#             time.sleep(1)
#             # self.robot.MoveCart( [ 110.0 , -758.0 +  a_bias, 100.4, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             # time.sleep(1)
#             # self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             # time.sleep(1)

#         def New_Gua_A_goback():
#             '''
#                 以下为左机械臂的操作：
#                 机械臂回位
#             '''   
#             self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], 70.0, 200.0)
#             time.sleep(1)
#             self.Safe_move( [300.0 , -400.0 + a_bias, 420.0, 90.0, 0.0, 0.0], "ym", )
#             time.sleep(1)
#             self.robot.MoveCart( [ 300.0 , -400.0 +  a_bias, 420.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ 295.0 , -500.0+2.0, 370.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#             time.sleep(1)
#             self.MoveL(0.0,0.0,-50.0,5.0)
#             time.sleep(1)
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#             time.sleep(3)
#             self.MoveL(0.0,0.0,100.0,35.0)
#             time.sleep(1)
#             self.Go_to_start_zone()

#         def New_Gua_B_goback():
#             fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)# rA归位
#             fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 500.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             fr5_B.MoveL(0.0,0.0,-290.0,5.0)
#             time.sleep(1)
#             fr5_B.MoveGripper(1, 100, 50, 100, 10000, 1)  # open
#             time.sleep(3)
#             fr5_B.MoveL(0.0,0.0,100.0,25.0)
#             time.sleep(1)
#             fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#             time.sleep(1)

#         force_max = 1.0
#         # force_max = input('设定材料受力上限，范围0～2N')
#         # force_max = float(force_max)
#         # 创建两个线程并分别绑定对象的函数
#         thread_1 = threading.Thread(target=New_Gua_B_1)
#         thread_2 = threading.Thread(target=New_Gua_A_1)

#         # 启动线程
#         thread_1.start()
#         thread_2.start()

#         # 等待两个线程执行完毕
#         thread_1.join()
#         thread_2.join()

#         print("---------------准备刮取------------------")
#         '''
#             以下为左机械臂的操作：
#             机械臂缓慢伸入，同时根据力传感器反馈的数据随时进行停止，当末端z方向力达到1N时，停止前伸
#         ''' 
#         force_max = 1.0
#         is_to_limit1 = True # 是否达到第一段刮取力度极限的标志  
#         is_to_limit2 = True # 是否达到第二段刮取力度极限的标志
#         test_time1 = 0
#         test_time2 = 0
#         delta_y1 = 1.0
#         delta_y2 = 0.05
#         # input('-----------输入以开始-------------')
#         self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(3)
#         # 第一段
#         # utest()
#         force_sensor.zero_calibration()
#         input('n确认无误即可按下--------')
#         while is_to_limit1:
#             force_sensor.read_force_sensor()
#             if float(force_sensor.real_force_data[2]) > 10:
#                 force_sensor.zero_calibration() # 数据漂的严重，重新校准
#             if float(force_sensor.real_force_data[2]) < 2*(-1.0) :
#                 print("受到外力干扰！")
#                 is_to_limit1 = False
#                 input('-----请处理-----')
#             self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias - test_time1 * delta_y1 , 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 5.0, -1.0, -1, )
#             # self.MoveL(0.0,delta_y1*(-1.0),0.0,1.0)
#             test_time1 = test_time1 + 1
#             if test_time1 > 100:
#                 is_to_limit1 = False

#         self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 5.0, -1.0, -1, )
#         time.sleep(2)
#         # 第二段
#         force_sensor.zero_calibration()
#         while is_to_limit2 :
#             force_sensor.read_force_sensor()
#             if float(force_sensor.real_force_data[2]) > 10:
#                 force_sensor.zero_calibration() # 数据漂的严重，重新校准
#             if float(force_sensor.real_force_data[2]) < force_max*(-0.5) :
#                 print("超过材料受力上限！")
#                 is_to_limit2 = False
#             self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias - test_time2 * delta_y2 , 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             test_time2 = test_time2 + 1
#             # if test_time2 > 30 :
#             #     is_to_limit2 = False
#         print("到达指定位置！")
#         gua_position = self.robot.GetActualToolFlangePose(0)
#         gua_position = gua_position[-6:]
#         print("当前刮取位置位姿：",gua_position)

#         # 进入旋转刮取
#         for i in range(int(gua_times)):
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 90.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 92.829, -89.897, -2.832], 0, 0, 0.0, 0.0, 90.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 90.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 90.0, -1.0, -1, )
#             time.sleep(1)
#             # 退出，准备下次垂直刮取
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 3.0 , gua_position[2] + 10.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] , gua_position[2] + 10.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] , gua_position[2] - 5.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 20.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 3.0 , gua_position[2] - 5.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             # 对齐初始位置
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 3.0 , gua_position[2] , 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#         '''
#             以下为左机械臂的操作：
#             机械臂暂时退出一定距离
#         '''   
#         self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 50.0 , gua_position[2] , 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         self.MoveL(0.0,150.0,0.0,50.0)
#         time.sleep(1)
#         '''
#             以下为右机械臂的操作：
#             机械臂执行抖动、倾倒的动作
#         '''   
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.83,49.816,91.33], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.884,12.735,91.223], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         # input("-------------------")
#         #todo
#         joint_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#         pos_record = fr5_B.robot.GetActualTCPPose(0)[1:7]
#         max_angel = joint_pos[5] + 3.0
#         min_angel = joint_pos[5] - 3.0
#         shakes = 0
#         i = -1
#         while shakes < 200:
#             fr5_B.robot.ServoJ(joint_pos, 0.0, 0.0, 0.02, 0.0, 0.0)
#             if joint_pos[5] > max_angel:
#                 i = -1
#             if joint_pos[5] < min_angel:
#                 i = 1
#             joint_pos[5] += i

#             new_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#             time.sleep(0.002)
#             shakes += 1

#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.83,49.816,91.33], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.884,12.735,91.223], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)

#         joint_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#         pos_record = fr5_B.robot.GetActualTCPPose(0)[1:7]
#         max_angel = joint_pos[5] + 3.0
#         min_angel = joint_pos[5] - 3.0
#         shakes = 0
#         i = -1
#         while shakes < 200:
#             fr5_B.robot.ServoJ(joint_pos, 0.0, 0.0, 0.02, 0.0, 0.0)
#             if joint_pos[5] > max_angel:
#                 i = -1
#             if joint_pos[5] < min_angel:
#                 i = 1
#             joint_pos[5] += i

#             new_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#             time.sleep(0.002)
#             shakes += 1
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)

#         # 创建两个线程并分别绑定对象的函数
#         thread_3 = threading.Thread(target=New_Gua_A_goback)
#         thread_4 = threading.Thread(target=New_Gua_B_goback)

#         # 启动线程
#         thread_3.start()
#         thread_4.start()

#         # 等待两个线程执行完毕
#         thread_3.join()
#         thread_4.join()



#     def New_Gua2(self,gua_times,fr5_B):
#        # input("--------------------")
#         # 连接第二个机械臂
#         # 材料感度上限
#         force_max = 1.0 
#         # 实际刮取时候的偏移量
#         bias = 4.0
#         # -----------------------------------------------------以下为右机械臂的操作-----------------------------------------------------#
#         '''
#             以下为右机械臂的操作：
#             机械臂归位、移动到砂芯漏斗处，抓住漏斗，拿起，横放到指定位置等待刮取
#         '''    
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -250.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 210.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 0, 50, 100, 10000, 1)  # close
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,140.0,30.0)
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 0.0 + rA_ym_to_x_bias, -340.0, 350.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 0.0 + rA_ym_to_x_bias, -440.0, 350.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 60.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,244.643,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
        
#         '''
#             以下为左机械臂的操作：
#             机械臂归位、移动到刮刀处，抓住刮刀，拿起，伸到砂芯漏斗开口处
#         '''   
#         self.Go_to_start_zone()
#         time.sleep(1)
#         self.Safe_move( [300.0 , -400.0 + a_bias, 400.0, 90.0, 0.0, 0.0], "ym", )
#         time.sleep(1)
#         self.robot.MoveCart( [ 300.0 , -400.0 +  a_bias, 400.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 295.0 , -500.0, 370.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 295.0 , -500.0, 320.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#         time.sleep(3)
#         self.MoveL(0.0,0.0,100.0,30.0)
#         time.sleep(1)
#         self.MoveL(0.0,200.0,0.0,50.0)
#         time.sleep(1)
#         self.robot.MoveCart( [ 300.0 , -300.0 +  a_bias, 300.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 100.0 , -500.0 +  a_bias, 250.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 100.0 , -650.0 +  a_bias, 250.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 110.0 , -650.0 +  a_bias, 100.4, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 50.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 110.0 , -758.0 +  a_bias, 100.4, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias, 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         print("---------------准备刮取------------------")
#         '''
#             以下为左机械臂的操作：
#             机械臂缓慢伸入，同时根据力传感器反馈的数据随时进行停止，当末端z方向力达到1N时，停止前伸
#         '''   
#         is_to_limit2 = True # 是否达到刮取力度极限的标志
#         force_sensor.zero_calibration()
#         test_time2 = 0
#         delta_y2 = 0.01
#         while is_to_limit2 :
#             force_sensor.read_force_sensor()
#             if float(force_sensor.real_force_data[2]) > 1.0:
#                 force_sensor.zero_calibration() # 数据漂的严重，重新校准
#             if float(force_sensor.real_force_data[2]) < force_max*(-1.0) :
#                 print("超过材料受力上限！")
#                 is_to_limit2 = False
#             self.robot.MoveCart( [ 110.0 , -760.0 +  a_bias - test_time2 * delta_y2 , 100.4, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             test_time2 = test_time2 + 1
#             # if test_time2 > 30 :
#             #     is_to_limit2 = False
#         print("到达指定位置！")
#         gua_position = self.robot.GetActualToolFlangePose(0)
#         gua_position = gua_position[-6:]
#         print("当前刮取位置位姿：",gua_position)

#         # 进入旋转刮取
#         for i in range(int(gua_times)):
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 40.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 92.829, -89.897, -2.832], 0, 0, 0.0, 0.0, 40.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 40.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1], gua_position[2], 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 40.0, -1.0, -1, )
#             time.sleep(1)
#             # 退出，准备下次垂直刮取
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 3.0 , gua_position[2] + 10.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] , gua_position[2] + 10.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] , gua_position[2] - 5.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 3.0 , gua_position[2] - 5.0, 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#             # 对齐初始位置
#             self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 3.0 , gua_position[2] , 91.864, 89.902, 1.864 ], 0, 0, 0.0, 0.0, 2.0, -1.0, -1, )
#             time.sleep(1)
#         '''
#             以下为左机械臂的操作：
#             机械臂暂时退出一定距离
#         '''   
#         self.robot.MoveCart( [ gua_position[0] , gua_position[1] + 50.0 , gua_position[2] , 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         self.MoveL(0.0,150.0,0.0,50.0)
#         time.sleep(1)
#         '''
#             以下为右机械臂的操作：
#             机械臂执行抖动、倾倒的动作
#         '''   
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.83,49.816,91.33], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.884,12.735,91.223], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         # input("-------------------")
#         #todo
#         joint_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#         pos_record = fr5_B.robot.GetActualTCPPose(0)[1:7]
#         max_angel = joint_pos[5] + 3.0
#         min_angel = joint_pos[5] - 3.0
#         shakes = 0
#         i = -1
#         while shakes < 200:
#             fr5_B.robot.ServoJ(joint_pos, 0.0, 0.0, 0.02, 0.0, 0.0)
#             if joint_pos[5] > max_angel:
#                 i = -1
#             if joint_pos[5] < min_angel:
#                 i = 1
#             joint_pos[5] += i

#             new_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#             time.sleep(0.002)
#             shakes += 1

#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.83,49.816,91.33], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-89.884,12.735,91.223], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)

#         joint_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#         pos_record = fr5_B.robot.GetActualTCPPose(0)[1:7]
#         max_angel = joint_pos[5] + 3.0
#         min_angel = joint_pos[5] - 3.0
#         shakes = 0
#         i = -1
#         while shakes < 200:
#             fr5_B.robot.ServoJ(joint_pos, 0.0, 0.0, 0.02, 0.0, 0.0)
#             if joint_pos[5] > max_angel:
#                 i = -1
#             if joint_pos[5] < min_angel:
#                 i = 1
#             joint_pos[5] += i

#             new_pos = fr5_B.robot.GetActualJointPosDegree(0)[1:7]
#             time.sleep(0.002)
#             shakes += 1
#         fr5_B.robot.MoveCart( [ 33.123,-459.412,100.0,-68.713,89.697,112.487], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         '''
#             以下为左机械臂的操作：
#             机械臂回位
#         '''   
#         self.point_safe_move([0.0, -250.0, 400.0, 90.0, 0.0, 0.0], 70.0, 200.0)
#         time.sleep(1)
#         self.Safe_move( [300.0 , -400.0 + a_bias, 420.0, 90.0, 0.0, 0.0], "ym", )
#         time.sleep(1)
#         self.robot.MoveCart( [ 300.0 , -400.0 +  a_bias, 420.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.robot.MoveCart( [ 295.0 , -500.0, 370.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 70.0, -1.0, -1, )
#         time.sleep(1)
#         self.MoveL(0.0,0.0,-50.0,5.0)
#         time.sleep(1)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         time.sleep(3)
#         self.MoveL(0.0,0.0,100.0,35.0)
#         time.sleep(1)
#         self.Go_to_start_zone()


#         '''
#             以下为右机械臂的操作：
#             机械臂把砂芯漏斗插入抽滤瓶
#         '''
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)# rA归位
#         fr5_B.robot.MoveCart( [ -200.0 + rA_ym_to_x_bias, -340.0, 500.0, 90.0, 0.0, 0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveL(0.0,0.0,-290.0,15.0)
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 100, 50, 100, 10000, 1)  # open
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,100.0,25.0)
#         time.sleep(1)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)
        
#         '''
#             以下为左机械臂的操作：
#             机械臂把培养皿放到指定位置
#         '''   
#         # self.Go_to_start_zone()
#         # self.robot.MoveCart( [ 95.0 , -400.0 , 300.0, -179.0, 0.0, -179.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         # time.sleep(1)
#         # self.robot.MoveCart( [ 95.0 , -740.0, 175.0, -179.0,0.0,-179.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         # time.sleep(1)
#         # self.MoveGripper(1, 0, 50, 100, 10000, 1)  # close
#         # time.sleep(3)
#         # self.MoveL(0.0,0.0,200.0,10.0)
#         # time.sleep(1)
#         # self.robot.MoveCart( [ 90.0 , -300.0, 175.0, -179.0,0.0,-179.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         # time.sleep(1)
        
#         # self.MoveGripper(1,100, 50, 100, 10000, 1)  # OPEN
#         # time.sleep(3)
#         # self.MoveL(0.0,0.0,200.0,10.0)
#         # time.sleep(1)
#         # self.robot.MoveCart( [ 95.0 , -300.0, 175.0, 90.0,0.0,0.0 ], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         # time.sleep(1)
#         # self.Go_to_start_zone()
#         #       

#     def Get_Cylinder(self,index):
#         '''
#         机械臂A抓取量筒，可以抓取x=-500，y=-400～100、一共6个位置的量筒，其中y=100*(2-index)
#         index取值1～6代表6个量筒
#         '''

#         self.Go_to_start_zone()
#         time.sleep(1)
#         id = index -1
#         if id <= 5:
#             self.robot.MoveCart( [0.0 + ym_to_x_bias, -300.0, 300.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0 , -300.0, 300.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 300.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 70.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-350.0, 100.0 - id * 100, 70.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
#             time.sleep(3)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 100.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 400.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, -300.0, 400.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, -300.0, 400.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
      
#     def Put_Cylinder(self,index):
#         '''
#         机械臂A放置量筒，可以放置x=-500，y=-400～100、一共6个位置的量筒，其中y=100*(2-index)
#         index取值1～6代表6个量筒
#         '''
#         id = index - 1
#         if id <= 5:
#             self.robot.MoveCart( [-300.0 , -300.0, 400.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 400.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 100.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-350.0, 100.0 - id * 100, 70.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#             time.sleep(3)
#             self.robot.MoveCart( [-300.0, 100.0 - id * 100, 100.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.MoveL(0.0,0.0,300.0,30.0)
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, -300.0, 400.0, 90.0, 0.0, -90.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)
#             self.robot.MoveCart( [-300.0, -300.0, 400.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#             time.sleep(1)

#     def press(self,pump_index = 0,btm_index = 0, v = 30.0):
#         height = 400.0
        
#         bias = {
#             0: [30.5 - 150.0, -71.5],#启动按键
#             1: [49.5 - 150.0, -74.5],#反向按键
#         }
#         pump_loc = {
#             0: [410.0, -300.0],
#             1: [410.0, -100.0],
#         } 
        
#         self.MoveL(pump_loc[pump_index][0] + bias[btm_index][0] - self.robot.GetActualTCPPose(0)[1], pump_loc[pump_index][1] + bias[btm_index][1] - self.robot.GetActualTCPPose(0)[2], 0.0,v)
#         # 按键
#         self.MoveL(0.0,0.0,-9.0,v)
#         time.sleep(0.5)
#         self.MoveL(0.0,0.0,9.0,v)
    
#     def catch_stick(self, index = 0,v = 30.0,stick_x = 0.0,stick_y = -300.0):
#         height = 400.0
#         stick_y -= 40.0
#         stick_pos = [stick_x, stick_y, 65.0 ,179.9, 0.0, 0.0]
#         # self.Go_to_start_zone()
        
#         # 移动到触控棒上空
#         pos = stick_pos[0:2]
#         pos += [height,179.9,0.0,0.0]
        
#         self.robot.MoveCart(pos, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         self.MoveL(0.0,0.0,(stick_pos[2] - self.robot.GetActualTCPPose(0)[3] + 150.0),v)
        
#         # 抓取并爬升到安全高度
#         self.robot.MoveGripper(1, 0, 50, 10, 10000, 1)
#         self.MoveL(0.0,0.0,(height - self.robot.GetActualTCPPose(0)[3]),v) 
#         # 设定抓取棒后的朝向，0朝蠕动泵，1朝抽气泵
#         if index == 0:
#             self.robot.MoveCart(self.robot.GetActualTCPPose(0)[1:4]+[90.0 ,0.0 ,89.9], 0, 0, 0.0, 0.0, 100.0, -1.0, -1)
#             self.point_safe_move([250.0, -250.0,height - 170, 90.0 ,0.0 ,90.0], v)
#         if index == 1:
#             self.robot.MoveCart(self.robot.GetActualTCPPose(0)[1:4]+[90.0 ,0.0 ,-89.9], 0, 0, 0.0, 0.0, 100.0, -1.0, -1)
#             time.sleep(0.5)
#             self.MoveL(-100.0,0.0,0.0,v)
    
#     def press_pump(self,switch = 0,v = 30.0, pump_x = -500.0,pump_y = -100.0):
#         height = 400.0
#         off_bias = [20.0, 45.0]
#         on_bias = [15.0, 45.0]
#         if switch == 0:
#             pos = [pump_x + off_bias[0] + 150.0, pump_y + off_bias[1], height - 220, 90.0 ,0.0 ,-89.9]
#         if switch == 1:
#             pos = [pump_x + on_bias[0] + 150.0, pump_y + on_bias[1], height - 220, 90.0 ,0.0 ,-89.9]
#         self.point_safe_move(pos, v)
#         self.MoveL(0.0,0.0,-31.0,v)
#         time.sleep(0.5)
#         self.MoveL(0.0,0.0,10.0,v)

#     def put_stick(self, v = 30.0,stick_x = 0.0,stick_y = -300.0):
#         height = 400.0
#         stick_y -= 40.0
#         stick_pos = [stick_x, stick_y, 65.0 ,179.9, 0.0, 0.0]
#         pos = stick_pos[0:2]
#         pos += [height,179.9,0.0,0.0]
#         self.MoveL(0.0,0.0,(height - self.robot.GetActualTCPPose(0)[3]),v)
        
        
#         self.robot.MoveCart(self.robot.GetActualTCPPose(0)[1:4]+[179.9,0.0,0.0], 0, 0, 0.0, 0.0, 100.0, -1.0, -1)
#         # 移动到触控棒上空
#         self.robot.MoveCart(pos, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         self.MoveL(0.0,0.0,(stick_pos[2] - self.robot.GetActualTCPPose(0)[3] + 150.0),v)
#         # 松开并归位
#         self.MoveGripper(1, 100, 50, 10, 10000, 1) 
#         self.MoveL(0.0,0.0,(height - self.robot.GetActualTCPPose(0)[3]),v)
#         self.Go_to_start_zone(v)
    
#     def EX_to_mix(self, v=50.0, mix_v=80.0,t = 5):
#         print("------------搅拌实验------------\n")
#         print("请将200ml烧杯放置在-200 -700处")
#         self.Go_to_start_zone(v)
#         rA = ChemistryTest(2)
#         #rA.Go_to_start_zone(v)
#         # time.sleep(0.5) # 防止夹爪超时
#         # rA.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
        
#         # 运动至初始位置
#         # rA.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], v)

#         pre_pos = [280.0, -570.0, 590.0, 89.9, 0.0, 89.9]
#         # 辅助机械臂就位
#         rA.robot.MoveCart(pre_pos, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         time.sleep(0.5)
#         rA.MoveL(28.0, 0.0, 0.0, v)

#         ### 夹按钮测试
#         # while True:
#         #     rA.MoveGripper(1, 0, 15, 10, 10000, 1)
#         #     time.sleep(1)
            
#         #     joint_pos = rA.robot.GetActualJointPosDegree(0)[1:7]
#         #     max_angel = joint_pos[5] + mix_v

#         #     while joint_pos[5] < max_angel:
#         #         rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #         joint_pos[5] = joint_pos[5] + 1
#         #         time.sleep(0.008)

#         #     time.sleep(1)

#         #     while joint_pos[5] > 0:
#         #         rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#         #         joint_pos[5] = joint_pos[5] - 1
#         #         time.sleep(0.008)
                
#         #     rA.MoveGripper(1, 100, 30, 10, 10000, 1)

#         ####
#         bias = 150.0
#         # 能抓烧杯的大小
#         self.catch_bottle(0)
#         self.point_safe_move([-200.0, -450.0, 100.0, 90.0, 0.0, 0.0],v)
#         self.MoveL(0.0, -100.0, 0.0, v)
#         self.catch_bottle(1)
#         catch_pos = self.robot.GetActualTCPPose(0)[1:7]

#         mix_pos = [-507.0, -737.00, 270.00, 90.0, 0.0, 0.0]
#         mix_pos[1] += bias
#         route1 = copy.deepcopy(mix_pos)
#         route1[0] += 60.0
#         route1[2] -= 150.0
#         self.MoveL(0.0, 0.0, (300 - self.robot.GetActualTCPPose(0)[3]), v)
        
#         route2 = copy.deepcopy(mix_pos)
#         route2[2] -= 150.0
        
#         self.point_safe_move(route1,v)
#         self.robot.MoveCart(route2, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         time.sleep(1)
#         self.MoveL(0.0, 0.0, (mix_pos[2] - self.robot.GetActualTCPPose(0)[3]), v)
#         time.sleep(1)
#         self.MoveL(0.0, 0.0, 20.0, 30.0)
#         time.sleep(1)
#         self.MoveL(0.0, 0.0, -20.0, 30.0)
#         ####
        
#         # 搅拌开始
#         # 抓住旋钮
#         # t = int(input("搅拌即将开始，请确认搅拌安全，并请输入搅拌时间："))
#         rA.MoveGripper(1, 0, 50, 0, 30000, 0)
#         # input("请确认已经抓住旋钮，按回车继续")
#         joint_pos = rA.robot.GetActualJointPosDegree(0)[1:7]
#         max_angel = joint_pos[5] + mix_v

#         while joint_pos[5] < max_angel + 30:
#             rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#             joint_pos[5] = joint_pos[5] + 1
#             time.sleep(0.008)

#         while joint_pos[5] > max_angel:
#             rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#             joint_pos[5] = joint_pos[5] - 1
#             time.sleep(0.008)
        

#         rA.MoveGripper(1, 100, 50, 0, 30000, 0)
#         time.sleep(1)
#         rA.MoveL(-20.0, 0.0, 0.0, v)
#         # 搅拌结束

#         # 运动至初始位置
#         rA.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], v)

#     def EX_to_mix_no_rA(self, v=50.0,reaction_index = 1, mix_v=80.0,t = 5):
#         print("------------搅拌实验------------\n")
#         print("请将200ml烧杯放置在-200 -700处")
#         self.Go_to_start_zone(v)
#         self.catch_bottle(0)
        
#         catch_pos = reaction_pos[reaction_index][:]
#         catch_pos += [90.0, 0.0, 0.0]
#         catch_pos[1] += (30.0 + a_bias)
        
#         # 能抓烧杯的大小
#         self.catch_bottle(0)
#         self.point_safe_move(catch_pos,v)
#         self.MoveL(0.0, -40.0, 0.0, v)
#         print(self.robot.GetActualTCPPose(0)[1:7])
#         # input("按回车继续")
#         self.catch_bottle(1)
#         input("按回车继续")
#         catch_pos = self.robot.GetActualTCPPose(0)[1:7]

#         bias = a_bias
#         mix_pos = [-510.0, -742.00, 270.00, 90.0, 0.0, 0.0]
#         mix_pos[1] += bias
#         route1 = copy.deepcopy(mix_pos)
#         route1[0] += 100.0
#         route1[2] -= 150.0
#         self.MoveL(0.0, 0.0, (300 - self.robot.GetActualTCPPose(0)[3]), v)
#         self.point_safe_move(route1,v)

#         route2 = copy.deepcopy(mix_pos)
#         route2[2] -= 150.0
#         self.robot.MoveCart(route2, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         time.sleep(1)
#         # input("按回车继续")
#         self.MoveL(0.0, 0.0, (mix_pos[2] - self.robot.GetActualTCPPose(0)[3]), v)
#         time.sleep(1)
#         self.MoveL(0.0, 0.0, 20.0, 30.0)
#         time.sleep(1)
#         self.MoveL(0.0, 0.0, -20.0, 30.0) 
        
#     def Ex_back_mix(self, v=50.0, mix_v=80.0,t = 5):
#         rA = ChemistryTest(2)
#         rA.Go_to_start_zone(v)
#         pre_pos = [280.0, -570.0, 590.0, 89.9, 0.0, 89.9]
#         # 辅助机械臂就位
#         rA.robot.MoveCart(pre_pos, 0, 0, 0.0, 0.0, v, -1.0, -1)
#         time.sleep(0.5)
#         rA.MoveL(28.0, 0.0, 0.0, v)
#         rA.MoveGripper(1, 0, 50, 0, 30000, 0)
        
#         bias = a_bias
#         mix_pos = [-507.0, -739.00, 270.00, 90.0, 0.0, 0.0]
#         mix_pos[1] += bias
#         route1 = copy.deepcopy(mix_pos)
#         route1[0] += 60.0
#         route1[2] -= 150.0
#         route2 = copy.deepcopy(mix_pos)
#         route2[2] -= 150.0
        
        
#         joint_pos = rA.robot.GetActualJointPosDegree(0)[1:7]
#         max_angel = joint_pos[5] - mix_v
#         while joint_pos[5] > max_angel:
#             rA.robot.ServoJ(joint_pos, 0.0, 0.0, 0.008, 0.0, 0.0)
#             joint_pos[5] = joint_pos[5] - 1
#             time.sleep(0.008)
        
#         rA.MoveGripper(1, 100, 50, 0, 30000, 0)
#         time.sleep(1)
        
#         rA.MoveL(-20.0, 0.0, 0.0, v)    

#         self.MoveL(0.0, 0.0, (route2[2] - self.robot.GetActualTCPPose(0)[3]), v)
    
#         self.robot.MoveCart(route1, 0, 0, 0.0, 0.0, 30.0, -1.0, -1)
        
#         self.point_safe_move([-200.0, -550.0, 100.0, 90.0, 0.0, 0.0],v)
#         # self.catch_bottle(0)
#         # self.Go_to_start_zone(v)
        
#         rA.Go_to_start_zone(v)


#     def Ex_back_mix_no_rA(self, v=50.0,reaction_index = 1, loose = 0):
  


        
#         bias = 150.0
#         mix_pos = [-507.0, -742.00, 270.00, 90.0, 0.0, 0.0]
#         mix_pos[1] += bias
#         route1 = copy.deepcopy(mix_pos)
#         route1[0] += 100.0
#         route1[2] -= 150.0
#         route2 = copy.deepcopy(mix_pos)
#         route2[2] -= 150.0
        
       

#         self.MoveL(0.0, 0.0, (route2[2] - self.robot.GetActualTCPPose(0)[3]), v)
    
#         self.robot.MoveCart(route1, 0, 0, 0.0, 0.0, 30.0, -1.0, -1)
        
#         put_pos = reaction_pos[reaction_index][:] + [90.0, 0.0, 0.0]
        
#         put_pos[1] += (a_bias - 10.0)
        
#         # 若需要松开
#         if loose:
#             self.point_safe_move(put_pos,v)
#             self.catch_bottle(0)
#             self.MoveL(0.0, 50.0, 0.0, v)
#             self.Go_to_start_zone(v)
            
#     def F101_wash(self,v = 50.0):
#         fr5 = ChemistryTest(1)
#         rA = ChemistryTest(2)

#         thread_pour = threading.Thread(target=fr5.Pour_Reaction)
#         thread_wash = threading.Thread(target=rA.wash)

#         thread_pour.start()
#         thread_wash.start()

#         thread_pour.join()
#         thread_wash.join()

#     def Pour_Reaction(self,v=50.0,f101 = 1, reaction_index = 1,h = 70.0, x = 230.0, y = -810.0, z = 310.0):
        
#         start_pos = reaction_pos[reaction_index][:] + [90.0, 0.0, 0.0]
#         if reaction_index == 2:
#             start_pos[1] -= 100.0
#         start_pos[1] += (a_bias + 100.0)
        
#         if f101 ==1:
#             self.catch_bottle(0)
#             self.point_safe_move(start_pos,v)
#             self.MoveL(0.0, -110.0, 0.0, v)
#             self.catch_bottle(1)
#             # print(self.robot.GetActualTCPPose(0)[1:7])
#             # input("按回车继续")

#         # start_pos = self.robot.GetActualTCPPose(0)[1:7]
#         start_pos = reaction_pos[reaction_index][:] + [90.0, 0.0, 0.0]
#         start_pos[1] += (a_bias - 10.0)
        
#         if reaction_index == 2:
#             z -= 20.0 
        
#         end_catch_position = [x,y,z]
#         bias = 70.0
#         height = -5.0
#         pre_angle = 30.0

#         end_catch_position[0] = end_catch_position[0] - bias - 15.0
#         end_catch_position[1] = end_catch_position[1] + a_bias - 10.0
#         end_catch_position += [ 89.9, pre_angle, 0.0]

#         print(end_catch_position)
#         self.point_safe_move(end_catch_position, v, 310.0)

#         # 执行倾倒
#         self.pour(np.abs(bias), height, -float(2 * np.sign(bias)), 110 - np.abs(pre_angle), 30.0, 70.0, 0, 0)
        
#         reaction_done.set()

#         wash_done.wait()
#         wash_done.clear()
#         start_pos[0] -= 10.0
#         self.point_safe_move(start_pos, v)
#         self.catch_bottle(0)
#         self.MoveL(0.0, 50.0, 0.0, v)
#         self.Go_to_start_zone(v)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)

#     def wash(self, v = 50.0, reaction_index = 1, wash = 0):
#         catch_pos = [-400.0, -280.0,70.0, 90.0, 0.0, 0.0]
#         height = 300.0
#         if reaction_index == 2:
#             height -= 20.0
#         self.Go_to_start_zone(v)
#         # # 先水平移动
#         # self.MoveL(catch_pos[0] - self.robot.GetActualTCPPose(0)[1],0.0,0.0,v)

#         # self.MoveL(0.0,catch_pos[1] - self.robot.GetActualTCPPose(0)[2] + 100.0 + 100.0,0.0,v)

#         # #再垂直移动
#         # self.MoveL(0.0,0.0,(catch_pos[2] - self.robot.GetActualTCPPose(0)[3]),v)



#         self.point_safe_move(catch_pos,v,550.0)

#         self.MoveL(0.0, -50.0, 0.0, v)

#         put_pos = self.robot.GetActualTCPPose(0)[1:7]

#         # 抓住洗瓶
#         self.MoveGripper(1, 65, 50, 10, 10000, 1)

#         self.MoveL(0.0,0.0,(height - self.robot.GetActualTCPPose(0)[3]),v)


#         reaction_done.wait()
#         reaction_done.clear()

#         pos = self.robot.GetActualTCPPose(0)[1:4]
        
#         pos[0] += 95.0

#         self.robot.MoveCart(pos + [90.0, 15.0, 0.0], 0, 0, 0.0, 0.0, v, -1.0, -1)
        
#         # while True:
#         #     move = input("输入垂直运动的方向和位移，输入q退出")
#         #     if move == 'q':
#         #         break
#         #     self.MoveL(0.0,0.0,float(move),v)

#         # self.MoveL(90.0,0.0,0.0,30.0)
        
 

#         self.pump_water(0)

#         joint_pos = self.robot.GetActualJointPosDegree(0)[1:7]
#         now_pos = self.robot.GetActualTCPPose(0)[1:7]
#         max_angel = joint_pos[5] + 4.0
#         min_angel = joint_pos[5] - 4.0
#         shakes = 0
#         rate = 0.3
#         i = -2
#         k = 1
#         while shakes < 400:
#             self.robot.ServoJ(joint_pos, 0.0, 0.0, 0.004, 0.0, 0.0)
#             if joint_pos[5] > max_angel:
#                 i = -1 * rate
#             if joint_pos[5] < min_angel:
#                 i = 1 * rate
#             if self.robot.GetActualTCPPose(0)[2] < now_pos[1] - 3.0:
#                 k = 1 * rate
#             if self.robot.GetActualTCPPose(0)[2] > now_pos[1] + 3.0:
#                 k = -1 * rate
            
#             joint_pos[5] += i

#             new_pos = self.robot.GetActualJointPosDegree(0)[1:7]
#             time.sleep(0.004)
#             shakes += 1
        
        

        
#         self.pump_water(1)
        
#         # 冲洗结束
#         time.sleep(0.5)

#         self.MoveL(-90.0,0.0,0.0,v)
#         wash_done.set()
#         # input("输入继续")
#         if wash == 1:
#             pos = self.robot.GetActualTCPPose(0)[1:4]
#             pos2 = self.robot.GetActualTCPPose(0)[1:7] 
#             pos[0] += 65.0
#             pos[2] -= 20.0
#             pos += [90.0, 80.0, 0.0]
#             self.robot.MoveCart(pos, 0, 0, 0.0, 0.0, v, -1.0, -1)
#             joint_pos = self.robot.GetActualJointPosDegree(0)[1:7]
#             time.sleep(0.5)
            
#             self.pump_water(0)
#             max_angel = joint_pos[5] + 8.0
#             min_angel = joint_pos[5] - 8.0
#             shakes = 0
#             rate = 0.3
#             i = -2
#             k = 1
#             while shakes < 400:
#                 self.robot.ServoJ(joint_pos, 0.0, 0.0, 0.004, 0.0, 0.0)
#                 if joint_pos[5] > max_angel:
#                     i = -1 * rate
#                 if joint_pos[5] < min_angel:
#                     i = 1 * rate
#                 if self.robot.GetActualTCPPose(0)[2] < now_pos[1] - 1.5:
#                     k = 1 * rate
#                 if self.robot.GetActualTCPPose(0)[2] > now_pos[1] + 1.5:
#                     k = -1 * rate
                
#                 joint_pos[5] += i

#                 new_pos = self.robot.GetActualJointPosDegree(0)[1:7]
#                 time.sleep(0.004)
#                 shakes += 1 
            
            
#             self.pump_water(1)
#             time.sleep(0.5)
            
#             input("输入继续")
            
#             self.robot.MoveCart(pos2, 0, 0, 0.0, 0.0, v, -1.0, -1)
            
            
        
        

#         self.robot.MoveCart(put_pos, 0, 0, 0.0, 0.0, v, -1.0, -1)

#         self.MoveGripper(1, 100, 50, 10, 10000, 1)
        
#         self.MoveL(0.0, 50.0, 0.0, v)

#         self.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0],v,550.0)

#     def Ex_Get_Cylinder(self,index = 1):
#         input("enter to execute")
#         #get 1
#         self.Get_Cylinder(index)
#         time.sleep(3)

#     def Ex_Put_Cylinder(self,index = 1):
#         # 假设此时物体在-200，-450左右空中
#         self.Put_Cylinder(index)
#         time.sleep(3)
    
#     # 倾倒固体--参数为速度，反应瓶编号,试剂瓶编号
#     def Ex_Pour_solid(self, v=50.0, reaction_index = 1, catch_index = 1, go_to_start = 1, triple = 0):

#         h = 70.0
#         # 选择要倾倒的反应瓶

#         # 选择抓取的试剂的位置
#         catch_pos = {
#             1 : [-500.0 + a_bias + 30.0, -200.0, h, 90.0, 0.0, -90.0],
#             2 : [-600.0 + a_bias + 30.0, -100.0, h, 90.0, 0.0, -90.0],
#             3 : [-600.0 + a_bias + 30.0, 0.0, h, 90.0, 0.0, -90.0],
            
#         }
        
#         # self.Go_to_start_zone(v)
#         # self.MoveL(-300.0, 0.0, 0.0, v)
#         pre_pos = [-500.0 + a_bias + 30.0, -200.0, 400.0, 90.0, 0.0, -90.0]
        
#         self.point_safe_move(pre_pos, v)
        
#         self.point_safe_move(catch_pos[catch_index], v)
#         self.MoveL(-45.0, 0.0, 0.0, v)
#         start_pos = self.robot.GetActualTCPPose(0)[1:7]
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close
        
#         end_catch_position = reaction_pos[reaction_index][:]
        
#         bias = 39
#         height = 10
#         pre_angle = 30.0
#         end_catch_position[0] = end_catch_position[0] + a_bias - 15.0
#         end_catch_position[1] = end_catch_position[1] + bias + 20.0
#         end_catch_position[2] = 170.0
#         end_catch_position += [ 90.0, pre_angle, -90.0]

        
#         self.point_safe_move(end_catch_position, v, 300.0)
#         # self.test_pour(v, pre_angle)

#         # 执行倾倒
#         self.pour(np.abs(bias), height, -float(2 * np.sign(bias)), 110 - np.abs(pre_angle), 25.0)
        
#         self.point_safe_move(start_pos, v)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         self.MoveL(30.0, 0.0, 0.0, v)
#         if go_to_start == 1:
#             self.Go_to_start_zone(v)
        
#     # 倾倒液体--参数为速度，反应瓶编号，试剂瓶编号
#     def Ex_Pour(self, v=50.0, reaction_index = 1, catch_index = 1):
#         h = 70.0
#         # 选择要倾倒的反应瓶
        
#         # 选择抓取的试剂的位置
#         catch_pos = {
#             1 : [-500.0 + a_bias + 30.0, -300.0, h, 90.0, 0.0, -90.0],
#             2 : [-500.0 + a_bias + 30.0, -200.0, h, 90.0, 0.0, -90.0],
#             3 : [-600.0 + a_bias + 30.0, -100.0, h, 90.0, 0.0, -90.0],
#         }

#         pre_pos = [-500.0 + a_bias + 30.0, -200.0, 400.0, 90.0, 0.0, -90.0]
        
#         self.point_safe_move(pre_pos, v)
        
#         self.point_safe_move(catch_pos[catch_index], v)
#         self.MoveL(-45.0, 0.0, 0.0, v)
#         start_pos = self.robot.GetActualTCPPose(0)[1:7]
#         self.MoveGripper(1, 0, 50, 10, 10000, 1)  # close

#         end_catch_position = reaction_pos[reaction_index][:]
#         bias = 20.0
#         height = 169.0 - h
#         pre_angle = 45.0

#         end_catch_position[0] = end_catch_position[0] + a_bias - 10.0
#         end_catch_position[1] = end_catch_position[1] + bias + 65
#         end_catch_position += [ 90.0, pre_angle, -90.0]

#         print(end_catch_position)
#         self.point_safe_move(end_catch_position, v, 300.0)

#         # while True:
#         #     parameter = input("请输入倾倒参数，格式为：bias height,输入q退出\n")
#         #     if parameter == 'q':
#         #         break
#         #     parameter = parameter.split(' ')
#         #     self.pour(np.abs(float(parameter[0])), float(parameter[1]), -float(2 * np.sign(float(parameter[0]))), 110 - np.abs(pre_angle), 100.0, 70.0, 0, 0)
#         #     self.robot.MoveCart(end_catch_position, 0, 0, 0.0, 0.0, v, -1.0, -1)
        
#         # 执行倾倒
#         self.pour(np.abs(bias), height, -float(2 * np.sign(bias)), 110 - np.abs(pre_angle), 25.0)
        
#         self.point_safe_move(start_pos, v)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         self.MoveL(60.0, 0.0, 0.0, v)
#         # self.Go_to_start_zone(v)

#     # 倾倒液体--参数为速度，反应瓶编号，试剂瓶编号
#     def Ex_Put_Funnel(self, v=50.0, reaction_index = 1, catch_index = 1):
#         h = 135.0
#         # 选择要倾倒的反应瓶
        
#         # 选择抓取的试剂的位置
#         catch_pos = {
#             1 : [400.0 - a_bias - 50.0, -200.0, h, 90.0, 0.0, 90.0],
#             2 : [-600.0 + a_bias + 30.0, -200.0, h, 90.0, 0.0, -90.0],
#         }

#         # self.Go_to_start_zone(v)
        
#         self.point_safe_move(catch_pos[catch_index], v)
        
#         self.MoveL(65.0, 0.0, 0.0, v)
#         start_pos = self.robot.GetActualTCPPose(0)[1:7]
#         self.MoveGripper(1, 53, 50, 10, 10000, 1)  # close

#         end_catch_position = reaction_pos[reaction_index][:]
 

#         end_catch_position[0] = end_catch_position[0] - 6.0
#         end_catch_position[1] = end_catch_position[1] + a_bias - 3.0
#         end_catch_position[2] = 154.0
#         end_catch_position += [ 90.0, 0.0, 0.0]

#         print(end_catch_position)
#         self.point_safe_move(end_catch_position, 30.0, 400.0, 5.0)


#         # self.adjust()
#         self.MoveGripper(1, 100, 30, 10, 10000, 1)  # open 
#         time.sleep(1.5)
#         self.MoveL(0.0, 60.0, 0.0, v)
#         self.Go_to_start_zone(v)
        
#     def Ex_Back_Funnel(self, v=50.0, reaction_index = 1, catch_index = 1):
#         start_pos = [204.0, -203.0, 130.0, 90.0, 0.0, 90.0]
#         end_catch_position = reaction_pos[reaction_index][:]

#         end_catch_position[0] = end_catch_position[0] - 6.0
#         end_catch_position[1] = end_catch_position[1] + a_bias - 3.0
#         end_catch_position[2] = 144.0
#         end_catch_position += [ 90.0, 0.0, 0.0]
        
#         end_catch_position[1] = end_catch_position[1] + 60.0
#         self.point_safe_move(end_catch_position, v, 400.0)
#         self.MoveL(0.0, reaction_pos[reaction_index][1] + a_bias - 10.0 - self.robot.GetActualTCPPose(0)[2], 0.0, v)
#         self.MoveGripper(1, 53, 50, 10, 10000, 1)  # close
        
#         self.point_safe_move(start_pos, 30.0, 400.0, 5.0)
#         # self.adjust()
#         self.MoveGripper(1, 100, 30, 10, 10000, 1)  # open
#         time.sleep(1.5)
#         self.MoveL(-60.0, 0.0, 0.0, v)
#         self.Go_to_start_zone(v)
        

#     def Ex_Pour_Reaction(self, v=50.0,reaction_index = 1, x = 235.0, y = -810.0, z = 330.0):
#         self.Go_to_start_zone(v)
#         self.catch_bottle(0)
        
#         catch_pos = reaction_pos[reaction_index][:]
#         catch_pos += [90.0, 0.0, 0.0]
#         catch_pos[1] += (30.0 + a_bias)
#         # print(catch_pos)
#         # 能抓烧杯的大小
#         self.catch_bottle(0)
#         self.point_safe_move(catch_pos,v)
#         self.MoveL(0.0, -30.0, 0.0, v)
        
#         self.catch_bottle(1)
#         start_pos = reaction_pos[reaction_index][:] + [90.0, 0.0, 0.0]
#         start_pos[1] += (a_bias - 10.0)
        
        
#         # start_pos = [-200.0, -550.0, 100.0, 90.0, 0.0, 0.0]
        
        
#         end_catch_position = [x,y,z]
#         bias = 70.0
#         height = 5.0
#         pre_angle = 30.0

#         end_catch_position[0] = end_catch_position[0] - bias - 15.0
#         end_catch_position[1] = end_catch_position[1] + a_bias -20.0
#         end_catch_position += [ 90.0, pre_angle, 0.0]

#         print(end_catch_position)
#         self.point_safe_move(end_catch_position, v, 310.0)

#         # 执行倾倒
#         self.pour(np.abs(bias), height, -float(2 * np.sign(bias)), 110 - np.abs(pre_angle), 30.0, 70.0, 0)

#         self.point_safe_move(start_pos, v)
#         self.catch_bottle(0)
#         self.Go_to_start_zone(v)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
        
#         print(reaction_pos)


#     def Ex_Move_Reaction_To_Pour(self, v = 50.0, reaction_index = 1):
#         self.catch_bottle(0)
        
#         catch_pos = reaction_pos[reaction_index][:]
#         catch_pos += [90.0, 0.0, 0.0]
#         catch_pos[1] += (30.0 + a_bias)
        
#         # 能抓烧杯的大小
#         self.catch_bottle(0)
#         self.point_safe_move(catch_pos,v)
#         self.MoveL(0.0, -40.0, 0.0, v)
#         print(self.robot.GetActualTCPPose(0)[1:7])
#         # input("按回车继续")
#         self.catch_bottle(1)
        
#         end_pos = [200.0, -800.0, 100.0, 90.0, 0.0, 90.0]
        
#         end_pos[0] -= a_bias
        
#         self.point_safe_move(end_pos,v)
        
#         self.catch_bottle(0)
        
#         self.MoveL(-50.0, 0.0, 0.0, v)

#         self.Go_to_start_zone(v)        
        

#     def Ex_Pour_filter(self, v = 50.0, reaction_index = 1):
#         self.Go_to_start_zone(v)
        
#         self.point_safe_move([200.0,-800.0 + a_bias + 30.0, 122.0, 90.0, 0.0, 0.0], v)
#         self.MoveL(0.0, -50.0, 0.0, v)
#         start_pos = self.robot.GetActualTCPPose(0)[1:7]
#         self.MoveGripper(1, 0, 100, 100, 10000, 1)  # close

#         end_catch_position = reaction_pos[reaction_index][:]
#         bias = 20.0
#         height = -15.0
#         pre_angle = 45.0

#         end_catch_position[0] = end_catch_position[0] + a_bias - 22.0
#         end_catch_position[1] = end_catch_position[1] + bias + 17.0
#         end_catch_position[2] = 250.0
#         end_catch_position += [ 90.0, pre_angle, -90.0]

#         print(end_catch_position)
#         self.point_safe_move(end_catch_position, v, 400.0)
#         # self.adjust()ß
        
#         # while True:
#         #     parameter = input("请输入倾倒参数，格式为：bias height,输入q退出\n")
#         #     if parameter == 'q':
#         #         break
#         #     parameter = parameter.split(' ')
#         #     self.pour(np.abs(float(parameter[0])), float(parameter[1]), -float(2 * np.sign(float(parameter[0]))), 150 - np.abs(pre_angle), 100.0, 70.0, 0, 0)
#         #     self.robot.MoveCart(end_catch_position, 0, 0, 0.0, 0.0, v, -1.0, -1)
        
        
#         # 执行倾倒
#         self.pour(np.abs(bias), height, -float(2 * np.sign(bias)), 125.0 - np.abs(pre_angle), 30.0, 70.0, 1, 1)
        
#         self.point_safe_move(start_pos, v, 400.0)
#         self.MoveGripper(1, 100, 50, 10, 10000, 1)  # open
#         self.MoveL(0.0, 50.0, 0.0, v) 
#         self.Go_to_start_zone(v)
        
#     # 双臂多线程--倾倒与洗瓶
#     def dou_pour_wash(self, fr5_B, v=50.0, catch = 0, reaction_index = 1, wash = 0):
#         fr5_B.pump_air(1)
#         thread_pour = threading.Thread(target=self.Pour_Reaction,args=(v,catch, reaction_index))
#         thread_wash = threading.Thread(target=fr5_B.wash,args=(v,reaction_index,wash))

#         thread_pour.start()
#         thread_wash.start()

#         thread_pour.join()
#         thread_wash.join()
#         input("抽滤完毕后回车继续")
#         fr5_B.pump_air(0)
#         input("等待气压平衡,回车继续")
#         # print("等待60s气压平衡")
#         # time.sleep(60)

#     def dou_go_start(self, fr5_B, v=50.0):
#         # thread_go = threading.Thread(target=self.Go_to_start_zone,args=(v,))
#         # thread_go_B = threading.Thread(target=fr5_B.Go_to_start_zone,args=(v,))
#         fr5_B.Go_to_start_zone(v)

#         # thread_go.start()
#         # thread_go_B.start()

#         # thread_go.join()
#         # thread_go_B.join()

#     def put_sp_to_choulv(self,fr5_B,v=50.0):
#         '''
#         sp-----代表烧瓶
#         将三经烧瓶拿到抽滤瓶所在位置
#         '''
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)

#         fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.robot.MoveCart( [200.0 + rA_ym_to_x_bias, -300.0, 100.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveL(0.0,-150.0,0.0,30.0)
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 0, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,300.0,50.0)
#         fr5_B.MoveL(0.0,100.0,0.0,50.0)
#         # 拿到抽滤瓶的位置
#         # 由于原来抽滤瓶中心有偏移，y方向需要加偏移量
#         fr5_B.robot.MoveCart( [-200.0 + rA_ym_to_x_bias, -500.0 + b_bias + 5.0, 400.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [-200.0 + rA_ym_to_x_bias, -500.0 + b_bias + 5.0, 100.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.MoveL(0.0,100.0,0.0,30.0)
#         time.sleep(1)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)

#     def Ex_Back_Funnel_v2(self,fr5_B):
#         '''
#         拿走布什漏斗的第二个方案：
#         放到另一个圆台上（洗瓶附近的圆台）
#         '''
#         fr5_B.MoveGripper(1, 100, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [-200.0 + rA_ym_to_x_bias, -350.0 + b_bias + 5.0, 150.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveL(0.0,-150.0,0.0,30.0)
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 0, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,100.0,30.0)
#         time.sleep(1)
#         fr5_B.MoveL(0.0,100.0,0.0,30.0)
#         time.sleep(1)
#         # 拿到圆台的上空
#         fr5_B.robot.MoveCart( [-500.0 + rA_ym_to_x_bias, -400.0 + b_bias, 300.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         # 下降
#         fr5_B.MoveL(0.0,0.0,-100.0,20.0)
#         time.sleep(1)
#         fr5_B.MoveL(0.0,0.0,-62.0,20.0)
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 100, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,200.0,20.0)
#         time.sleep(1)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)

#     def put_sp_to_start(self,fr5_B):
#         '''
#         把抽滤瓶处的三经烧瓶拿到 烧瓶 的起始位置（此时烧瓶头上没有布什漏斗）
#         '''
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         time.sleep(1)

#         fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
#         time.sleep(3)
#         # 由于原来抽滤瓶中心有偏移，y方向需要加偏移量
#         fr5_B.robot.MoveCart( [-208.0, -250.0 + 5.0, 100.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveL(0.0,-100.0,0.0,20.0)
#         fr5_B.MoveGripper(1, 0, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.MoveL(0.0,0.0,50.0,20.0)
#         fr5_B.MoveL(0.0,100.0,0.0,20.0)
#         fr5_B.robot.MoveCart( [-208.0, -250.0, 400.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [192.0, -250.0, 400.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [200.0+rA_ym_to_x_bias, -250.0, 150.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.robot.MoveCart( [200.0+rA_ym_to_x_bias, -450.0, 100.0, 90.0, 0.0, 0.0], 0, 0, 0.0, 0.0, 30.0, -1.0, -1, )
#         time.sleep(1)
#         fr5_B.MoveGripper(1, 50, 50, 10, 10000, 1)
#         time.sleep(3)
#         fr5_B.MoveL(0.0,100.0,0.0,20.0)
#         fr5_B.point_safe_move([0.0, -300.0, 400.0, 90.0, 0.0, 0.0], 20.0)#rA归位
#         fr5_B.MoveGripper(1, 100, 50, 10, 10000, 1)
#         time.sleep(1)

#     def Ex(self,fr5_B,v = 50.0):
#         # input("-------------完整实验演示流程-------------")
#         # self.dou_go_start(fr5_B, v)
#         # # 添加固体液体
#         self.Ex_Pour(v,1)
        
#         self.Ex_Pour_solid(v,1,1)
        
#         # 反应瓶拿去搅拌
#         self.EX_to_mix_no_rA(v)
        
#         # # # 滴加液体
#         # fr5_B.catch_stick(0,v)
#         # # 启动移液泵1
#         # fr5_B.press(0,0,v)

#         # input("请将移液泵1的管子放入反应瓶中，按回车继续")
#         fr5_B.mix_ctr(0)
#         self.rd_ctr(1)
#         time.sleep(5)
#         fr5_B.mix_ctr(1)
#         self.rd_ctr(0)
#         time.sleep(1)
#         # # 关闭移液泵1
#         # fr5_B.press(0,0,v)   
#         # # 放回触摸棒
#         # fr5_B.put_stick(v)
        
#         self.Ex_back_mix_no_rA(v)

#         # fr5_B.catch_stick(1,v)
#         # # 启动气泵
#         # fr5_B.press_pump(1,v)

#         # fr5_B.put_stick(v)

#         self.dou_pour_wash(fr5_B, v)

#         # # 关闭气泵
#         # fr5_B.catch_stick(1,v)

#         # fr5_B.press_pump(0,v)

#         # fr5_B.put_stick(v)

#         self.New_Gua("1",fr5_B)

#     def flow2_1(self, fr5_B, v = 50.0):
#         self.dou_go_start(fr5_B, v)
#         # 加料到A
#         self.Ex_Pour(v)        
#         self.Ex_Pour_solid(v, 1, 1, 0)
#         self.Ex_Pour_solid(v, 1, 2)

#         # 抽滤
#         self.dou_pour_wash(fr5_B, v, 1)
        
#         ## 待办 ##
        
#     def flow2_2(self, fr5_B, v = 50.0):
#         # 加料到B
#         self.Ex_Pour(v, 1, 1)
#         self.Ex_Pour_solid(v, 1, 1)

#         # 搅拌
#         self.EX_to_mix_no_rA(v, 1)

#         fr5_B.mix_ctr(0)
#         self.rd_ctr(1)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         self.rd_ctr(0)
#         time.sleep(1)
#         # 加液
#         # input("手动加液，按回车继续")
#         # 移液
#         self.Ex_back_mix_no_rA(v, 1, 1)
        
#         self.grab_rubber(fr5_B)
#         # 抽滤瓶内液体倒入反应瓶A

#         # 辅助机械臂上移让位
#         # pos = fr5_B.robot.GetActualTCPPose(0)[1:7]
#         # pos[2] += 200.0
#         # print(pos)
#         # fr5_B.robot.MoveCart(pos, 0, 0, 0.0, 0.0, v, -1.0, -1)

#         self.Ex_Put_Funnel(v)
        
#         self.Ex_Pour_filter(v)
        
#         self.put_rubber(fr5_B)

#         self.Ex_Back_Funnel(v)

#         self.dou_pour_wash(fr5_B, v, catch=1)

#         # 刮取烘干
#         self.New_Gua("1", fr5_B)
#         ## 待办 ##
#         self.dou_go_start(fr5_B, v)
    
#     def flow2_3(self, fr5_B, v = 50.0):
#         self.Ex(fr5_B, v)

#     def flow2_4(self, fr5_B, v = 50.0):
#         # 加料到c
#         self.Ex_Pour(v,1,1)
#         self.Ex_Pour_solid(v,1,1)


#         # 搅拌
#         self.EX_to_mix_no_rA(v)

#         fr5_B.mix_ctr(0)
#         self.rd_ctr(1)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         self.rd_ctr(0)
#         time.sleep(1)
#         # 加液
#         # input("手动加液，按回车继续")

#         # 移液
#         self.Ex_back_mix_no_rA(v)
#         # 抽滤取固
#         self.dou_pour_wash(fr5_B, v)
        
#     def Ex2(self, fr5_B, v = 50.0):
#         # 初始化
#         self.Go_to_start_zone(v)
#         fr5_B.Go_to_start_zone(v)
#         # 流程1
#         self.flow2_1(fr5_B, v)

#         # 流程2
#         self.flow2_2(fr5_B, v)

#         # 流程3
#         self.flow2_3(fr5_B, v)

#         # 流程4
#         self.flow2_4(fr5_B, v)

#         # 结束
#         self.Go_to_start_zone(v)
#         fr5_B.Go_to_start_zone(v)

#     def flow3_1(self,fr5_B,v=50.0):
#         print()
#         # 加水
#         self.Ex_Pour(v, 1, 1)
#         # A将固体A倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 1)
#         # 搅拌
#         self.EX_to_mix_no_rA(v)

#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
#         # input("手动搅拌，按回车继续")
        
#         self.Ex_back_mix_no_rA(v, 1, 1)

#     def flow3_2(self,fr5_B,v=50.0):
#         print()
#         self.Ex_Pour(v, 1, 1)
#         # A将固体D倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 1, 0)
#         # A将固体E倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 2)
        
#         # 开启蠕动泵（滴加两种液体）
#         self.EX_to_mix_no_rA(v, 1)
#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
#         # input("滴加液体，回车继续")
#         self.Ex_back_mix_no_rA(v, 1, 1)

#     def flow3_3(self,fr5_B,v=50.0):
#         print()
#         # A将水加入三经烧瓶C
#         self.Ex_Pour(v, 1)
#         # A将固体G倒入三颈烧瓶C
#         self.Ex_Pour_solid(v, 1, 1, 0)
#         # 搅拌同时，B将固体F分两次加入三经烧瓶C
#         self.Ex_Pour_solid(v, 1, 2)
    
#         # 搅拌
#         self.EX_to_mix_no_rA(v, 1)

#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
#         # 停止搅拌

#         self.Ex_back_mix_no_rA(v, 1, 1)

#         # 插入漏斗
#         self.Ex_Put_Funnel(v, 1)
        
#         # B移动反应瓶C到抽滤处
#         self.put_sp_to_choulv(fr5_B)
#         # 把三经烧瓶B的液体倒入三经烧瓶C中 对三经烧瓶B冲洗，冲洗液倒入三经烧瓶C
#         self.dou_pour_wash(fr5_B, v, catch=1, reaction_index=2)
        
#         # B拿走布什漏斗
#         self.Ex_Back_Funnel_v2(fr5_B)
#         # B将三颈烧瓶C放回原位
#         self.put_sp_to_start(fr5_B)

#         # self.Ex_Back_Funnel(fr5_B)
#         # # A将固体H倒入三颈烧瓶C
#         # self.Ex_Pour_solid(v, 1, 3)

#         self.dou_go_start(fr5_B, v)
        


#     def flow3_4(self,fr5_B,v=50.0):
#         print()    
#         # A将液体C加入烧杯
#         # A将液体I加入烧杯
#         self.Ex_Pour(v,1,1)
#         self.Ex_Pour(v,1,2)
#         # 搅拌
#         self.EX_to_mix_no_rA(v, 1)
        
#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
#         # input("手动搅拌，按回车继续")

#         self.Ex_back_mix_no_rA(v)
        
#         # A将三经烧瓶C的液体倒入砂芯漏斗
#         self.dou_pour_wash(fr5_B, v, wash = 1)
#         # 分四次 用水对漏斗里的滤饼进行洗涤 
#         # 批示：滤饼里的水滤干净都难，还要冲洗吗  
        
        
#         # 刮取分离
#         self.New_Gua("1", fr5_B)

        
        
#     def flow3_5(self,fr5_B,v=50.0):
#         # 以下四条建议单独开一个流程
#         # A把得到的固体倒入烧杯
#         self.Ex_Pour_solid(v, 1, 1)
#         # 对烧杯的东西进行搅拌
#         self.EX_to_mix_no_rA(v, 1)
        
#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
#         # input("手动搅拌，按回车继续")
        
#         self.Ex_back_mix_no_rA(v)
#         # 对烧杯里的物料倒入漏斗进行抽滤
#         self.dou_pour_wash(fr5_B, v)
#         # 刮取分离
#         # input('-----------抽滤完了吗？wjj给我按回车！！！！---------')
#         self.New_Gua("1", fr5_B)

#     def Ex3(self,fr5_B, v=50.0):
#         # 初始化
#         self.Go_to_start_zone(v)
#         fr5_B.Go_to_start_zone(v)

#         self.flow3_1(fr5_B)

#         self.flow3_2(fr5_B)

#         self.flow3_3(fr5_B)

#         self.flow3_4(fr5_B)
        
#     def flow4_1(self,fr5_B,v=50.0):
#         print()
#         # 加水
#         self.Ex_Pour(v, 1, 1)
#         # A将固体A倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 1)
#         # 搅拌
#         self.EX_to_mix_no_rA(v)
#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
#         # input("开启搅拌，回车继续")
#         self.Ex_back_mix_no_rA(v, 1, 1)

#     def flow4_2(self,fr5_B,v=50.0):
#         print()
#         # A将固体D倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 1, 0)
#         # A将固体E倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 2)
        
#         # 搅拌，开启蠕动泵（滴加两种液体）
#         self.EX_to_mix_no_rA(v, 1)
#         fr5_B.mix_ctr(0)
#         self.rd_ctr(1)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         self.rd_ctr(0)
#         time.sleep(1)
#         # input("开启搅拌，滴加液体，回车继续")
#         self.Ex_back_mix_no_rA(v, 1, 1)

#     def flow4_3(self,fr5_B,v=50.0):
#         print()
#         # A将液体I加入三经烧瓶C
#         self.Ex_Pour(v, 1, 1)
#         # A将液体M加入三经烧瓶C
#         self.Ex_Pour(v, 1, 2)

#         self.Ex_Pour(v, 1, 3)
#         # A将固体k倒入三颈烧瓶
#         self.Ex_Pour_solid(v, 1, 3)
        

#         # 搅拌
#         self.EX_to_mix_no_rA(v, 1)
#         # 开启蠕动泵（滴加两种液体）
#         # 开启蠕动泵（滴加液体k）
#         fr5_B.mix_ctr(0)
#         self.rd_ctr(1)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         self.rd_ctr(0)
#         time.sleep(1)
#         # input("开启搅拌，滴加液体，回车继续")
        
#         self.Ex_back_mix_no_rA(v, 1, 1)
        
#     def flow4_4(self,fr5_B,v=50.0):
#         print()    
#         # A将液体M加入三经烧瓶D
#         self.Ex_Pour(v, 1, 1)
#         # A将液体O加入三经烧瓶D
#         self.Ex_Pour(v, 1, 2) 
#         # A将三经烧瓶D的液体倒入砂芯漏斗,抽滤
        
#         self.dou_pour_wash(fr5_B, v, catch = 1)
        
#         # 分两次 用液体M对漏斗里的滤饼进行洗涤(选做)

#         # 刮取分离
#         self.New_Gua("1", fr5_B)
        
        
#     def flow4_5(self,fr5_B,v=50.0):
#         # 以下四条建议单独开一个流程
#         # A把得到的固体倒入三经烧瓶D
#         self.Ex_Pour_solid(v, 1, 1, 0)
#         # A将固体P加入三经烧瓶D
#         self.Ex_Pour_solid(v, 1, 2, 0) 
#         # A分三次把固体Q加入到三经烧瓶D中
#         self.Ex_Pour_solid(v, 1, 3 )
#         # 对三经烧瓶的东西进行搅拌
#         self.EX_to_mix_no_rA(v, 1)
        
#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)
#         time.sleep(1)
        
#         self.Ex_back_mix_no_rA(v, 1, 1)
        

#     def Ex4(self,fr5_B, v=50.0):
#         print()
#         self.flow4_1(fr5_B)

#         self.flow4_2(fr5_B)

#         self.flow4_3(fr5_B)

#         self.flow4_4(fr5_B)

#     # 金属盐
#     def Ex5(self,fr5_B,v=50.0):
#         # A将液体M加入三经烧瓶D
#         self.Ex_Pour(v, 1, 1)
#         # A将液体O加入三经烧瓶D
#         self.Ex_Pour(v, 1, 2) 
        
#         self.Ex_Pour(v, 1, 3) 

#         self.Ex_Pour_solid(v, 1, 3)

#         # 对三经烧瓶的东西进行搅拌
#         self.EX_to_mix_no_rA(v, 1)
        
#         fr5_B.mix_ctr(0)
#         time.sleep(10)
#         fr5_B.mix_ctr(1)

#         time.sleep(1)
#         self.Ex_back_mix_no_rA(v)
#         # 对烧杯里的物料倒入漏斗进行抽滤
        
#         self.dou_pour_wash(fr5_B, v)
        
#         # 刮取分离
#         input('-----------抽滤完了吗？wjj给我按回车！！！！---------')
#         self.New_Gua("1", fr5_B)




# def main():
#     print("---------------FR5机械臂化学协作实验演示------------------\n")
#     input("----------按下回车开始运行-----------\n")
#     # 初始化机械臂A
#     fr5 = ChemistryTest() 
#     # fr5.EX_to_mix()
#     while True:
#         print(
#             "--------请选择要执行的任务：--------\n\
#                 1---4种对象抓取\n\
#                 2---10次抓取 指定位置指定对象\n\
#                 3---10次抓取 100ml水\n\
#                 4---10次移动\n\
#                 5---10次移动 70ml水\n\
#                 6---10次移动 100ml水\n\
#                 7---预设放置\n\
#                 8---指定放置\n\
#                 9---10次预设倾倒\n\
#                 10---10次指定倾倒\n\
#                 11---搅拌\n\
#                 13---刮取\n\
#             "
#         )

#         task_index = input("----输入任务序号（输入q以退出）----\n")
#         if task_index == "1":
#             fr5.F101_catch_02()  # 4种对象抓取
#         elif task_index == "2":
#             fr5.F101_catch_03()  # 10次抓取 指定位置指定对象
#         elif task_index == "3":
#             fr5.F101_catch_03()  # 10次抓取 100ml水
#         elif task_index == "4":
#             fr5.F101_move_01()  # 10次移动
#         elif task_index == "5":
#             fr5.F101_move_01()  # 10次移动 70ml水
#         elif task_index == "6":
#             fr5.F101_move_01()  # 10次移动 100ml水
#         elif task_index == "7":
#             fr5.F101_put_01()  # 10次预设放置
#         elif task_index == "8":
#             fr5.F101_put_01()  # 10次指定放置
#         elif task_index == "9":
#             fr5.F101_pour_03()  # 10次预设倾倒
#             print()
#         elif task_index == "10":
#             fr5.F101_pour_03()  # 10次指定倾倒
#             print()
#         elif task_index == "11":
#             fr5.F101_mix_01()
#         elif task_index == "12":
#             fr5.F101_catch_04()  # 水平方向抓取
#         elif task_index == "13":
#             fr5.Gua()  # 水平方向抓取
#         elif task_index == "q":
#             exit()
#         else:
#             input("任务不存在，请重新输入,回车确认\n")

# if __name__ == "__main__":
#     main()
