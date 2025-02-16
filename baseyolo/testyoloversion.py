# """
# @author: 李文皓
# @function: 测试YOLO_version.py代码的正确性
# """
# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import Int32

# def stage_controller():
#     # 初始化ROS节点
#     rospy.init_node('stage_controller', anonymous=True)
    
#     # 创建发布器
#     pub = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)
    
#     # 初始阶段状态
#     stage_status = 0
#     pub.publish(stage_status)
    
#     print("阶段控制器已启动")
#     print("当前阶段: 0 (眼在手外)")
#     print("按 '1' 切换到阶段1 (眼在手上)")
#     print("按 'q' 退出程序")

#     try:
#         while not rospy.is_shutdown():
#             # 获取用户输入
#             key = input()
            
#             if key == '1':
#                 if stage_status == 0:
#                     stage_status = 1
#                     pub.publish(stage_status)
#                     print("\n切换到阶段1 (眼在手上)")
#                     print("正在发送信号: 1")
#                 else:
#                     print("\n已经是阶段1，无需切换")
                    
#             elif key == 'q':
#                 print("\n退出程序")
#                 break
                
#             else:
#                 print(f"\n无效输入: {key}")
#                 print("请输入 '1' 或 'q'")

#     except KeyboardInterrupt:
#         print("\n程序被中断")

# if __name__ == '__main__':
#     try:
#         stage_controller()
#     except rospy.ROSInterruptException:
#         pass
# """
# @author: 李文皓
# @function: 测试YOLO_version.py代码的正确性
# """
# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import Int32, String

# def cup_pose_callback(msg):
#     """杯子坐标回调函数"""
#     print(f"接收到杯子坐标: {msg.data}")

# def button_pose_callback(msg):
#     """按钮坐标回调函数"""
#     print(f"接收到按钮坐标: {msg.data}")

# def stage_controller():
#     # 初始化ROS节点
#     rospy.init_node('stage_controller', anonymous=True)
    
#     # 创建发布器
#     pub = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)
    
#     # 创建订阅器
#     rospy.Subscriber('/vision/cup_pose', String, cup_pose_callback)
#     rospy.Subscriber('/vision/button_pose', String, button_pose_callback)
    
#     # 初始阶段状态
#     stage_status = 0
#     pub.publish(stage_status)
    
#     print("阶段控制器已启动")
#     print("当前阶段: 0 (眼在手外)")
#     print("按 '1' 切换到阶段1 (眼在手上)")
#     print("按 'q' 退出程序")

#     try:
#         while not rospy.is_shutdown():
#             # 获取用户输入
#             key = input()
            
#             if key == '1':
#                 if stage_status == 0:
#                     stage_status = 1
#                     pub.publish(stage_status)
#                     print("\n切换到阶段1 (眼在手上)")
#                     print("正在发送信号: 1")
#                 else:
#                     print("\n已经是阶段1，无需切换")
                    
#             elif key == 'q':
#                 print("\n退出程序")
#                 break
                
#             else:
#                 print(f"\n无效输入: {key}")
#                 print("请输入 '1' 或 'q'")

#     except KeyboardInterrupt:
#         print("\n程序被中断")

# if __name__ == '__main__':
#     try:
#         stage_controller()
#     except rospy.ROSInterruptException:
#         pass
"""
@author: 李文皓
@function: 测试YOLO_version.py代码的正确性
"""
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String
import threading

def cup_pose_callback(msg):
    """杯子坐标回调函数"""
    print(f"接收到杯子坐标: {msg.data}")

def button_pose_callback(msg):
    """按钮坐标回调函数"""
    print(f"接收到按钮坐标: {msg.data}")

def input_handler(pub):
    """用户输入处理函数"""
    stage_status = 0
    pub.publish(stage_status)
    
    print("阶段控制器已启动")
    print("当前阶段: 0 (眼在手外)")
    print("按 '1' 切换到阶段1 (眼在手上)")
    print("按 'q' 退出程序")

    try:
        while not rospy.is_shutdown():
            # 获取用户输入
            key = input()
            
            if key == '1':
                if stage_status == 0:
                    stage_status = 1
                    pub.publish(stage_status)
                    print("\n切换到阶段1 (眼在手上)")
                    print("正在发送信号: 1")
                else:
                    print("\n已经是阶段1，无需切换")
                    
            elif key == 'q':
                print("\n退出程序")
                rospy.signal_shutdown("用户退出")
                break
                
            else:
                print(f"\n无效输入: {key}")
                print("请输入 '1' 或 'q'")

    except KeyboardInterrupt:
        print("\n程序被中断")

def stage_controller():
    # 初始化ROS节点
    rospy.init_node('stage_controller', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/arm/stage_status', Int32, queue_size=10)
    
    # 创建订阅器
    rospy.Subscriber('/vision/cup_pose', String, cup_pose_callback)
    rospy.Subscriber('/vision/button_pose', String, button_pose_callback)
    
    # 启动用户输入线程
    input_thread = threading.Thread(target=input_handler, args=(pub,))
    input_thread.daemon = True  # 设置为守护线程，主线程退出时自动退出
    input_thread.start()
    
    # 主线程保持运行，处理ROS回调
    rospy.spin()

if __name__ == '__main__':
    try:
        stage_controller()
    except rospy.ROSInterruptException:
        pass