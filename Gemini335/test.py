#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray

def tag_trans_mat_callback(msg):
    # 接收到tag_trans_mat消息后打印
    rospy.loginfo("收到tag_trans_mat数据: %s" % str(msg.data))

def listener():
    # 初始化ROS节点
    rospy.init_node('tag_trans_mat_listener', anonymous=True)
    # 订阅tag_trans_mat话题
    rospy.Subscriber('tag_trans_mat', Float32MultiArray, tag_trans_mat_callback)
    # 保持节点运行，等待回调
    rospy.spin()

if __name__ == '__main__':
    listener()
