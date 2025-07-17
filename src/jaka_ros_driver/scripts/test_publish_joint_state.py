#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def joint_state_publisher():
    """
    一个简单的节点，用于发布连续变化的/joint_states消息，以测试RViz中的机器人模型。
    """
    rospy.init_node('test_joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # === 关键：这个列表必须与你的URDF文件中的关节顺序完全一致 ===
    # (7 right arm + 2 right gripper + 7 left arm + 2 left gripper)
    joint_names = [
        "r-j1", "r-j2", "r-j3", "r-j4", "r-j5", "r-j6", "r-j7",
        "right_finger1_joint", "right_finger2_joint",  
        "l-j1", "l-j2", "l-j3", "l-j4", "l-j5", "l-j6", "l-j7",
        "left_finger1_joint", "left_finger2_joint" 
    ]

    # 设置发布频率，例如50Hz
    rate = rospy.Rate(50) 

    rospy.loginfo("启动关节状态测试发布器...")
    rospy.loginfo("将在RViz中使机器人挥手。")

    # 初始关节位置 (18个自由度)
    positions = [0.0] * 18

    # 动画参数
    amplitude = 0.7  # 摆动幅度 (弧度)
    frequency = 0.5  # 摆动频率 (Hz)

    while not rospy.is_shutdown():
        # 创建一个JointState消息
        js_msg = JointState()
        js_msg.header = Header()
        js_msg.header.stamp = rospy.Time.now()

        # 填充关节名
        js_msg.name = joint_names

        # 计算当前时间的正弦值来模拟摆动
        # rospy.Time.now().to_sec() 获取从ROS时间起点开始的总秒数
        current_time = rospy.Time.now().to_sec()

        # 让右臂的第2个关节和左臂的第2个关节动起来
        wave_angle = amplitude * math.sin(2 * math.pi * frequency * current_time)

        # 更新位置数组中对应关节的值
        # 索引1对应 "right_arm_joint2"
        # 索引10对应 "left_arm_joint2"
        positions[1] = wave_angle
        positions[10] = -wave_angle # 让左臂反向摆动，看起来更协调

        # 将计算好的位置赋值给消息
        js_msg.position = positions

        # 发布消息
        pub.publish(js_msg)

        # 按指定频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass