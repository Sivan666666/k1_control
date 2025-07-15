#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32

def gripper_position_publisher():
    # 初始化ROS节点
    rospy.init_node('gripper_sequence_publisher', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('gripper_target_position', Int32, queue_size=10)
    rospy.loginfo("夹爪序列发布器已启动，等待订阅者连接...")
    
    # 等待订阅者连接（防止首次发布丢失）
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    # 预设位置序列 [全闭，半开，全开，抓取位置]
    position_sequence = [0, 500, 1000, 300]
    sequence_delay = 1  # 每个位置之间的延迟（秒）
    
    try:
        rospy.loginfo("开始执行预设夹爪位置序列...")
        while not rospy.is_shutdown():
            for idx, position in enumerate(position_sequence):
                # 发布位置指令
                pub.publish(position)
                rospy.loginfo(f"发布位置 #{idx+1}: {position}")
                
                # 等待动作完成（固定延迟）
                rospy.sleep(sequence_delay)
                
            rospy.loginfo("序列完成，重新开始循环...")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")

if __name__ == '__main__':
    gripper_position_publisher()