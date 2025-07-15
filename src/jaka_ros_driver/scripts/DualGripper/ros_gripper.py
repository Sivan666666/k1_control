#!/usr/bin/env python
import rospy
import threading
import dh_modbus_gripper
from std_msgs.msg import Int32

class GripperROSNode:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('dh_gripper_node', anonymous=True)
        
        # 参数配置（从ROS参数服务器获取）
        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.force = rospy.get_param('~force', 100)
        self.speed = rospy.get_param('~speed', 100)
        
        # 初始化夹爪
        self.m_gripper = dh_modbus_gripper.dh_modbus_gripper()
        self._initialize_gripper()
        
        # ROS话题订阅和发布
        rospy.Subscriber('gripper_target_position', Int32, self._target_position_callback)
        self.state_publisher = rospy.Publisher('gripper_state', Int32, queue_size=10)
        
        # 状态跟踪变量
        self.current_target = None
        self.is_moving = False

    def _initialize_gripper(self):
        """初始化夹爪连接和参数设置"""
        try:
            self.m_gripper.open(self.port, self.baudrate)
            self.m_gripper.Initialization()
            rospy.loginfo("等待夹爪初始化...")
            
            initstate = 0
            while not rospy.is_shutdown() and initstate != 1:
                initstate = self.m_gripper.GetInitState()
                rospy.sleep(0.2)
                
            self.m_gripper.SetTargetForce(self.force)
            self.m_gripper.SetTargetSpeed(self.speed)
            rospy.loginfo("夹爪初始化完成")
        except Exception as e:
            rospy.logerr(f"夹爪初始化失败: {str(e)}")
            rospy.signal_shutdown("硬件连接失败")

    def _target_position_callback(self, msg):
        """处理目标位置订阅消息"""
        target = msg.data
        rospy.loginfo(f"收到目标位置指令: {target}")
        self.current_target = target
        self.m_gripper.SetTargetPosition(target)
        self.is_moving = True

    def _publish_gripper_state(self):
        """持续发布夹爪状态（仅数字）"""
        rate = rospy.Rate(10)  # 10Hz发布频率
        while not rospy.is_shutdown():
            if self.is_moving:
                state = self.m_gripper.GetGripState()
                state_msg = Int32()
                state_msg.data = state
                self.state_publisher.publish(state_msg)
                # 0 means moving 1 means arrive 2 means grasping 3 means drop
                # 状态变化时记录日志
                if state != 0:  # 0表示移动中
                    self.is_moving = False
                    rospy.loginfo(f"夹爪状态更新: {state}")
            rate.sleep()

    def run(self):
        """启动状态发布线程"""
        state_thread = threading.Thread(target=self._publish_gripper_state)
        state_thread.daemon = True
        state_thread.start()
        rospy.spin()

    def shutdown(self):
        """节点关闭时的清理操作"""
        self.m_gripper.close()
        rospy.loginfo("夹爪连接已关闭")

if __name__ == '__main__':
    try:
        node = GripperROSNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass