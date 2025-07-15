#!/usr/bin/env python
import rospy
import threading
import dh_modbus_gripper
import dh_device
from std_msgs.msg import Int32

class DualGripperROSNode:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('dh_dual_gripper_node', anonymous=True)
        
        # 初始化两个独立的dh_device实例
        self.devices = [
            dh_device.dh_device(),
            dh_device.dh_device()
        ]
        
        # 初始化两个夹爪
        self.grippers = [
            self._init_single_gripper(
                device=self.devices[0],
                port=rospy.get_param('~gripper1_port', '/dev/ttyUSB0'),
                baudrate=rospy.get_param('~baudrate', 115200),
                force=rospy.get_param('~force', 100),
                speed=rospy.get_param('~speed', 100),
                gripper_id=1
            ),
            self._init_single_gripper(
                device=self.devices[1],
                port=rospy.get_param('~gripper2_port', '/dev/ttyUSB1'),
                baudrate=rospy.get_param('~baudrate', 115200),
                force=rospy.get_param('~force', 100),
                speed=rospy.get_param('~speed', 100),
                gripper_id=2
            )
        ]
        
        # ROS话题订阅和发布
        rospy.Subscriber('gripper_target_position', Int32, self._target_position_callback)
        self.state_publishers = [
            rospy.Publisher('gripper1_state', Int32, queue_size=10),
            rospy.Publisher('gripper2_state', Int32, queue_size=10)
        ]
        
        # 状态跟踪变量
        self.current_target = None
        self.is_moving = [False, False]

    def _init_single_gripper(self, device, port, baudrate, force, speed, gripper_id):
        """初始化单个夹爪"""
        try:
            rospy.loginfo(f"正在初始化夹爪{gripper_id} ({port})...")
            gripper = dh_modbus_gripper.dh_modbus_gripper()
            
            # 使用独立的device实例
            gripper.m_device = device  # 替换默认的全局m_device
            
            ret = gripper.open(port, baudrate)
            if ret < 0:
                raise Exception(f"打开端口失败: {port}")
            
            gripper.Initialization()
            
            # 等待初始化完成
            initstate = 0
            while not rospy.is_shutdown() and initstate != 1:
                initstate = gripper.GetInitState()
                rospy.sleep(0.2)
            
            gripper.SetTargetForce(force)
            gripper.SetTargetSpeed(speed)
            rospy.loginfo(f"夹爪{gripper_id} 初始化完成")
            return gripper
        except Exception as e:
            rospy.logerr(f"夹爪{gripper_id} 初始化失败: {str(e)}")
            raise

    def _target_position_callback(self, msg):
        """处理目标位置订阅消息"""
        target = msg.data
        rospy.loginfo(f"收到目标位置指令: {target}")
        self.current_target = target
        
        # 同时控制两个夹爪
        for i, gripper in enumerate(self.grippers):
            try:
                gripper.SetTargetPosition(target)
                self.is_moving[i] = True
            except Exception as e:
                rospy.logerr(f"夹爪{i+1} 控制失败: {str(e)}")

    def _publish_gripper_state(self):
        """持续发布两个夹爪的状态"""
        rate = rospy.Rate(10)  # 10Hz发布频率
        while not rospy.is_shutdown():
            for i, gripper in enumerate(self.grippers):
                if self.is_moving[i]:
                    state = gripper.GetGripState()
                    state_msg = Int32()
                    state_msg.data = state
                    self.state_publishers[i].publish(state_msg)
                    
                    # 状态变化时记录日志
                    if state != 0:  # 0表示移动中
                        self.is_moving[i] = False
                        rospy.loginfo(f"夹爪{i+1} 状态更新: {state}")
            rate.sleep()

    def run(self):
        """启动状态发布线程"""
        state_thread = threading.Thread(target=self._publish_gripper_state)
        state_thread.daemon = True
        state_thread.start()
        rospy.spin()

    def shutdown(self):
        """节点关闭时的清理操作"""
        for i, gripper in enumerate(self.grippers):
            try:
                gripper.close()
                rospy.loginfo(f"夹爪{i+1} 连接已关闭")
            except Exception as e:
                rospy.logerr(f"夹爪{i+1} 关闭失败: {str(e)}")

if __name__ == '__main__':
    try:
        node = DualGripperROSNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点启动失败: {str(e)}")