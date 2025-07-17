# -*- coding: utf-8 -*-

import rospy
import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from visualization_msgs.msg import Marker, MarkerArray
import os
import time
from spatialmath import SE3
import mujoco
import mujoco.viewer

def pose_to_se3(x, y, z, rx, ry, rz):
        """
        将 x, y, z, rx, ry, rz 转换为 SE3 对象
        :param x: 平移 x
        :param y: 平移 y
        :param z: 平移 z
        :param rx: 绕 x 轴的旋转角度（单位：度）
        :param ry: 绕 y 轴的旋转角度（单位：度）
        :param rz: 绕 z 轴的旋转角度（单位：度）
        :return: SE3 对象
        """
        # 将角度从度转换为弧度
        rx, ry, rz = np.deg2rad(rx), np.deg2rad(ry), np.deg2rad(rz)
        
        # 创建旋转矩阵
        R = SE3.Rz(rz) * SE3.Ry(ry) * SE3.Rx(rx)
        
        # 创建齐次变换矩阵
        T = SE3.Rt(R.R, [x, y, z])
        
        return T



class K1DualArmController:
    def __init__(self, urdf_path=None):
        """
        初始化K1双臂机器人控制器
        
        参数:
            urdf_path: URDF文件路径，如果为None则尝试默认路径
        """
        if urdf_path is None:
            # 尝试自动查找URDF文件
            urdf_path = '/home/hwk/ros_ws/k1_ros/src/jaka_ros_driver/scripts/model/K1/urdf/k1_pgc_j4_limit.urdf'
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        
        # 从URDF加载机器人模型
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        self.model = mujoco.MjModel.from_xml_path("/home/hwk/ros_ws/k1_ros/src/jaka_ros_driver/scripts/k1_simulation/model/K1/k1.xml")
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # 定义左右臂的关节索引（根据URDF实际结构调整）
        self.right_arm_joints = [0, 1, 2, 3, 4, 5, 6]    # 左臂关节索引
        self.left_arm_joints = [7, 8, 9, 10, 11, 12, 13]  # 右臂关节索引
        
        # 末端执行器名称（根据URDF中的link名称）
        self.left_ee_link = "lt"  # 根据实际URDF调整
        self.right_ee_link = "rt"  # 根据实际URDF调整
        self.qnow = np.zeros(14) 
        self.qnow = np.array([0.83988522,-0.66850441,-0.69920311,-2.42284396,-1.10251352,0.89649283,-1.9211578,-0.94049207,-0.73311629,0.86677897,-2.42284663,1.05591172,-0.78310933,-1.13897499])
        self.Kp = 100.0  # 比例增益
        self.Kd = 5.0   # 微分增益

        # 关节速度限制 (rad/s)
        self.velocity_limits = np.array([3.0] * 14)
        
        # 关节加速度限制 (rad/s^2)
        self.acceleration_limits = np.array([3.0] * 14)


        self.joint_limits = {
            'left': [
                (-6.2832, 6.2832), (-1.8325, 1.8325), (-6.2832, 6.2832), (-2.5307, 0.5235), (-6.2832, 6.2832), (-1.8325, 1.8325), (-6.2832, 6.2832)
            ],
            'right': [
                (-6.2832, 6.2832), (-1.8325, 1.8325), (-6.2832, 6.2832), (-2.5307, 0.5235), (-6.2832, 6.2832), (-1.8325, 1.8325), (-6.2832, 6.2832)
            ]
        }
    
    def inverse_kinematics(self, T_desired, arm='left', q0=None, tol=1e-6, max_iter=10000):
        """
            逆运动学求解
            
            参数:
                T_desired: 期望的末端位姿(SE3)
                arm: 'left'或'right'，指定左臂或右臂
                q0: 初始关节角度猜测(可选)
                tol: 容差
                max_iter: 最大迭代次数
                
            返回:
                q_sol: 解得的关节角度
                success: 是否成功求解
        """
        if arm == 'left':
            #print("Left ARM")
            joint_indices = self.left_arm_joints
            ee_link = self.left_ee_link
            joint_limits = self.joint_limits['left']
        else:
            #print("Right ARM")
            joint_indices = self.right_arm_joints
            ee_link = self.right_ee_link
            joint_limits = self.joint_limits['right']
        
        # 如果没有提供初始猜测，使用零位
        if q0 is None:
            q0 = np.zeros(self.robot.n)
        
        # 设置QP参数
        kq = 1.0  # 关节限制避免增益
        km = 0.0  # 可操作性最大化增益 (0表示禁用)
        

        # 使用机器人工具箱的IK求解
        sol = self.robot.ikine_LM(
            T_desired, 
            end=ee_link,
            q0=q0[joint_indices],
            mask=[1, 1, 1, 1, 1, 1],  # 控制位置和方向
            tol=tol,
            joint_limits=True,
            ilimit=max_iter,
            kq = 1.0,  # 关节限制避免增益
            km = 0.0,  # 可操作性最大化增益 (0表示禁用)
            method='sugihara' 
        )

        # # 使用机器人工具箱的IK求解
        # sol = self.robot.ikine_LM(
        #     T_desired, 
        #     end=ee_link,
        #     q0=q0[joint_indices],
        #     mask=[1, 1, 1, 1, 1, 1],  # 控制位置和方向
        #     tol=tol,
        #     ilimit=max_iter
        # )
        #print("solution:",sol)
        if sol.success:
            # 只返回对应臂的关节角度
            q_sol = sol.q[:7]
            self.print_ik_result(q_sol)
            #print("IK求解成功！各个关节的度数:",np.rad2deg(q_sol))
            for i, (q, limit) in enumerate(zip(q_sol, joint_limits)):
                if q < limit[0] or q > limit[1]:
                    print(f"关节 {i} 超出限制范围: {q} 不在 {limit} 内")
                    return None, False
            return q_sol, True
        else:
            return None, False




def main():
    rospy.init_node('manipulability_map_generator', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    
    # 1. 实例化您自己的控制器
    try:
        controller = K1DualArmController()
        robot = controller.robot # 从控制器中获取rtb机器人对象
    except FileNotFoundError as e:
        rospy.logerr(e)
        return
        
    # 2. 定义扫描参数
    ARM_TO_VISUALIZE = 'right' # 可以改为 'left'
    
    if ARM_TO_VISUALIZE == 'right':
        ee_link_name = controller.right_ee_link
        arm_joint_indices = controller.right_arm_joints
        # 只扫描右侧空间
        y_range = np.arange(-0.6, 0.0, 0.08)
    else:
        ee_link_name = controller.left_ee_link
        arm_joint_indices = controller.left_arm_joints
        # 只扫描左侧空间
        y_range = np.arange(0.6, 0.5, 0.08)

    rospy.loginfo(f"将为 '{ARM_TO_VISUALIZE}' 臂, 末端 '{ee_link_name}' 生成地图...")
    
    x_range = np.arange(0.2, 0.9, 0.08)
    z_range = np.arange(0.2, 0.8, 0.08)
    IK_RESTARTS = 5

    # 3. 开始扫描
    marker_array = MarkerArray()
    marker_id = 0
    total_points = len(x_range) * len(y_range) * len(z_range)
    rospy.loginfo(f"开始扫描工作空间，共 {total_points} 个点...")
    
    rospy.sleep(2.0)

    for x in x_range:
        for y in y_range:
            for z in z_range:
                if rospy.is_shutdown():
                    return

                T_desired = sm.SE3(x, y, z) * sm.SE3.Ry(-np.pi / 2) * sm.SE3.Rz(-np.pi / 2)
                max_manip_at_point = 0.0

                for _ in range(IK_RESTARTS):
                    # 创建一个随机的、完整的14/18关节初始猜测
                    q0_full = np.random.uniform(low=robot.qlim[0, :], high=robot.qlim[1, :])
                    
                    # 使用您自己的、可靠的IK函数
                    q_sol_arm, success = controller.inverse_kinematics(
                        T_desired, arm=ARM_TO_VISUALIZE, q0=q0_full
                    )
                    
                    if success:
                        # 成功后，需要构建完整的关节状态来计算雅可比
                        q_full_state = np.zeros(robot.n)
                        q_full_state[arm_joint_indices] = q_sol_arm
                        
                        J_full = robot.jacob0(q_full_state, end=ee_link_name)
                        J_arm = J_full[:, arm_joint_indices]
                        J_pos = J_arm[:3, :]
                        
                        try:
                            identity_matrix = np.eye(J_pos.shape[0]) * 1e-6
                            manipulability = np.sqrt(np.linalg.det(J_pos @ J_pos.T + identity_matrix))
                            
                            if manipulability > max_manip_at_point:
                                max_manip_at_point = manipulability
                        except np.linalg.LinAlgError:
                            pass
                
                # 创建并发布Marker (逻辑同前)
                if max_manip_at_point > 1e-3:
                    marker = Marker()
                    marker.header.frame_id = robot.base_link.name
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "manipulability_map"
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = z
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.045
                    marker.scale.y = 0.045
                    marker.scale.z = 0.045
                    val = min(max_manip_at_point / 0.4, 1.0)
                    marker.color.r = min(1.0, 2.0 * (1 - val))
                    marker.color.g = min(1.0, 2.0 * val)
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                    marker_array.markers.append(marker)
                    marker_id += 1

                rospy.loginfo_throttle(1.0, f"进度: {marker_id}/{total_points}, 当前点({x:.2f},{y:.2f},{z:.2f}) 最大可操作性: {max_manip_at_point:.4f}")
    
    if len(marker_array.markers) > 0:
        rospy.loginfo(f"扫描完成！共生成 {len(marker_array.markers)} 个可视化标记。正在发布到RViz...")
        marker_pub.publish(marker_array)
        rospy.loginfo("发布完成！请在RViz中添加一个MarkerArray显示，并订阅 /visualization_marker_array 话题。")
    else:
        rospy.loginfo("扫描完成，但没有找到任何可达且可操作的点。请检查你的扫描范围或IK设置。")
        
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
