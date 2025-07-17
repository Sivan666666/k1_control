import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import q2r
import matplotlib.pyplot as plt
from math import pi
import mujoco
import mujoco.viewer
import time
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def pose_to_se3(x, y, z, rx, ry, rz):

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

        rospy.init_node('k1_dual_arm_controller', anonymous=True)
        
        # 创建轨迹发布者 (左右臂各一个)
        self.right_arm_pub = rospy.Publisher('/right_arm_controller/command', 
                                           JointTrajectory, 
                                           queue_size=10)
        self.left_arm_pub = rospy.Publisher('/left_arm_controller/command', 
                                          JointTrajectory, 
                                          queue_size=10)
        
        # 设置发布频率 (125Hz对应8ms间隔)
        self.publish_rate = rospy.Rate(125)  # 125Hz = 8ms

        
        if urdf_path is None:
            # 尝试自动查找URDF文件
            urdf_path = os.path.abspath(os.getcwd()) + '/model/K1/K1/urdf/k1_pgc.urdf'
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        
        # 从URDF加载机器人模型
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        
        self.model = mujoco.MjModel.from_xml_path("model/K1/k1_pgc_35_50_new.xml")
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
       
        
         # 关节索引定义 (根据实际URDF结构调整)
        self.right_arm_joints = [0, 1, 2, 3, 4, 5, 6]     # 右臂7个关节
        self.left_arm_joints = [9, 10, 11, 12, 13, 14, 15] # 左臂7个关节 
        self.right_gripper_joints = [7, 8]                  # 右夹爪2个关节
        self.left_gripper_joints = [16, 17]                 # 左夹爪2个关节
        
        # 末端执行器link名称
        self.right_ee_link = "right_gripper_base"
        self.left_ee_link = "left_gripper_base"
        
        # 初始关节位置 (示例值)
        self.qnow = np.zeros(18)
        self.qnow[self.right_arm_joints] = [1.31851687, -0.41459478, -0.32868551, -2.17855411, -0.68782104,  1.55630035, 2.65536049]
        self.qnow[self.left_arm_joints] = [-1.57412261, -0.80421948,  1.17120901, -1.96090677,  0.3898956,  -1.37025403, -2.08752968]
        #self.qnow[self.right_arm_joints] = [0.84, -0.67, -0.70, -2.42, -1.10, 0.90, -1.92]
        #self.qnow[self.left_arm_joints] = [-0.94, -0.73, 0.87, -2.42, 1.06, -0.78, -1.14]
        self.qnow[self.right_gripper_joints] = [0.01875, -0.01875]
        self.qnow[self.left_gripper_joints] = [0.01875, -0.01875]



        # 关节速度限制 (rad/s)
        self.velocity_limits = np.array([5.0] * 18)
        
        # 关节加速度限制 (rad/s^2)
        self.acceleration_limits = np.array([5.0] * 18)
        
        
        
        
        

        
        
        
    def forward_kinematics(self, q, arm='left'):
        """
        计算正运动学
        
        参数:
            q: 关节角度向量(rad)
            arm: 'left'或'right'，指定左臂或右臂
            
        返回:
            SE3: 末端执行器的位姿
        """
        if arm == 'left':
            ee_link = self.left_ee_link
            q_arm = q[self.left_arm_joints]
        else:
            ee_link = self.right_ee_link
            q_arm = q[self.right_arm_joints]
        
        # 计算正运动学
        T = self.robot.fkine(q, end=ee_link)
        return T
    
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
            
        else:
            #print("Right ARM")
            joint_indices = self.right_arm_joints
            ee_link = self.right_ee_link
            
        
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
            
            return q_sol, True
        else:
            return None, False

    def print_ik_result(self, q_sol):
        deg_values = np.rad2deg(q_sol)
        header = "IK求解成功！各个关节的度数:"
        
        # # 方法1：紧凑格式
        # with np.printoptions(formatter={'float': '{: .2f}'.format}, linewidth=1000):
        #     print(header, deg_values)
        
        # 方法2：表格格式（更专业）
        print(f"\n{header}")
        for i, val in enumerate(deg_values, 1):
            print(f"关节{i}: {val:7.2f}°", end=' | ')
        print()  # 换行

    def publish_trajectory(self, trajectory_data, arm='left'):
        """
        发布轨迹到ROS（8ms间隔发布，时间戳严格同步）
        
        参数:
            trajectory_data: 轨迹数据字典，包含time/position/velocity/acceleration
            arm: 'left'或'right'，指定左臂或右臂
        """
        # 创建轨迹消息
        traj_msg = JointTrajectory()
        
        # 设置关节名称 (根据实际机械臂调整)
        if arm == 'left':
            traj_msg.joint_names = [
                'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
                'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6',
                'left_arm_joint7'
            ]
            publisher = self.left_arm_pub
        else:
            traj_msg.joint_names = [
                'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3',
                'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6',
                'right_arm_joint7'
            ]
            publisher = self.right_arm_pub
        
        # 获取轨迹起始时间
        start_time = rospy.Time.now()
        
        # 计算总预期持续时间
        expected_duration = trajectory_data['time'][-1]
        
        # 逐步发布轨迹点
        for i in range(len(trajectory_data['time'])):
            point = JointTrajectoryPoint()
            point.positions = trajectory_data['position'][i].tolist()
            point.velocities = trajectory_data['velocity'][i].tolist()
            point.accelerations = trajectory_data['acceleration'][i].tolist()
            
            # 关键修改：time_from_start必须严格使用规划时的时间戳
            point.time_from_start = rospy.Duration(trajectory_data['time'][i])
            
            # 每次只发布一个点
            traj_msg.points = [point]
            traj_msg.header.stamp = start_time  # 使用统一的起始时间戳
            publisher.publish(traj_msg)
            
            # 计算实际已运行时间
            elapsed = (rospy.Time.now() - start_time).to_sec()
            expected = trajectory_data['time'][i]
            
            # 动态调整发布间隔（确保时间戳同步）
            if i < len(trajectory_data['time']) - 1:
                next_point_dt = trajectory_data['time'][i+1] - trajectory_data['time'][i]
                sleep_time = max(0.0, next_point_dt - (rospy.Time.now() - start_time).to_sec() + expected)
                rospy.sleep(sleep_time)
            
            # 调试日志（每50个点打印一次）
            if i % 50 == 0:
                rospy.loginfo(f"{arm}臂进度: {elapsed:.3f}/{expected_duration:.3f}s | 点 {i+1}/{len(trajectory_data['time'])}")

        rospy.loginfo(f"{arm}臂轨迹发布完成 | 实际用时: {(rospy.Time.now() - start_time).to_sec():.3f}s")

    def run_fling_with_toppra(self):
        """
        运行 Fling 动作
        """
        # 规划左臂轨迹
        left_traj = self.generate_fling_trajectory(arm='left')
        if left_traj is None:
            print("左臂轨迹规划失败")
            return
        
        # 规划右臂轨迹
        right_traj = self.generate_fling_trajectory(arm='right')
        if right_traj is None:
            print("右臂轨迹规划失败")
            return
            
        # 创建并启动两个线程分别发布左右臂轨迹
        import threading
        left_thread = threading.Thread(
            target=self.publish_trajectory,
            args=(left_traj, 'left')
        )
        right_thread = threading.Thread(
            target=self.publish_trajectory,
            args=(right_traj, 'right')
        )
        
        left_thread.start()
        right_thread.start()
        
        # 等待线程完成
        left_thread.join()
        right_thread.join()


        

    def visualize_dual_arm_trajectory(self, left_trajectory, right_trajectory):
        """
        在Mujoco仿真窗口中同时绘制左右机械臂末端轨迹
        
        参数:
            left_trajectory: 左臂轨迹数据，包含关节角度序列
            right_trajectory: 右臂轨迹数据，包含关节角度序列
        """
        # 定义左右臂的视觉属性
        left_arm_color = [1, 0, 0, 1]  # 红色
        right_arm_color = [0, 1, 0, 1]  # 绿色
        sphere_size = 0.003
        
        # 获取左右臂的父body ID
        left_parent_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "l_pgc50_base")
        right_parent_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "r_pgc50_base")
        
        if left_parent_id == -1 or right_parent_id == -1:
            print("Error: Cannot find arm body IDs")
            return
        
        # 计算左右臂的末端轨迹点
        left_positions = []
        right_positions = []
        
        max_length = max(len(left_trajectory), len(right_trajectory))
        
        for i in range(max_length):
            # 设置左臂关节角度
            if i < len(left_trajectory):
                self.data.qpos[9:16] = left_trajectory[i]  # 左臂关节索引7-13
            # 设置右臂关节角度
            if i < len(right_trajectory):
                self.data.qpos[0:7] = right_trajectory[i]  # 右臂关节索引0-6
                
            # 前向动力学计算
            mujoco.mj_forward(self.model, self.data)
            
            # 计算左臂末端位置
            left_geom_offset = np.array([0, 0, 0.1175])  # 根据实际XML调整
            left_parent_pos = self.data.xpos[left_parent_id]
            left_parent_mat = self.data.xmat[left_parent_id].reshape(3,3)
            left_end_pos = left_parent_pos + np.dot(left_parent_mat, left_geom_offset)
            if i < len(left_trajectory):
                left_positions.append(left_end_pos)
            
            # 计算右臂末端位置
            right_geom_offset = np.array([0, 0, 0.1175])  # 根据实际XML调整
            right_parent_pos = self.data.xpos[right_parent_id]
            right_parent_mat = self.data.xmat[right_parent_id].reshape(3,3)
            right_end_pos = right_parent_pos + np.dot(right_parent_mat, right_geom_offset)
            if i < len(right_trajectory):
                right_positions.append(right_end_pos)
        
        # 初始化场景几何体
        if not hasattr(self, 'viewer') or not hasattr(self.viewer, 'user_scn'):
            print("Error: Mujoco viewer not initialized")
            return
        
        # 重置场景中的几何体
        self.viewer.user_scn.ngeom = 0
        
        # 添加左臂轨迹点（红色）
        for i, pos in enumerate(left_positions):
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[i],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[sphere_size, 0, 0],
                pos=pos,
                mat=np.eye(3).flatten(),
                rgba=np.array(left_arm_color)
            )
        
        # 添加右臂轨迹点（绿色）
        for i, pos in enumerate(right_positions):
            mujoco.mjv_initGeom(
                self.viewer.user_scn.geoms[len(left_positions) + i],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[sphere_size, 0, 0],
                pos=pos,
                mat=np.eye(3).flatten(),
                rgba=np.array(right_arm_color)
            )
        
        # 设置总几何体数量
        self.viewer.user_scn.ngeom = len(left_positions) + len(right_positions)
        print(f"Added {len(left_positions)} left arm and {len(right_positions)} right arm trajectory points")


    def plan_trajectory_with_toppra(self, waypoints, arm='left'):
        """
        使用TOPPRA规划轨迹
        
        参数:
            waypoints: 路径点列表，每个路径点是关节角度向量
            arm: 'left'或'right'，指定左臂或右臂
            
        返回:
            q_trajectory: 规划后的轨迹，包含时间序列和关节角度
        """
        if arm == 'left':
            joint_indices = self.left_arm_joints
        else:
            joint_indices = self.right_arm_joints
            
        # 提取对应臂的关节角度
        waypoints_arm = [wp[joint_indices] for wp in waypoints]
        
        # 创建路径
        path_scalars = np.linspace(0, 1, len(waypoints_arm))
        path = ta.SplineInterpolator(path_scalars, waypoints_arm)
        print(path)
        # 创建约束
        vlim = np.vstack([-self.velocity_limits[joint_indices], 
                          self.velocity_limits[joint_indices]]).T
        alim = np.vstack([-self.acceleration_limits[joint_indices], 
                          self.acceleration_limits[joint_indices]]).T
        
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(
            alim, discretization_scheme=constraint.DiscretizationType.Interpolation)
        
        # 创建TOPPRA实例
        instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, solver_wrapper="seidel")
        
        # 计算轨迹
        jnt_traj = instance.compute_trajectory(0, 0)
        
        # 采样轨迹
        ts_sample = np.linspace(0, jnt_traj.get_duration(), 1000)
        qs_sample = jnt_traj.eval(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)
        print(jnt_traj.get_duration())
        print(qs_sample.shape[0])
        # # 绘制轨迹图
        # time_steps = np.arange(qs_sample.shape[0]) * 0.01

        fig, axs = plt.subplots(3, 1, figsize=(10, 8))

        for i in range(7):
            axs[0].plot(ts_sample, qs_sample[:, i], label=f'Joint {i+1}')
            axs[1].plot(ts_sample, qds_sample[:, i], label=f'Joint {i+1}')
            axs[2].plot(ts_sample, qdds_sample[:, i], label=f'Joint {i+1}')

        axs[0].set_title('Position')
        axs[1].set_title('Velocity')
        axs[2].set_title('Acceleration')

        for ax in axs:
            ax.set_xlabel('Time [s]')
            ax.legend()
            ax.grid()

        plt.tight_layout()
        plt.show()

        return {
            'time': ts_sample,
            'position': qs_sample,
            'velocity': qds_sample,
            'acceleration': qdds_sample,
            'duration': jnt_traj.get_duration()
        }
    
    def generate_fling_trajectory(self, arm='left'):
        """
        生成Fling动作的轨迹点并规划轨迹
        
        参数:
            arm: 'left'或'right'，指定左臂或右臂
            
        返回:
            trajectory: 规划后的轨迹
        """
        if arm == 'left':
            motion = [
                pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0),
                pose_to_se3(0.2, 0.3, 0.2, 160, 0, 0),
                pose_to_se3(0.32, 0.3, 0.22, 160, 0, 0),
                pose_to_se3(0.52, 0.3, 0.42, 130, -20, 35),
                pose_to_se3(0.35, 0.3, 0.20, 160, 0, 0),
                pose_to_se3(0.30, 0.3, 0.17, 160, 0, 0),
                pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0)
            ]
            print("Left ARM")
        else:  # right arm (mirrored version)
            motion = [
                pose_to_se3(0.20, -0.22, 0.15, -160, 0, 0),      # Y坐标取反，roll取反
                pose_to_se3(0.2, -0.3, 0.2, -160, 0, 0),         # Y坐标取反，roll取反
                pose_to_se3(0.32, -0.3, 0.22, -160, 0, 0),       # Y坐标取反，roll取反
                pose_to_se3(0.52, -0.3, 0.42, -130, -20, -35),   # Y坐标取反，roll/yaw取反
                pose_to_se3(0.35, -0.3, 0.20, -160, 0, 0),       # Y坐标取反，roll取反
                pose_to_se3(0.30, -0.3, 0.17, -160, 0, 0),       # Y坐标取反，roll取反
                pose_to_se3(0.20, -0.22, 0.15, -160, 0, 0)        # Y坐标取反，roll取反
            ]
            print("Right ARM")


        # 求解逆运动学得到关节空间路径点
        waypoints = []
        for pose in motion:
            q_sol, success = self.inverse_kinematics(pose, arm=arm, q0=self.qnow)
            if not success:
                print(f"逆运动学求解失败: {pose}")
                return None
            
            # 创建完整的关节角度向量
            full_q = self.qnow.copy()
            if arm == 'left':
                full_q[self.left_arm_joints] = q_sol
            else:
                full_q[self.right_arm_joints] = q_sol
                
            waypoints.append(full_q)
        
        # 使用TOPPRA规划轨迹
        trajectory = self.plan_trajectory_with_toppra(waypoints, arm=arm)
        return trajectory
        





if __name__ == "__main__":
    # 创建控制器实例
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")
        
        controller.run_fling_with_toppra()

        
    except Exception as e:
        print(f"初始化失败: {str(e)}")