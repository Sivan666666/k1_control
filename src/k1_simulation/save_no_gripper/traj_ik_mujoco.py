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

    
    def test_kinematics(self):
        """测试正逆运动学"""
        print("=== K1双臂机器人运动学测试 ===")
        
        # 初始关节角度（零位）
        q_home = np.zeros(self.robot.n)
        q_home = np.array([np.deg2rad(85),np.deg2rad(-42), np.deg2rad(4.6), np.deg2rad(-97), np.deg2rad(-3.7),np.deg2rad(44.7),np.deg2rad(-1.05),np.deg2rad(-92),np.deg2rad(-26), np.deg2rad(5), np.deg2rad(-97), np.deg2rad(-3.7),np.deg2rad(-55),np.deg2rad(65)])
        print(f"初始关节角度: {np.degrees(q_home)} deg")
        
        
        
        # ===== 左臂测试 =====
        print("\n--- 左臂测试 ---")
        
        # 计算初始正运动学
        T_left_init = self.forward_kinematics(q_home, arm='left')
        print(f"左臂初始末端位姿:\n{T_left_init}")
        
        # 设置目标位姿（在初始位姿基础上偏移）
        # 给定的数据（按行排列）
        matrix_data = [
            [-0.87, -0.4929, -0.01306, 0.005789],
            [0.03005, -0.02658, -0.9992, 0.189],
            [0.4922, -0.8697, 0.03793, 0.5666],
            [0, 0, 0, 1]
        ]
        # 创建SE3对象
        T_left_goal = SE3(matrix_data)
        T_left_goal = pose_to_se3(0.013, -0.958, 0.583, -140.6, 86.7, -50)
        print(T_left_goal)
        #T_left_goal = T_left_init * SE3.Trans(0.1, 0.1, 0.1) * SE3.Rx(pi/4)
        print(f"\n左臂目标位姿:\n{T_left_goal}")
        
        # 求解逆运动学
        q_left_sol, success = self.inverse_kinematics(T_left_goal, arm='left', q0=q_home)
        
        if success:
            print(f"\n左臂逆解成功! 关节角度: {np.degrees(q_left_sol)} deg")
            
            # 更新整个机器人的关节角度（只更新左臂）
            q_new = q_home.copy()
            q_new[self.left_arm_joints] = q_left_sol
            
            # 计算验证正运动学
            T_left_achieved = self.forward_kinematics(q_new, arm='left')
            print(f"\n左臂实际达到位姿:\n{T_left_achieved}")
            print(f"位置误差: {np.linalg.norm(T_left_goal.t - T_left_achieved.t):.6f} m")
            
            
        else:
            print("左臂逆解失败!")
        
        # ===== 右臂测试 =====
        print("\n--- 右臂测试 ---")
        
        # 计算初始正运动学
        T_right_init = self.forward_kinematics(q_home, arm='right')
        print(f"右臂初始末端位姿:\n{T_right_init}")
        
        # 设置目标位姿
        T_right_goal = T_right_init * SE3.Trans(0.3, -0.1, 0.1) * SE3.Rx(-pi/4)
        print(f"\n右臂目标位姿:\n{T_right_goal}")
        
        # 求解逆运动学
        q_right_sol, success = self.inverse_kinematics(T_right_goal, arm='right', q0=q_home)
        
        if success:
            print(f"\n右臂逆解成功! 关节角度: {np.degrees(q_right_sol)} deg")
            
            # 更新整个机器人的关节角度（只更新右臂）
            q_new = q_home.copy()
            q_new[self.right_arm_joints] = q_right_sol
            
            # 计算验证正运动学
            T_right_achieved = self.forward_kinematics(q_new, arm='right')
            print(f"\n右臂实际达到位姿:\n{T_right_achieved}")
            print(f"位置误差: {np.linalg.norm(T_right_goal.t - T_right_achieved.t):.6f} m")
            
        else:
            print("右臂逆解失败!")


        self.qnow = q_new
        
    
    def run_fling_with_toppra(self):
        """
        运行 Fling 动作
        """
        while self.viewer.is_running():
            left_traj = self.generate_fling_trajectory(arm='left')
            if left_traj is None:
                print("左臂轨迹规划失败")
                return
            
            # 规划右臂轨迹
            right_traj = self.generate_fling_trajectory(arm='right')
            if right_traj is None:
                print("右臂轨迹规划失败")
                return
                
            self.viewer.sync()
            time.sleep(0.03)
            # # 可视化轨迹
            self.visualize_dual_arm_trajectory(left_traj['position'], right_traj['position'])

            self.execute_trajectory(left_traj, right_traj)
            #self.execute_trajectory(right_traj, arm='right')

        self.viewer.close()

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
        left_parent_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "l7")
        right_parent_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "r7")
        
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
                self.data.qpos[7:14] = left_trajectory[i]  # 左臂关节索引7-13
            # 设置右臂关节角度
            if i < len(right_trajectory):
                self.data.qpos[0:7] = right_trajectory[i]  # 右臂关节索引0-6
                
            # 前向动力学计算
            mujoco.mj_forward(self.model, self.data)
            
            # 计算左臂末端位置
            left_geom_offset = np.array([0, 0, 0.203172])  # 根据实际XML调整
            left_parent_pos = self.data.xpos[left_parent_id]
            left_parent_mat = self.data.xmat[left_parent_id].reshape(3,3)
            left_end_pos = left_parent_pos + np.dot(left_parent_mat, left_geom_offset)
            if i < len(left_trajectory):
                left_positions.append(left_end_pos)
            
            # 计算右臂末端位置
            right_geom_offset = np.array([0, 0, 0.203172])  # 根据实际XML调整
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
        # 生成Fling动作的路径点

        # bu dui cheng
        # if arm == 'left':
        #     motion = []
        #     motion.append(pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0))
        #     motion.append(pose_to_se3(0.2, 0.20, 0.2, 160, 0, 0))
        #     motion.append(pose_to_se3(0.32, 0.20, 0.22, 160, 0, 0))
        #     motion.append(pose_to_se3(0.52, 0.20, 0.42, 130, -20, 35))
        #     motion.append(pose_to_se3(0.35, 0.20, 0.20, 160, 0, 0))
        #     motion.append(pose_to_se3(0.30, 0.20, 0.17, 160, 0, 0))
        #     motion.append(pose_to_se3(0.20, 0.20, 0.15, 160, 0, 0))
        # else:
        #     motion = []
        #     motion.append(pose_to_se3(0.20, -0.22, 0.15, -160, -15, 0))
        #     motion.append(pose_to_se3(0.2, -0.2, 0.2, -160, -15, 0))
        #     motion.append(pose_to_se3(0.32, -0.2, 0.22, -160, -15, 0))
        #     motion.append(pose_to_se3(0.52, -0.2, 0.42, -150, -30, -40))
        #     motion.append(pose_to_se3(0.35, -0.2, 0.20, -160, -15, 0))
        #     motion.append(pose_to_se3(0.30, -0.2, 0.17, -160, -15, 0))
        #     motion.append(pose_to_se3(0.20, -0.22, 0.15, -160, -15, 0))
        
        # dui cheng 
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
    
    def execute_trajectory(self, trajectory_left, trajectory_right):
        """
        执行规划好的轨迹
        
        参数:
            trajectory: 规划好的轨迹
            arm: 'left'或'right'，指定左臂或右臂
        """
        if trajectory_left is None or trajectory_right is None:
            print("无法执行轨迹: 轨迹规划失败")
            return
        


        # if arm == 'left':
        #     joint_indices = self.left_arm_joints
        #     ee_link = self.left_ee_link
        # else:mujoco.mj_forw
        #     joint_indices = self.right_arm_joints
        #     ee_link = self.right_ee_link

        start_time = time.time()
        
        while self.viewer.is_running():
            current_time = time.time() - start_time
            # left
            if current_time > trajectory_left['duration']:
                start_time = time.time()
                current_time = 0
                
            
            # 找到最近的轨迹点
            idx = np.searchsorted(trajectory_left['time'], current_time, side='left')
            if idx >= len(trajectory_left['time']):
                idx = len(trajectory_left['time']) - 1
            
            #print("Y")
            # 更新关节角度
            q_target = trajectory_left['position'][idx]
            self.qnow = q_target
            self.data.qpos[self.left_arm_joints] = self.qnow
            
            
            # right

            # 找到最近的轨迹点
            idx = np.searchsorted(trajectory_right['time'], current_time, side='left')
            if idx >= len(trajectory_right['time']):
                idx = len(trajectory_right['time']) - 1
            
            #print("Y")
            # 更新关节角度
            q_target = trajectory_right['position'][idx]
            self.qnow = q_target
            self.data.qpos[self.right_arm_joints] = self.qnow


            # 步进仿真
            mujoco.mj_step(self.model, self.data)
            
            # 渲染画面
            self.viewer.sync()
            
            # 控制仿真速度
            time.sleep(0.01)

        





if __name__ == "__main__":
    # 创建控制器实例
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")
        
        controller.run_fling_with_toppra()

        
    except Exception as e:
        print(f"初始化失败: {str(e)}")