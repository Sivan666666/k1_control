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
from loguru import logger
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

def get_pose_components(T):
    """
    从一个SE3对象中分解出位置(x, y, z)和RPY姿态(roll, pitch, yaw, 单位:度)。

    :param T: spatialmath.SE3 对象
    :return: 包含6个键值对的字典
    """
    if not isinstance(T, SE3):
        raise TypeError("输入必须是SE3对象")

    position = T.t
    rpy_angles = T.eul(unit='deg')  # 获取Roll, Pitch, Yaw角度
    return position, rpy_angles



class K1DualArmController:
    def __init__(self, urdf_path=None):
        """ros
        初始化K1双臂机器人控制器
        
        参数:
            urdf_path: URDF文件路径，如果为None则尝试默认路径
        """
        try:
            PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
        except NameError:
            PROJECT_ROOT = os.getcwd()
        
        urdf_path = os.path.join(PROJECT_ROOT,"model","K1","K1","urdf","k1.urdf")
        xml_path = os.path.join(PROJECT_ROOT,"model","K1","k1_new.xml")


        # 初始状态
        self.qnow = np.zeros(14) 
        self.qnow = np.array([0.83988522,-0.66850441,-0.69920311,-2.42284396,-1.10251352,0.89649283,-1.9211578,-0.94049207,-0.73311629,0.86677897,-2.42284663,1.05591172,-0.78310933,-1.13897499])
        self.q_init = self.qnow.copy()  # 保存初始关节角度
        

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"XML文件未找到: {xml_path}")

        
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 启动viewer
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # 定义左右臂的关节索引（根据URDF实际结构调整）
        self.right_arm_joints = [0, 1, 2, 3, 4, 5, 6]    # 左臂关节索引
        self.left_arm_joints = [7, 8, 9, 10, 11, 12, 13]  # 右臂关节索引
        
        # 末端执行器名称（根据URDF中的link名称）
        self.left_ee_link = "lt"  # 根据实际URDF调整
        self.right_ee_link = "rt"  # 根据实际URDF调整
      
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

    # ================================================================= #
    # ================         High-Level API Functions         ==================== #
    # ================================================================= #

    def move_to_joint_target(self, q_target, velocity_scaling_factor=1.0):
        """
        (API 1) Plans and moves to the specified target joint position.
        """
        print(f"\n[API] Moving to Joint Target (Speed: {velocity_scaling_factor*100:.0f}%)")
        q_target = np.array(q_target)
        if q_target.size != self.robot.n:
            print(f"Error: Target joint dimension should be {self.robot.n}, but received {q_target.size}.")
            return

        # Create a path from the current point to the target point
        waypoints = [self.qnow, q_target]

        # Plan trajectory
        full_traj = self.plan_full_robot_trajectory(waypoints, velocity_scaling_factor)

        if full_traj:
            self.visualize_unified_trajectory(full_traj)
            # Execute trajectory
            self.execute_full_robot_trajectory_in_mujoco(full_traj)


    def move_to_cartesian_pose(self, T_desired, arm, velocity_scaling_factor=1.0,control_orientation=True):
        """
        API 2: Plans and moves one or both arms to a specified Cartesian pose(s).

        Args:
            T_desired: The target pose.
                       - If arm is 'left' or 'right', this is a single SE3 object.
                       - If arm is 'both', this is a tuple (T_right_desired, T_left_desired).
            arm (str): Which arm to move ('left', 'right', or 'both').
            velocity_scaling_factor (float): Factor to scale motion speed.
            control_orientation: If True, controls orientation; if False, only position.
        """
        print(f"\n[API] Moving '{arm}' arm(s) to Cartesian Pose (Speed: {velocity_scaling_factor * 100:.0f}%)")

        q_target_full = self.qnow.copy()
        if control_orientation:
            mask = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        else:
            mask = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]

        if arm == 'both':
            if not isinstance(T_desired, (list, tuple)) or len(T_desired) != 2:
                print("Error: For 'both' arms, T_desired must be a tuple of (T_right, T_left).")
                return

            T_right_desired, T_left_desired = T_desired

            # Solve for both arms
            q_sol_r, success_r = self.inverse_kinematics(T_right_desired, 'right', self.qnow, mask=mask)
            q_sol_l, success_l = self.inverse_kinematics(T_left_desired, 'left', self.qnow, mask=mask)

            if not (success_r and success_l):
                print("Aborting move: IK solution failed for one or both arms.")
                return

            q_target_full[self.right_arm_joints] = q_sol_r
            q_target_full[self.left_arm_joints] = q_sol_l

        elif arm in ['left', 'right']:
            q_sol_arm, success = self.inverse_kinematics(T_desired, arm, self.qnow, mask=mask)
            if not success:
                print("Aborting move: Cannot find an IK solution.")
                return

            if arm == 'left':
                q_target_full[self.left_arm_joints] = q_sol_arm
            else:
                q_target_full[self.right_arm_joints] = q_sol_arm
        else:
            print(f"Error: Invalid arm specified: '{arm}'. Use 'left', 'right', or 'both'.")
            return

        # Move the whole robot to the new unified joint state
        self.move_to_joint_target(q_target_full,velocity_scaling_factor)

    def move_to_cartesian_pose_robust(self, T_desired, arm, velocity_scaling_factor=1.0, wait=True):
        """
        [更智能的API] 移动到笛卡尔位姿。如果完整位姿不可达，则自动尝试只控制位置。
        """
        logger.info(f"API: 尝试移动 '{arm}' 臂到目标位姿 (智能模式)")
        
        # 步骤1: 尝试求解完整位姿 (位置+姿态)
        is_reachable = self.is_pose_reachable(T_desired, arm, control_orientation=True)
        
        if is_reachable:
            logger.info("完整位姿可达，将按指定位姿移动。")
            self.move_to_cartesian_pose(T_desired, arm, velocity_scaling_factor, wait, control_orientation=True)
            return

        # 步骤2: 如果上一步失败，尝试只求解位置
        logger.warning("完整位姿不可达，正在尝试仅控制位置...")
        is_reachable_pos_only = self.is_pose_reachable(T_desired, arm, control_orientation=False)

        if is_reachable_pos_only:
            logger.info("仅位置可达，将移动到该位置（姿态自适应）。")
            self.move_to_cartesian_pose(T_desired, arm, velocity_scaling_factor, wait, control_orientation=False)
        else:
            logger.error("任务中止: 即便只控制位置，目标点依然不可达。")

    def run_waypoint_sequence(self, q_start, left_motion, right_motion):
        """
        (API 2) First moves to a starting point, then executes the specified Cartesian waypoint sequence.
        """
        print("\n(API 2) Request: Execute waypoint sequence task.")

        # --- Stage 1: Move to the starting position of the sequence ---
        print("\n--- Stage 1: Moving to the sequence start position ---")
        self.move_to_joint_target(q_start)

        if not self.viewer.is_running(): return
        print("\n--- Stage 2: Executing dual-arm waypoint trajectory from the start position ---")

        # --- Stage 2: Plan and execute dual-arm trajectory from the starting point ---
        # Now self.qnow should be very close to q_start
        # Plan trajectories for both arms from the current position (which is q_start)
        left_traj = self.plan_single_arm_trajectory_from_waypoints(
            self.qnow, left_motion, arm='left'
        )
        right_traj = self.plan_single_arm_trajectory_from_waypoints(
            self.qnow, right_motion, arm='right'
        )

        if left_traj and right_traj:
            self.execute_dual_arm_trajectory(left_traj, right_traj)
        else:
            print("Failed to execute dual-arm motion due to trajectory planning failure on one or both arms.")

    # =================================================================== #
    # ===========  Execute Function  in sim (mujoco)  ==================================== #
    # =================================================================== # 
    def execute_full_robot_trajectory_in_mujoco(self, trajectory):
        """plan in mujoco"""
        if trajectory is None:
            print("trajectory is empty")
            return
        
        print(f"execute trajectory with {len(trajectory['time'])} points, duartion: {trajectory['duration']:.2f} s")
        start_time = time.time()
        
        while self.viewer.is_running() and (time.time() - start_time) < trajectory['duration']:
            current_time = time.time() - start_time
            idx = np.searchsorted(trajectory['time'], current_time, side='left')
            if idx >= len(trajectory['time']):
                idx = len(trajectory['time']) - 1

            q_target = trajectory['position'][idx]
            self.data.qpos[:14] = q_target
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(0.001) 

        # ensure the final state is set
        self.qnow = trajectory['position'][-1]
        self.data.qpos[:14] = self.qnow
        mujoco.mj_forward(self.model, self.data)
        print("Trajectory execution finished.")

    def execute_dual_arm_trajectory(self, left_traj, right_traj):
        """Synchronously executes dual-arm trajectories."""
        if left_traj is None or right_traj is None:
            print("At least one arm trajectory is empty, cannot execute.")
            return

        duration = max(left_traj['duration'], right_traj['duration'])
        print(f"Executing dual-arm synchronized trajectory, total duration: {duration:.2f} seconds...")
        start_time = time.time()

        while self.viewer.is_running() and (time.time() - start_time) < duration:
            current_time = time.time() - start_time

            # Update left arm
            if current_time < left_traj['duration']:
                idx_l = np.searchsorted(left_traj['time'], current_time, side='left')
                q_target_l = left_traj['position'][idx_l]
                self.data.qpos['7':'14'] = q_target_l

            # Update right arm
            if current_time < right_traj['duration']:
                idx_r = np.searchsorted(right_traj['time'], current_time, side='left')
                q_target_r = right_traj['position'][idx_r]
                self.data.qpos['0':'7'] = q_target_r

            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(0.001)

        # Update final state after motion
        final_q = self.qnow.copy()
        final_q['7':'14'] = left_traj['position'][-1]
        final_q['0':'7'] = right_traj['position'][-1]
        self.qnow = final_q
        self.data.qpos[:14] = self.qnow
        mujoco.mj_forward(self.model, self.data)
        print("Dual-arm trajectory execution completed.")

    # =================================================================== #
    # ===========  Plan Function   ==================================== #
    # =================================================================== # 
    def plan_full_robot_trajectory(self, waypoints,velocity_scaling_factor=1.0):
        """plan bimanual robot trajectory with TOPPRA"""
        if len(waypoints) < 2:
            print("wrong waypoints, need at start and end point at least")
            return None
        
        path_scalars = np.linspace(0, 1, len(waypoints))
        path = ta.SplineInterpolator(path_scalars, np.array(waypoints))

        vlim = np.vstack([-self.velocity_limits, self.velocity_limits]).T * velocity_scaling_factor
        alim = np.vstack([-self.acceleration_limits, self.acceleration_limits]).T* velocity_scaling_factor
        
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)
        
        instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, solver_wrapper="seidel")
        jnt_traj = instance.compute_trajectory(0, 0)
        
        if jnt_traj is None:
            print("TOPPRA failed")
            return None

        duration = jnt_traj.get_duration()
        num_samples = max(2, int(duration * 100))
        ts_sample = np.linspace(0, duration, num_samples)
        qs_sample = jnt_traj.eval(ts_sample)

        return {'time': ts_sample, 'position': qs_sample, 'duration': duration}

    def plan_single_arm_trajectory_from_waypoints(self, q_start, motion_list, arm):
        """从一个起始关节位置开始，为单个臂规划笛卡尔路径点轨迹"""
        print(f"正在为 {arm} 臂规划路径...")
        waypoints_arm = []
        q_previous_full = q_start.copy()

        for i, pose in enumerate(motion_list):
            q_sol, success = self.inverse_kinematics(pose, arm=arm, q0=q_previous_full)
            if not success:
                print(f"逆解失败于路径点 {i}，轨迹规划中止。")
                return None

            q_current_full = q_previous_full.copy()
            if arm == 'left':
                q_current_full[self.left_arm_joints] = q_sol
            else:
                q_current_full[self.right_arm_joints] = q_sol

            waypoints_arm.append(q_current_full)
            q_previous_full = q_current_full

        # 将起始点加入路径点列表的开头
        waypoints_arm.insert(0, q_start)

        # 提取对应臂的轨迹
        if arm == 'left':
            joint_indices = self.left_arm_joints
        else:
            joint_indices = self.right_arm_joints

        waypoints_arm_only = [wp[joint_indices] for wp in waypoints_arm]

        # 使用TOPPRA规划
        path_scalars = np.linspace(0, 1, len(waypoints_arm_only))
        path = ta.SplineInterpolator(path_scalars, waypoints_arm_only)

        vlim = np.vstack([-self.velocity_limits[joint_indices], self.velocity_limits[joint_indices]]).T
        alim = np.vstack([-self.acceleration_limits[joint_indices], self.acceleration_limits[joint_indices]]).T
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)
        instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, solver_wrapper="seidel")
        jnt_traj = instance.compute_trajectory(0, 0)

        if jnt_traj is None: return None
        duration = jnt_traj.get_duration()
        ts_sample = np.linspace(0, duration, int(duration * 100))
        qs_sample = jnt_traj.eval(ts_sample)

        return {'time': ts_sample, 'position': qs_sample, 'duration': duration}
    

    # =================================================================== #
    # ===========  Basic Function   ==================================== #
    # =================================================================== #
    def is_pose_reachable(self, T_desired, arm, control_orientation=True):
        """
        辅助函数：检查一个位姿是否可达，但不实际移动机器人。
        返回 True 如果可达, False 如果不可达。
        """
        print(f"正在检查可达性: Arm '{arm}', 姿态控制: {control_orientation}")
        mask = [1.0]*6 if control_orientation else [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
        _ , success = self.inverse_kinematics(T_desired, arm, self.qnow, mask)
        return success
        
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
    
    def inverse_kinematics(self, T_desired, arm='left', q0=None, mask=None, tol=1e-6, max_iter=10000):
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
        
        if mask is None:
            mask = [1, 1, 1, 1, 1, 1]
        
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
        
    def visualize_unified_trajectory(self, trajectory):
        """
        [NEW] Visualizes a unified 14-DOF trajectory in the MuJoCo viewer.
        """
        if trajectory is None or len(trajectory['position']) == 0:
            print("无法可视化空轨迹 (Cannot visualize an empty trajectory).")
            return

        print("正在可视化轨迹路径... (Visualizing trajectory path...)")
        left_path, right_path = [], []

        # Calculate the Cartesian path for both end-effectors
        for q_state in trajectory['position']:
            left_path.append(self.forward_kinematics(q_state, 'left').t)
            right_path.append(self.forward_kinematics(q_state, 'right').t)

        # Use MuJoCo's user geoms to draw spheres
        self.viewer.user_scn.ngeom = 0
        left_color = [1, 0, 0, 0.5]  # Transparent Red
        right_color = [0, 1, 0, 0.5]  # Transparent Green
        sphere_size = [0.005, 0, 0]

        max_geoms = len(self.viewer.user_scn.geoms)
        num_points = 0
        for i, pos in enumerate(left_path):
            if num_points >= max_geoms: break
            mujoco.mjv_initGeom(self.viewer.user_scn.geoms[num_points], type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                size=sphere_size, pos=pos, mat=np.eye(3).flatten(), rgba=left_color)
            num_points += 1
        for i, pos in enumerate(right_path):
            if num_points >= max_geoms: break
            mujoco.mjv_initGeom(self.viewer.user_scn.geoms[num_points], type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                size=sphere_size, pos=pos, mat=np.eye(3).flatten(), rgba=right_color)
            num_points += 1

        self.viewer.user_scn.ngeom = num_points
        print(f"路径已显示, 共 {num_points} 个点 (Path displayed with {num_points} points).")


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
            q_sol, success = self.inverse_kinematics(pose, arm=arm, q0=self.qnow,tol=1e-3,max_iter=300)
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

    # =================================================================== #
    # ==============     Initial      ============== #
    # =================================================================== #
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")
    except Exception as e:
        print(f"初始化失败: {str(e)}")
        
    q_left_arm_deg = np.array([1.707, -78.003, 72.538, -82.305, 50.506, -5.6, -126.290])
    q_right_arm_deg = np.array([-1.554, -78.013, -72.530, -82.317, -50.502, 5.610, 126.298])

    q_left_arm_rad = q_left_arm_deg * np.pi / 180
    q_right_arm_rad = q_right_arm_deg * np.pi / 180

    # Concatenate into the full 14 DOF robot joint target
    q_target_initial = np.concatenate((q_right_arm_rad, q_left_arm_rad))


    # =================================================================== #
    # ==============     Move to Initial points      ============== #
    # =================================================================== #

    # Debug Test
    controller.move_to_joint_target(q_target_initial)
    print("\n已到达指定的初始位置。暂停3秒...")
    time.sleep(3)

    
    # =================================================================== #
    # ==============      Test moving to the specified initial point  ============== #
    # =================================================================== #

    print("\n--- DEBUG: Moving to a point slightly above the initial target ---")

    # Define a target pose by moving 15cm forward (X) and 10cm up (Z)
    current_pose_left = controller.forward_kinematics(controller.qnow, arm='left')
    current_pose_right = controller.forward_kinematics(controller.qnow, arm='right')
    target_offset = SE3.Tx(0.15) * SE3.Tz(0.40)
    target_pose_left = target_offset * current_pose_left
    target_pose_right = target_offset * current_pose_right
    # Pack the two target poses into a tuple: (Right_Target, Left_Target)
    # and call the function with arm='both'
    controller.move_to_cartesian_pose((target_pose_right, target_pose_left), arm='both', velocity_scaling_factor=0.5)
    print("--- DEBUG: Reached the debug target. Pausing for 2 seconds... ---")
    time.sleep(2)

    
    # --- Now move to the actual initial target ---
    # print("\n--- Moving to the actual specified initial position ---")
    # controller.move_to_joint_target(q_target_initial)
    #
    # print("\nReached the specified initial position. Pausing for 3 seconds...")
    # time.sleep(3)


    # =================================================================== #
    # ==============     Run Waypoint Sequence      ============== #    
    # =================================================================== #
    if controller.viewer.is_running():
        print("\n" + "="*60)
        print("Preparing to test (API 2): Executing Fling motion from the current position")
        print("="*60)

        # Define Cartesian waypoints for the left and right arm Fling motion
        left_fling_motion = [
            pose_to_se3(0.2, 0.3, 0.2, np.rad2deg(160), 0, 0),
            pose_to_se3(0.32, 0.3, 0.22, np.rad2deg(160), 0, 0),
            pose_to_se3(0.52, 0.3, 0.42, np.rad2deg(130), np.rad2deg(-20), np.rad2deg(35)),
            pose_to_se3(0.35, 0.3, 0.20, np.rad2deg(160), 0, 0),
            pose_to_se3(0.30, 0.3, 0.17, np.rad2deg(160), 0, 0),
            pose_to_se3(0.20, 0.22, 0.15, np.rad2deg(160), 0, 0)
        ]

        right_fling_motion = [
            pose_to_se3(0.2, -0.3, 0.2, np.rad2deg(-160), 0, 0),
            pose_to_se3(0.32, -0.3, 0.22, np.rad2deg(-160), 0, 0),
            pose_to_se3(0.52, -0.3, 0.42, np.rad2deg(-130), np.rad2deg(-20), np.rad2deg(-35)),
            pose_to_se3(0.35, -0.3, 0.20, np.rad2deg(-160), 0, 0),
            pose_to_se3(0.30, -0.3, 0.17, np.rad2deg(-160), 0, 0),
            pose_to_se3(0.20, -0.22, 0.15, np.rad2deg(-160), 0, 0)
        ]

        # Execute the task: q_start is the target initial position reached in the previous step
        controller.run_waypoint_sequence(q_target_initial, left_fling_motion, right_fling_motion)

    print("\nAll tasks demonstrated.")


