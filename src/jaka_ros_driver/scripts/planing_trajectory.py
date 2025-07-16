import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import q2r
import matplotlib.pyplot as plt
from math import pi
import time
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from loguru import logger
from nav_msgs.msg import Path
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import threading

try:
    PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:
    PROJECT_ROOT = os.getcwd()

def SE3_to_end_pose(T):
    """
    将SE3变换矩阵转换为机械臂末端位姿格式(x,y,z,rx,ry,rz)

    参数:
        T: SE3变换矩阵

    返回:
        str: 格式为"x, y, z, rx, ry, rz"的字符串，角度单位为度
    """
    if not isinstance(T, SE3):
        raise ValueError("输入必须是SE3对象")

    # 获取位置(x,y,z)
    pos = T.t

    # 获取欧拉角(假设是ZYX顺序，即RPY)
    eul = T.eul(unit='deg')

    # 组合位置和欧拉角
    pose_data = np.concatenate([pos, eul])

    # 格式化为字符串，保留1位小数
    pose_str = ", ".join([f"{x:.1f}" for x in pose_data])

    return pose_str


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
        if urdf_path is None:
            urdf_path = os.path.join(PROJECT_ROOT, "scripts", 'model/K1/urdf/k1_pgc_j4_limit.urdf')
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        print(f"Robot model loaded from: {urdf_path}")


        # Ros publish
        rospy.init_node('k1_dual_arm_controller', anonymous=True)
        self.right_arm_pub = rospy.Publisher('/right_arm_controller/command',
                                             JointTrajectory,
                                             queue_size=10)
        self.left_arm_pub = rospy.Publisher('/left_arm_controller/command',
                                            JointTrajectory,
                                            queue_size=10)
        self.vis_pub_left = rospy.Publisher('/planned_path_left', Path, queue_size=1, latch=True)
        self.vis_pub_right = rospy.Publisher('/planned_path_right', Path, queue_size=1, latch=True)

        # 18自由度关节定义
        self.right_arm_joints_indices = [0, 1, 2, 3, 4, 5, 6]  # 右臂7个关节
        self.left_arm_joints_indices = [9, 10, 11, 12, 13, 14, 15]  # 左臂7个关节
        self.right_gripper_joints = [7, 8]  # 右夹爪2个关节
        self.left_gripper_joints = [16, 17]  # 左夹爪2个关节
        self.right_arm_joint_names = [f'right_arm_joint{i + 1}' for i in range(7)]        # ros controller name
        self.left_arm_joint_names = [f'left_arm_joint{i + 1}' for i in range(7)]        # ros controller name


        self.traj_index = 1
        self.traj_num = 1
        # gripper
        self.gripper_pub = rospy.Publisher('gripper1_target_position', Int32, queue_size=10)
        self.gripper_pub2 = rospy.Publisher('gripper2_target_position', Int32, queue_size=10)
        # 预设位置序列 [全闭，半开，全开，抓取位置]
        self.position_sequence = [0, 500, 1000, 300]

        # self.right_ee_link = "right_gripper_base"
        self.left_ee_link = "lt"
        self.right_ee_link = "rt"

        # 轨迹数据存储
        self.qnow = np.zeros(self.robot.n)
        self.left_trajectory = None
        self.right_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_completion_event = threading.Event()
        self.timer = None

        self.left_arm_finished = False
        self.right_arm_finished = False


        # 初始关节位置 (示例值)
        self.qnow_left = np.array([-1.554, -78.013, -72.530, -82.317, -50.502, 5.610, 126.298])* np.pi/180  # 左臂关节角度
        self.qnow_right = np.array([1.707, -78.003, 72.538, -82.305, 50.506, -5.6, -126.290])* np.pi/180  # 右臂关节角度
        self.qnow[self.right_gripper_joints] = [0.01875, -0.01875]
        self.qnow[self.left_gripper_joints] = [0.01875, -0.01875]
        self.qnow[self.right_arm_joints_indices] = self.qnow_right
        self.qnow[self.left_arm_joints_indices] = self.qnow_left
        logger.info(f"Initial joint state set: {self.qnow}")

        # 关节速度限制 (rad/s)
        self.velocity_limits = np.array([np.deg2rad(80)] * 18)
        # 关节加速度限制 (rad/s^2)
        self.acceleration_limits = np.array([np.deg2rad(100)] * 18)

    # ================================================================= #
    # ================         High-Level API Functions         ==================== #
    # ================================================================= #
    def move_to_joint_target(self, q_target, velocity_scaling_factor=1.0,wait=True):
        """
        (API 1) Plans and moves to the specified target joint position.
        """
        print(f"\n[API] Moving to Joint Target (Speed: {velocity_scaling_factor * 100:.0f}%)")
        q_target = np.array(q_target)
        if q_target.size != self.robot.n:
            print(f"Error: Target joint dimension should be {self.robot.n}, but received {q_target.size}.")
            return

        # Create a path from the current point to the target point
        waypoints = [self.qnow, q_target]

        # Plan trajectory
        full_traj = self.plan_full_robot_trajectory(waypoints, velocity_scaling_factor)

        if full_traj and self.split_and_store_trajectory(full_traj):
            self.visualize_trajectory_ros(full_traj)  # <--- 在此可视化
            rospy.loginfo("路径已发布供RViz预览。将在2秒后开始运动...")
            rospy.sleep(2)
            self.start_trajectory_execution()
            if wait:
                self.wait_for_trajectory_completion()
            self.qnow = q_target  # 更新当前状态

    def move_to_cartesian_pose(self, T_desired, arm, velocity_scaling_factor=1.0,wait=True):
        """
        API 2: Plans and moves one or both arms to a specified Cartesian pose(s).

        Args:
            T_desired: The target pose.
                       - If arm is 'left' or 'right', this is a single SE3 object.
                       - If arm is 'both', this is a tuple (T_right_desired, T_left_desired).
            arm (str): Which arm to move ('left', 'right', or 'both').
            velocity_scaling_factor (float): Factor to scale motion speed.
        """
        print(f"\n[API] Moving '{arm}' arm(s) to Cartesian Pose (Speed: {velocity_scaling_factor * 100:.0f}%)")

        q_target_full = self.qnow.copy()

        if arm == 'both':
            if not isinstance(T_desired, (list, tuple)) or len(T_desired) != 2:
                print("Error: For 'both' arms, T_desired must be a tuple of (T_right, T_left).")
                return

            T_right_desired, T_left_desired = T_desired

            # Solve for both arms
            q_sol_r, success_r = self.inverse_kinematics(T_right_desired, 'right', self.qnow)
            q_sol_l, success_l = self.inverse_kinematics(T_left_desired, 'left', self.qnow)

            if not (success_r and success_l):
                print("Aborting move: IK solution failed for one or both arms.")
                return

            q_target_full[self.right_arm_joints_indices] = q_sol_r
            q_target_full[self.left_arm_joints_indices] = q_sol_l

        elif arm in ['left', 'right']:
            q_sol_arm, success = self.inverse_kinematics(T_desired, arm, self.qnow)
            if not success:
                print("Aborting move: Cannot find an IK solution.")
                return

            if arm == 'left':
                q_target_full[self.left_arm_joints_indices] = q_sol_arm
            else:
                q_target_full[self.right_arm_joints_indices] = q_sol_arm
        else:
            print(f"Error: Invalid arm specified: '{arm}'. Use 'left', 'right', or 'both'.")
            return

        # Move the whole robot to the new unified joint state
        self.move_to_joint_target(q_target_full, velocity_scaling_factor)

    # ================================================================= #
    # ================         planning and visualization        ================ #
    # ================================================================= #
    def plan_full_robot_trajectory(self, waypoints, velocity_scaling_factor=1.0):
        """plan bimanual robot trajectory with TOPPRA"""
        if len(waypoints) < 2:
            print("wrong waypoints, need at start and end point at least")
            return None

        path_scalars = np.linspace(0, 1, len(waypoints))
        path = ta.SplineInterpolator(path_scalars, np.array(waypoints))

        vlim = np.vstack([-self.velocity_limits, self.velocity_limits]).T * velocity_scaling_factor
        alim = np.vstack([-self.acceleration_limits, self.acceleration_limits]).T * velocity_scaling_factor

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
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)

        return {'time': ts_sample, 'position': qs_sample, 'velocity': qds_sample, 'acceleration': qdds_sample}

    def split_and_store_trajectory(self, unified_trajectory):
        """轨迹拆分为左右臂两个轨迹并存储"""
        if unified_trajectory is None:
            self.left_trajectory = None
            self.right_trajectory = None
            return False

        self.left_trajectory = {
            'time': unified_trajectory['time'],
            'position': unified_trajectory['position'][:, self.left_arm_joints_indices],
            'velocity': unified_trajectory['velocity'][:, self.left_arm_joints_indices],
            'acceleration': unified_trajectory['acceleration'][:, self.left_arm_joints_indices],
        }
        self.right_trajectory = {
            'time': unified_trajectory['time'],
            'position': unified_trajectory['position'][:, self.right_arm_joints_indices],
            'velocity': unified_trajectory['velocity'][:, self.right_arm_joints_indices],
            'acceleration': unified_trajectory['acceleration'][:, self.right_arm_joints_indices],
        }
        return True

    def visualize_trajectory_ros(self, trajectory):
        """将存储的轨迹发布为Path消息，供RViz可视化"""
        if self.left_trajectory is None or self.right_trajectory is None: return

        rospy.loginfo("发布可视化路径到 /planned_path_left 和 /planned_path_right ...")

        # 为左右臂创建Path消息
        header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        left_path_msg = Path(header=header)
        right_path_msg = Path(header=header)

        # 计算左臂的笛卡尔路径点
        for q_18dof in trajectory['position']:
            T_left = self.forward_kinematics(q_18dof, 'left')
            pose_l = PoseStamped(header=left_path_msg.header)
            pose_l.pose.position.x, pose_l.pose.position.y, pose_l.pose.position.z = T_left.t
            pose_l.pose.orientation.w = 1.0 # TODO： simple orientation
            left_path_msg.poses.append(pose_l)

            T_right = self.forward_kinematics(q_18dof, 'right')
            pose_r = PoseStamped(header=right_path_msg.header)
            pose_r.pose.position.x, pose_r.pose.position.y, pose_r.pose.position.z = T_right.t
            pose_r.pose.orientation.w = 1.0
            right_path_msg.poses.append(pose_r)

        self.vis_pub_left.publish(left_path_msg)
        self.vis_pub_right.publish(right_path_msg)
        rospy.loginfo("可视化路径已发布。请在RViz中添加Path显示并订阅该话题。")

    # ================================================================= #
    # ================         ROS执行回调        ================ #
    # ================================================================= #
    def start_trajectory_execution(self):
        """启动一个rospy.Timer来发送轨迹点"""
        if self.timer:
            self.timer.shutdown()

        self.trajectory_completion_event.clear()
        self.trajectory_start_time = rospy.Time.now()

        # 启动高频发送定时器
        self.timer = rospy.Timer(rospy.Duration(0.008), self._execution_callback)
        rospy.loginfo(f"开始执行轨迹，时长: {self.left_trajectory['time'][-1]:.2f}s")

    def _execution_callback(self, event):
        """高频定时器回调，发送轨迹点"""
        if self.left_trajectory is None or self.right_trajectory is None:
            self.timer.shutdown()
            return

        elapsed_time = (rospy.Time.now() - self.trajectory_start_time).to_sec()
        total_duration = self.left_trajectory['time'][-1]

        if elapsed_time >= total_duration:
            rospy.loginfo("轨迹执行完毕。")
            self.timer.shutdown()
            self.trajectory_completion_event.set()  # 发送完成信号
            # 确保发送最后一个点
            self._send_trajectory_point(-1)  # -1代表最后一个点
            return

        self._send_trajectory_point(elapsed_time)

    def _send_trajectory_point(self, elapsed_time):
        """根据给定时间查找并发送左右臂的轨迹点"""
        # 找到对应时间的索引
        if elapsed_time == -1:  # 特殊标志，发送最后一个点
            left_idx = len(self.left_trajectory['time']) - 1
            right_idx = len(self.right_trajectory['time']) - 1
            time_from_start_l = rospy.Duration(self.left_trajectory['time'][left_idx])
            time_from_start_r = rospy.Duration(self.right_trajectory['time'][right_idx])
        else:
            left_idx = np.searchsorted(self.left_trajectory['time'], elapsed_time, side='left')
            right_idx = left_idx  # 因为时间轴是统一的
            time_from_start_l = rospy.Duration(elapsed_time)
            time_from_start_r = rospy.Duration(elapsed_time)

        # 创建并发布左臂消息
        header = Header(stamp=rospy.Time.now())
        left_msg = JointTrajectory(joint_names=self.left_arm_joint_names, header=header)
        point_l = JointTrajectoryPoint()
        point_l.positions = self.left_trajectory['position'][left_idx]
        point_l.velocities = self.left_trajectory['velocity'][left_idx]
        point_l.accelerations = self.left_trajectory['acceleration'][left_idx]
        point_l.time_from_start = time_from_start_l
        left_msg.points.append(point_l)
        self.left_arm_pub.publish(left_msg)

        # 创建并发布右臂消息
        right_msg = JointTrajectory(joint_names=self.right_arm_joint_names, header=header)
        point_r = JointTrajectoryPoint()
        point_r.positions = self.right_trajectory['position'][right_idx]
        point_r.velocities = self.right_trajectory['velocity'][right_idx]
        point_r.accelerations = self.right_trajectory['acceleration'][right_idx]
        point_r.time_from_start = time_from_start_r
        right_msg.points.append(point_r)
        self.right_arm_pub.publish(right_msg)

    def wait_for_trajectory_completion(self):
        """等待当前轨迹执行完成"""
        rospy.loginfo("等待轨迹执行完成...")
        self.trajectory_completion_event.wait()
        rospy.loginfo("轨迹完成，继续。")


    def dual_arm_command_j_send_callback(self, event):
        """
        8ms定时器回调函数，根据实际经过时间发布对应轨迹点
        """
        if self.left_trajectory is None or self.right_trajectory is None:
            return

        if self.trajectory_start_time is None:
            return

        # 计算当前时间点（从轨迹开始算起）
        current_time = (rospy.Time.now() - self.trajectory_start_time).to_sec()
        total_duration_left = self.left_trajectory['time'][-1]
        total_duration_right = self.right_trajectory['time'][-1]

        # 初始化标志位
        left_arm_active = True
        right_arm_active = True

        # 检查左臂是否已完成轨迹
        if current_time >= total_duration_left:
            if not self.left_arm_finished:
                rospy.loginfo("LEFT 轨迹执行完成")
                self.left_arm_finished = True
            left_arm_active = False

        # 检查右臂是否已完成轨迹
        if current_time >= total_duration_right:
            if not self.right_arm_finished:
                rospy.loginfo("RIGHT 轨迹执行完成")
                self.right_arm_finished = True
            right_arm_active = False

        # 如果两条臂都完成，则停止定时器
        if not left_arm_active and not right_arm_active:
            self.timer.shutdown()
            self.prepare_next_trajectory()
            return

        # 分别找到左右臂当前时间对应的轨迹点索引
        if left_arm_active:
            left_idx = np.searchsorted(self.left_trajectory['time'], current_time, side='left')
            left_idx = min(left_idx, len(self.left_trajectory['time']) - 1)

        if right_arm_active:
            right_idx = np.searchsorted(self.right_trajectory['time'], current_time, side='left')
            right_idx = min(right_idx, len(self.right_trajectory['time']) - 1)

        # 发布左臂指令（如果活跃）
        if left_arm_active:
            left_msg = JointTrajectory()
            left_msg.joint_names = [
                'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
                'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6',
                'left_arm_joint7'
            ]
            left_msg.header.stamp = rospy.Time.now()

            left_point = JointTrajectoryPoint()
            left_point.positions = self.left_trajectory['position'][left_idx].tolist()
            left_point.velocities = self.left_trajectory['velocity'][left_idx].tolist()
            left_point.accelerations = self.left_trajectory['acceleration'][left_idx].tolist()
            left_point.time_from_start = rospy.Duration(self.left_trajectory['time'][left_idx])
            left_msg.points = [left_point]
            self.left_arm_pub.publish(left_msg)

            left_end_control = PoseStamped()
            left_end_control.pose.position.x = left_point.positions[6]
            self.left_end_joint_pub.publish(left_end_control)

        # 发布右臂指令（如果活跃）
        if right_arm_active:
            right_msg = JointTrajectory()
            right_msg.joint_names = [
                'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3',
                'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6',
                'right_arm_joint7'
            ]
            right_msg.header.stamp = rospy.Time.now()

            right_point = JointTrajectoryPoint()
            right_point.positions = self.right_trajectory['position'][right_idx].tolist()
            right_point.velocities = self.right_trajectory['velocity'][right_idx].tolist()
            right_point.accelerations = self.right_trajectory['acceleration'][right_idx].tolist()
            right_point.time_from_start = rospy.Duration(self.right_trajectory['time'][right_idx])
            right_msg.points = [right_point]
            self.right_arm_pub.publish(right_msg)

        # 打印进度（每50个点打印一次）
        if left_arm_active and left_idx % 50 == 0:
            rospy.loginfo(
                f"左臂轨迹进度: {current_time:.3f}/{total_duration_left:.3f}s | 点 {left_idx}/{len(self.left_trajectory['time'])}")
        if right_arm_active and right_idx % 50 == 0:
            rospy.loginfo(
                f"右臂轨迹进度: {current_time:.3f}/{total_duration_right:.3f}s | 点 {right_idx}/{len(self.right_trajectory['time'])}")

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
            q_arm = q[self.left_arm_joints_indices]
        else:
            ee_link = self.right_ee_link
            q_arm = q[self.right_arm_joints_indices]

        # 计算正运动学
        T = self.robot.fkine(q, end=ee_link)
        return T

    def inverse_kinematics(self, T_desired, arm='left', q0=None, tol=1e-5, max_iter=100000):
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
            # print("Left ARM")
            joint_indices = self.left_arm_joints_indices
            ee_link = self.left_ee_link

        else:
            # print("Right ARM")
            joint_indices = self.right_arm_joints_indices
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
            kq=1.0,  # 关节限制避免增益
            km=0.0,  # 可操作性最大化增益 (0表示禁用)
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
        # print("solution:",sol)
        if sol.success:
            # 只返回对应臂的关节角度
            q_sol = sol.q[:7]
            self.print_ik_result(q_sol)
            # print("IK求解成功！各个关节的度数:",np.rad2deg(q_sol))

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


if __name__ == "__main__":
    # 创建控制器实例
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")

        q_home  = np.deg2rad(np.array([
            -1.554, -78.013, -72.530, -82.317, -50.502, 5.610, 126.298, # 右
            0, 0,
            1.707, -78.003, 72.538, -82.305, 50.506, -5.6, -126.290,# 左
            0, 0
        ]))
        rospy.loginfo("======== Demo 开始: 移动到HOME位置 ========")

        controller.move_to_joint_target(q_home, velocity_scaling_factor=0.6, wait=True)

        # ---------- Demo
        if not rospy.is_shutdown():
            rospy.loginfo("\n======== Demo: 双臂移动  ========")

            # 获取当前位姿
            current_pose_left = controller.forward_kinematics(controller.qnow, arm='left')
            current_pose_right = controller.forward_kinematics(controller.qnow, arm='right')

            # 定义一个共同的目标偏移量
            target_offset = SE3.Tx(0.15) * SE3.Tz(0.20)
            target_pose_left = target_offset * current_pose_left
            target_pose_right = target_offset * current_pose_right

            # 使用 'both' 模式，统一规划和执行
            controller.move_to_cartesian_pose(
                (target_pose_right, target_pose_left),
                arm='both',
                velocity_scaling_factor=0.5,
                wait=True
            )

        rospy.loginfo("\n演示全部完成。")

        # T = controller.forward_kinematics(jVal, 'left')
        # print("T", T)
        # print(SE3_to_end_pose(T))
        # init open gripper
        # # 可靠初始化夹爪
        # rospy.sleep(1)  # 等待Publisher建立连接
        # for _ in range(5):  # 多次发布确保接收
        #     controller.gripper_pub.publish(1000)
        #     controller.gripper_pub2.publish(1000)
        #     rospy.loginfo("发送夹爪初始化指令: 1000 (全开)")
        #     rospy.sleep(0.1)
        # # control loop
        # controller.run_fling_with_toppra()

        # 添加rospy.spin()保持节点运行
        rospy.spin()


    except rospy.ROSInterruptException:
        print("程序被中断。")

    except Exception as e:
        print(f"发生未知错误: {e}")
        import traceback
        traceback.print_exc()