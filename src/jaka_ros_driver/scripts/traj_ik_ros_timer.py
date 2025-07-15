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
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped


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

        rospy.init_node('k1_dual_arm_controller', anonymous=True)
        
        # 创建轨迹发布者 (左右臂各一个)
        self.right_arm_pub = rospy.Publisher('/right_arm_controller/command', 
                                           JointTrajectory, 
                                           queue_size=10)
        self.left_arm_pub = rospy.Publisher('/left_arm_controller/command', 
                                          JointTrajectory, 
                                          queue_size=10)
        self.left_end_joint_pub = rospy.Publisher('/control/left_end_joint',PoseStamped,queue_size=10)
        self.traj_index = 1 
        self.traj_num = 1
        # gripper
        self.gripper_pub = rospy.Publisher('gripper1_target_position', Int32, queue_size=10)
        self.gripper_pub2 = rospy.Publisher('gripper2_target_position', Int32, queue_size=10)
        # 预设位置序列 [全闭，半开，全开，抓取位置]
        self.position_sequence = [0, 500, 1000, 300]

        
        # 轨迹数据存储
        self.left_trajectory = None
        self.right_trajectory = None
        self.trajectory_start_time = None
        self.timer = None

        self.left_arm_finished = False
        self.right_arm_finished = False
        
        if urdf_path is None:
            # 尝试自动查找URDF文件
            urdf_path = os.path.abspath(os.getcwd()) + '/model/K1/urdf/k1_pgc_j4_limit.urdf'
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        
        # 从URDF加载机器人模型
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)


         # 关节索引定义 (根据实际URDF结构调整)
        self.right_arm_joints = [0, 1, 2, 3, 4, 5, 6]     # 右臂7个关节
        self.left_arm_joints = [9, 10, 11, 12, 13, 14, 15] # 左臂7个关节 
        self.right_gripper_joints = [7, 8]                  # 右夹爪2个关节
        self.left_gripper_joints = [16, 17]                 # 左夹爪2个关节
        
        # 末端执行器link名称
        self.right_ee_link = "right_gripper_base"
        self.left_ee_link = "lt"

        # 初始关节位置 (示例值)
        self.qnow = np.zeros(18)
        self.qnow[self.right_arm_joints] = [np.deg2rad(75.72), np.deg2rad(-48.64), np.deg2rad(-69.35), np.deg2rad(-132.21), np.deg2rad(-33.56),  np.deg2rad(47.04), np.deg2rad(-126.27)]
        self.qnow[self.left_arm_joints] = [np.deg2rad(-75.72), np.deg2rad(-48.64), np.deg2rad(69.35), np.deg2rad(-132.21), np.deg2rad(33.56),  np.deg2rad(-47.04), np.deg2rad(-126.27)]
        #self.qnow[self.right_arm_joints] = [0.84, -0.67, -0.70, -2.42, -1.10, 0.90, -1.92]
        #self.qnow[self.left_arm_joints] = [-0.94, -0.73, 0.87, -2.42, 1.06, -0.78, -1.14]
        self.qnow[self.right_gripper_joints] = [0.01875, -0.01875]
        self.qnow[self.left_gripper_joints] = [0.01875, -0.01875]



        # 关节速度限制 (rad/s)
        # self.velocity_limits = np.array([np.deg2rad(90), np.deg2rad(90), np.deg2rad(120), np.deg2rad(120), np.deg2rad(150), np.deg2rad(150), np.deg2rad(150), np.deg2rad(120), np.deg2rad(120),
        #                                  np.deg2rad(90), np.deg2rad(90), np.deg2rad(120), np.deg2rad(120), np.deg2rad(150), np.deg2rad(150), np.deg2rad(150), np.deg2rad(120), np.deg2rad(120)])
        # self.velocity_limits = np.array([1.5] * 18)
        self.velocity_limits = np.array([np.deg2rad(80)] * 18)
        # 关节加速度限制 (rad/s^2)
        self.acceleration_limits = np.array([np.deg2rad(100)] * 18)
        
    def prepare_next_trajectory(self):
        """
        准备下一条轨迹
        """
        # 更新轨迹索引
        # self.traj_index = self.traj_index + 1
        self.traj_index = 1
        if self.traj_index>self.traj_num:
            return
        if self.traj_index == 2:
            #self.gripper_pub.publish(self.position_sequence[0])
            rospy.sleep(1)
            rospy.loginfo(f"准备执行下一条轨迹，轨迹索引: {self.traj_index}")
            for _ in range(5):  # 多次发布确保接收
                self.gripper_pub.publish(0)
                self.gripper_pub2.publish(0)
                rospy.loginfo("发送夹爪初始化指令: 0 (全闭)")
                rospy.sleep(0.1)
            #self.gripper_pub.publish(0)
            rospy.sleep(1)
        
        
        # 生成并发布新轨迹
        self.run_fling_with_toppra()
        
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
            left_idx = 0
        
        if right_arm_active:
            right_idx = np.searchsorted(self.right_trajectory['time'], current_time, side='left')
            right_idx = min(right_idx, len(self.right_trajectory['time']) - 1)
            right_idx = 0
        
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
            rospy.loginfo(f"左臂轨迹进度: {current_time:.3f}/{total_duration_left:.3f}s | 点 {left_idx}/{len(self.left_trajectory['time'])}")
        if right_arm_active and right_idx % 50 == 0:
            rospy.loginfo(f"右臂轨迹进度: {current_time:.3f}/{total_duration_right:.3f}s | 点 {right_idx}/{len(self.right_trajectory['time'])}")
        

        
        
        
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


    def run_fling_with_toppra(self):
        """
        运行 Fling 动作
        """
        # 规划左臂轨迹
        self.left_arm_finished = False
        self.right_arm_finished = False

        self.left_trajectory = self.generate_fling_trajectory(arm='left')  # 直接赋值给实例变量
        if self.left_trajectory is None:
            rospy.logerr("左臂轨迹规划失败")
            return
        
        # 规划右臂轨迹
        self.right_trajectory = self.generate_fling_trajectory(arm='right')  # 直接赋值给实例变量
        if self.right_trajectory is None:
            rospy.logerr("右臂轨迹规划失败")
            return
            
        # 重置轨迹执行状态
        self.trajectory_start_time = rospy.Time.now()
        
        # 创建8ms定时器（只在轨迹生成后启动）
        self.timer = rospy.Timer(rospy.Duration(0.002), self.dual_arm_command_j_send_callback)
        # self.timer = rospy.Timer(rospy.Duration(0.002), self.dual_arm_command_j_send_callback)
        
        rospy.loginfo(f"轨迹规划完成，开始执行... 总时长: {self.left_trajectory['time'][-1]:.3f}s")



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
        
        # 计算轨迹持续时间
        duration = jnt_traj.get_duration()

        # 生成时间序列（从0开始，每8ms一个点，最后一个点不超过duration）
        ts_sample = np.arange(0, duration + 0.002, 0.002)
        ts_sample[-1] = duration  # 确保最后一个点正好是duration
        # 采样轨迹
        #ts_sample = np.linspace(0, jnt_traj.get_duration(), int(np.ceil(jnt_traj.get_duration()/0.008)))
        qs_sample = jnt_traj.eval(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)
        print(jnt_traj.get_duration())
        print(qs_sample.shape[0])
        # # 绘制轨迹图
        

        # fig, axs = plt.subplots(3, 1, figsize=(10, 8))

        # for i in range(7):
        #     axs[0].plot(ts_sample, qs_sample[:, i], label=f'Joint {i+1}')
        #     axs[1].plot(ts_sample, qds_sample[:, i], label=f'Joint {i+1}')
        #     axs[2].plot(ts_sample, qdds_sample[:, i], label=f'Joint {i+1}')

        # axs[0].set_title('Position')
        # axs[1].set_title('Velocity')
        # axs[2].set_title('Acceleration')

        # for ax in axs:
        #     ax.set_xlabel('Time [s]')
        #     ax.legend()
        #     ax.grid()

        # plt.tight_layout()
        # plt.show()

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
        jVal = np.zeros(18)
        jVal[0] = np.deg2rad(1.4)
        jVal[1] = np.deg2rad(-78)
        jVal[2] = np.deg2rad(72.5)
        jVal[3] = np.deg2rad(-82.3)
        jVal[4] = np.deg2rad(50.5)
        jVal[5] = np.deg2rad(-5.6)
        jVal[6] = np.deg2rad(-126.3)
        q_left = jVal[0:7]

        T = controller.forward_kinematics(jVal, 'left')
        print("T", T)
        print(SE3_to_end_pose(T))
        if self.traj_index == 1:
            if arm == 'left':
                motion = [

                    
                    # pose_to_se3(0.38, 0.19, -0.21, 174.3, 12.0, 92.0),
                    # pose_to_se3(0.38, 0.19, -0.2, 174.3, 12.0, 92.0)
                    #pose_to_se3(0.38, 0.19, 0.2, 174.3, 12, 88.3)
                    # pose_to_se3(0.2, 0.3, 0.2, 160, 0, 0),
                    # pose_to_se3(0.32, 0.3, 0.22, 160, 0, 0),
                    # pose_to_se3(0.52, 0.3, 0.42, 130, -20, 35),
                    # pose_to_se3(0.35, 0.3, 0.20, 160, 0, 0),
                    # pose_to_se3(0.30, 0.3, 0.17, 160, 0, 0),
                    # pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0)
                ]
                print("Left ARM")
            else:  # right arm (mirrored version)
                motion = [
                    pose_to_se3(0.38, -0.19, -0.21, 174.3, -12.0, 92.0),
                    pose_to_se3(0.38, -0.19, -0.2, 174.3, -12.0, 92.0)
                    #pose_to_se3(0.38, -0.19, 0.2, 174, -12, 92)    # Y坐标取反，roll取反
                    # pose_to_se3(0.2, -0.3, 0.2, 160, 0, 0),         # Y坐标取反，roll取反
                    # pose_to_se3(0.32, -0.3, 0.22, -160, 0, 0),       # Y坐标取反，roll取反
                    # pose_to_se3(0.52, -0.3, 0.42, -130, -20, -35),   # Y坐标取反，roll/yaw取反
                    # pose_to_se3(0.35, -0.3, 0.20, -160, 0, 0),       # Y坐标取反，roll取反
                    # pose_to_se3(0.30, -0.3, 0.17, -160, 0, 0),       # Y坐标取反，roll取反
                    # pose_to_se3(0.197, -0.225, 0.152, -160, 6, -16)        # Y坐标取反，roll取反
                ]
                print("Right ARM")
            #     motion = [
            #         pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0),
            #         pose_to_se3(0.2, 0.3, 0.2, 160, 0, 0),
            #         pose_to_se3(0.32, 0.3, 0.22, 160, 0, 0),
            #         pose_to_se3(0.52, 0.3, 0.42, 130, -20, 35),
            #         pose_to_se3(0.35, 0.3, 0.20, 160, 0, 0),
            #         pose_to_se3(0.30, 0.3, 0.17, 160, 0, 0),
            #         pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0)
            #     ]
            #     print("Left ARM")
            # else:  # right arm (mirrored version)
            #     motion = [
            #         pose_to_se3(0.2, -0.22, 0.15, 160, 0, 0),      # Y坐标取反，roll取反
            #         pose_to_se3(0.2, -0.3, 0.2, 160, 0, 0),         # Y坐标取反，roll取反
            #         pose_to_se3(0.32, -0.3, 0.22, -160, 0, 0),       # Y坐标取反，roll取反
            #         pose_to_se3(0.52, -0.3, 0.42, -130, -20, -35),   # Y坐标取反，roll/yaw取反
            #         pose_to_se3(0.35, -0.3, 0.20, -160, 0, 0),       # Y坐标取反，roll取反
            #         pose_to_se3(0.30, -0.3, 0.17, -160, 0, 0),       # Y坐标取反，roll取反
            #         pose_to_se3(0.197, -0.225, 0.152, -160, 6, -16)        # Y坐标取反，roll取反
            #     ]
            #     print("Right ARM")
        elif self.traj_index == 2:
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
                    pose_to_se3(0.197, -0.225, 0.152, -160, 6, -16),      # Y坐标取反，roll取反
                    pose_to_se3(0.2, -0.3, 0.2, -160, 0, 0),         # Y坐标取反，roll取反
                    pose_to_se3(0.32, -0.3, 0.22, -160, 0, 0),       # Y坐标取反，roll取反
                    pose_to_se3(0.52, -0.3, 0.42, -130, -20, -35),   # Y坐标取反，roll/yaw取反
                    pose_to_se3(0.35, -0.3, 0.20, -160, 0, 0),       # Y坐标取反，roll取反
                    pose_to_se3(0.30, -0.3, 0.17, -160, 0, 0),       # Y坐标取反，roll取反
                    pose_to_se3(0.197, -0.225, 0.152, -160, 6, -16)        # Y坐标取反，roll取反
                ]
                print("Right ARM")
                


        # 求解逆运动学得到关节空间路径点
        waypoints = []
        for pose in motion:
            q_sol, success = self.inverse_kinematics(pose, arm=arm, q0=self.qnow)
            if not success:
                rospy.logerr(f"逆运动学求解失败: {pose}")
                return None
            
            # 创建完整的关节角度向量
            full_q = self.qnow.copy()
            if arm == 'left':
                full_q[self.left_arm_joints] = q_sol
            else:
                full_q[self.right_arm_joints] = q_sol
                
            waypoints.append(full_q)
        
        # 使用TOPPRA规划轨迹
        try:
            trajectory = self.plan_trajectory_with_toppra(waypoints, arm=arm)
            if trajectory is None:
                rospy.logerr("TOPPRA轨迹规划失败")
                return None
            return trajectory
        except Exception as e:
            rospy.logerr(f"轨迹规划异常: {str(e)}")
            return None
        





if __name__ == "__main__":
    # 创建控制器实例
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")
        jVal = np.zeros(18)
        jVal[0] = np.deg2rad(1.4)
        jVal[1] = np.deg2rad(-78)
        jVal[2] = np.deg2rad(72.5)
        jVal[3] = np.deg2rad(-82.3)
        jVal[4] = np.deg2rad(50.5)
        jVal[5] = np.deg2rad(-5.6)
        jVal[6] = np.deg2rad(-126.3)

        T = controller.forward_kinematics(jVal, 'left')
        print("T", T)
        print(SE3_to_end_pose(T))
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
        
    except Exception as e:
        print(f"初始化失败: {str(e)}")