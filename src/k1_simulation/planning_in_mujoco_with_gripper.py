import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import time
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R

try:
    PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
except NameError:
    PROJECT_ROOT = os.getcwd()

# --- 辅助函数 ---
def pose_to_se3(x, y, z, rx, ry, rz):
    rx_rad, ry_rad, rz_rad = np.deg2rad([rx, ry, rz])
    return SE3.Tz(z) * SE3.Ty(y) * SE3.Tx(x) * SE3.Rz(rz_rad) * SE3.Ry(ry_rad) * SE3.Rx(rx_rad)

def SE3_to_end_pose(T, order='XYZ'):
    pos = T.t
    rotation_matrix = T.R
    r = R.from_matrix(rotation_matrix)
    eul = r.as_euler(order.lower(), degrees=True)
    return pos, eul

# --- MuJoCo控制器 ---
class K1DualArmMujocoController:
    def __init__(self, urdf_path=None, xml_path=None):
        if urdf_path is None:
            urdf_path = 'model/K1/urdf/k1_pgc_j4_limit.urdf'
        if xml_path is None:
            xml_path = 'model/K1/k1_new.xml' # 假设你的XML文件在这里

        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        print(f"Robotics Toolbox模型加载成功, 共 {self.robot.n} 个自由度。")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # --- 关节定义 (18-DOF) ---
        self.right_arm_indices = list(range(7))
        self.right_gripper_indices = [7, 8]
        self.left_arm_indices = list(range(9, 16))
        self.left_gripper_indices = [16, 17]
        self.right_ee_link = "rt"
        self.left_ee_link = "lt"
        
        # --- 状态与运动学限制 ---
        self.qnow = np.zeros(self.robot.n)
        self.velocity_limits = np.array([np.deg2rad(80)] * self.robot.n)
        self.acceleration_limits = np.array([np.deg2rad(50)] * self.robot.n)
        print("MuJoCo控制器初始化成功。")

    def forward_kinematics(self, q_18dof, arm):
        ee_link = self.left_ee_link if arm == 'left' else self.right_ee_link
        return self.robot.fkine(q_18dof, end=ee_link)

    def inverse_kinematics(self, T_desired, arm, q0_18dof, mask=None):
        if mask is None: mask = [1.0] * 6
        sol = self.robot.ikine_LM(
            T_desired, end=(self.left_ee_link if arm == 'left' else self.right_ee_link),
            q0=q0_18dof, mask=mask, joint_limits=True, tol=1e-4, ilimit=100
        )
        if sol.success:
            indices = self.left_arm_indices if arm == 'left' else self.right_arm_indices
            return sol.q[indices], True
        return None, False

    def plan_full_robot_trajectory(self, waypoints, velocity_scaling_factor=1.0):
        if len(waypoints) < 2: return None
        vel_limits = self.velocity_limits * velocity_scaling_factor
        accel_limits = self.acceleration_limits * velocity_scaling_factor
        path = ta.SplineInterpolator(np.linspace(0, 1, len(waypoints)), np.array(waypoints))
        pc_vel = constraint.JointVelocityConstraint(np.vstack([-vel_limits, vel_limits]).T)
        pc_acc = constraint.JointAccelerationConstraint(np.vstack([-accel_limits, accel_limits]).T)
        instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, solver_wrapper="seidel")
        jnt_traj = instance.compute_trajectory(0, 0)
        if jnt_traj is None: return None
        duration = jnt_traj.get_duration()
        ts_sample = np.linspace(0, duration, max(2, int(duration * 100)))
        qs = jnt_traj.eval(ts_sample)
        return {'time': ts_sample, 'position': qs, 'duration': duration}

    def execute_trajectory_mujoco(self, trajectory):
        """在MuJoCo中执行轨迹动画"""
        if trajectory is None: return
        print(f"正在MuJoCo中执行轨迹, 时长: {trajectory['duration']:.2f}s...")
        start_time = time.time()
        while self.viewer.is_running():
            elapsed_time = time.time() - start_time
            if elapsed_time > trajectory['duration']:
                break
            idx = np.searchsorted(trajectory['time'], elapsed_time, side='left')
            idx = min(idx, len(trajectory['position']) - 1)
            self.data.qpos[:self.robot.n] = trajectory['position'][idx]
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
        self.qnow = trajectory['position'][-1]
        self.data.qpos[:self.robot.n] = self.qnow
        mujoco.mj_forward(self.model, self.data)
        print("轨迹执行完毕。")

    # ================================================================= #
    # ================         高层运动API (与ROS版一致)        ==================== #
    # ================================================================= #
    
    def move_to_joint_target(self, q_target, velocity_scaling_factor=1.0):
        print(f"\n[API] 移动到关节目标 (速度: {velocity_scaling_factor*100:.0f}%)")
        q_target = np.array(q_target)
        if q_target.size != self.robot.n:
            print(f"错误: 目标关节维度应为 {self.robot.n}, 但接收到 {q_target.size}.")
            return
        
        traj = self.plan_full_robot_trajectory([self.qnow, q_target], velocity_scaling_factor)
        if traj:
            input("规划完成，按回车在MuJoCo中预览...")
            self.execute_trajectory_mujoco(traj)

    def move_to_cartesian_pose(self, T_desired, arm, velocity_scaling_factor=1.0, control_orientation=True):
        print(f"\n[API] 移动 '{arm}' 臂到笛卡尔位姿 (姿态控制: {control_orientation})")
        q_target_18dof = self.qnow.copy()
        ik_success = False
        mask = [1.0]*6 if control_orientation else [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
        
        if arm == 'both':
            T_right, T_left = T_desired
            q_sol_r, success_r = self.inverse_kinematics(T_right, 'right', self.qnow, mask)
            q_sol_l, success_l = self.inverse_kinematics(T_left, 'left', self.qnow, mask)
            if success_r and success_l:
                q_target_18dof[self.right_arm_indices] = q_sol_r
                q_target_18dof[self.left_arm_indices] = q_sol_l
                ik_success = True
        elif arm in ['left', 'right']:
            q_sol, success = self.inverse_kinematics(T_desired, arm, self.qnow, mask)
            if success:
                indices = self.left_arm_indices if arm == 'left' else self.right_arm_indices
                q_target_18dof[indices] = q_sol
                ik_success = True
        
        if ik_success:
            self.move_to_joint_target(q_target_18dof, velocity_scaling_factor)
        else:
            print("任务中止: 逆解失败。")

    def grasp_from_table(self, q_start, velocity_scaling_factor=1.0):
        """与ROS版本逻辑完全一致的抓取函数"""
        print("\n--- 开始执行桌面抓取任务 ---")
        current_T_left = self.forward_kinematics(q_start, arm='left')
        current_T_right = self.forward_kinematics(q_start, arm='right')
        
        current_left_pos, _ = SE3_to_end_pose(current_T_left)
        current_right_pos, _ = SE3_to_end_pose(current_T_right)

        target_left_pos = current_left_pos.copy()
        target_right_pos = current_right_pos.copy() 
        target_left_pos[2] = 0.7
        target_right_pos[2] = 0.7
        target_left_euler = np.array([81, 26, 70])  
        target_right_euler = np.array([81, -26, 110])
        
        T_target_left = pose_to_se3(target_left_pos[0], target_left_pos[1], target_left_pos[2], 
                                     target_left_euler[0], target_left_euler[1], target_left_euler[2])
        T_target_right = pose_to_se3(target_right_pos[0], target_right_pos[1], target_right_pos[2],
                                     target_right_euler[0], target_right_euler[1], target_right_euler[2])
        
        self.move_to_cartesian_pose((T_target_right, T_target_left), 'both', velocity_scaling_factor)

if __name__ == "__main__":
    try:
        controller = K1DualArmMujocoController()
        
        q_home_18dof = np.zeros(18)
        q_home_18dof[controller.right_arm_indices] = np.deg2rad([-1.554, -78.013, -72.530, -82.317, -50.502, 5.610, 126.298])
        q_home_18dof[controller.left_arm_indices] = np.deg2rad([1.707, -78.003, 72.538, -82.305, 50.506, -5.6, -126.290])
        q_home_18dof[controller.right_gripper_indices] = [0.01875, -0.01875]
        q_home_18dof[controller.left_gripper_indices] = [0.01875, -0.01875]

        print("======== 仿真开始: 移动到HOME位置 ========")
        controller.move_to_joint_target(q_home_18dof, velocity_scaling_factor=1.0)
        
        print("\n======== 仿真: 执行桌面抓取 ========")
        controller.grasp_from_table(controller.qnow, velocity_scaling_factor=0.8)

        print("\n仿真全部完成。")
        time.sleep(10) # 保持窗口打开10秒

    except Exception as e:
        print(f"发生未知错误: {e}")
        import traceback
        traceback.print_exc()