import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import mujoco
import mujoco.viewer
import time
import toppra as ta
import toppra.constraint as constraint

def pose_to_se3(x, y, z, rx, ry, rz):
    """将位姿转换为SE3对象"""
    return SE3(x, y, z) * SE3.Rx(np.deg2rad(rx)) * SE3.Ry(np.deg2rad(ry)) * SE3.Rz(np.deg2rad(rz))

class K1DualArmController:
    def __init__(self, urdf_path=None):
        # 初始化机器人模型

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
        self.right_ee_link = "right_gripper_adapter"
        self.left_ee_link = "left_gripper_adapter"
        
        # 初始关节位置 (示例值)
        self.qnow = np.zeros(18)
        self.qnow[self.right_arm_joints] = [0.84, -0.67, -0.70, -2.42, -1.10, 0.90, -1.92]
        self.qnow[self.left_arm_joints] = [-0.94, -0.73, 0.87, -2.42, 1.06, -0.78, -1.14]
        
        # 控制参数
        self.Kp = 100.0
        self.Kd = 5.0
        self.velocity_limits = np.array([3.0] * 14)
        self.acceleration_limits = np.array([3.0] * 14)
        
        # 关节限制
        self.joint_limits = {
            'left': [(-6.28, 6.28), (-1.83, 1.83), (-6.28, 6.28), 
                    (-2.53, 0.52), (-6.28, 6.28), (-1.83, 1.83), (-6.28, 6.28)],
            'right': [(-6.28, 6.28), (-1.83, 1.83), (-6.28, 6.28), 
                     (-2.53, 0.52), (-6.28, 6.28), (-1.83, 1.83), (-6.28, 6.28)]
        }

    def inverse_kinematics(self, T_desired, arm='left', q0=None):
        """改进的逆运动学求解"""
        joint_indices = self.left_arm_joints if arm == 'left' else self.right_arm_joints
        ee_link = self.left_ee_link if arm == 'left' else self.right_ee_link
        
        tol=1e-6
        max_iter=10000
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
        
        if sol.success:
            q_sol = sol.q
            # 检查关节限制
            limits = self.joint_limits[arm]
            for i, (q, (low, high)) in enumerate(zip(q_sol, limits)):
                if not low <= q <= high:
                    print(f"关节 {i} 超出限制: {q:.2f} not in [{low:.2f}, {high:.2f}]")
                    return None, False
            return q_sol, True
        return None, False

    def plan_trajectory(self, waypoints, arm='left'):
        """使用TOPPRA规划轨迹"""
        joint_indices = self.left_arm_joints if arm == 'left' else self.right_arm_joints
        
        # 提取对应臂的关节角度
        waypoints_arm = [wp[joint_indices] for wp in waypoints]
        
        # 创建路径和约束
        path = ta.SplineInterpolator(np.linspace(0, 1, len(waypoints_arm)), waypoints_arm)
        vlim = np.vstack([-self.velocity_limits, self.velocity_limits]).T[:7]
        alim = np.vstack([-self.acceleration_limits, self.acceleration_limits]).T[:7]
        
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)
        
        # 轨迹规划
        instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path)
        jnt_traj = instance.compute_trajectory(0, 0)
        
        # 采样轨迹
        ts = np.linspace(0, jnt_traj.get_duration(), 100)
        return {
            'time': ts,
            'position': jnt_traj.eval(ts),
            'duration': jnt_traj.get_duration()
        }

    def generate_symmetric_trajectory(self, arm='left'):
        """生成对称的双臂轨迹"""
        # 对称轨迹定义 (左臂)
        left_motion = [
            pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0),
            pose_to_se3(0.2, 0.3, 0.2, 160, 0, 0),
            pose_to_se3(0.32, 0.3, 0.22, 160, 0, 0),
            pose_to_se3(0.52, 0.3, 0.42, 130, -20, 35),
            pose_to_se3(0.35, 0.3, 0.20, 160, 0, 0),
            pose_to_se3(0.30, 0.3, 0.17, 160, 0, 0),
            pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0)
        ]
        
        # 右臂镜像轨迹 (Y坐标和roll角取反)
        right_motion = [
            pose_to_se3(0.20, -0.22, 0.15, -160, 0, 0),
            pose_to_se3(0.2, -0.3, 0.2, -160, 0, 0),
            pose_to_se3(0.32, -0.3, 0.22, -160, 0, 0),
            pose_to_se3(0.52, -0.3, 0.42, -130, -20, -35),
            pose_to_se3(0.35, -0.3, 0.20, -160, 0, 0),
            pose_to_se3(0.30, -0.3, 0.17, -160, 0, 0),
            pose_to_se3(0.20, -0.22, 0.15, -160, 0, 0)
        ]
        
        motion = left_motion if arm == 'left' else right_motion
        waypoints = []
        
        for pose in motion:
            q_sol, success = self.inverse_kinematics(pose, arm=arm, q0=self.qnow)
            if not success:
                print(f"{arm} arm IK failed at pose: {pose}")
                return None
                
            full_q = self.qnow.copy()
            joint_indices = self.left_arm_joints if arm == 'left' else self.right_arm_joints
            full_q[joint_indices] = q_sol
            waypoints.append(full_q)
            
        return self.plan_trajectory(waypoints, arm)

    def execute_dual_arm_trajectory(self, left_traj, right_traj):
        """同步执行双臂轨迹"""
        start_time = time.time()
        
        while self.viewer.is_running():
            t = (time.time() - start_time) % max(left_traj['duration'], right_traj['duration'])
            
            # 左臂轨迹执行
            idx_left = np.searchsorted(left_traj['time'], t, side='left')
            if idx_left < len(left_traj['position']):
                self.data.qpos[self.left_arm_joints] = left_traj['position'][idx_left]
            
            # 右臂轨迹执行
            idx_right = np.searchsorted(right_traj['time'], t, side='left')
            if idx_right < len(right_traj['position']):
                self.data.qpos[self.right_arm_joints] = right_traj['position'][idx_right]
            
            # 夹爪控制 (示例: 保持半开)
            self.data.qpos[self.left_gripper_joints] = [0.01875, -0.01875]  # 左夹爪
            self.data.qpos[self.right_gripper_joints] = [0.01875, -0.01875] # 右夹爪
            
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(0.01)

    def run(self):
        """主运行函数"""
        print("Generating trajectories...")
        left_traj = self.generate_symmetric_trajectory(arm='left')
        right_traj = self.generate_symmetric_trajectory(arm='right')
        
        if left_traj and right_traj:
            print("Executing dual-arm motion...")
            self.execute_dual_arm_trajectory(left_traj, right_traj)
        else:
            print("Trajectory generation failed")
        
        self.viewer.close()

if __name__ == "__main__":
    try:
        controller = K1DualArmController()
        controller.run()
    except Exception as e:
        print(f"Error: {str(e)}")