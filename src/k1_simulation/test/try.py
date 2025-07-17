import numpy as np
import mujoco
import mujoco.viewer
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH

# ---------------------- 运动学模型 ---------------------- #
class K1Kinematics:
    def __init__(self):
        self.right_arm = self._create_right_arm()
        self.left_arm = self._create_left_arm()

    def _create_right_arm(self):
        return DHRobot([
            RevoluteDH(a=0, alpha=np.pi/2, d=0.217, qlim=[-2*np.pi, 2*np.pi]),
            RevoluteDH(a=0, alpha=-np.pi/2, d=0.2075, qlim=[-1.8325, 1.8325]),
            RevoluteDH(a=0, alpha=np.pi/2, d=0, qlim=[-2*np.pi, 2*np.pi]),
            RevoluteDH(a=0.0004, alpha=-np.pi/2, d=0.33028, qlim=[-2.5307, 0.5235]),
            RevoluteDH(a=0, alpha=np.pi/2, d=0, qlim=[-2*np.pi, 2*np.pi]),
            RevoluteDH(a=0, alpha=-np.pi/2, d=0.23494, qlim=[-1.8325, 1.8325]),
            RevoluteDH(a=0, alpha=np.pi/2, d=0, qlim=[-2*np.pi, 2*np.pi]),
        ], name="RightArm")

    def _create_left_arm(self):
        return DHRobot([
            RevoluteDH(a=0, alpha=np.pi/2, d=0.217, offset=np.pi, qlim=[-2*np.pi, 2*np.pi]),
            RevoluteDH(a=0, alpha=-np.pi/2, d=0.2075, qlim=[-1.8325, 1.8325]),
            RevoluteDH(a=0, alpha=np.pi/2, d=0, qlim=[-2*np.pi, 2*np.pi]),
            RevoluteDH(a=0.0004, alpha=-np.pi/2, d=0.33028, qlim=[-2.5307, 0.5235]),
            RevoluteDH(a=0, alpha=np.pi/2, d=0, qlim=[-2*np.pi, 2*np.pi]),
            RevoluteDH(a=0, alpha=-np.pi/2, d=0.23494, qlim=[-1.8325, 1.8325]),
            RevoluteDH(a=0, alpha=np.pi/2, d=0, qlim=[-2*np.pi, 2*np.pi]),
        ], name="LeftArm")

    def right_fk(self, q):
        return self.right_arm.fkine(q)
    
    def left_fk(self, q):
        return self.left_arm.fkine(q).data[0]
    
    def right_ik(self, T, q0=None):
        if q0 is None: q0 = np.zeros(7)
        sol = self.right_arm.ikine_LM(T, q0=q0)
        return sol.q if sol.success else None
    
    def left_ik(self, T, q0=None):
        if q0 is None: q0 = np.zeros(7)
        sol = self.left_arm.ikine_LM(T, q0=q0)
        return sol.q if sol.success else None

# ---------------------- 轨迹生成 ---------------------- #
def generate_square_trajectory(center, side_length, height=0.2, n_points=50):
    """生成正方形轨迹（XY平面，Z为高度）"""
    points = []
    # 起点
    x, y = center
    points.append((x, y, height))
    # 右边缘
    for _ in range(n_points):
        x += side_length / n_points
        points.append((x, y, height))
    # 上边缘
    for _ in range(n_points):
        y += side_length / n_points
        points.append((x, y, height))
    # 左边缘
    for _ in range(n_points):
        x -= side_length / n_points
        points.append((x, y, height))
    # 下边缘
    for _ in range(n_points):
        y -= side_length / n_points
        points.append((x, y, height))
    return np.array(points)

# ---------------------- Mujoco 仿真 ---------------------- #
class K1Simulator:
    def __init__(self):
        # 加载模型
        
        self.model = mujoco.MjModel.from_xml_path("model/K1/k1.xml")
        self.data = mujoco.MjData(self.model)
        
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        print("3")
        # 运动学求解器
        self.kinematics = K1Kinematics()
        print("4")
        # 轨迹参数
        self.right_trajectory = generate_square_trajectory(center=(0.3, -0.3), side_length=0.2)
        self.left_trajectory = generate_square_trajectory(center=(0.3, 0.3), side_length=0.2, height=0.15)
        
        self.trajectory_idx = 0
        self.q_right = np.zeros(7)
        self.q_left = np.zeros(7)
        

    def update_controls(self):
        """更新关节角度控制指令"""
        # 右臂轨迹
        print("1")
        if self.trajectory_idx < len(self.right_trajectory):
            pos = self.right_trajectory[self.trajectory_idx]

            # 输入参数
            X = 0.17
            Y = 0.086
            Z = 0.421
            rx_deg = 91   # 绕X轴旋转角度（度数）
            ry_deg = -12  # 绕Y轴旋转角度（度数）
            rz_deg = -9   # 绕Z轴旋转角度（度数）

            # 角度转弧度
            rx = np.deg2rad(rx_deg)
            ry = np.deg2rad(ry_deg)
            rz = np.deg2rad(rz_deg)

            # 绕Z轴旋转矩阵
            Rz = np.array([
                [np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz), np.cos(rz), 0],
                [0, 0, 1]
            ])

            # 绕Y轴旋转矩阵
            Ry = np.array([
                [np.cos(ry), 0, np.sin(ry)],
                [0, 1, 0],
                [-np.sin(ry), 0, np.cos(ry)]
            ])

            # 绕X轴旋转矩阵
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx), np.cos(rx)]
            ])

            # 计算总旋转矩阵（ZYX顺序：R = Rx * Ry * Rz）
            R = np.dot(Rx, np.dot(Ry, Rz))

            # 构建齐次变换矩阵
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [X, Y, Z]

            #T = SE3(0.65, 0.05, 0.4)   # 添加姿态（示例）
            q = self.kinematics.right_ik(T)
            print(q)
            if q is not None:
                self.q_right = q
        
        # 左臂轨迹
        if self.trajectory_idx < len(self.left_trajectory):
            pos = self.left_trajectory[self.trajectory_idx]
            #T = SE3(pos[0], pos[1], pos[2]) * SE3.Rz(-np.pi/2)  # 镜像姿态
            q = self.kinematics.left_ik(T)
            print("q",q)
            q = self.kinematics.left_fk(np.array([np.deg2rad(-90),np.deg2rad(-75), np.deg2rad(92), np.deg2rad(-120), np.deg2rad(-70),np.deg2rad(-92),np.deg2rad(41)]))
            print("q2",q)
            if q is not None:
                self.q_left = q
        
        self.trajectory_idx += 1
        if self.trajectory_idx >= len(self.right_trajectory):
            self.trajectory_idx = 0  # 循环轨迹

    def run(self):
        while self.viewer.is_running():
            # 更新控制指令
            print("1")
            self.update_controls()
            
            # 设置关节角度
            self.data.qpos[:7] = self.q_right
            self.data.qpos[7:14] = np.array([np.deg2rad(-90),np.deg2rad(-75), np.deg2rad(92), np.deg2rad(-120), np.deg2rad(-70),np.deg2rad(-92),np.deg2rad(41)])
            print("left",self.kinematics.left_fk([np.deg2rad(-90),np.deg2rad(-75), np.deg2rad(92), np.deg2rad(-120), np.deg2rad(-70),np.deg2rad(-92),np.deg2rad(41)]))
            # 步进仿真
            mujoco.mj_step(self.model, self.data)
            
            # 渲染画面
            self.viewer.sync()
            
            # 控制速度
            # mujoco.mj_energy(self.model, self.data)  # 保持仿真稳定

        self.viewer.close()

# ---------------------- 主函数 ---------------------- #
if __name__ == "__main__":
    print("1")
    simulator = K1Simulator()
    print("2")
    simulator.run()