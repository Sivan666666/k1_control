import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import q2r
import matplotlib.pyplot as plt
from math import pi

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
            # 尝试自动查找URDF文件
            urdf_path = os.path.abspath(os.getcwd()) + '/model/K1/K1/urdf/k1_pgc.urdf'
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        
        # 从URDF加载机器人模型
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        print(self.robot)


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
        
        # 初始化图形环境
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
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
            joint_indices = self.left_arm_joints
            ee_link = self.left_ee_link
        else:
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
        print("solution:",sol)
        if sol.success:
            # 只返回对应臂的关节角度
            q_sol = sol.q[:7]
            print("q_sol:",q_sol)
            return q_sol, True
        else:
            return None, False
        
    def inverse_kinematics_QP(self, T_desired, arm='left', q0=None, tol=1e-6, max_iter=1000):
        """
        基于二次规划(QP)的逆运动学求解，严格考虑关节限制
        
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
            joint_indices = self.left_arm_joints
            ee_link = self.left_ee_link
            # 左臂关节限制 [min, max] 每个关节
            joint_limits = np.array([
                [-6.2832, 6.2832],   # l-j1
                [-1.8325, 1.8325],   # l-j2
                [-6.2832, 6.2832],   # l-j3
                [-2.5307, 0.5235],   # l-j4
                [-6.2832, 6.2832],   # l-j5
                [-1.8325, 1.8325],   # l-j6
                [-6.2832, 6.2832]    # l-j7
            ])
        else:
            joint_indices = self.right_arm_joints
            ee_link = self.right_ee_link
            # 右臂关节限制 [min, max] 每个关节
            joint_limits = np.array([
                [-6.2832, 6.2832],   # r-j1
                [-1.8325, 1.8325],   # r-j2
                [-6.2832, 6.2832],   # r-j3
                [-2.5307, 0.5235],   # r-j4
                [-6.2832, 6.2832],   # r-j5
                [-1.8325, 1.8325],   # r-j6
                [-6.2832, 6.2832]    # r-j7
            ])

        # 如果没有提供初始猜测，使用关节范围的中点
        if q0 is None:
            q0 = np.mean(joint_limits, axis=1)
        else:
            q0 = q0[joint_indices]

        # 设置QP参数
        kq = 1.0  # 关节限制避免增益
        km = 0.0  # 可操作性最大化增益 (0表示禁用)
        kj = 1.0  # 关节速度范数最小化增益
        ks = 0.1  # 松弛变量权重
        
        # 计算关节影响距离 (根据关节范围自动设置)
        pi = 0.3 * (joint_limits[:, 1] - joint_limits[:, 0])  # 30%的关节范围
        
        # 调用QP求解器
        sol = self.robot.ikine_QP(
            T_desired,
            end=ee_link,
            q0=q0,
            ilimit=max_iter,
            tol=tol,
            joint_limits=True,
            mask=[1, 1, 1, 1, 1, 1],  # 全自由度控制
            kq=kq,
            km=km,
            kj=kj,
            ks=ks,
            pi=pi
        )

        if sol.success:
            q_sol = sol.q
            # 验证关节限制
            for i, (q, (qmin, qmax)) in enumerate(zip(q_sol, joint_limits)):
                if q < qmin or q > qmax:
                    print(f"警告: 关节 {i} 超出限制: {q:.4f} 不在 [{qmin:.4f}, {qmax:.4f}] 内")
                    # 强制限制在范围内
                    q_sol[i] = np.clip(q, qmin, qmax)
            
            # 验证末端误差
            T_achieved = self.robot.fkine(q_sol, end=ee_link)
            error = np.linalg.norm(T_desired.t - T_achieved.t)
            if error > tol:
                print(f"达到的位姿误差较大: {error:.6f} > {tol:.6f}")
                return q_sol, False
            
            return q_sol, True
        else:
            print("QP求解失败:", sol.reason)
            return None, False
    
    def plot_robot(self, q, block=False):
        """可视化机器人当前状态"""
        self.robot.plot(q, backend='pyplot', fig=self.fig, block=block)
        plt.title("K1 Dual-Arm Robot")
        plt.draw()
        plt.pause(0.01)
    
    

    def test_kinematics(self):
        """测试正逆运动学"""
        print("=== K1双臂机器人运动学测试 ===")
        
        # 初始关节角度（零位）
        q_home = np.zeros(self.robot.n)
        q_home = np.array([np.deg2rad(85),np.deg2rad(-42), np.deg2rad(4.6), np.deg2rad(-97), np.deg2rad(-3.7),np.deg2rad(44.7),np.deg2rad(-1.05),0 ,0, np.deg2rad(-92),np.deg2rad(-26), np.deg2rad(5), np.deg2rad(-97), np.deg2rad(-3.7),np.deg2rad(-55),np.deg2rad(65),0 ,0])
        print(f"初始关节角度: {np.degrees(q_home)} deg")
        
        # 可视化初始状态
        self.plot_robot(q_home)
        
        # ===== 左臂测试 =====
        print("\n--- 左臂测试 ---")
        
        # 计算初始正运动学
        T_left_init = self.forward_kinematics(q_home, arm='left')
        print(f"左臂初始末端位姿:\n{T_left_init}")
        
        # 设置目标位姿（在初始位姿基础上偏移）
        # 给定的数据（按行排列）
        
        T_left_goal = pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0)
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
            
            # 可视化结果
            self.plot_robot(q_new)
        else:
            print("左臂逆解失败!")
        


        # ===== 右臂测试 =====
        print("\n--- 右臂测试 ---")
        
        # 计算初始正运动学
        T_right_init = self.forward_kinematics(q_home, arm='right')
        print(f"右臂初始末端位姿:\n{T_right_init}")
        
        # 设置目标位姿
        T_right_goal = pose_to_se3(0.52, -0.2, 0.42, -150, -30, -40)
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
            
            # 可视化结果
            self.plot_robot(q_new)
        else:
            print("右臂逆解失败!")
        
        # 保持图形窗口打开
        plt.show(block=True)
        



if __name__ == "__main__":
    # 创建控制器实例
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")
        
        # 运行运动学测试
        controller.test_kinematics()
        
    except Exception as e:
        print(f"初始化失败: {str(e)}")