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

    return pos, eul

def circular_trajectory(radius, center, axis, period, t):
    """
    生成圆形轨迹的位姿
    :param radius: 圆的半径
    :param center: 圆心位置 [x, y, z]
    :param axis: 圆的轴方向 [x, y, z]
    :param period: 圆的周期（单位：秒）
    :param t: 当前时间（单位：秒）
    :return: 位姿 [x, y, z, rx, ry, rz]（单位：米和度）
    """
    # 角度随时间变化
    theta = 2 * np.pi * t / period
    
    # 圆的参数方程
    if axis == 'x':
        x = center[0]
        y = center[1] + radius * np.cos(theta)
        z = center[2] + radius * np.sin(theta)
    elif axis == 'y':
        x = center[0] + radius * np.sin(theta)
        y = center[1]
        z = center[2] + radius * np.cos(theta)
    elif axis == 'z':
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        z = center[2]
    else:
        raise ValueError("轴方向必须是 'x', 'y' 或 'z'")
    
    # 旋转角度（保持固定方向）
    rx, ry, rz = 180, 0, 0
    
    return [x, y, z, rx, ry, rz]

def interpolate_se3(T1, T2, t):
    """
    在两个 SE3 位姿之间插值
    :param T1: 起始位姿 (SE3)
    :param T2: 目标位姿 (SE3)
    :param t: 插值参数 (0 到 1)
    :return: 插值后的位姿 (SE3)
    """
    # 插值平移部分
    p1 = T1.t
    p2 = T2.t
    p = p1 + t * (p2 - p1)
    
    # 插值旋转部分
    R1 = T1.R
    
    R = R1
    
    # 创建插值后的位姿
    T = SE3.Rt(R, p)
    return T

    return site_pos, base_pos

class K1DualArmController:
    def __init__(self, urdf_path=None):
        """
        初始化K1双臂机器人控制器
        
        参数:
            urdf_path: URDF文件路径，如果为None则尝试默认路径
        """
        try:
            PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
        except NameError:
            PROJECT_ROOT = os.getcwd()
        

        urdf_path = os.path.join(PROJECT_ROOT,"model","K1","K1","urdf","k1.urdf")
        urdf_path = os.path.join(PROJECT_ROOT,"model","K1","K1","urdf","k1_pgc.urdf")
        xml_path = os.path.join(PROJECT_ROOT,"model","K1","k1_new.xml")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")
        
        # 从URDF加载机器人模型
        self.robot = rtb.ERobot.URDF(file_path=urdf_path)
        print(self.robot)
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
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

    def test_kinematics(self):
        # q_left_arm = np.array([1.707,-78.003,72.538,-82.305,50.506,-5.6,-126.290]) *np.pi/180
        # q_right_arm =  np.array([-1.554,-78.013,-72.530,-82.317,-50.502,5.610,126.298]) *np.pi/180
        
        q_left_arm = np.array([1.4,-78.000,72.500,-82.305,50.506,-5.6,-126.30]) *np.pi/180
        q_right_arm =  np.array([-1.4,-78.00,-72.530,-82.3,-50.50,5.610,126.298]) *np.pi/180
        
        
        # q_left_arm = np.zeros(7)
        # q_right_arm = np.zeros(7)
        q_full_robot= np.concatenate((q_right_arm, q_left_arm))
        
        q_full_robot = np.array([-1.4, -77.999,  -72.5,  -82.299,  -50.5,      5.599,  126.298,
                                1.399, -77.998,   72.498,  -82.299,   50.5,     -5.599, -126.299 ])*np.pi/180
        
        T = self.forward_kinematics(q_full_robot, arm='left')
        print(f"左臂末端执行器位姿: {T}")
        self.qnow =q_full_robot

        end_pose, end_state = SE3_to_end_pose(T)
        print(f"左臂末端执行器位姿: {end_pose}, 欧拉角: {end_state}")
  
        self.data.qpos[:14] = self.qnow
        mujoco.mj_forward(self.model, self.data)
        left_site_name = "wrist_flange_left"  # 左臂末端执行器的site名称
        right_site_name = "wrist_flange_right"  # 右臂末端执行器的site名称
        left_pos= self.data.site(left_site_name).xpos
        print(f"左臂末端执行器位置: {left_pos}")
        right_pos = self.data.site(right_site_name).xpos
        print(f"右臂末端执行器位置: {right_pos}")
        self.run()

        
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
            print("Left BEGIN")
            joint_indices = self.left_arm_joints
            ee_link = self.left_ee_link
            joint_limits = self.joint_limits['left']
        else:
            print("Right BEGIN")
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
            print("q_sol:",q_sol)
            for i, (q, limit) in enumerate(zip(q_sol, joint_limits)):
                if q < limit[0] or q > limit[1]:
                    print(f"关节 {i} 超出限制范围: {q} 不在 {limit} 内")
                    return None, False
            return q_sol, True
        else:
            return None, False
        
        
    def traj_run(self):
        """运行仿真循环"""
        # 定义左右臂的圆形轨迹参数
        left_circle_params = {
            'radius': 0.1,  # 左臂圆的半径
            'center': [0.2, 0.2, 0.2],  # 左臂圆心位置
            'axis': 'x',  # 左臂圆的轴方向
            'period': 10  # 左臂圆的周期（秒）
        }
        
        right_circle_params = {
            'radius': 0.1,  # 右臂圆的半径
            'center': [0.2, -0.2, 0.2],  # 右臂圆心位置
            'axis': 'x',  # 右臂圆的轴方向
            'period': 10  # 右臂圆的周期（秒）
        }
        
        start_time = time.time()
        
        while self.viewer.is_running():
            current_time = time.time() - start_time
            
            # 生成左臂的圆形轨迹位姿
            left_pose = circular_trajectory(**left_circle_params, t=current_time)
            T_left_goal = pose_to_se3(*left_pose)
            
            # 生成右臂的圆形轨迹位姿
            right_pose = circular_trajectory(**right_circle_params, t=current_time)
            T_right_goal = pose_to_se3(*right_pose)
            
            # 求解逆运动学
            q_left_sol, _ = self.inverse_kinematics(T_left_goal, arm='left', q0=self.qnow)
            q_right_sol, _ = self.inverse_kinematics(T_right_goal, arm='right', q0=self.qnow)
            
            if q_left_sol is not None and q_right_sol is not None:
                # 更新整个机器人的关节角度
                q_new = self.qnow.copy()
                q_new[self.left_arm_joints] = q_left_sol
                q_new[self.right_arm_joints] = q_right_sol
                self.qnow = q_new
            # 遍历所有接触点


            for i in range(self.data.ncon):
                contact = self.data.contact[i]
                # 获取几何体对应的body_id
                body1_id = self.model.geom_bodyid[contact.geom1]
                body2_id = self.model.geom_bodyid[contact.geom2]
                
                # 通过mj_id2name转换body_id为名称
                body1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
                body2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2_id)
                
                print(f"接触点 {i}: 几何体 {contact.geom1} 名字 {body1_name} 和 {contact.geom2} 名字 {body2_name} 在位置 {contact.pos} 发生接触")
                
            
            # 更新仿真数据
            self.data.qpos[:14] = self.qnow
            mujoco.mj_step(self.model, self.data)
            
            # 渲染画面
            self.viewer.sync()
            
            # 控制仿真速度
            time.sleep(0.01)
        
        self.viewer.close()
    
    def fling_trajectory(self, arm='left'):
        """
        生成 Fling 动作的关键点位并求解逆运动学
        :param arm: 'left' 或 'right'
        :return: 关节角度列表
        """
        if arm == 'left':
            joint_indices = self.left_arm_joints
            ee_link = self.left_ee_link
        else:
            joint_indices = self.right_arm_joints
            ee_link = self.right_ee_link

        if arm == 'left':
            motion = []
            motion.append(pose_to_se3(0.20, 0.22, 0.15, 160, 0, 0))
            motion.append(pose_to_se3(0.2, 0.20, 0.2, 160, 0, 0))
            motion.append(pose_to_se3(0.32, 0.20, 0.22, 160, 0, 0))
            motion.append(pose_to_se3(0.52, 0.20, 0.42, 130, -20, 35))
            motion.append(pose_to_se3(0.35, 0.20, 0.20, 160, 0, 0))
            motion.append(pose_to_se3(0.30, 0.20, 0.17, 160, 0, 0))
            motion.append(pose_to_se3(0.20, 0.20, 0.15, 160, 0, 0))

            # 插值生成更多关键点位
            interpolated_motion = []
            for i in range(len(motion) - 1):
                interpolated_motion.append(motion[i])
                # T1 = motion[i]
                # T2 = motion[i + 1]
                # for t in np.linspace(0, 1, 20):
                #     interpolated_motion.append(interpolate_se3(T1, T2, t))
            interpolated_motion.append(motion[-1])
        else:
            motion = []
            motion.append(pose_to_se3(0.20, -0.22, 0.15, -160, -15, 0))
            motion.append(pose_to_se3(0.2, -0.2, 0.2, -160, -15, 0))
            motion.append(pose_to_se3(0.32, -0.2, 0.22, -160, -15, 0))
            motion.append(pose_to_se3(0.52, -0.2, 0.42, -150, -30, -40))
            motion.append(pose_to_se3(0.35, -0.2, 0.20, -160, -15, 0))
            motion.append(pose_to_se3(0.30, -0.2, 0.17, -160, -15, 0))
            motion.append(pose_to_se3(0.20, -0.22, 0.15, -160, -15, 0))

            # 插值生成更多关键点位
            interpolated_motion = []
            for i in range(len(motion) - 1):
                interpolated_motion.append(motion[i])
                # T1 = motion[i]
                # T2 = motion[i + 1]
                # for t in np.linspace(0, 1, 20):
                #     interpolated_motion.append(interpolate_se3(T1, T2, t))
            interpolated_motion.append(motion[-1])

        # 求解逆运动学
        q_solutions = []
        for T in interpolated_motion:
            q_sol, success = self.inverse_kinematics(T, arm=arm, q0=self.qnow)
            self.qnow[joint_indices] = q_sol
            if success:
                q_solutions.append(q_sol)
            else:
                print(len(q_solutions))
                print(f"逆运动学求解失败: {T}")
                return None

        return q_solutions
    
    def run_fling(self):
        """
        运行 Fling 动作
        """
        while self.viewer.is_running():
            left_q_solutions = self.fling_trajectory(arm='left')
            right_q_solutions = self.fling_trajectory(arm='right')

            if left_q_solutions is None:
                print("Fling 动作规划失败")
                return

            for left_q, right_q in zip(left_q_solutions, right_q_solutions):
                # left_control = self.pd_control(left_q, self.qnow[self.left_arm_joints], self.data.qvel[self.left_arm_joints])
                # right_control = self.pd_control(right_q, self.qnow[self.right_arm_joints], self.data.qvel[self.right_arm_joints])

                # # 应用控制输入
                # self.data.ctrl[self.left_arm_joints] = left_control
                # self.data.ctrl[self.right_arm_joints] = right_control


                self.qnow[self.left_arm_joints] = left_q
                self.qnow[self.right_arm_joints] = right_q
                self.data.qpos[:14] = self.qnow
                #self.data.ctrl[:14] = self.qnow
                mujoco.mj_step(self.model, self.data)
                self.viewer.sync()
                time.sleep(0.5)

        self.viewer.close()



    def run(self):
        """运行仿真循环"""
        while self.viewer.is_running():
            self.data.qpos[:14] = self.qnow
            # 步进仿真
            mujoco.mj_step(self.model, self.data)
            
            # 渲染画面
            self.viewer.sync()
            
            # 控制仿真速度
            time.sleep(0.01)
        
        self.viewer.close()
        





if __name__ == "__main__":
    # 创建控制器实例
    try:
        controller = K1DualArmController()
        print("机器人模型加载成功!")
        
        # 运行运动学测试
        controller.test_kinematics()
        #controller.traj_run()
        # controller.run_fling()

        
    except Exception as e:
        print(f"初始化失败: {str(e)}")