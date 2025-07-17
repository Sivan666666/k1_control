import roboticstoolbox as rtb
import numpy as np
import os
from spatialmath.base import r2q

# --- File Path Setup ---
# Replace with your actual file paths
try:
    PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
except NameError:
    PROJECT_ROOT = os.getcwd()

# We only need to load the most complete URDF file
URDF_FILE_PATH = os.path.join(PROJECT_ROOT,"model","K1","K1","urdf","k1_pgc.urdf")
URDF_WITHOUT_GRIPPER_FILE_PATH = os.path.join(PROJECT_ROOT,"model","K1","K1","urdf","k1.urdf")



# --- 1. Load a SINGLE robot object from the complete URDF ---
# The toolbox will parse the entire structure, including both arms and grippers.
print("--- Loading full K1 dual-arm model ---")
robot = rtb.ERobot.URDF(file_path=URDF_WITHOUT_GRIPPER_FILE_PATH)
print("--- Model Loaded ---")

# --- 2. Inspect the model to understand the joint order and count ---
# This is a crucial step to ensure your joint vector 'q' is correct.
print("\nModel Details:")
print(robot)
print(f"\nTotal Degrees of Freedom (n): {robot.n}")
print("Please ensure your joint vector 'q' matches this structure and count.")


# --- 3. Define the FULL joint vector 'q' ---
# Based on the URDF structure for the K1 dual-arm with grippers,
# there are 7 joints for the right arm, 7 for the left, and 1 for each gripper.
# Total joints = 7 (r-arm) + 7 (l-arm) + 1 (r-gripper) + 1 (l-gripper) = 16
q_left_arm = np.array([1.707,-78.003,72.538,-82.305,50.506,-5.6,-126.290]) *np.pi/180
q_right_arm =  np.array([-1.554,-78.013,-72.530,-82.317,-50.502,5.610,126.298]) *np.pi/180
q_full_robot= np.concatenate((q_right_arm, q_left_arm))
# if robot.n == 16:
#     q_right_arm = np.array([1.4,-78,72.5,-82.3,50.499,-5.6,-126.3]) *np.pi/180
#     q_left_arm =  np.array([-1.4,78,-72.5,82.3,-50.499,5.6,126.3]) *np.pi/180
#     # q_right_gripper = [0.01]  # right_finger1_joint
#     # q_left_gripper = [0.01]   # left_finger1_joint

#     # The order must match the order shown in `print(robot)`
#     # Typically, it's all joints from the first branch, then all from the second.
#     # r-j1..7, l-j1..7, right_finger1, left_finger1 (example order)
#     # q_full_robot = np.array(
#     #     q_right_arm + q_left_arm + q_right_gripper + q_left_gripper
#     # )
# else:
#     # If the joint count is different, create a zero vector as a placeholder
#     print(f"\n[Warning] robot.n is {robot.n}, not 16. Using a zero vector for FK calculation.")
#     q_full_robot = np.zeros(robot.n)


print("\n==================== FK Calculation Test ====================\n")


# --- 4. Call fkine, dynamically specifying the 'end' link ---

# A. Get pose for the right arm's flange
T_rt = robot.fkine(q_full_robot, end='rt')
print(f"Pose of 'rt':\n  -> Position = {T_rt.t}, Orientation = {r2q(T_rt.R)}")

# B. Get pose for the left arm's flange
T_lt = robot.fkine(q_full_robot, end='lt')
print(f"Pose of 'lt':\n  -> Position = {T_lt.t}, Orientation = {r2q(T_lt.R)}")

# # C. Get pose for the right gripper's TCP
# T_r_gripper = robot.fkine(q_full_robot, end='right_gripper_adapter')
# print(f"Pose of 'right_gripper_adapter':\n  -> Position = {np.round(T_r_gripper.t, 4)}")

# # D. Get pose for the left gripper's TCP
# T_l_gripper = robot.fkine(q_full_robot, end='left_gripper_adapter')
# print(f"Pose of 'left_gripper_adapter':\n  -> Position = {np.round(T_l_gripper.t, 4)}")

print("\n==================== Test Complete ====================")