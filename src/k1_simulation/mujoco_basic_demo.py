import time

import mujoco
import mujoco.viewer

# m = mujoco.MjModel.from_xml_path('/home/hwk/ros_ws/k1_ros/src/k1_simulation/model/K1/k1_new.xml')

m = mujoco.MjModel.from_xml_path('/home/hwk/ros_ws/k1_ros/src/k1_simulation/model/K1/k1_pgc_fix_v1.xml')
# m = mujoco.MjModel.from_xml_path('/home/hwk/ros_ws/k1_ros/src/k1_simulation/model/K1/k1_pgc_35_50_new.xml')
# m = mujoco.MjModel.from_xml_path('/home/hwk/program/cloth_simulation/mujoco_model/Style3dCloth_Fixed_Point.xml')
# m = mujoco.MjModel.from_xml_path('/home/hwk/program/cloth_simulation/mujoco_model/poncho_flex_cloth.xml')
# m = mujoco.MjModel.from_xml_path('/home/hwk/program/cloth_simulation/mujoco_model/test_vertid.xml')
# m = mujoco.MjModel.from_xml_path('/home/hwk/program/cloth_simulation/mujoco_model/piper_bimanual_urdf.xml')
# m = mujoco.MjModel.from_xml_path('/home/hwk/program/cloth_simulation/mujoco_model/piper_bimanual_description.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 10000:
    step_start = time.time()


    import numpy as np
    left_joint_positions = np.append(np.array([-21.3, -51.7, 65.7, -134.2, 121.25, -40.5, -136.6])*np.pi/180,[0.01875,-0.01875])
    right_joint_positions = np.append(np.array([21.3, -51.7, -65.7, -134.2, -121.25, 40.5, 136.6])*np.pi/180,[0.01875,-0.01875]) 
                              

    left_joint_name = ["l-j1","l-j2","l-j3","l-j4","l-j5","l-j6","l-j7", "left_finger1_joint", "left_finger2_joint"]
    right_joint_name = ["r-j1","r-j2","r-j3","r-j4","r-j5","r-j6","r-j7", "right_finger1_joint", "right_finger2_joint"]
    for i, joint_name in enumerate(left_joint_name):
      d.qpos[m.joint(joint_name).id] = left_joint_positions[i]
    for i, joint_name in enumerate(right_joint_name):
        d.qpos[m.joint(joint_name).id] = right_joint_positions[i]
    
    dof_index = 10
    joint_id = m.dof_jntid[dof_index]
    joint_name = m.joint(joint_id).name
    print(f"问题关节名称: {joint_name}")


    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)