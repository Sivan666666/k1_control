import time

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('/home/hwk/ros_ws/k1_ros/src/k1_simulation/model/K1/k1_new.xml')
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