#!/home/sivan/miniconda3/envs/mujo_ros/bin/python3

import mujoco.viewer
import time
 
def main():
    model = mujoco.MjModel.from_xml_path('model/K1/k1_scene.xml')
    data = mujoco.MjData(model)
    #data.ctrl[:14] = [-1.57, -1.34, 2.65, -1.3, 1.55, 1.2, 0, -1.57, -1.34, 2.65, -1.3, 1.55, 1.2, 0]
    #data.ctrl[:14] = [-2.89, -1.43, -1.07, -0.164, 1.76, 1.26, -4.02, 2.89, -1.43, -1.07, -0.164, -1.26, -1.26, -1.45]
    #data.qpos[:2] = [0.01875, 0.01875]
    #data.ctrl[14:16] = [0.035 , 0.035]
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            data.qpos[7] = 0.01875
            data.qpos[8] = -0.01875
            mujoco.mj_step(model, data)
            for i in range(data.ncon):
                contact = data.contact[i]
                # 获取几何体对应的body_id
                body1_id = model.geom_bodyid[contact.geom1]
                body2_id = model.geom_bodyid[contact.geom2]
                
                # 通过mj_id2name转换body_id为名称
                body1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
                body2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2_id)
                
                print(f"接触点 {i}: 几何体 {contact.geom1} 名字 {body1_name} 和 {contact.geom2} 名字 {body2_name} 在位置 {contact.pos} 发生接触")

            viewer.sync()
            time.sleep(0.002) # 让动画速度变慢，不然更新太快看不清机械臂的运动过程
 
if __name__ == "__main__":
    main()

