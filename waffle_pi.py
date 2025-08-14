import mujoco 
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("/home/kalyan/robotis_mujoco_menagerie/robotis_tb3/scene_turtlebot3_waffle_pi.xml")
data = mujoco.MjData(model)

body_name = "base"
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)

force = [100, 0, 0]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.xfrc_applied[body_id, :3] = force
        mujoco.mj_step(model, data)
        viewer.sync()