import mujoco
import mujoco.viewer
import time

# Load the model from XML file
model = mujoco.MjModel.from_xml_path("/home/kalyan/mujoco_menagerie/franka_fr3/fr3.xml")  # replace with your file path

# Create a data object for simulation
data = mujoco.MjData(model)

# Launch the viewer for simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() :
        mujoco.mj_step(model, data)
        viewer.sync()
