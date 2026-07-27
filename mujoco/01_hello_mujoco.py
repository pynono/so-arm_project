import mujoco
import mujoco.viewer

xml = "<mujoco><worldbody/></mujoco>"
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
