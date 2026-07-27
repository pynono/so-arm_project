import mujoco
import mujoco.viewer

SCENE_XML = "scenes/01_6_geoms.xml"

model = mujoco.MjModel.from_xml_path(SCENE_XML)
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
