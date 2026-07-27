import mujoco
import mujoco.viewer
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)  # 팔 모양 확인
