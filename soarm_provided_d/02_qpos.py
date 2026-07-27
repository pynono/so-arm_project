import mujoco
import mujoco.viewer
import numpy as np
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)

data.qpos[:5] = np.radians([0, 30, -45, 0, 0])
data.ctrl[:5] = np.radians([0, 30, -45, 0, 0])  # qpos 고정을 위한 코드

mujoco.mj_forward(model, data)

mujoco.viewer.launch(model, data)  # 팔 모양 확인
