import time
import mujoco
import mujoco.viewer
import numpy as np
from soarm_lab import SCENE

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)


data.ctrl[:5] = np.radians([0, 30, -45, 0, 0])  # qpos 고정을 위한 코드

with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
        time.sleep(model.opt.timestep)
