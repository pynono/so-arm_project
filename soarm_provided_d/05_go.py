import time
import numpy as np
import mujoco
import mujoco.viewer
from soarm_lab import SCENE
from soarm_lab.ik_core import IKSo101

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)
ik = IKSo101()

deg, err = ik.solve([0.25, 0.0, 0.15])  # 좌표 -> 각도(도), 잔차
data.ctrl[:5] = np.radians(deg)  # qpos 고정을 위한 코드

with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
        time.sleep(model.opt.timestep)
