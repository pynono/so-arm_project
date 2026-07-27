from soarm_lab.grasp_scene import build, BALL_XY, BALL_R
import numpy as np
import mujoco
import mujoco.viewer

model, data = build(basket_xy=(0.16, -0.12))
bid = model.body("obj").id
print("공 위치:", data.xpos[bid])

mujoco.viewer.launch(model, data)
