import time
import numpy as np
from soarm_lab.ik_core import IKSo101
from soarm_lab import RealBackend

ik = IKSo101()
robot = RealBackend(port="/dev/ttyACM0")  # 포트번호 - ls /dev/ttyACM*

deg, err = ik.solve([0.25, 0.0, 0.15])
print("각도", np.round(deg, 1), "잔차(mm):")

robot.move(deg)
time.sleep(2)
