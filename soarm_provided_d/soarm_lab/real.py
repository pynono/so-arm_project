# -*- coding: utf-8 -*-
"""real.py — 실물 백엔드(시리얼 직결).

시뮬과 같은 인터페이스(move/ee/wait)라 arm.go(real=True) 로 전환된다.
driver_sdk 로 STS3215 서보에 직접 명령한다.
관절 id: 1 base·2 shoulder·3 elbow·4 wrist·5 roll·6 gripper. 각도=도, 0=중앙.
"""
import numpy as np
from fk_core import FKSo101, ARM_JOINT_NAMES
from driver_sdk import STS3215Driver


class RealBackend:
    def __init__(self, port="/dev/ttyACM0"):
        lim = FKSo101().limits_deg()               # 관절한계(도) — 명령 전 클램프
        self._lo = [lim[n][0] for n in ARM_JOINT_NAMES]
        self._hi = [lim[n][1] for n in ARM_JOINT_NAMES]
        self.drv = STS3215Driver(port=port)
        self.drv.connect()

    def _clamp_arm(self, angles_deg):
        return [float(np.clip(a, lo, hi))
                for a, lo, hi in zip(angles_deg, self._lo, self._hi)]

    def move(self, angles_deg, grip=None, secs=None):
        arm5 = self._clamp_arm(angles_deg)                     # 안전 클램프
        pos = {i + 1: STS3215Driver.degrees_to_position(arm5[i]) for i in range(5)}
        self.drv.set_all_positions(pos)
        # 그리퍼(id6)는 비선형 링키지라 percent 매핑을 따로 쓴다

    def ee(self):
        """현재 관절각을 읽어 FK로 손끝 위치를 추정한다(실물은 site 를 못 읽음)."""
        pos = self.drv.get_all_positions()
        arm5 = [STS3215Driver.position_to_degrees(pos.get(i + 1)) or 0.0
                for i in range(5)]
        p, _ = FKSo101().fk_deg(arm5)
        return p

    def wait(self):
        pass
