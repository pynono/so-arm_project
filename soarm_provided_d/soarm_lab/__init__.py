# -*- coding: utf-8 -*-
"""
soarm_lab — SO-ARM101 실습 제공물(패키지).
사용 예:

    from soarm_lab import arm
    arm.go([0.25, 0.0, 0.15])     # 손끝을 그 좌표로 (시뮬)
    arm.wait()

내부: fk_core(POE FK) · ik_core(DLS IK) · sim(MuJoCo) · real(실물 드라이버) · arm(제어 진입점).
"""
import os, sys
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:            # 내부 모듈끼리 flat import 되게
    sys.path.insert(0, _HERE)

SCENE = os.path.join(_HERE, "models", "SO101", "scene.xml")   # 공식 SO-101 MJCF

from arm import arm                   # noqa: E402  (학생이 쓰는 단 하나)

__all__ = ["arm", "SCENE"]
