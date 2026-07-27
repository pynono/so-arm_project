# -*- coding: utf-8 -*-
"""grasp.py — 빨간 공 파지용 접근점 계산.

고정집게가 공 '옆'(베이스 쪽)에 내려오게 접근점을 잡는다.
IK는 ik_core.IKSo101.solve(..., down=True) 하나로 쓴다(아래보기 파지).
씬은 grasp_scene.build 로 만든다.
"""
import numpy as np


def approach_xy(obj_xy, r=0.02, margin=0.014):
    """공을 잡으러 갈 접근점 xy.

    고정집게가 공 '옆'(베이스 쪽)에 내려오도록, 공에서 베이스 방향으로
    (반지름 + margin)만큼 비켜 잡는다. 두 단계로 쓴다:
      하강점 = approach_xy(obj, r, margin=0.014)  # 여유 있게 옆에 내려간 뒤
      파지점 = approach_xy(obj, r, margin=0.0)    # 옆으로 붙이고 닫기
    """
    obj_xy = np.asarray(obj_xy, float)
    th = np.arctan2(obj_xy[1], obj_xy[0])
    u = -np.array([np.cos(th), np.sin(th)])   # 공→베이스 방향
    return obj_xy + u * (r + margin)
