# -*- coding: utf-8 -*-
"""grasp_scene.py — 파지 실습용 씬 빌더.

기본 SCENE(SO-ARM101)에 빨간 공 하나와 '실제로 잡히는 물리'를 얹어
(model, data)를 돌려준다.

파지 물리가 필요한 이유: 기본 마찰로는 공이 미끄러워 잡히지 않는다.
  - cone=ELLIPTIC, impratio=10 : 접촉이 옆으로 덜 미끄러지도록
  - 공 condim=6, friction 큼    : 회전·비틀림 마찰까지 살려 쥐면 물리도록
"""
import os
import mujoco
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(_HERE, "models", "SO101", "scene.xml")

BALL_XY = (0.24, 0.05)     # 공을 놓을 자리 (x, y)
BALL_R = 0.02              # 공 반지름(m) → 지름 4cm


_CONES = {"elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC,
          "pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL}


def build(ball_xy=BALL_XY, ball_r=BALL_R, basket_xy=None,
          cone="elliptic", impratio=10, condim=6, friction=(1.5, 0.1, 0.01)):
    """SCENE + 빨간 공 + 파지 물리 → (model, data).

    공은 freejoint 로 붙는다. obj body id 는 model.body('obj').id,
    공 위치는 data.xpos[그 id] 로 읽는다.
    basket_xy=(x,y) 를 주면 그 자리에 박스 5개로 된 바구니를 얹는다(놓기용).

    파지 물리를 인자로 바꿔 실험할 수 있다(기본값이 '잘 잡히는' 세팅):
      cone     : "elliptic"(기본) / "pyramidal"(→ 미끄러져 놓침 비교용)
      impratio : 미끄럼 저항 비율(기본 10, 1로 낮추면 미끄러짐)
      condim   : 공 접촉 마찰 차원(6=미끄럼+비틀림+구름, 3=미끄럼만)
      friction : [미끄럼, 비틀림, 구름] 계수
    """
    spec = mujoco.MjSpec.from_file(SCENE)
    spec.option.cone = _CONES[str(cone).lower()]        # 마찰뿔 모양
    spec.option.impratio = impratio                     # 접촉 강성 비율
    b = spec.worldbody.add_body(name="obj",
                                pos=[ball_xy[0], ball_xy[1], ball_r])
    b.add_freejoint()
    g = b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[ball_r, 0, 0],
                   rgba=[1, 0, 0, 1], mass=0.01)
    g.condim = condim                  # 켤 마찰 종류 수
    g.friction = list(friction)        # [미끄럼, 비틀림, 구름] 세기
    if basket_xy is not None:
        _add_basket(spec, basket_xy)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _add_basket(spec, xy, half=0.06, wall=0.03, t=0.005):
    """박스 5개(바닥 1 + 벽 4)로 뚜껑 없는 바구니를 만든다. 고정 물체."""
    bx, by = xy
    body = spec.worldbody.add_body(name="basket", pos=[bx, by, 0.0])
    col = [0.55, 0.35, 0.15, 1]
    box = mujoco.mjtGeom.mjGEOM_BOX
    body.add_geom(type=box, size=[half, half, t], pos=[0, 0, t], rgba=col)          # 바닥
    body.add_geom(type=box, size=[t, half, wall], pos=[half, 0, wall], rgba=col)    # +x 벽
    body.add_geom(type=box, size=[t, half, wall], pos=[-half, 0, wall], rgba=col)   # -x 벽
    body.add_geom(type=box, size=[half, t, wall], pos=[0, half, wall], rgba=col)    # +y 벽
    body.add_geom(type=box, size=[half, t, wall], pos=[0, -half, wall], rgba=col)   # -y 벽
