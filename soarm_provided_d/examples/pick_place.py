# -*- coding: utf-8 -*-
"""pick & place 참고 실행 — 좌표함수로 공을 집어 바구니에 담는다.

핵심 아이디어:
  · 순서표(STATES)를 '공 좌표의 함수'(make_states)로 만든다 → 좌표만 바꾸면 어디든.
  · 좌표는 시뮬이 알려준다 → arm.ball_xy(). (현실에선 나중에 카메라가 준다)

실행:
    cd ~/soarm_provided
    .venv/bin/python examples/pick_place.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from soarm_lab import arm
from soarm_lab.grasp import approach_xy
from soarm_lab.grasp_scene import BALL_R

BASKET = (0.16, -0.12)


def make_states(ball_xy, basket_xy, r=BALL_R):
    """공·바구니 좌표 → pick&place 순서표. (학생이 설계하는 부분)"""
    hx, hy = approach_xy(ball_xy,   r=r, margin=0.014)   # 하강점(공 옆, 여유)
    gx, gy = approach_xy(ball_xy,   r=r, margin=0.0)     # 파지점(공에 붙임)
    px, py = approach_xy(basket_xy, r=r, margin=0.0)     # 놓을 곳
    return [
        ("hover",   [hx, hy, 0.12],  100),   # 공 옆 위
        ("descend", [hx, hy, 0.005], 100),   # 옆으로 하강(공 안 누름)
        ("slide",   [gx, gy, 0.005], 100),   # 파지점으로 슬라이드
        ("close",   None,             15),   # 닫기(공 크기에 맞춤)
        ("lift",    [gx, gy, 0.14],   15),   # 들기
        ("carry",   [px, py, 0.14],   15),   # 바구니 위로
        ("lower",   [px, py, 0.07],   15),   # 내리기
        ("open",    None,            100),   # 놓기
    ]


if __name__ == "__main__":
    arm.grasp(basket_xy=BASKET)               # 공 + 바구니 씬 (창이 뜬다)
    xy = arm.ball_xy()                        # 시뮬이 알려주는 공 좌표
    print("공 좌표:", xy.round(3))
    arm.run(make_states(xy, BASKET), down=True)   # 파지는 아래보기(down) 필요
    arm.wait()                                # 창을 열어둔 채 대기 (닫으면 종료)
