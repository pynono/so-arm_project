# -*- coding: utf-8 -*-
"""
arm.py — SO-ARM101 제어 진입점. 좌표만 주면 팔이 움직인다(시뮬/실물 공통).

    from soarm_lab import arm

    # 한 점으로 이동
    arm.go([0.25, 0.0, 0.15])              # 손끝을 좌표로 (시뮬)
    arm.go([0.25, 0.0, 0.15], real=True)   # 같은 좌표를 실물로

    # pick & place
    arm.grasp(basket_xy=(0.16, -0.12))     # 공 + 바구니 씬을 연다
    xy = arm.ball_xy()                     # 시뮬이 알려주는 공 좌표
    arm.run(states)                        # 순서표(STATES)를 부드럽게 실행

내부: 좌표 → ik_core(자코비안 DLS) → 관절각 → 백엔드(sim 또는 real).move
좌표만 바꾸면 같은 코드가 시뮬에서 실물로 간다(real=True).
"""
import numpy as np
from ik_core import IKSo101
from sim import SimBackend

_SEED = (0, 30, -45, 0, 0)     # IK 시작 추정치(도) — 자연스러운 팔꿈치 자세


class Arm:
    REACH_TOL = 0.02           # 위치 IK 잔차 2cm 넘으면 '팔 밖'으로 판단

    def __init__(self):
        self.ik = IKSo101()    # 제공물: DLS 수치 IK
        self._sim = None
        self._real = None

    # ── 이동 ──────────────────────────────────────────────────────────────
    def go(self, xyz, grip=None, real=False, seed=_SEED, down=False, secs=1.2):
        """손끝을 좌표[x,y,z](m)로 이동. 반환 (관절각5[도], 잔차[m]).
        grip : 그리퍼 각도(도, None이면 유지)
        down : True면 그리퍼가 수직 아래를 봄(파지용)
        real : True면 실물로,  secs : 이동에 줄 시간(초)"""
        angles_deg, err = self.ik.solve(xyz, seed_deg=list(seed), down=down)
        tol = 0.12 if down else self.REACH_TOL     # 아래보기는 방향까지 맞춰 잔차가 큼
        if err > tol:
            raise ValueError(
                f"도달 불가: {list(xyz)} 는 팔 범위 밖 같아요 (잔차 {err*1000:.0f}mm).")
        self._backend(real).move(angles_deg, grip, secs=secs)
        return angles_deg, err

    def run(self, states, secs=0.3, grip_secs=0.25, down=False):
        """순서표(STATES)를 부드럽게 실행. 이동은 지금 자세→목표를 각도 보간해 잇는다.
        각 상태 = (이름, 목표[x,y,z] 또는 None, grip[도]).  목표 None = 그리퍼만(닫기/열기).
        secs=이동 속도, grip_secs=여닫기 속도(작을수록 빠름).
        down=True면 그리퍼가 아래를 보게 IK를 푼다(느림). 기본은 위치만(빠름)."""
        b = self._backend(False)
        for _name, target, grip in states:
            if target is None:
                self.grip(grip, grip_secs)
            else:
                self._glide(b, target, grip, secs, down)

    def grip(self, deg, secs=0.25):
        """그리퍼만 deg(도)까지 여닫는다(팔 고정). secs=여닫는 데 줄 시간(작으면 빠름).
        너무 빠르면 공을 튕겨내니 0.2초 아래로는 권하지 않는다."""
        import mujoco, time
        b = self._backend(False)
        cur = float(np.degrees(b.data.ctrl[5]))
        n = max(1, int(secs / b.model.opt.timestep))
        for i in range(n + 1):
            b.data.ctrl[5] = np.radians(cur + (deg - cur) * i / n)
            mujoco.mj_step(b.model, b.data)
            if getattr(b, "viewer", None):
                b.viewer.sync(); time.sleep(b.model.opt.timestep)

    # ── 파지 씬 ───────────────────────────────────────────────────────────
    def grasp(self, ball_xy=None, basket_xy=(0.16, -0.12), key_callback=None):
        """빨간 공(+바구니)이 있는 파지 씬으로 창을 연다. 이후 run(STATES)로 집기.
        ball_xy=(x,y)로 공 위치를 바꿀 수 있다(기본은 grasp_scene.BALL_XY).
        key_callback: 뷰어 키 입력 콜백(리플레이 등)."""
        from grasp_scene import build, BALL_XY
        model, data = build(ball_xy=ball_xy or BALL_XY, basket_xy=basket_xy)
        self._sim = SimBackend(model, data, key_callback=key_callback)
        return self

    def ball_xy(self):
        """파지 씬 속 공(obj)의 현재 xy 좌표를 시뮬에서 읽어 준다.
        현실에선 눈으로 감 잡을 좌표를 시뮬은 참값으로 알려준다."""
        import mujoco
        b = self._backend(False)
        bid = b.model.body("obj").id
        mujoco.mj_forward(b.model, b.data)
        return b.data.xpos[bid][:2].copy()

    # ── 창 ────────────────────────────────────────────────────────────────
    def live(self):
        """창 하나를 계속 띄워두고 실시간 조종(arm.go 칠 때마다 같은 창에서 움직임)."""
        from sim import LiveSim
        self._sim = LiveSim()
        return self

    def wait(self, real=False):
        """시뮬 창을 열어둔 채 대기(마우스 조작). 창을 닫으면 종료."""
        self._backend(real).wait()

    # ── 내부 ──────────────────────────────────────────────────────────────
    def _glide(self, b, target, grip, secs, down=False):
        """지금 관절각 → target 관절각을 매 프레임 보간(끊김 없이). IK는 목표 1번만 푼다."""
        import mujoco, time
        d0 = np.degrees(b.data.ctrl[:5]).astype(float)               # 지금 자세
        d1 = np.array(self.ik.solve(target, seed_deg=d0, down=down)[0])   # 지금 자세로 warm-start
        n = max(1, int(secs / b.model.opt.timestep))
        for i in range(n + 1):
            t = i / n
            b.data.ctrl[:5] = np.radians(d0 * (1 - t) + d1 * t)      # 각도 보간
            b.data.ctrl[5] = np.radians(grip)
            mujoco.mj_step(b.model, b.data)
            if getattr(b, "viewer", None):
                b.viewer.sync(); time.sleep(b.model.opt.timestep)

    def _backend(self, real):
        if real:
            if self._real is None:
                from real import RealBackend
                self._real = RealBackend()
            return self._real
        if self._sim is None:
            self._sim = SimBackend()
        return self._sim


arm = Arm()     # from soarm_lab import arm  → 이 인스턴스를 쓴다
