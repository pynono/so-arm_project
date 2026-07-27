# -*- coding: utf-8 -*-
"""
sim.py — MuJoCo 시뮬 백엔드.
관절각(도)을 받아 position 액추에이터(ctrl=목표각·라디안)로 팔을 '물리적으로' 이동시키고,
인터랙티브 뷰어에 실시간 표시한다. 검증된 규약: ctrl[:5] = radians(deg5).
"""
import os, time
import numpy as np
import mujoco

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(_HERE, "models", "SO101", "scene.xml")


class SimBackend:
    """시뮬 팔. move(각도)로 이동, wait()로 창 열어둠. 손끝 site='gripperframe'.
    model·data 를 주면 그 씬을 쓴다(파지 씬 등). 안 주면 기본 SCENE."""
    def __init__(self, model=None, data=None, view=True):
        if model is None:
            self.model = mujoco.MjModel.from_xml_path(SCENE)
            self.data  = mujoco.MjData(self.model)
        else:
            self.model, self.data = model, data
        self.eid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.viewer = None
        if view and not os.environ.get("SOARM_HEADLESS"):   # 헤드리스 테스트면 창 없이
            import mujoco.viewer as mjv
            self.viewer = mjv.launch_passive(self.model, self.data)
            self.viewer.sync()

    def move(self, angles_deg, grip=None, secs=1.2):
        """팔 5축(도)으로 이동. grip=도(None이면 유지). secs=이동에 줄 시간."""
        self.data.ctrl[:5] = np.radians(angles_deg)
        if grip is not None:
            self.data.ctrl[5] = np.radians(grip)
        steps = int(secs / self.model.opt.timestep)
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            if self.viewer:
                self.viewer.sync()
                time.sleep(self.model.opt.timestep)

    def ee(self):
        """현재 손끝 월드좌표[m]."""
        mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.eid].copy()

    def wait(self):
        """창을 열어둔 채 대기(마우스 조작 유지). 창을 닫으면 종료."""
        if not self.viewer:
            return
        while self.viewer.is_running():
            self.viewer.sync()
            time.sleep(0.02)


class LiveSim:
    """라이브 뷰어 — 창 하나를 백그라운드로 계속 띄워두고 move()로 실시간 조종.
    물리+렌더 루프가 별도 스레드에서 돌아, 창은 늘 마우스 조작이 가능하다."""
    def __init__(self):
        import threading
        self.model = mujoco.MjModel.from_xml_path(SCENE)
        self.data  = mujoco.MjData(self.model)
        self.eid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self._lock = threading.Lock()
        self._alive = True
        self._headless = bool(os.environ.get("SOARM_HEADLESS"))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        if self._headless:                      # 창 없이 물리만(테스트용)
            while self._alive:
                with self._lock:
                    mujoco.mj_step(self.model, self.data)
                time.sleep(self.model.opt.timestep)
            return
        import mujoco.viewer as mjv
        with mjv.launch_passive(self.model, self.data) as v:
            while v.is_running() and self._alive:
                with self._lock:
                    mujoco.mj_step(self.model, self.data)
                v.sync()
                time.sleep(self.model.opt.timestep)
        self._alive = False

    def move(self, angles_deg, grip=None, secs=None):
        """목표각을 걸어두면(ctrl) 백그라운드 루프가 그리로 몰고 간다. 즉시 반환."""
        with self._lock:
            self.data.ctrl[:5] = np.radians(angles_deg)
            if grip is not None:
                self.data.ctrl[5] = np.radians(grip)

    def ee(self):
        with self._lock:
            mujoco.mj_forward(self.model, self.data)
            return self.data.site_xpos[self.eid].copy()

    def wait(self):
        while self._alive and self._thread.is_alive():
            time.sleep(0.1)
