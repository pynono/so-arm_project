# -*- coding: utf-8 -*-
"""
ik_core.py — SO-ARM101 위치 역기구학(IK). fk_core(POE FK) 재사용.

IK = FK의 반대: "손끝을 (x,y,z)로 보내려면 관절각이 몇?"
5-DOF라 위치(3D)만 목표로 삼는다(방향은 남는 자유도로 자유). 방법 = 수치 IK:
  현재 각도에서 조금씩 고쳐가며 목표에 수렴 (감쇠 최소자승 DLS + 자코비안).
자코비안은 fk_core.fk 를 유한차분으로 미분 → FK와 100% 일치하는 IK.
"""
import math
import numpy as np
from fk_core import FKSo101, ARM_JOINT_NAMES


def _fd_position_jacobian(fk, q, d=1e-5):
    """손끝 위치의 자코비안 3x5 (유한차분). ∂p/∂qᵢ."""
    J = np.zeros((3, len(q)))
    for i in range(len(q)):
        qp = q.copy(); qp[i] += d
        qm = q.copy(); qm[i] -= d
        J[:, i] = (fk.fk(qp)[:3, 3] - fk.fk(qm)[:3, 3]) / (2 * d)
    return J


# 그리퍼가 '뻗는' 방향 (fk 손끝 좌표계 기준, 실측 상수).
# down 모드에서 이 방향을 월드 [0,0,-1](수직 아래)로 맞춘다.
APPROACH_FK = np.array([0.0801, -0.0029, 0.9968])




class IKSo101:
    def __init__(self, fk=None):
        self.fk = fk or FKSo101()
        # 관절 한계(rad) 순서대로
        self.lo = np.array([self.fk.limits_rad[n][0] for n in self.fk.names])
        self.hi = np.array([self.fk.limits_rad[n][1] for n in self.fk.names])

    def solve(self, target_m, seed_deg=None, iters=300, tol=5e-5, damp=0.03,
              down=False):
        """목표 위치(m, 3벡터) → 관절각(도 5개), 잔차(m).
        seed_deg: 시작 추정치(도). None이면 여러 랜덤시드로 재시도.
        down=True: 그리퍼가 '수직 아래를 보는' 자세 조건 추가 (파지용).
                   오차·자코비안에 방향 3줄이 더 붙을 뿐, 나머지는 동일한 DLS."""
        target = np.asarray(target_m, float)
        seeds = []
        if seed_deg is not None:
            seeds.append(np.radians(seed_deg))
        # 결정적 시드 몇 개(랜덤 대신 고정) — 재현성
        seeds += [np.zeros(5),
                  np.radians([0, 30, -45, 0, 0]),
                  np.radians([0, -30, 45, 0, 0]),
                  np.radians([0, 60, -90, 30, 0]),
                  np.radians([45, 20, -20, 0, 0])]
        if down:
            return self._solve_down(target, seeds)
        best_q, best_err = None, np.inf
        for q0 in seeds:
            q = np.clip(q0.copy(), self.lo, self.hi)
            for _ in range(iters):
                p = self.fk.fk(q)[:3, 3]
                e = target - p
                err = np.linalg.norm(e)
                if err < tol:
                    break
                J = _fd_position_jacobian(self.fk, q)
                # 감쇠 최소자승(DLS): dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
                dq = J.T @ np.linalg.solve(J @ J.T + damp**2 * np.eye(3), e)
                # 스텝 제한(과도한 점프 방지)
                dq = np.clip(dq, -0.3, 0.3)
                q = np.clip(q + dq, self.lo, self.hi)
            p = self.fk.fk(q)[:3, 3]
            err = np.linalg.norm(target - p)
            if err < best_err:
                best_q, best_err = q.copy(), err
            if best_err < tol:
                break
        return np.degrees(best_q), best_err

    # ---------- down 모드 내부 (파지용) ----------
    def _prep_down(self):
        """down 모드 준비(첫 사용 때 한 번). 방향 자코비안은 MuJoCo(mj_jacSite)로 계산
        — 원리는 위와 같은 DLS에 '방향 3줄'을 쌓는 것이고, 계산만 MuJoCo가 해준다."""
        import os
        import mujoco
        scene = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "models", "SO101", "scene.xml")
        m = mujoco.MjModel.from_xml_path(scene)
        d = mujoco.MjData(m)
        sid = m.site("gripperframe").id
        gb = m.body("gripper").id
        # 그리퍼가 '뻗는' 방향 (site 로컬, 자세와 무관한 고정 기하)
        d.qpos[:5] = np.radians([0, 30, -45, 0, 0])
        mujoco.mj_forward(m, d)
        R = d.site_xmat[sid].reshape(3, 3)
        v = d.site_xpos[sid] - d.xpos[gb]
        appr = R.T @ (v / np.linalg.norm(v))
        self._mj = (mujoco, m, d, sid, appr)

    def _solve_down(self, target, seeds, iters=200, kr=0.5, damp=0.04):
        """위치 + '그리퍼가 수직 아래를 본다' 자세 조건의 DLS.
        오차 6개(위치3+방향3), 자코비안 6x5 — 구조는 위치 IK와 동일."""
        if not hasattr(self, "_mj"):
            self._prep_down()
        mujoco, m, d, sid, appr = self._mj
        down_dir = np.array([0.0, 0.0, -1.0])
        best_q, best_err = None, np.inf
        for q0 in seeds:
            q = np.clip(np.asarray(q0, float).copy(), self.lo, self.hi)
            best_e, stall = np.inf, 0
            for _ in range(iters):
                d.qpos[:5] = q
                mujoco.mj_kinematics(m, d)
                mujoco.mj_comPos(m, d)
                sp = d.site_xpos[sid]
                R = d.site_xmat[sid].reshape(3, 3)
                ep = target - sp                       # 위치 오차 3
                er = np.cross(R @ appr, down_dir)      # 방향 오차 3(회전축 형태)
                en = np.linalg.norm(ep) + 0.1 * np.linalg.norm(er)
                stall = 0 if en < best_e - 1e-5 else stall + 1
                best_e = min(best_e, en)
                if (np.linalg.norm(ep) < 1e-3 and np.linalg.norm(er) < 1e-2) or stall >= 8:
                    break                              # 수렴 or 개선 정체(도달한계) → 그만
                jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
                mujoco.mj_jacSite(m, d, jp, jr, sid)
                J = np.vstack([jp[:, :5], kr * jr[:, :5]])   # 6x5
                e = np.concatenate([ep, kr * er])
                dq = J.T @ np.linalg.solve(J @ J.T + damp**2 * np.eye(6), e)
                q = np.clip(q + np.clip(dq, -0.2, 0.2), self.lo, self.hi)
            d.qpos[:5] = q
            mujoco.mj_kinematics(m, d)
            perr = np.linalg.norm(target - d.site_xpos[sid])
            derr = np.linalg.norm(np.cross(d.site_xmat[sid].reshape(3, 3) @ appr, down_dir))
            err = perr + 0.1 * derr
            if err < best_err:
                best_q, best_err = q.copy(), err
            if perr < 3e-3 and derr < 0.2:     # 이 시드로 충분히 좋음 → 나머지 시드 생략
                break
        d.qpos[:5] = best_q
        mujoco.mj_kinematics(m, d)
        return np.degrees(best_q), np.linalg.norm(target - d.site_xpos[sid])


if __name__ == "__main__":
    ik = IKSo101()
    np.set_printoptions(precision=2, suppress=True)
    print("=== IK 자기검증: FK로 만든 목표를 IK가 되찾나 ===")
    tests_deg = [
        [0, 30, -45, 0, 0],
        [20, 45, -60, 15, 0],
        [-30, 10, -20, 30, 0],
        [40, 60, -80, -20, 0],
    ]
    for td in tests_deg:
        target, _ = ik.fk.fk_deg(td)            # 이 각도의 손끝 위치를 목표로
        sol_deg, err = ik.solve(target)         # 그 위치로 IK
        p2, _ = ik.fk.fk_deg(sol_deg)           # IK 해로 다시 FK
        print(f"목표(FK{td}) = {np.round(target*1000,1)} mm")
        print(f"  IK해(도) = {sol_deg}   잔차 = {err*1000:.2f} mm   재FK = {np.round(p2*1000,1)}")

    print("\n=== down 모드(아래보기, 파지용) ===")
    for t in ([0.24, 0.05, 0.06], [0.24, 0.05, 0.01], [0.16, -0.12, 0.02]):
        ang, err = ik.solve(t, seed_deg=[0, 30, -45, 0, 0], down=True)
        a = ik.fk.fk(np.radians(ang))[:3, :3] @ APPROACH_FK
        tilt = np.degrees(np.arccos(np.clip(-a[2], -1, 1)))
        print(f"{t}: 잔차 {err*1000:.1f} mm, 수직아래서 {tilt:.1f}° 기울음, 각도 {ang}")
