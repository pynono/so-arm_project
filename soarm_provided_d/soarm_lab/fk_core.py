# -*- coding: utf-8 -*-
"""
fk_core.py — SO-ARM101 순기구학(POE) 핵심. numpy만 필요, 자급식.

URDF(so101.urdf, = 로봇 도면)에서 홈자세 M 과 관절 스크루축 S를 뽑아
관절각 → 손끝 위치를 계산한다. (Modern Robotics Ch.4 규약)

컨트롤러 각도 규약과의 관계 (검증됨):
  컨트롤러 3D 트윈은 서보위치를  positionToRad=(pos-2048)/4095·2π  로 URDF 관절에 적용.
  SDK가 주는 angle_deg 를 라디안으로 바꾸면 정확히 같은 값 →  q_urdf = radians(angle_deg).
  즉 bot.angles()(도)를 그대로 radians 해서 넣으면 컨트롤러 트윈과 동일한 손끝좌표가 나온다.
"""
import math
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

URDF_DEFAULT = str(Path(__file__).resolve().parent / "so101.urdf")

# 관절 id(1~5) = 팔 회전관절, base→tip 순서 (kinematics.json drive_order)
ARM_JOINT_IDS = [1, 2, 3, 4, 5]
ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


# ---------- SE(3)/so(3) 도구 ----------
def _rpy_to_R(r, p, y):
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(xyz, rpy):
    T = np.eye(4); T[:3, :3] = _rpy_to_R(*rpy); T[:3, 3] = xyz
    return T


def _skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def _exp3(w, th): # 회전만
    W = _skew(w)
    return np.eye(3) + math.sin(th) * W + (1 - math.cos(th)) * (W @ W)


def _exp6(S, th): # 회전하고 이동
    w, v = S[:3], S[3:]
    T = np.eye(4)
    if np.linalg.norm(w) < 1e-12:
        T[:3, 3] = v * th; return T
    W = _skew(w)
    T[:3, :3] = _exp3(w, th)
    G = np.eye(3) * th + (1 - math.cos(th)) * W + (th - math.sin(th)) * (W @ W)
    T[:3, 3] = G @ v
    return T


class FKSo101:
    """URDF를 한 번 읽어 POE FK를 제공."""

    def __init__(self, urdf=URDF_DEFAULT, base="base_link", tip="gripper_frame_link"):
        root = ET.parse(urdf).getroot()
        joints = {}
        for j in root.findall("joint"):
            ch = j.find("child").get("link")
            o = j.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0")).split()] if o is not None else [0, 0, 0]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0")).split()] if o is not None else [0, 0, 0]
            ax = j.find("axis")
            axis = np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else np.zeros(3)
            lim = j.find("limit")
            lohi = (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else (None, None)
            joints[ch] = {"name": j.get("name"), "type": j.get("type"),
                          "parent": j.find("parent").get("link"),
                          "T": _T(xyz, rpy), "axis": axis, "limit": lohi}
        # 체인 구성 base→tip
        chain, link = [], tip
        while link != base:
            chain.append(joints[link]); link = joints[link]["parent"]
        chain.reverse()
        self.chain = chain
        self._extract()

    def _extract(self):
        T = np.eye(4); Slist = []; names = []; limits = []
        for j in self.chain:
            T = T @ j["T"]
            if j["type"] == "revolute":
                w = T[:3, :3] @ (j["axis"] / np.linalg.norm(j["axis"]))
                q = T[:3, 3]
                Slist.append(np.concatenate([w, -np.cross(w, q)]))
                names.append(j["name"]); limits.append(j["limit"])
        self.M = T.copy()
        self.Slist = np.array(Slist)
        self.names = names
        self.limits_rad = dict(zip(names, limits))

    def fk(self, q_rad):
        """관절각(rad) 리스트 → 4x4 변환 (base→손끝)."""
        T = np.eye(4)
        for i, th in enumerate(q_rad):
            T = T @ _exp6(self.Slist[i], th)
        return T @ self.M

    def fk_deg(self, deg5):
        """컨트롤러 각도(도) 5개 → (손끝 위치[m] np(3,), 방향행렬 R 3x3)."""
        T = self.fk([math.radians(d) for d in deg5])
        return T[:3, 3], T[:3, :3]

    def limits_deg(self):
        return {n: (math.degrees(lo), math.degrees(hi))
                for n, (lo, hi) in self.limits_rad.items()}


if __name__ == "__main__":
    fk = FKSo101()
    print("관절:", fk.names)
    print("홈 M 손끝[m]:", np.round(fk.M[:3, 3], 4))
    print("한계(도):")
    for n, (lo, hi) in fk.limits_deg().items():
        print(f"  {n:14s} [{lo:7.1f}, {hi:7.1f}]")
    p, _ = fk.fk_deg([0, 0, 0, 0, 0])
    print("FK(0,0,0,0,0) 손끝[mm]:", np.round(p * 1000, 1))
    p, _ = fk.fk_deg([30, 45, 0, 0, 0])
    print("FK(30,45,0,0,0) 손끝[mm]:", np.round(p * 1000, 1))
