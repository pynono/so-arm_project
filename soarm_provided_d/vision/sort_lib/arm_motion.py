# -*- coding: utf-8 -*-
"""Arm motion: arrive wait, adaptive IK hover/seed/tilt, go_xyz / grip / safe."""
from __future__ import annotations

import time

import numpy as np
from soarm_lab import arm
from soarm_lab.driver_sdk import STS3215Driver

from .constants import (
    DOWN_TILT_FAR_DEG,
    GRIP_OPEN,
    JOINT_ARRIVE_EXTRA,
    JOINT_ARRIVE_POLL,
    JOINT_ARRIVE_TOL,
    R_FAR,
    R_NEAR,
    SAFE_POSE,
    SEED,
    Z_HOVER_FAR,
    Z_HOVER_NEAR,
)


def wait_joints_arrived(
    drv,
    target: dict[int, int],
    *,
    tol: int = JOINT_ARRIVE_TOL,
    timeout: float = JOINT_ARRIVE_EXTRA,
    label: str = "",
) -> bool:
    """Poll present positions until within tol of target (or timeout)."""
    sids = [s for s in sorted(target) if target.get(s) is not None]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        cur = drv.get_all_positions()
        if all(
            cur.get(sid) is not None and abs(cur[sid] - target[sid]) <= tol
            for sid in sids
        ):
            tag = f" ({label})" if label else ""
            print(f"  [ARRIVED] 관절 도착 확인{tag}")
            return True
        time.sleep(JOINT_ARRIVE_POLL)
    tag = f" ({label})" if label else ""
    print(f"  [ARRIVED] timeout{tag} — proceeding")
    return False


def xy_reach(x: float, y: float) -> float:
    return float(np.hypot(x, y))


def reach_t(r: float) -> float:
    """0 at R_NEAR (or closer), 1 at R_FAR (or farther)."""
    return float(np.clip((r - R_NEAR) / max(R_FAR - R_NEAR, 1e-6), 0.0, 1.0))


def hover_z_for(r: float) -> float:
    """Far XY → higher hover so the arm is less stretched before descend."""
    t = reach_t(r)
    return float(Z_HOVER_NEAR + t * (Z_HOVER_FAR - Z_HOVER_NEAR))


def pick_seed_for(x: float, y: float, r: float | None = None) -> list[float]:
    """Distance-biased IK seed: near=folded j2/j3/j4, far=more extended wrist.

    Encourages different solution families before DLS; j1 from atan2(y,x).
    """
    if r is None:
        r = xy_reach(x, y)
    t = reach_t(r)
    j1 = float(np.degrees(np.arctan2(y, x)))
    # Empirically from hover IK / taught postures (near folded → far softer wrist)
    j2 = -55.0 + t * 60.0   # ~-55 → +5
    j3 = 30.0 - t * 35.0    # ~30 → -5
    j4 = 85.0 - t * 35.0    # ~85 → 50
    return [j1, j2, j3, j4, 0.0]


def down_tilt_for(r: float) -> float:
    """Soften pure vertical down at long reach (radial outward tilt, deg)."""
    return float(DOWN_TILT_FAR_DEG * reach_t(r))


def go_xyz(
    drv,
    xyz,
    down=False,
    seed=SEED,
    secs=0.8,
    j5=None,
    j6=None,
    label: str = "",
    down_tilt_deg: float = 0.0,
):
    ang, err = arm.ik.solve(
        list(xyz),
        seed_deg=list(seed),
        down=down,
        down_tilt_deg=float(down_tilt_deg),
    )
    # Soft-down still constrains orientation — keep the looser down tolerance
    tol = 0.12 if down else arm.REACH_TOL
    if err > tol:
        raise ValueError(f"도달 불가 {xyz} 잔차 {err*1000:.0f}mm")
    pos = {i + 1: STS3215Driver.degrees_to_position(float(ang[i])) for i in range(5)}
    if j5 is not None:
        pos[5] = int(j5)
    cur = drv.get_all_positions()
    pos[6] = int(j6) if j6 is not None else (cur.get(6) or GRIP_OPEN)
    drv.set_all_positions(pos)
    time.sleep(secs)
    wait_joints_arrived(
        drv, pos, timeout=JOINT_ARRIVE_EXTRA, label=label or f"xyz={list(xyz)}"
    )
    print(
        f"  [IK] {label or 'go'} xyz=({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:.3f}) "
        f"down={down} tilt={float(down_tilt_deg):.0f}° "
        f"j2={ang[1]:.1f} j3={ang[2]:.1f} j4={ang[3]:.1f} "
        f"err={err*1000:.1f}mm"
    )
    return ang, err


def set_grip(drv, raw: int, secs=0.3, wait_arrive: bool = False, label: str = ""):
    """Close/open gripper by writing joint 6 only — other joints stay put."""
    goal = int(raw)
    drv.set_position(6, goal)
    time.sleep(secs)
    if wait_arrive:
        wait_joints_arrived(
            drv, {6: goal}, timeout=JOINT_ARRIVE_EXTRA,
            label=label or f"j6={goal}",
        )


def go_safe(drv):
    print("[SAFE]")
    drv.set_all_positions(dict(SAFE_POSE))
    time.sleep(0.7)
