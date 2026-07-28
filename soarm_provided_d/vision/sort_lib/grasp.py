# -*- coding: utf-8 -*-
"""Grasp verify, GRIP stage, and --measure-load helpers."""
from __future__ import annotations

import time

import numpy as np

from .arm_motion import go_xyz, pick_seed_for, set_grip, xy_reach, down_tilt_for
from .constants import (
    GRIP_CLOSE,
    GRIP_CLOSE_SETTLE_SECS,
    GRIP_DOWN_SECS,
    GRIP_LIFT_SECS,
    GRIP_LOAD_MAJORITY,
    GRIP_LOAD_SAMPLE_DT,
    GRIP_LOAD_SAMPLES,
    GRIP_LOAD_THRESH,
    GRIP_POST_CLOSE_STABILIZE,
    GRIP_TEST_LIFT_SECS,
    GripFailure,
    J6_OPEN,
    JOINT_ARRIVE_TOL,
    MEASURE_LOAD_DT,
    MEASURE_LOAD_N,
    PICK_J5,
    Z_CARRY,
    Z_PICK,
    Z_TEST_LIFT,
)
from .detect_plan import pump


def load_stats(samples: list) -> dict:
    vals = [v for v in samples if v is not None]
    if not vals:
        return {"n": 0, "mean": None, "std": None, "min": None, "max": None}
    arr = np.array(vals, dtype=float)
    return {
        "n": len(vals),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "max": float(arr.max()),
    }


def sample_grip_loads(drv, n: int = MEASURE_LOAD_N, dt: float = MEASURE_LOAD_DT):
    """Collect n get_load(6) samples; print each; return list."""
    samples = []
    for i in range(n):
        time.sleep(dt)
        load = drv.get_load(6)
        print(f"    sample[{i + 1}/{n}] get_load(6)={load}")
        samples.append(load)
    return samples


def measure_grip_load_profiles(drv):
    """Interactive: sample empty / good grasp / wrong catch load distributions.

    Helps choose GRIP_LOAD_THRESH. Prints mean/std/min/max per scenario.
    """
    print("\n======== MEASURE GRIP LOAD PROFILES ========")
    print(f"Default GRIP_LOAD_THRESH={GRIP_LOAD_THRESH} until you re-tune from these stats.")
    print("For each scenario: position the gripper as prompted, then Enter.\n")

    scenarios = [
        ("empty", "Empty gripper — close on NOTHING (air), then Enter"),
        ("good_grasp", "Successful grasp — close firmly on a ball, then Enter"),
        ("wrong_catch", "Wrong catch / snag — grip something unintended, then Enter"),
    ]
    results = {}
    for key, prompt in scenarios:
        input(f"  [{key}] {prompt} ")
        print(f"  [{key}] sampling {MEASURE_LOAD_N}× get_load(6)…")
        samples = sample_grip_loads(drv)
        st = load_stats(samples)
        results[key] = st
        print(
            f"  [{key}] n={st['n']} mean={st['mean']:.1f} std={st['std']:.1f} "
            f"min={st['min']:.0f} max={st['max']:.0f}"
            if st["n"]
            else f"  [{key}] no valid samples"
        )
        print()

    print("======== SUMMARY (set GRIP_LOAD_THRESH between empty and good) ========")
    for key, st in results.items():
        if st["n"]:
            print(
                f"  {key:12s}  mean={st['mean']:7.1f}  std={st['std']:6.1f}  "
                f"min={st['min']:6.0f}  max={st['max']:6.0f}  n={st['n']}"
            )
        else:
            print(f"  {key:12s}  (no data)")
    print(
        f"\nGuidance: empty max should be << thresh << good mean; "
        f"current default thresh={GRIP_LOAD_THRESH}."
    )
    return results


def verify_grasp_load(
    drv,
    thresh: int = GRIP_LOAD_THRESH,
):
    """Multi-sample grasp check (call AFTER test lift, not at bottom).

    Sample get_load(6) 5× over ~1s (0.2s apart). Grasped when majority
    (>= GRIP_LOAD_MAJORITY) of samples have load >= thresh.
    Returns True on success, False on miss (caller handles fail path).
    """
    print(
        f"  [GRIP] verify after test lift "
        f"(need >= {GRIP_LOAD_MAJORITY}/{GRIP_LOAD_SAMPLES} "
        f"get_load(6)>={thresh})"
    )
    samples = []
    last_load = None
    for _ in range(GRIP_LOAD_SAMPLES):
        time.sleep(GRIP_LOAD_SAMPLE_DT)
        load = drv.get_load(6)
        print(load)  # one line per sample (5×)
        samples.append(load)
        last_load = load

    ok = sum(1 for v in samples if v is not None and v >= thresh)
    grasped = ok >= GRIP_LOAD_MAJORITY
    verdict = "GRASP OK" if grasped else "MISS"
    print(
        f"  [GRIP] samples={samples} ok={ok}/{GRIP_LOAD_SAMPLES} "
        f"thresh={thresh} last={last_load} → {verdict}"
    )
    return grasped


def stage_grip(
    drv,
    xy,
    win,
    grabber,
    color: str,
    *,
    reach_r: float | None = None,
    hover_ang=None,
):
    """After MOVE_TO_BALL: soft-down descend → close(j6) → test lift → verify → carry.

    Flow:
      거리적응 soft-down 하강(Z_PICK, gripper OPEN) → 관절 도착 확인
      → 그리퍼 닫기(j6 only) → 닫힘 대기 → 짧은 안정화
      → 시험 상승(Z_TEST_LIFT ~2.5cm)
      → get_load(6) majority 검증
         성공: Z_CARRY 리프트
         실패: 내려놓고 열기 → GripFailure (caller: SAFE→재검출→재시도)
    """
    stage = f"{color.upper()} GRIP"
    print(f"[{stage}] {xy}")
    x, y = float(xy[0]), float(xy[1])
    r = float(reach_r) if reach_r is not None else xy_reach(x, y)
    tilt = down_tilt_for(r)
    seed = list(hover_ang) if hover_ang is not None else pick_seed_for(x, y, r)
    rgb, _ = grabber.get()
    if rgb is not None:
        pump(rgb, win, stage, [
            "descend→close(j6)→settle→test_lift→verify",
            f"xy=({x:+.3f},{y:+.3f}) r={r:.3f} tilt={tilt:.0f}",
            f"Z_TEST_LIFT={Z_TEST_LIFT:.3f} thresh={GRIP_LOAD_THRESH}",
        ])

    # 1) Soft-down descend to pick height — gripper stays OPEN (J6_OPEN)
    #    Near: tilt≈0 (pure vertical). Far: mild outward tilt so j4 varies with reach.
    print(
        f"  [{stage}] soft-down 하강 → xy=({x:+.3f},{y:+.3f}) z={Z_PICK:.3f} "
        f"r={r:.3f} tilt={tilt:.0f}° j6=OPEN({J6_OPEN})"
    )
    go_xyz(
        drv, [x, y, Z_PICK], down=True, seed=seed, down_tilt_deg=tilt,
        secs=GRIP_DOWN_SECS, j5=PICK_J5, j6=J6_OPEN, label="Z_PICK open",
    )
    # Confirm XY center + Z_PICK with gripper still open before closing
    cur = drv.get_all_positions()
    j6_now = cur.get(6)
    open_ok = j6_now is not None and abs(j6_now - J6_OPEN) <= JOINT_ARRIVE_TOL
    print(
        f"  [{stage}] pre-close confirm: target_xy=({x:+.3f},{y:+.3f}) "
        f"Z_PICK={Z_PICK:.3f} j6={j6_now} J6_OPEN={J6_OPEN} "
        f"open={'OK' if open_ok else 'WARN'}"
    )
    if not open_ok:
        print(f"  [{stage}] j6 not open — forcing J6_OPEN before close")
        set_grip(drv, J6_OPEN, 0.4, wait_arrive=True, label="ensure open")

    # 2) Close gripper (j6 only); wait close complete; short stabilize
    print(f"  [{stage}] 그리퍼 닫기 (j6 only) close={GRIP_CLOSE}")
    set_grip(drv, GRIP_CLOSE, GRIP_CLOSE_SETTLE_SECS, wait_arrive=True, label="닫힘")
    print(f"  [{stage}] 닫힘 완료 — stabilize {GRIP_POST_CLOSE_STABILIZE:.2f}s")
    time.sleep(GRIP_POST_CLOSE_STABILIZE)

    # 3) Test lift ~2.5cm only (NOT full Z_CARRY yet)
    print(f"  [{stage}] 시험 상승 → z={Z_TEST_LIFT:.3f} (+{Z_TEST_LIFT - Z_PICK:.3f}m)")
    go_xyz(
        drv, [x, y, Z_TEST_LIFT], down=False, secs=GRIP_TEST_LIFT_SECS,
        j5=PICK_J5, j6=GRIP_CLOSE, label="Z_TEST_LIFT",
    )

    # 4) Grasp verification AFTER test lift
    print(f"  [{stage}] 파지 성공 검증")
    grasped = verify_grasp_load(drv)
    if not grasped:
        print(f"  [{stage}] FAIL — 내려놓고 열기 → SAFE/재검출")
        try:
            go_xyz(
                drv, [x, y, Z_PICK], down=True, seed=seed, down_tilt_deg=tilt,
                secs=GRIP_TEST_LIFT_SECS, j5=PICK_J5, j6=GRIP_CLOSE, label="fail-down",
            )
            set_grip(drv, J6_OPEN, GRIP_CLOSE_SETTLE_SECS, wait_arrive=True, label="열기")
            print(f"  [{stage}] opened J6_OPEN={J6_OPEN}")
        except Exception as e:
            print(f"  [{stage}] fail-path cleanup error: {e}")
        raise GripFailure(
            color,
            f"grasp miss after test lift "
            f"(get_load(6) majority < {GRIP_LOAD_THRESH})",
        )

    # 5) Success: lift to carry height, then caller MOVE_TO_BOX
    print(f"  [{stage}] OK — 상자로 이동 준비 (lift → Z_CARRY={Z_CARRY:.3f})")
    go_xyz(
        drv, [x, y, Z_CARRY], down=False, secs=GRIP_LIFT_SECS,
        j6=GRIP_CLOSE, label="Z_CARRY",
    )
