# -*- coding: utf-8 -*-
"""ACT stages: MOVE_TO_BALL / pick-until-grasp / MOVE_TO_BOX / PLACE / recover."""
from __future__ import annotations

from .arm_motion import (
    go_safe,
    go_xyz,
    hover_z_for,
    pick_seed_for,
    set_grip,
    xy_reach,
)
from .constants import (
    BOX_APPROACH_SECS,
    COLOR_ORDER,
    DROP_J5,
    DROP_J6,
    GRIP_CLOSE,
    GRIP_MAX_RETRIES,
    GripFailure,
    PICK_APPROACH_SECS,
    PICK_J5,
    PICK_J6,
    RedetectFailure,
    Z_CARRY,
    Z_HOVER,
    Z_PLACE,
)
from .detect_plan import pump, stage_redetect_ball, stage_scan_all
from .grasp import stage_grip


def stage_move_to_ball(drv, xy, win, grabber, color: str):
    """Phase 1: distance-adaptive hover (down=False); arrive before descend.

    Far reaches use higher Z_HOVER + extended j2/j3/j4 seed so the arm does not
    lock into a stiff pure-down family before the short final descend.
    Returns (x, y, r, hover_z, hover_ang) for warm-start descend.
    """
    stage = f"{color.upper()} MOVE_TO_BALL"
    print(f"[{stage}] 공 위로 이동 {xy}")
    x, y = float(xy[0]), float(xy[1])
    r = xy_reach(x, y)
    z_h = hover_z_for(r)
    seed = pick_seed_for(x, y, r)
    rgb, _ = grabber.get()
    if rgb is not None:
        pump(rgb, win, stage, [
            f"hover approach {PICK_APPROACH_SECS:.0f}s",
            f"xy=({x:+.3f},{y:+.3f}) z={z_h:.3f} r={r:.3f}",
        ])
    print(
        f"  [{stage}] adaptive hover r={r:.3f}m z={z_h:.3f} "
        f"seed j2/j3/j4=({seed[1]:.0f},{seed[2]:.0f},{seed[3]:.0f})"
    )
    # Approach above ball: position IK (no hard down), taught j5/j6
    hover_ang, _ = go_xyz(
        drv, [x, y, z_h], down=False, seed=seed, secs=PICK_APPROACH_SECS,
        j5=PICK_J5, j6=PICK_J6, label="hover",
    )
    print(f"  [{stage}] ready for descend")
    return x, y, r, z_h, hover_ang


def stage_pick(drv, xy, win, grabber, color: str):
    """Pick = MOVE_TO_BALL then GRIP (descend/close/test_lift/verify/carry)."""
    x, y, r, _z_h, hover_ang = stage_move_to_ball(drv, xy, win, grabber, color)
    stage_grip(
        drv, (x, y), win, grabber, color, reach_r=r, hover_ang=hover_ang,
    )


def stage_pick_until_grasp(drv, plan_entry: dict, win, grabber, color: str, H):
    """MOVE_TO_BALL → GRIP(descend/close/test_lift/verify); on miss retry.

    Fail path (after stage_grip already lowered+opened):
      SAFE → redetect ball → full pick sequence again.
    Redetect timeout is recoverable (RedetectFailure ⊂ GripFailure).
    Raises GripFailure after GRIP_MAX_RETRIES cycles for outer re-SCAN.
    """
    ball_xy = plan_entry["ball_xy"]
    last_err: Exception | None = None
    for attempt in range(1, GRIP_MAX_RETRIES + 1):
        print(
            f"  [PICK] cycle {attempt}/{GRIP_MAX_RETRIES} "
            f"{color} xy=({ball_xy[0]:+.3f},{ball_xy[1]:+.3f})"
        )
        try:
            x, y, r, _z_h, hover_ang = stage_move_to_ball(
                drv, ball_xy, win, grabber, color
            )
            stage_grip(
                drv, (x, y), win, grabber, color,
                reach_r=r, hover_ang=hover_ang,
            )
            plan_entry["ball_xy"] = ball_xy
            return ball_xy
        except GripFailure as e:
            last_err = e
            kind = "REDETECT" if isinstance(e, RedetectFailure) else "MISS"
            print(
                f"  [GRIP] {kind} cycle {attempt}/{GRIP_MAX_RETRIES} — {e}"
            )
            if attempt >= GRIP_MAX_RETRIES:
                break
            print(
                f"  → SAFE → 재검출 {color} ball → "
                f"MOVE_TO_BALL → GRIP 재시도 ({attempt + 1}/{GRIP_MAX_RETRIES})"
            )
            go_safe(drv)
            try:
                ball_xy = stage_redetect_ball(grabber, H, win, color)
                plan_entry["ball_xy"] = ball_xy
            except RedetectFailure as re:
                last_err = re
                print(
                    f"  [REDETECT] recoverable fail — {re} "
                    f"(will retry pick with cached xy or re-SCAN)"
                )
                continue

    raise GripFailure(
        color,
        f"grasp failed after {GRIP_MAX_RETRIES} SAFE/redetect pick cycles"
        + (f" — {last_err}" if last_err else ""),
    )


def stage_move_to_box(drv, xy, win, grabber, color: str):
    """Phase 1 of place: move to box XY at carry/hover; grip stays closed until PLACE."""
    stage = f"{color.upper()} MOVE_TO_BOX"
    print(f"[{stage}] {xy} ({BOX_APPROACH_SECS:.0f}s, grip closed)")
    x, y = xy
    rgb, _ = grabber.get()
    if rgb is not None:
        pump(rgb, win, stage, [
            f"carry hover {BOX_APPROACH_SECS:.0f}s (grip closed)",
            f"xy=({x:+.3f},{y:+.3f}) z={Z_CARRY:.3f}",
        ])
    # Approach only — do NOT open gripper here (DROP_J6 reserved for PLACE after arrive)
    # Override IK j5 with taught DROP_J5 (same idea as DROP_J6 for gripper)
    go_xyz(
        drv, [x, y, Z_CARRY], down=False, secs=BOX_APPROACH_SECS,
        j5=DROP_J5, j6=GRIP_CLOSE,
    )
    print(f"  [{stage}] arrived (hover) — ready for PLACE")


def stage_place(drv, xy, win, grabber, color: str):
    """Phase 2 of place: after MOVE_TO_BOX — lower (closed) → open DROP_J6 → lift."""
    stage = f"{color.upper()} PLACE"
    print(f"[{stage}] {xy} DROP_J5={DROP_J5} DROP_J6={DROP_J6}")
    x, y = xy
    rgb, _ = grabber.get()
    if rgb is not None:
        pump(rgb, win, stage, [
            "lower(closed)→open(DROP_J6)→lift",
            f"xy=({x:+.3f},{y:+.3f}) z={Z_PLACE:.3f}",
        ])
    # Keep closed while lowering; open only after Z_PLACE motion has finished
    go_xyz(
        drv, [x, y, Z_PLACE], down=True, secs=0.65,
        j5=DROP_J5, j6=GRIP_CLOSE,
    )
    print(f"  [{stage}] lowered — opening DROP_J6={DROP_J6}")
    set_grip(drv, DROP_J6, 0.3)
    go_xyz(
        drv, [x, y, Z_HOVER], down=False, secs=0.5,
        j5=DROP_J5, j6=DROP_J6,
    )
    print(f"  [{stage}] OK")


def run_sort_from_plan(grabber, drv, plan: dict, win: str, done: set[str], H):
    """Execute pick/place for colors not yet in done. Mutates done on success.

    Blue: re-detect ball (update cache) before first MOVE_TO_BALL / GRIP.
    On grip miss: SAFE → redetect → MOVE → GRIP (up to GRIP_MAX_RETRIES).
    Raises GripFailure if a color still fails (caller recovers with full re-SCAN).
    """
    for color in COLOR_ORDER:
        if color in done:
            print(f"  skip {color}: already done")
            continue
        if color not in plan:
            print(f"  skip {color}: not in plan")
            continue
        print(f"\n======== {color.upper()} ACT (cached) ========")
        print(f"  progress: done={sorted(done) or 'none'} now={color}")
        p = plan[color]
        # Blue often drifts after red ACT — refresh ball xy once before pick
        if color == "blue":
            p["ball_xy"] = stage_redetect_ball(grabber, H, win, color)
        stage_pick_until_grasp(drv, p, win, grabber, color, H)
        stage_move_to_box(drv, p["box_xy"], win, grabber, color)
        stage_place(drv, p["box_xy"], win, grabber, color)
        go_safe(drv)
        done.add(color)
        print(f"  [{color.upper()}] completed — done={sorted(done)}")


def run_sort_with_recover(grabber, drv, H, win: str, taught: dict):
    """SCAN ALL → ACT; after GRIP_MAX_RETRIES pick cycles fail → SAFE → re-SCAN → resume."""
    done: set[str] = set()
    while True:
        pending = [c for c in COLOR_ORDER if c not in done]
        if not pending:
            print("\n모든 색상 완료")
            return
        print(f"\n======== SCAN ALL (pending: {pending}, done: {sorted(done) or 'none'}) ========")
        plan = stage_scan_all(grabber, H, win, taught)
        try:
            run_sort_from_plan(grabber, drv, plan, win, done, H)
            return
        except GripFailure as e:
            kind = "REDETECT" if isinstance(e, RedetectFailure) else "GRIP"
            print(
                f"\n[RECOVER] {e.color.upper()} {kind} failed after "
                f"{GRIP_MAX_RETRIES} SAFE/redetect pick cycles — {e}"
            )
            print(f"  completed: {sorted(done) or 'none'}")
            print(f"  → SAFE → re-SCAN → resume from remaining colors")
            go_safe(drv)
            # loop: re-scan and only act on colors not in done
