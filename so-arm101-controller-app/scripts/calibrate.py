#!/usr/bin/env python3
"""
SO-ARM101 per-robot calibration wizard (LeRobot-style).

Runs the same three steps LeRobot's follower calibration does:

  1. Centre — you put the arm in its neutral middle pose and press ENTER. Each
     servo's homing offset is rewritten so that pose reads POS_CENTER (2048).
     THIS is the step that makes commanding work: the servo's 0/4095 seam is a
     hard wall to its position controller, so a joint whose travel straddles
     the seam can't be driven end to end. Centring moves the seam to the far
     side of every joint's range, out of the way.

  2. Range — with torque off you move every joint through its full travel while
     a live MIN/POS/MAX table updates in place. Press ENTER to finish.

  3. Save — offsets, ranges and gripper direction are written to
     data/calibration.json. The driver loads it at startup.

The homing offsets are written to servo EEPROM and persist across power
cycles, so each physical arm keeps its own centring even when you swap
calibration files between robots.

Usage:
    # GUI server must NOT be running (single USB owner constraint).
    pkill -f "app.py"
    python3 scripts/calibrate.py --port /dev/ttyACM0 --label team1
"""
from __future__ import annotations

import argparse
import json
import platform
import select
import sys
import time
from pathlib import Path

# Make 'sdk' importable when run from project root.
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sdk.driver_sdk import (
    STS3215Driver, JOINT_IDS, JOINT_NAMES, JOINT_LIMITS,
    POS_CENTER, POS_MAX, POS_RANGE,
)


CALIBRATION_FILE = ROOT / "data" / "calibration.json"

# Printed once up front so the operator sees the whole arm before starting.
JOINT_BLURBS = {
    1: "받침대 회전 — 팔 전체가 좌우로 돕니다",
    2: "어깨 — 윗팔을 들어올리고 내립니다",
    3: "팔꿈치 — 아랫팔을 접고 폅니다",
    4: "손목 까딱 — 집게가 위아래로 꺾입니다",
    5: "손목 비틀기 — 집게가 축을 중심으로 돕니다",
    6: "집게 — 열고 닫습니다",
}


# ── Console helpers ───────────────────────────────────────────────────────────

def enter_pressed() -> bool:
    """True if the user has pressed ENTER, without blocking. Same idea as
    LeRobot's helper: select() on POSIX, msvcrt on Windows."""
    if platform.system() == "Windows":
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch() in (b"\r", b"\n")
        return False
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip() == ""
    return False


def move_cursor_up(lines: int) -> None:
    print(f"\033[{lines}A", end="")


def wait_enter(msg: str) -> None:
    try:
        input(msg)
    except (KeyboardInterrupt, EOFError):
        print("\n중단됨.")
        raise SystemExit(1)


# ── Step 1: centre ────────────────────────────────────────────────────────────

def center_all(arm: STS3215Driver) -> dict[int, int]:
    """Write each servo's homing offset so its current pose reads POS_CENTER.
    Returns {sid: offset written}."""
    offsets = {}
    for sid in JOINT_IDS:
        off = arm.center_half_turn(sid)
        if off is None:
            print(f"  ✗ ID {sid} ({JOINT_NAMES[sid-1]}) 오프셋 기록 실패 — 케이블 확인")
            raise SystemExit(1)
        offsets[sid] = off
        time.sleep(0.03)
    return offsets


# ── Step 2: ranges ────────────────────────────────────────────────────────────

def record_ranges(arm: STS3215Driver) -> tuple[dict[int, int], dict[int, int]]:
    """Stream live positions while the operator sweeps every joint, tracking
    the min and max each reaches. ENTER ends it. Because the arm was centred
    first, positions no longer cross the seam, so plain min/max is enough."""
    start = {sid: (arm.get_position(sid) or POS_CENTER) for sid in JOINT_IDS}
    mins = dict(start)
    maxes = dict(start)

    print()  # the redraw region starts here
    drawn = False
    while True:
        for sid in JOINT_IDS:
            p = arm.get_position(sid)
            if p is not None:
                mins[sid] = min(mins[sid], p)
                maxes[sid] = max(maxes[sid], p)

        if drawn:
            move_cursor_up(len(JOINT_IDS) + 2)
        print(f"  {'관절':<14} | {'MIN':>5} | {'POS':>5} | {'MAX':>5} | {'범위':>5}")
        print(f"  {'-'*14}-+-{'-'*5}-+-{'-'*5}-+-{'-'*5}-+-{'-'*5}")
        for sid in JOINT_IDS:
            pos = arm.get_position(sid)
            pos_s = f"{pos:>5}" if pos is not None else f"{'?':>5}"
            span = maxes[sid] - mins[sid]
            print(f"  {JOINT_NAMES[sid-1]:<14} | {mins[sid]:>5} | {pos_s} | "
                  f"{maxes[sid]:>5} | {span / POS_RANGE * 360:>4.0f}°")
        drawn = True

        if enter_pressed():
            return mins, maxes
        time.sleep(0.02)


# ── Step 3 helper: gripper direction ─────────────────────────────────────────

def detect_gripper_flip(arm: STS3215Driver, lo: int, hi: int) -> bool:
    """The gripper reports 0..100 % open, so it needs to know which raw end is
    'closed'. Ranges alone can't say. Ask for the closed pose and see which end
    it lands on: closer to max ⇒ the servo counts down from closed to open ⇒
    flip. (Rotation joints don't need this — their direction is handled by
    check_direction.py's invert flag.)"""
    wait_enter("\n  집게를 완전히 닫고 Enter → ")
    closed = arm.get_position(6)
    if closed is None:
        return False
    return abs(closed - hi) < abs(closed - lo)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--port",  default="/dev/ttyACM0")
    parser.add_argument("--label", default="", help="robot name (informational)")
    parser.add_argument("--out",   default=str(CALIBRATION_FILE))
    args = parser.parse_args()

    arm = STS3215Driver(port=args.port)
    print(f"\n{args.port} 여는 중 ...")
    if not arm.connect():
        print("  ✗ 열 수 없습니다. GUI 서버가 포트를 잡고 있나요? "
              "pkill -f app.py 후 다시 실행하세요.")
        sys.exit(1)
    print("  ✓ 연결됨")

    print(f"\n{'=' * 62}")
    print("SO-ARM101 캘리브레이션 (LeRobot 방식)")
    print("=" * 62)
    for i, sid in enumerate(JOINT_IDS, start=1):
        print(f"  {i}. ID {sid} · {JOINT_NAMES[sid-1]:<12} {JOINT_BLURBS[sid]}")

    arm.set_all_torque(False)
    print("\n모든 관절 잠금 해제됨 — 손으로 움직일 수 있습니다.")
    print("⚠ 중력에 팔이 떨어지지 않도록 한 손으로 받쳐주세요.")

    try:
        # 1. Centre
        print(f"\n{'─' * 62}")
        print("[1/3] 센터 자세")
        print("─" * 62)
        print("  팔을 중립(가운데) 자세로 맞추세요. 각 관절이 가동범위의")
        print("  한가운데 오도록 — 팔을 똑바로 세우고 집게는 정면을 보게 하면 됩니다.")
        print("  이 자세가 서보 영점(2048)이 되어 이후 모든 동작의 기준이 됩니다.")
        wait_enter("\n  센터 자세가 되면 Enter → ")
        offsets = center_all(arm)
        print("  ✓ 센터링 완료 (관절별 오프셋 기록됨)")

        # 2. Ranges
        print(f"\n{'─' * 62}")
        print("[2/3] 가동범위 측정")
        print("─" * 62)
        print("  모든 관절을 각각 끝에서 끝까지 천천히 움직이세요.")
        print("  아래 표가 실시간으로 갱신됩니다. 다 돌렸으면 Enter.")
        mins, maxes = record_ranges(arm)

        # 3. Gripper direction
        print(f"\n{'─' * 62}")
        print("[3/3] 집게 방향 확인")
        print("─" * 62)
        flip = detect_gripper_flip(arm, mins[6], maxes[6])
        print(f"  ✓ 집게 flip = {flip}")
    finally:
        arm.set_all_torque(True)
        arm.disconnect()

    # Build calibration
    cal = {"robot": args.label or "unnamed", "joints": {}}
    for sid in JOINT_IDS:
        entry = {"min": int(mins[sid]), "max": int(maxes[sid]),
                 "homing_offset": int(offsets[sid])}
        if sid == 6 and flip:
            entry["flip"] = True
        cal["joints"][str(sid)] = entry

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(cal, indent=2))

    print(f"\n✓ 저장됨: {out}")
    print("\n측정 결과:")
    print(f"  {'ID':<3} {'관절':<12} {'min':>5} {'max':>5} {'범위':>6} {'각도':>6} {'offset':>7}")
    wrapped = False
    for sid in JOINT_IDS:
        e = cal["joints"][str(sid)]
        span = e["max"] - e["min"]
        seam = "  ⚠이음매" if e["max"] > POS_MAX else ""
        if e["max"] > POS_MAX:
            wrapped = True
        print(f"  {sid:<3} {JOINT_NAMES[sid-1]:<12} {e['min']:>5} {e['max']:>5} "
              f"{span:>6} {span / POS_RANGE * 360:>5.0f}° {e['homing_offset']:>7}{seam}")

    if wrapped:
        print("\n⚠ 이음매를 걸친 관절이 있습니다. 센터 자세가 실제 중앙이 아니었을")
        print("  가능성이 큽니다. 팔을 더 정확히 중립으로 놓고 다시 실행하세요.")
    else:
        print("\n각 관절의 각도가 실제 가동범위와 비슷한지 확인하세요.")
    print("서버를 재시작하면(python3 app.py) 드라이버가 자동으로 읽습니다.")


if __name__ == "__main__":
    main()
