#!/usr/bin/env python3
"""
SO-ARM101 direction calibration — figures out which joints need
``"invert": true`` in data/calibration.json by moving each joint a
small amount and asking the user which direction the **real** robot
went vs which direction the **twin** showed.

Run this after scripts/calibrate.py (or any time the twin direction
feels wrong on a particular robot). The script writes the invert
flags into the existing calibration.json (or creates one if needed).

Usage (GUI server must NOT be running — single USB owner):
    pkill -f app.py
    python3 scripts/check_direction.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sdk.driver_sdk import STS3215Driver, JOINT_IDS, JOINT_NAMES


CAL_FILE = ROOT / "data" / "calibration.json"
NUDGE = 200  # how many raw counts to nudge per direction test


def load_or_init() -> dict:
    if CAL_FILE.exists():
        try:
            return json.loads(CAL_FILE.read_text())
        except Exception:
            pass
    return {"robot": "unnamed", "joints": {}}


def save(cal: dict) -> None:
    CAL_FILE.parent.mkdir(parents=True, exist_ok=True)
    CAL_FILE.write_text(json.dumps(cal, indent=2))


def ask(question: str) -> str:
    while True:
        try:
            ans = input(question).strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nAborted.")
            sys.exit(1)
        if ans in ("y", "n", "s"):
            return ans
        print("  (y = 같은 방향, n = 반대 방향, s = 이 관절 skip)")


def check_one(d: STS3215Driver, sid: int) -> bool | None:
    """Returns:
        True  → invert needed (twin moved opposite to real)
        False → direction OK
        None  → skipped
    """
    name = JOINT_NAMES[sid - 1]
    p0 = d.get_position(sid)
    if p0 is None:
        print(f"  joint {sid} {name}: no response, skipping")
        return None

    target = p0 + NUDGE
    if target > 4000:
        target = p0 - NUDGE   # avoid hitting full-range limit

    print(f"\n── joint {sid} {name} ──")
    print(f"  현재 raw={p0} → 명령 raw={target}")
    print(f"  실물 + 트윈 동시에 보세요. 어떻게 움직이는지...")
    d.set_torque(sid, True)
    d.set_speed(sid, 200)
    d.set_position(sid, target)
    time.sleep(2.5)
    p1 = d.get_position(sid)
    delta_raw = (p1 - p0) if p1 is not None else None
    real_dir = "increased" if (delta_raw or 0) > 0 else "decreased"
    print(f"  서보 raw {p0} → {p1}  (delta {delta_raw})")
    print(f"  웹 UI 의 트윈을 보세요. 실물 로봇과 같은 방향으로 움직였나요?")
    ans = ask(f"  같은 방향? [y/n/s]: ")

    # Return joint to original position
    d.set_position(sid, p0)
    time.sleep(2.0)

    if ans == "s":
        return None
    return ans == "n"   # n = twin opposite of real → need invert


def main():
    d = STS3215Driver(port="/dev/ttyACM0")
    print("Connecting to robot...")
    if not d.connect():
        print("✗ 시리얼 포트 열기 실패. GUI 서버가 점유 중이면 끄세요.")
        sys.exit(1)

    cal = load_or_init()
    joints = cal.setdefault("joints", {})

    print("\n각 관절을 하나씩 시험하고 y/n 으로 답해주세요.")
    print("브라우저에서 디지털 트윈 페이지가 열려있고 서버가 떠 있어야 시각 비교 가능.")
    print("⚠ 서버는 끄고 이 스크립트만 실행 중이라면 트윈은 정지 상태일 수 있어요.")
    print("  그 경우엔 별도 PC 에서 같은 robot 에 접속해서 봐야 합니다.")
    input("\n준비되면 Enter →")

    for sid in JOINT_IDS:
        needs_invert = check_one(d, sid)
        if needs_invert is None:
            continue
        entry = joints.setdefault(str(sid), {})
        if needs_invert:
            entry["invert"] = True
            print(f"  ✓ joint {sid}: invert=True 설정")
        else:
            entry.pop("invert", None)
            print(f"  ✓ joint {sid}: invert 제거 (방향 OK)")

    # 정리
    d.set_all_torque(False)
    d.disconnect()

    save(cal)
    print(f"\n✓ 저장 완료: {CAL_FILE}")
    print("\n관절별 invert 상태:")
    for sid in JOINT_IDS:
        e = joints.get(str(sid), {})
        flag = "INVERTED" if e.get("invert") else "normal"
        print(f"  {sid} {JOINT_NAMES[sid-1]:<12}  {flag}")
    print("\n서버 재시작 + 브라우저 새로고침 하면 적용됨.")


if __name__ == "__main__":
    main()
