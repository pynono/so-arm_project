#!/usr/bin/env python3
"""
Why isn't the digital twin tracking this robot?

Checks the three things that silently break twin alignment on a fresh
machine: wrong branch (no calibration loader), missing/unloaded
calibration file, and joints that ignore calibration by design.

Usage:
    python3 scripts/diagnose.py
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

CAL_FILE = ROOT / "data" / "calibration.json"
DRIVER = ROOT / "sdk" / "driver_sdk.py"
TWIN = ROOT / "frontend" / "js" / "twin.js"

ok, bad = "  [OK]  ", "  [FAIL]"


def branch() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=ROOT, text=True).strip()
    except Exception:
        return "<unknown>"


def main() -> None:
    print(f"\nrepo   : {ROOT}")
    print(f"branch : {branch()}\n")

    fatal = []

    # 1. Does this checkout even have the calibration machinery?
    print("1. 코드에 캘리브레이션 기능이 있나")
    has_loader = "_load_calibration_overrides" in DRIVER.read_text()
    has_flip = "lim.flip" in TWIN.read_text() if TWIN.exists() else False
    has_wizard = (ROOT / "scripts" / "calibrate.py").exists()
    for label, present in [
        ("driver_sdk.py: _load_calibration_overrides()", has_loader),
        ("twin.js: flip 처리", has_flip),
        ("scripts/calibrate.py", has_wizard),
    ]:
        print(f"{ok if present else bad} {label}")
    if not (has_loader and has_flip and has_wizard):
        fatal.append(
            "이 체크아웃에는 캘리브레이션 기능이 없습니다 (main 브랜치로 보임).\n"
            "     → git fetch origin && git checkout armtwin")

    # 2. Does the calibration file exist and parse?
    print("\n2. 캘리브레이션 파일")
    cal = None
    if not CAL_FILE.exists():
        print(f"{bad} data/calibration.json 없음")
        fatal.append("캘리브레이션 파일이 없습니다 → python3 scripts/calibrate.py")
    else:
        try:
            cal = json.loads(CAL_FILE.read_text())
            joints = cal.get("joints") or {}
            print(f"{ok} 있음 — robot='{cal.get('robot')}', "
                  f"관절 {sorted(joints, key=int)} 캘리브레이션됨")
            missing = [j for j in "12345" if j not in joints]
            if missing:
                print(f"{bad} 관절 {missing} 은 측정 안 됨")
        except Exception as exc:
            print(f"{bad} 파싱 실패: {exc}")
            fatal.append("calibration.json 이 깨졌습니다 → 재캘리브레이션")

    # 3. Did the driver ACTUALLY load it into memory?
    print("\n3. 드라이버가 실제로 로드했나 (import 시점 머지)")
    try:
        from sdk.driver_sdk import JOINT_LIMITS
    except Exception as exc:
        print(f"{bad} driver_sdk import 실패: {exc}")
        sys.exit(1)

    DEFAULT_G = {"min": 984, "max": 2318}
    g = JOINT_LIMITS[6]
    print(f"       실행 중인 JOINT_LIMITS[6] = min={g['min']} max={g['max']}")
    if g["min"] == DEFAULT_G["min"] and g["max"] == DEFAULT_G["max"]:
        if cal and "6" in (cal.get("joints") or {}):
            print(f"{bad} 파일에는 값이 있는데 메모리는 기본값 — 로드 실패")
            fatal.append(
                "캘리브레이션이 로드되지 않았습니다.\n"
                "     → 서버 재시작 필요 (import 시점에만 읽음)")
        else:
            print(f"{bad} 기본값 사용 중 (캘리브레이션 미적용)")
    else:
        print(f"{ok} 캘리브레이션 값이 메모리에 반영됨")

    # 4. The design gap: joints 1-5 ignore calibration in the twin.
    print("\n4. 트윈이 캘리브레이션을 반영하는 관절")
    for jid in range(1, 7):
        lim = JOINT_LIMITS[jid]
        uses_cal = "rad_min" in lim and "rad_max" in lim
        mark = ok if uses_cal else bad
        why = ("보간 경로 — min/max 반영됨" if uses_cal
               else "positionToRad() 고정 공식 — min/max 무시됨")
        print(f"{mark} joint {jid}: {why}")

    blind = [j for j in range(1, 6)
             if "rad_min" not in JOINT_LIMITS[j]]
    if blind:
        fatal.append(
            f"관절 {blind} 는 캘리브레이션해도 트윈에 반영되지 않습니다.\n"
            "     twin.js 가 rad_min/rad_max 없는 관절은 센터 2048 고정 공식으로\n"
            "     렌더링합니다. 서보 혼 장착 오프셋을 보정할 방법이 없습니다.")

    print("\n" + "─" * 60)
    if fatal:
        print("문제:\n")
        for i, f in enumerate(fatal, 1):
            print(f"  {i}. {f}\n")
    else:
        print("검사 통과 — 트윈이 안 맞으면 브라우저 강력 새로고침(Ctrl+Shift+R)")


if __name__ == "__main__":
    main()
