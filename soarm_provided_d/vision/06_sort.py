# -*- coding: utf-8 -*-
"""06_sort.py — 전체 검출 후 분류 (빨강→파랑 pick/place).

흐름:
  1 SCAN ALL  — 빨강/파랑 공·박스를 한 번에 검출·안정화 → 캐시
  2 ACT      — 캐시된 xy로
                MOVE_TO_BALL → GRIP(하강→닫기→안정화→시험상승→검증) → MOVE_TO_BOX → PLACE
  3 SAFE

집기: 거리적응 hover → 관절 도착 → soft-down 하강(Z_PICK, j6 OPEN) → 도착·개방 확인
      → 그리퍼 닫기(j6 only) → 닫힘 대기 → 짧은 안정화 → 시험 상승(Z_TEST_LIFT, ~2.5cm)
      → get_load(6) 5회 파지 검증
        ├─ 성공: Z_CARRY → MOVE_TO_BOX → PLACE
        └─ 실패: 내려놓고 열기 → SAFE → 재검출 → 재시도 (×GRIP_MAX_RETRIES)
          그래도 실패 → SAFE → 전체 재SCAN → 미완료 재개.
          재검출 타임아웃도 recoverable (RedetectFailure → SAFE/재시도/재SCAN).

놓기: MOVE_TO_BOX(박스 hover, BOX_APPROACH_SECS, j5=DROP_J5) 도착 후
      → PLACE(하강은 그립 유지·DROP_J5 → 도착 후 DROP_J6 개방 → 리프트).
      파랑은 MOVE_TO_BALL 직전 공 재검출로 캐시 갱신.

캘리브: data/H.npy + data/map_calib.json
키: q=중단
CLI: --measure-load  → empty/good/wrong-catch get_load(6) 분포 측정

구현은 vision/sort_lib/ 모듈로 분리 (이 파일은 엔트리포인트·오케스트레이터).
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false")
_cv2_fonts = Path(os.environ.get("VIRTUAL_ENV", "/home/rookie/D053/.venv")) / (
    "lib/python3.13/site-packages/cv2/qt/fonts"
)
if _cv2_fonts.is_dir() or _cv2_fonts.is_symlink():
    os.environ.setdefault("QT_QPA_FONTDIR", str(_cv2_fonts.resolve()))

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab.driver_sdk import STS3215Driver

_VISION = Path(__file__).resolve().parent
if str(_VISION) not in sys.path:
    sys.path.insert(0, str(_VISION))

from sort_lib.arm_motion import go_safe  # noqa: E402
from sort_lib.constants import (  # noqa: E402
    ACC,
    BOX_APPROACH_SECS,
    CALIB_PATH,
    DROP_J5,
    DROP_J6,
    GRIP_CLOSE,
    GRIP_LOAD_THRESH,
    GRIP_MAX_RETRIES,
    H_PATH,
    PICK_J5,
    PICK_J6,
    PORT,
    ROOT,
    SPEED,
    Z_CARRY,
    Z_PICK,
    Z_TEST_LIFT,
)
from sort_lib.detect_plan import FrameGrabber, load_taught_places  # noqa: E402
from sort_lib.grasp import measure_grip_load_profiles  # noqa: E402
from sort_lib.stages import run_sort_with_recover  # noqa: E402

os.chdir(ROOT)


def _connect_driver():
    drv = STS3215Driver(port=PORT)
    drv.connect()
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음")
    for sid in (1, 2, 3, 4, 5, 6):
        drv.set_torque(sid, True)
        if sid <= 5:
            drv.set_acceleration(sid, ACC)
            drv.set_speed(sid, SPEED)
    return drv


def main_measure_load():
    """CLI: --measure-load — sample empty / good / wrong-catch get_load(6) profiles."""
    print(
        f"--measure-load: sample get_load(6) for threshold tuning "
        f"(default GRIP_LOAD_THRESH={GRIP_LOAD_THRESH})"
    )
    drv = _connect_driver()
    try:
        go_safe(drv)
        measure_grip_load_profiles(drv)
    except KeyboardInterrupt:
        print("사용자 중단")
    finally:
        try:
            go_safe(drv)
        except Exception:
            pass
        drv.disconnect()


def main():
    if not H_PATH.is_file():
        raise SystemExit(f"없음: {H_PATH}")
    H = np.load(str(H_PATH))
    calib = json.loads(CALIB_PATH.read_text(encoding="utf-8")) if CALIB_PATH.is_file() else {}
    taught = load_taught_places(calib)
    print("티칭 place 폴백:", taught)
    print(
        f"PICK_J5={PICK_J5} DROP_J5={DROP_J5} "
        f"PICK_J6={PICK_J6} GRIP_CLOSE={GRIP_CLOSE} DROP_J6={DROP_J6}"
    )
    print(
        f"Z_PICK={Z_PICK} Z_TEST_LIFT={Z_TEST_LIFT} Z_CARRY={Z_CARRY} "
        f"grip load thresh={GRIP_LOAD_THRESH} pick_cycles={GRIP_MAX_RETRIES}"
    )
    print(f"BOX_APPROACH_SECS={BOX_APPROACH_SECS} (open DROP_J6 only after PLACE lower)")
    print(
        "흐름: SCAN ALL → ACT("
        "MOVE_TO_BALL→descend(open)→close(j6)→settle→test_lift→verify→"
        "MOVE_TO_BOX→PLACE) → SAFE "
        "(blue: re-detect before pick; miss → 내려놓고열기→SAFE+재검출+재시도×"
        f"{GRIP_MAX_RETRIES}; redetect timeout recoverable; then SAFE+재SCAN+미완료 재개)"
    )

    drv = _connect_driver()

    # Confirm remembered j5/j6 from live arm (informational)
    live = drv.get_all_positions()
    live_j5 = live.get(5)
    live_j6 = live.get(6)
    print(f"live j5={live_j5} (DROP_J5={DROP_J5}) live j6={live_j6} (PICK_J6={PICK_J6} DROP_J6={DROP_J6})")

    win = "sort stages (q=abort)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    go_safe(drv)

    # copy=True: SHM frames are owned copies → avoids BufferError on close
    with CameraReader(copy=True) as cam:
        grabber = FrameGrabber(cam)
        grabber.start()
        try:
            run_sort_with_recover(grabber, drv, H, win, taught)
            print("\n완료")
        except KeyboardInterrupt:
            print("사용자 중단")
        except Exception as e:
            print("오류:", e)
        finally:
            try:
                go_safe(drv)
            except Exception:
                pass
            grabber.stop()
            grabber.join(timeout=1.0)
            cv2.destroyAllWindows()
            drv.disconnect()


if __name__ == "__main__":
    if "--measure-load" in sys.argv:
        main_measure_load()
    else:
        main()
