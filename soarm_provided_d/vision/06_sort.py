# -*- coding: utf-8 -*-
"""06_sort.py — 전체 검출 후 분류 (빨강→파랑 pick/place).

흐름:
  1 SCAN ALL  — 빨강/파랑 공·박스를 한 번에 검출·안정화 → 캐시
  2 ACT      — 캐시된 xy로
                MOVE_TO_BALL → GRIP → MOVE_TO_BOX → PLACE  (red 후 blue)
  3 SAFE

집기: MOVE_TO_BALL(hover 접근 완료) → GRIP(하강→load 파지→리프트).
      j6 열림(기억값) → 닫으며 load 감시, |load|>=50 이면 파지 성공.
      실패(|load|<50) 시: SAFE → 해당 색 공 재검출 → MOVE_TO_BALL → GRIP
      을 최대 GRIP_MAX_RETRIES회 반복. 그래도 실패하면 SAFE → 전체 재SCAN →
      미완료 색상만 이어서 재개.

놓기: MOVE_TO_BOX(박스 hover, BOX_APPROACH_SECS) 도착 후
      → PLACE(하강은 그립 유지 → 도착 후 DROP_J6 개방 → 리프트).
      파랑은 MOVE_TO_BALL 직전 공 재검출로 캐시 갱신.

캘리브: data/H.npy + data/map_calib.json
키: q=중단
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
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
from soarm_lab import arm
from soarm_lab.driver_sdk import STS3215Driver

_VISION = Path(__file__).resolve().parent
if str(_VISION) not in sys.path:
    sys.path.insert(0, str(_VISION))
from detect_objects import annotate_objects, detect_colored_objects  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)

H_PATH = ROOT / "data" / "H.npy"
CALIB_PATH = ROOT / "data" / "map_calib.json"

SPEED = 500
ACC = 25
PORT = "/dev/ttyACM0"

Z_HOVER = 0.13
Z_PICK = 0.028
Z_CARRY = 0.14
Z_PLACE = 0.06
SETTLE_N = 8
DETECT_HOLD = 5       # 연속 N프레임 보이면 DETECT 통과
SCAN_TIMEOUT = 45.0
REDETECT_TIMEOUT = 12.0
_SEED = (0, 30, -45, 0, 0)

# Taught approach j5 + shared open j6 (live raw from /dev/ttyACM0)
PICK_J5 = 1021
J6_OPEN = 2350          # approach open + place/drop open
PICK_J6 = J6_OPEN       # MOVE_TO_BALL approach + GRIP open
DROP_J6 = J6_OPEN       # PLACE open after lower
PICK_APPROACH_SECS = 6.0

# Box approach: slow carry to box hover; grip stays CLOSED until PLACE opens
BOX_APPROACH_SECS = 6.0

# Grip: open = J6_OPEN; close for grasp
GRIP_OPEN = J6_OPEN
GRIP_CLOSE = 1950
GRIP_LOAD_THRESH = 50        # |get_load(6)| >= this → grasped
GRIP_INPLACE_TRIES = 1       # quick open→close at pose before leaving for redetect
GRIP_MAX_RETRIES = 3         # full pick cycles: SAFE→redetect→MOVE→GRIP; then re-SCAN
GRIP_CLOSE_POLL_SECS = 2.2

SAFE_POSE = {1: 2047, 2: 813, 3: 3192, 4: 1039, 5: 1137, 6: GRIP_OPEN}

# Scan targets: (color, kind) — balls required; boxes may fall back to taught
_SCAN_KEYS = (("red", "ball"), ("blue", "ball"), ("red", "box"), ("blue", "box"))
_COLOR_ORDER = ("red", "blue")


class GripFailure(RuntimeError):
    """Raised when pick/grip fails after GRIP_MAX_RETRIES redetect cycles.

    Caller (run_sort_with_recover) goes SAFE → full re-SCAN → resume pending.
    """

    def __init__(self, color: str, msg: str):
        super().__init__(msg)
        self.color = color


class FrameGrabber(threading.Thread):
    def __init__(self, cam: CameraReader):
        super().__init__(daemon=True, name="cam-grabber")
        self._cam = cam
        self._lock = threading.Lock()
        self._rgb = None
        self._fid = 0
        self._stop = threading.Event()

    def run(self):
        last = 0
        while not self._stop.is_set():
            rgb, _d, fid = self._cam.read()
            if rgb is not None and fid > last:
                with self._lock:
                    self._rgb = rgb
                    self._fid = fid
                last = fid
            else:
                time.sleep(0.001)

    def get(self):
        with self._lock:
            if self._rgb is None:
                return None, 0
            return self._rgb.copy(), self._fid

    def stop(self):
        self._stop.set()


def pixel_to_xy(u, v, H):
    x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
    return float(x), float(y)


def load_taught_places(calib: dict) -> dict[str, tuple[float, float]]:
    out: dict[str, list] = {"red": [], "blue": []}
    for b in calib.get("boxes") or []:
        c = b.get("color")
        if c in out and b.get("xyz"):
            out[c].append(b["xyz"][:2])
    places = {}
    for c, pts in out.items():
        if pts:
            arr = np.mean(np.array(pts), axis=0)
            places[c] = (float(arr[0]), float(arr[1]))
    return places


def go_xyz(drv, xyz, down=False, seed=_SEED, secs=1.8, j5=None, j6=None):
    ang, err = arm.ik.solve(list(xyz), seed_deg=list(seed), down=down)
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
    return ang, err


def set_grip(drv, raw: int, secs=1.45):
    cur = drv.get_all_positions()
    pos = {i: cur[i] for i in range(1, 7) if cur.get(i) is not None}
    pos[6] = int(raw)
    drv.set_all_positions(pos)
    time.sleep(secs)


def grip_until_load(
    drv,
    open_raw: int = GRIP_OPEN,
    close_raw: int = GRIP_CLOSE,
    thresh: int = GRIP_LOAD_THRESH,
    max_retries: int = GRIP_INPLACE_TRIES,
):
    """Close gripper while monitoring servo-6 load (in-place only).

    Each attempt: open → close → poll |load|.
    If |load| < thresh, optionally retry open→close in place (GRIP_INPLACE_TRIES).
    On failure, raises RuntimeError — caller should SAFE → redetect → MOVE → GRIP.
    """
    last_load = None
    for attempt in range(1, max_retries + 1):
        print(
            f"  [GRIP] inplace {attempt}/{max_retries} "
            f"open={open_raw} → close={close_raw} (need |load|>={thresh})"
        )
        set_grip(drv, open_raw, 1.35)
        cur = drv.get_all_positions()
        pos = {i: cur[i] for i in range(1, 7) if cur.get(i) is not None}
        pos[6] = int(close_raw)
        drv.set_all_positions(pos)

        deadline = time.monotonic() + GRIP_CLOSE_POLL_SECS
        while time.monotonic() < deadline:
            load = drv.get_load(6)
            last_load = load
            mag = abs(load) if load is not None else 0
            if mag >= thresh:
                print(f"  [GRIP] GRASP OK load={load} (|load|>={thresh})")
                return True
            time.sleep(0.05)

        miss = (
            f"  [GRIP] MISS inplace {attempt}/{max_retries} "
            f"load={last_load} (|load|<{thresh})"
        )
        if attempt < max_retries:
            print(f"{miss} → retry open→close in place")
            set_grip(drv, open_raw, 1.3)
        else:
            print(f"{miss} → leave for SAFE/redetect pick cycle")

    raise RuntimeError(
        f"grasp miss after {max_retries} inplace tries "
        f"(last load={last_load}, thresh={thresh})"
    )


def go_safe(drv):
    print("[SAFE]")
    drv.set_all_positions(dict(SAFE_POSE))
    time.sleep(2.0)


def draw_status(bgr, stage: str, lines):
    y = 22
    for c, tk in (((0, 0, 0), 3), ((0, 255, 255), 1)):
        cv2.putText(bgr, f"STAGE: {stage}", (8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, c, tk)
    y = 48
    for line in lines:
        for c, tk in (((0, 0, 0), 3), ((0, 255, 0), 1)):
            cv2.putText(bgr, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, tk)
        y += 22


def _pump(rgb, win, stage, lines):
    draw_status(rgb, stage, lines)
    cv2.imshow(win, rgb)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        raise KeyboardInterrupt


def _best_of(objs, color: str, kind: str):
    cands = [o for o in objs if o["color"] == color and o["kind"] == kind]
    if not cands:
        return None
    return max(cands, key=lambda x: x["area"])


def stage_scan_all(grabber, H, win: str, taught_places: dict, timeout=SCAN_TIMEOUT):
    """Detect all red/blue balls (+ boxes) first, settle xy, return plan cache.

    Plan: {color: {"ball_xy": (x,y), "box_xy": (x,y)}}
    Boxes may use taught_places if never seen.
    """
    stage = "SCAN ALL"
    print(f"\n[{stage}] 전체 검출…")
    deadline = time.monotonic() + timeout
    hit = {k: 0 for k in _SCAN_KEYS}
    buf: dict[tuple, list] = {k: [] for k in _SCAN_KEYS}
    last_fid = -1
    last_seen: dict[tuple, dict] = {}

    while time.monotonic() < deadline:
        rgb, fid = grabber.get()
        if rgb is None or fid == last_fid:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt
            time.sleep(0.005)
            continue
        last_fid = fid
        objs = detect_colored_objects(rgb)
        annotate_objects(rgb, objs)

        for key in _SCAN_KEYS:
            color, kind = key
            o = _best_of(objs, color, kind)
            if o is None:
                hit[key] = 0
                continue
            last_seen[key] = o
            hit[key] += 1
            if hit[key] < DETECT_HOLD:
                continue
            u, v = o["uv"]
            xy = pixel_to_xy(u, v, H)
            buf[key].append(xy)
            if len(buf[key]) > SETTLE_N:
                buf[key].pop(0)

        # Status lines
        lines = []
        for color, kind in _SCAN_KEYS:
            key = (color, kind)
            n = len(buf[key])
            h = hit[key]
            if n >= SETTLE_N:
                arr = np.array(buf[key])
                mean = arr.mean(axis=0)
                lines.append(
                    f"{color}_{kind}: OK xy=({mean[0]:+.3f},{mean[1]:+.3f})"
                )
            elif key in last_seen:
                lines.append(f"{color}_{kind}: hit={h}/{DETECT_HOLD} settle={n}/{SETTLE_N}")
            else:
                lines.append(f"{color}_{kind}: looking…")
        lines.append("q=abort")
        _pump(rgb, win, stage, lines)

        # Ready when balls settled; boxes settled OR taught fallback available
        balls_ok = all(
            len(buf[(c, "ball")]) >= SETTLE_N
            and float(np.std(np.array(buf[(c, "ball")]), axis=0).max()) < 0.008
            for c in ("red", "blue")
        )
        if not balls_ok:
            continue

        boxes_ready = True
        box_xy: dict[str, tuple[float, float]] = {}
        box_src: dict[str, str] = {}
        for c in ("red", "blue"):
            key = (c, "box")
            if (
                len(buf[key]) >= SETTLE_N
                and float(np.std(np.array(buf[key]), axis=0).max()) < 0.008
            ):
                mean = np.array(buf[key]).mean(axis=0)
                box_xy[c] = (float(mean[0]), float(mean[1]))
                box_src[c] = "detect"
            elif c in taught_places:
                box_xy[c] = taught_places[c]
                box_src[c] = "taught"
            else:
                boxes_ready = False
                break
        if not boxes_ready:
            continue

        plan = {}
        for c in ("red", "blue"):
            bmean = np.array(buf[(c, "ball")]).mean(axis=0)
            plan[c] = {
                "ball_xy": (float(bmean[0]), float(bmean[1])),
                "box_xy": box_xy[c],
            }
            if (c, "ball") in last_seen:
                plan[c]["ball_uv"] = last_seen[(c, "ball")]["uv"]
            if (c, "box") in last_seen:
                plan[c]["box_uv"] = last_seen[(c, "box")]["uv"]

        print(f"\n[{stage}] 완료 — 캐시 요약:")
        for c, p in plan.items():
            bx, by = p["ball_xy"]
            px, py = p["box_xy"]
            src = box_src.get(c, "?")
            print(f"  {c}: ball=({bx:+.3f},{by:+.3f}) → box=({px:+.3f},{py:+.3f}) [{src}]")
        _pump(rgb, win, stage, [
            "ALL CACHED",
            f"red ball→box",
            f"blue ball→box",
            "starting ACT…",
        ])
        time.sleep(1.5)
        return plan

    raise TimeoutError(f"{stage} 타임아웃 — 전체 검출 실패")


def stage_move_to_ball(drv, xy, win, grabber, color: str):
    """Phase 1: move to ball XY at hover; wait until approach finishes before grip."""
    stage = f"{color.upper()} MOVE_TO_BALL"
    print(f"[{stage}] {xy}")
    x, y = xy
    rgb, _ = grabber.get()
    if rgb is not None:
        _pump(rgb, win, stage, [
            f"hover approach {PICK_APPROACH_SECS:.0f}s",
            f"xy=({x:+.3f},{y:+.3f}) z={Z_HOVER:.3f}",
        ])
    # Approach above ball: IK for j1–4, taught j5/j6, slow entry — complete before GRIP
    go_xyz(
        drv, [x, y, Z_HOVER], down=False, secs=PICK_APPROACH_SECS,
        j5=PICK_J5, j6=PICK_J6,
    )
    print(f"  [{stage}] arrived (hover) — ready for GRIP")


def stage_grip(drv, xy, win, grabber, color: str):
    """Phase 2: after MOVE_TO_BALL — down → close with load feedback → lift.

    On grasp miss (|load| < thresh after inplace tries), raises GripFailure(color).
    Caller should recover with SAFE → redetect → MOVE_TO_BALL → GRIP.
    """
    stage = f"{color.upper()} GRIP"
    print(f"[{stage}] {xy}")
    x, y = xy
    rgb, _ = grabber.get()
    if rgb is not None:
        _pump(rgb, win, stage, [
            "down→grip(load)→lift",
            f"xy=({x:+.3f},{y:+.3f})",
            f"miss if |load|<{GRIP_LOAD_THRESH} → SAFE/redetect",
        ])
    go_xyz(drv, [x, y, Z_PICK], down=True, secs=2.0, j5=PICK_J5, j6=PICK_J6)
    try:
        grip_until_load(drv, open_raw=PICK_J6, close_raw=GRIP_CLOSE)
    except RuntimeError as e:
        # Open before leaving so SAFE/redetect is not holding a closed empty grip
        try:
            set_grip(drv, PICK_J6, 1.3)
        except Exception:
            pass
        raise GripFailure(color, str(e)) from e
    go_xyz(drv, [x, y, Z_CARRY], down=False, secs=1.9, j6=GRIP_CLOSE)
    print(f"  [{stage}] OK")


def stage_pick(drv, xy, win, grabber, color: str):
    """Pick = MOVE_TO_BALL (complete) then GRIP; phases are sequential, not merged."""
    stage_move_to_ball(drv, xy, win, grabber, color)
    stage_grip(drv, xy, win, grabber, color)


def stage_pick_until_grasp(drv, plan_entry: dict, win, grabber, color: str, H):
    """MOVE_TO_BALL → GRIP; on miss: SAFE → redetect ball → retry pick.

    Updates plan_entry['ball_xy'] with fresh detections. Raises GripFailure after
    GRIP_MAX_RETRIES full pick cycles so outer recover can re-SCAN.
    """
    ball_xy = plan_entry["ball_xy"]
    last_err: Exception | None = None
    for attempt in range(1, GRIP_MAX_RETRIES + 1):
        print(
            f"  [PICK] cycle {attempt}/{GRIP_MAX_RETRIES} "
            f"{color} xy=({ball_xy[0]:+.3f},{ball_xy[1]:+.3f})"
        )
        try:
            stage_move_to_ball(drv, ball_xy, win, grabber, color)
            stage_grip(drv, ball_xy, win, grabber, color)
            plan_entry["ball_xy"] = ball_xy
            return ball_xy
        except GripFailure as e:
            last_err = e
            print(
                f"  [GRIP] MISS cycle {attempt}/{GRIP_MAX_RETRIES} "
                f"(|load|<{GRIP_LOAD_THRESH}) — {e}"
            )
            if attempt >= GRIP_MAX_RETRIES:
                break
            print(
                f"  → SAFE → redetect {color} ball → "
                f"MOVE_TO_BALL → GRIP (retry {attempt + 1}/{GRIP_MAX_RETRIES})"
            )
            go_safe(drv)
            ball_xy = stage_redetect_ball(grabber, H, win, color)
            plan_entry["ball_xy"] = ball_xy

    raise GripFailure(
        color,
        f"grasp failed after {GRIP_MAX_RETRIES} SAFE/redetect pick cycles"
        + (f" — {last_err}" if last_err else ""),
    )


def stage_redetect_ball(grabber, H, win: str, color: str, timeout=REDETECT_TIMEOUT):
    """Quick re-detect of one color ball; settle xy and return (x, y).

    Used before blue MOVE_TO_BALL so pick uses a fresh ball position.
    """
    stage = f"{color.upper()} REDETECT"
    print(f"[{stage}] ball before pick…")
    deadline = time.monotonic() + timeout
    hit = 0
    buf: list[tuple[float, float]] = []
    last_fid = -1
    last_o = None

    while time.monotonic() < deadline:
        rgb, fid = grabber.get()
        if rgb is None or fid == last_fid:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt
            time.sleep(0.005)
            continue
        last_fid = fid
        objs = detect_colored_objects(rgb)
        annotate_objects(rgb, objs)
        o = _best_of(objs, color, "ball")
        if o is None:
            hit = 0
            _pump(rgb, win, stage, [f"{color} ball: looking…", "q=abort"])
            continue
        last_o = o
        hit += 1
        if hit >= DETECT_HOLD:
            u, v = o["uv"]
            buf.append(pixel_to_xy(u, v, H))
            if len(buf) > SETTLE_N:
                buf.pop(0)
        n = len(buf)
        _pump(rgb, win, stage, [
            f"{color} ball: hit={hit}/{DETECT_HOLD} settle={n}/{SETTLE_N}",
            "q=abort",
        ])
        if (
            n >= SETTLE_N
            and float(np.std(np.array(buf), axis=0).max()) < 0.008
        ):
            mean = np.array(buf).mean(axis=0)
            xy = (float(mean[0]), float(mean[1]))
            print(
                f"[{color.upper()}] re-detect ball before pick → "
                f"({xy[0]:+.3f},{xy[1]:+.3f})"
            )
            return xy

    raise TimeoutError(
        f"{stage} timeout — {color} ball not settled"
        + (f" (last uv={last_o['uv']})" if last_o else "")
    )


def stage_move_to_box(drv, xy, win, grabber, color: str):
    """Phase 1 of place: move to box XY at carry/hover; grip stays closed until PLACE."""
    stage = f"{color.upper()} MOVE_TO_BOX"
    print(f"[{stage}] {xy} ({BOX_APPROACH_SECS:.0f}s, grip closed)")
    x, y = xy
    rgb, _ = grabber.get()
    if rgb is not None:
        _pump(rgb, win, stage, [
            f"carry hover {BOX_APPROACH_SECS:.0f}s (grip closed)",
            f"xy=({x:+.3f},{y:+.3f}) z={Z_CARRY:.3f}",
        ])
    # Approach only — do NOT open gripper here (DROP_J6 reserved for PLACE after arrive)
    go_xyz(drv, [x, y, Z_CARRY], down=False, secs=BOX_APPROACH_SECS, j6=GRIP_CLOSE)
    print(f"  [{stage}] arrived (hover) — ready for PLACE")


def stage_place(drv, xy, win, grabber, color: str):
    """Phase 2 of place: after MOVE_TO_BOX — lower (closed) → open DROP_J6 → lift."""
    stage = f"{color.upper()} PLACE"
    print(f"[{stage}] {xy} DROP_J6={DROP_J6}")
    x, y = xy
    rgb, _ = grabber.get()
    if rgb is not None:
        _pump(rgb, win, stage, [
            "lower(closed)→open(DROP_J6)→lift",
            f"xy=({x:+.3f},{y:+.3f}) z={Z_PLACE:.3f}",
        ])
    # Keep closed while lowering; open only after Z_PLACE motion has finished
    go_xyz(drv, [x, y, Z_PLACE], down=True, secs=1.9, j6=GRIP_CLOSE)
    print(f"  [{stage}] lowered — opening DROP_J6={DROP_J6}")
    set_grip(drv, DROP_J6, 1.45)
    go_xyz(drv, [x, y, Z_HOVER], down=False, secs=1.7, j6=DROP_J6)
    print(f"  [{stage}] OK")


def run_sort_from_plan(grabber, drv, plan: dict, win: str, done: set[str], H):
    """Execute pick/place for colors not yet in done. Mutates done on success.

    Blue: re-detect ball (update cache) before first MOVE_TO_BALL / GRIP.
    On grip miss: SAFE → redetect → MOVE → GRIP (up to GRIP_MAX_RETRIES).
    Raises GripFailure if a color still fails (caller recovers with full re-SCAN).
    """
    for color in _COLOR_ORDER:
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
        pending = [c for c in _COLOR_ORDER if c not in done]
        if not pending:
            print("\n모든 색상 완료")
            return
        print(f"\n======== SCAN ALL (pending: {pending}, done: {sorted(done) or 'none'}) ========")
        plan = stage_scan_all(grabber, H, win, taught)
        try:
            run_sort_from_plan(grabber, drv, plan, win, done, H)
            return
        except GripFailure as e:
            print(
                f"\n[RECOVER] {e.color.upper()} GRIP failed after "
                f"{GRIP_MAX_RETRIES} SAFE/redetect pick cycles — {e}"
            )
            print(f"  completed: {sorted(done) or 'none'}")
            print(f"  → SAFE → re-SCAN → resume from remaining colors")
            go_safe(drv)
            # loop: re-scan and only act on colors not in done


def main():
    if not H_PATH.is_file():
        raise SystemExit(f"없음: {H_PATH}")
    H = np.load(str(H_PATH))
    calib = json.loads(CALIB_PATH.read_text(encoding="utf-8")) if CALIB_PATH.is_file() else {}
    taught = load_taught_places(calib)
    print("티칭 place 폴백:", taught)
    print(f"PICK_J5={PICK_J5} PICK_J6={PICK_J6} GRIP_CLOSE={GRIP_CLOSE} DROP_J6={DROP_J6}")
    print(
        f"grip load thresh={GRIP_LOAD_THRESH} "
        f"inplace={GRIP_INPLACE_TRIES} pick_cycles={GRIP_MAX_RETRIES}"
    )
    print(f"BOX_APPROACH_SECS={BOX_APPROACH_SECS} (open DROP_J6 only after PLACE lower)")
    print(
        "흐름: SCAN ALL → ACT(MOVE_TO_BALL→GRIP→MOVE_TO_BOX→PLACE) → SAFE "
        "(blue: re-detect before pick; grip miss → SAFE+재검출+MOVE+GRIP×"
        f"{GRIP_MAX_RETRIES}; then SAFE+재SCAN+미완료 재개)"
    )

    drv = STS3215Driver(port=PORT)
    drv.connect()
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음")
    for sid in (1, 2, 3, 4, 5, 6):
        drv.set_torque(sid, True)
        if sid <= 5:
            drv.set_acceleration(sid, ACC)
            drv.set_speed(sid, SPEED)

    # Confirm remembered j6 from live arm (informational)
    live = drv.get_all_positions()
    live_j6 = live.get(6)
    print(f"live j6={live_j6} (PICK_J6={PICK_J6} DROP_J6={DROP_J6})")

    win = "sort stages (q=abort)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    go_safe(drv)

    with CameraReader(copy=False) as cam:
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
    main()
