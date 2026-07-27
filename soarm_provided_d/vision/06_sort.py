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

Z_HOVER = 0.13          # default mid; pick uses distance-adaptive hover
Z_HOVER_NEAR = 0.11     # near reach: lower hover (less stretch)
Z_HOVER_FAR = 0.17      # far reach: higher hover so j2/j3/j4 stay softer
Z_PICK = 0.04
Z_TEST_LIFT = Z_PICK + 0.025  # ~2.5cm test lift after close (verify grasp)
Z_CARRY = 0.14
Z_PLACE = 0.06
# Radial reach (m) for near↔far blending of hover / seed / soft-down
R_NEAR = 0.16
R_FAR = 0.32
DOWN_TILT_FAR_DEG = 25.0  # soft-down outward tilt at R_FAR (0 at R_NEAR)
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
# Place/drop wrist: hold current live j5 (raw from /dev/ttyACM0) — do not use IK j5
DROP_J5 = 1019          # live present position id=5 (~-90.5°)
PICK_APPROACH_SECS = 5.0

# Box approach: slow carry to box hover; grip stays CLOSED until PLACE opens
BOX_APPROACH_SECS = 5.0

# Grip: open = J6_OPEN; close for grasp
GRIP_OPEN = J6_OPEN
GRIP_CLOSE = 1750
GRIP_LOAD_THRESH = 100       # get_load(6) >= this → grasped (signed, no abs)
GRIP_LOAD_SAMPLES = 5        # samples over ~1s after test lift
GRIP_LOAD_SAMPLE_DT = 0.2    # seconds between load samples (5 × 0.2 = 1.0s)
GRIP_LOAD_MAJORITY = 3       # need >= this many samples with load >= thresh
GRIP_MAX_RETRIES = 3         # full pick cycles: SAFE→redetect→MOVE→GRIP; then re-SCAN
GRIP_CLOSE_SETTLE_SECS = 1.35  # wait until close complete
GRIP_POST_CLOSE_STABILIZE = 0.35  # short settle after close confirm, before test lift
GRIP_DOWN_SECS = 3.0           # hover → pick z
GRIP_TEST_LIFT_SECS = 0.8      # pick → test lift (~2.5cm) before load verify
GRIP_LIFT_SECS = 1.9           # test → carry after grasp OK
JOINT_ARRIVE_TOL = 40          # raw units: present vs goal
JOINT_ARRIVE_POLL = 0.05
JOINT_ARRIVE_EXTRA = 2.0       # extra seconds beyond motion secs for arrive poll
REDETECT_SETTLE_STD = 0.008    # max xy std (m) for settle OK
MEASURE_LOAD_N = 30            # samples per --measure-load scenario
MEASURE_LOAD_DT = 0.1

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


class RedetectFailure(GripFailure):
    """Recoverable: ball re-detect timed out / did not settle.

    Treated like GripFailure so callers go SAFE → retry / re-SCAN.
    """


class FrameGrabber(threading.Thread):
    """Latest-frame grabber. Always stores a copy so SHM mmap can close cleanly."""

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
                # Copy even if CameraReader already copied — never hold SHM views.
                frame = rgb.copy()
                with self._lock:
                    self._rgb = frame
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
        with self._lock:
            self._rgb = None  # drop numpy refs before CameraReader.close()


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
    seed=_SEED,
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


def set_grip(drv, raw: int, secs=0.45, wait_arrive: bool = False, label: str = ""):
    """Close/open gripper by writing joint 6 only — other joints stay put."""
    goal = int(raw)
    drv.set_position(6, goal)
    time.sleep(secs)
    if wait_arrive:
        wait_joints_arrived(
            drv, {6: goal}, timeout=JOINT_ARRIVE_EXTRA,
            label=label or f"j6={goal}",
        )


def _xy_std(buf) -> tuple[float, float, float]:
    """Return (std_x, std_y, max_std) for xy sample buffer; zeros if empty."""
    if not buf:
        return 0.0, 0.0, 0.0
    arr = np.array(buf, dtype=float)
    sx, sy = float(np.std(arr[:, 0])), float(np.std(arr[:, 1]))
    return sx, sy, max(sx, sy)


def _load_stats(samples: list) -> dict:
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
        st = _load_stats(samples)
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


def go_safe(drv):
    print("[SAFE]")
    drv.set_all_positions(dict(SAFE_POSE))
    time.sleep(1.0)


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
        time.sleep(0.5)
        return plan

    raise TimeoutError(f"{stage} 타임아웃 — 전체 검출 실패")


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
        _pump(rgb, win, stage, [
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
        _pump(rgb, win, stage, [
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


def stage_redetect_ball(grabber, H, win: str, color: str, timeout=REDETECT_TIMEOUT):
    """Quick re-detect of one color ball; settle xy and return (x, y).

    On timeout: log hit/buffer/xy-std diagnostics and raise RedetectFailure
    (recoverable — SAFE / retry / re-SCAN), not a hard crash.
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
            sx, sy, smax = _xy_std(buf)
            _pump(rgb, win, stage, [
                f"{color} ball: looking… hit=0 buf={len(buf)} "
                f"std=({sx:.4f},{sy:.4f})",
                "q=abort",
            ])
            continue
        last_o = o
        hit += 1
        if hit >= DETECT_HOLD:
            u, v = o["uv"]
            buf.append(pixel_to_xy(u, v, H))
            if len(buf) > SETTLE_N:
                buf.pop(0)
        n = len(buf)
        sx, sy, smax = _xy_std(buf)
        _pump(rgb, win, stage, [
            f"{color} ball: hit={hit}/{DETECT_HOLD} buf={n}/{SETTLE_N} "
            f"std=({sx:.4f},{sy:.4f})",
            "q=abort",
        ])
        if n >= SETTLE_N and smax < REDETECT_SETTLE_STD:
            mean = np.array(buf).mean(axis=0)
            xy = (float(mean[0]), float(mean[1]))
            print(
                f"[{color.upper()}] re-detect ball before pick → "
                f"({xy[0]:+.3f},{xy[1]:+.3f}) "
                f"(hit={hit} buf={n} std=({sx:.4f},{sy:.4f}))"
            )
            return xy

    sx, sy, smax = _xy_std(buf)
    diag = (
        f"hit={hit} buf={len(buf)}/{SETTLE_N} "
        f"xy_std=({sx:.4f},{sy:.4f}) max_std={smax:.4f} "
        f"need_std<{REDETECT_SETTLE_STD}"
    )
    if last_o:
        diag += f" last_uv={last_o['uv']}"
    print(f"[{stage}] TIMEOUT recoverable — {diag}")
    raise RedetectFailure(
        color,
        f"{stage} timeout — {color} ball not settled ({diag})",
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
        _pump(rgb, win, stage, [
            "lower(closed)→open(DROP_J6)→lift",
            f"xy=({x:+.3f},{y:+.3f}) z={Z_PLACE:.3f}",
        ])
    # Keep closed while lowering; open only after Z_PLACE motion has finished
    go_xyz(
        drv, [x, y, Z_PLACE], down=True, secs=0.9,
        j5=DROP_J5, j6=GRIP_CLOSE,
    )
    print(f"  [{stage}] lowered — opening DROP_J6={DROP_J6}")
    set_grip(drv, DROP_J6, 0.45)
    go_xyz(
        drv, [x, y, Z_HOVER], down=False, secs=0.7,
        j5=DROP_J5, j6=DROP_J6,
    )
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
            kind = "REDETECT" if isinstance(e, RedetectFailure) else "GRIP"
            print(
                f"\n[RECOVER] {e.color.upper()} {kind} failed after "
                f"{GRIP_MAX_RETRIES} SAFE/redetect pick cycles — {e}"
            )
            print(f"  completed: {sorted(done) or 'none'}")
            print(f"  → SAFE → re-SCAN → resume from remaining colors")
            go_safe(drv)
            # loop: re-scan and only act on colors not in done


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
