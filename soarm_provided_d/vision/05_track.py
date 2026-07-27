# -*- coding: utf-8 -*-
"""05_track.py — 공 추적 (크기→Z, 고도에서 팔 펴기, 경량 파이프라인).

- 검출은 축소 영상에서, 카메라는 copy=False + 최신프레임만
- 높은 Z: elbow/wrist(3·4) 가 최대한 펴진 IK 해를 고르고 각도로 직행
- 도달 밖 → SAFE_POSE
키: q=종료
"""
import os
import sys
import threading
import time

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm
from soarm_lab.driver_sdk import STS3215Driver

_VISION = os.path.dirname(os.path.abspath(__file__))
if _VISION not in sys.path:
    sys.path.insert(0, _VISION)

# HSV는 detect_objects 공유 (이중 빨강). 추적은 가장 큰 red blob.
from detect_objects import MIN_AREA, color_mask  # noqa: E402

BALL_R = 0.02
R_REF_PX = 23.0
Z_TABLE = BALL_R
Z_CLEAR = 0.06
Z_MIN, Z_MAX = 0.06, 0.28
Z_SCALE_GAIN = 1.0
Z_DOWN_THRESH = 0.13
Z_HIGH_THRESH = 0.16          # 이상이면 3·4번 관절을 펴는 해 선택
R_EMA_ALPHA = 0.30
# 03_map 이 저장한 크기→Z / 공중 관절 (있으면 덮어씀)
CALIB_JSON = os.path.join("data", "map_calib.json")
_SIZE_Z = None                # {"a","b"} for z=a+b/r
_AIR_SEEDS = []               # 공중 warm-start 시드들

MARGIN_XY = 0.012
MARGIN_Z = 0.018
SPEED = 800
ACC = 30
REAL = True
DETECT_SCALE = 0.5            # 검출 해상도(속도)
IK_ITERS = 70                 # 추적용 짧은 IK
SHOW_EVERY = 1                # 표시 주기(프레임)

SAFE_POSE = {1: 2047, 2: 813, 3: 3192, 4: 1039, 5: 1137, 6: 2325}
_SEED = (0, 30, -45, 0, 0)

# 높은 목표에서 elbow/wrist 를 펴기 위한 시드 (관절 3·4 ≈ 0 쪽)
_EXTEND_SEEDS = [
    [0, 25, 0, 0, 0],
    [0, 35, -15, 10, 0],
    [0, 40, -30, 20, 0],
    [0, 45, -45, 0, 0],
    [0, 15, 5, -10, 0],
    [0, 30, -10, 30, 0],
    list(_SEED),
]


def detect_red(bgr, scale=DETECT_SCALE):
    """축소 영상에서 검출 후 원좌표로 환산. ((u,v,r), area) 또는 (None, area)."""
    if scale != 1.0:
        small = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    else:
        small = bgr
    # color_mask 는 morph open 포함 — 추적도 공유 HSV 유지
    mask = color_mask(small, "red")
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = MIN_AREA * (scale * scale)
    if not cnts:
        return None, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None, float(area / (scale * scale))
    (u, v), r = cv2.minEnclosingCircle(c)
    inv = 1.0 / scale
    return (int(u * inv), int(v * inv), float(r * inv)), float(area / (scale * scale))


def load_map_calib(path=CALIB_JSON):
    """03_map 결과로 크기→Z·공중 관절 시드 로드."""
    global R_REF_PX, Z_HIGH_THRESH, _SIZE_Z, _AIR_SEEDS
    if not os.path.isfile(path):
        print(f"[track] {path} 없음 — 기본 크기→Z 사용")
        return
    import json
    with open(path, encoding="utf-8") as f:
        cal = json.load(f)
    sz = cal.get("size_z")
    if sz and sz.get("model") == "z=a+b/r":
        _SIZE_Z = {"a": float(sz["a"]), "b": float(sz["b"])}
        if sz.get("r_ref_px"):
            R_REF_PX = float(sz["r_ref_px"])
        print(f"[track] size_z: z={_SIZE_Z['a']:.4f}+{_SIZE_Z['b']:.4f}/r  "
              f"r_ref≈{R_REF_PX:.1f}")
    if cal.get("z_air_thresh_m") is not None:
        Z_HIGH_THRESH = float(cal["z_air_thresh_m"])
    seeds = []
    if cal.get("air_joints_mean"):
        seeds.append([float(x) for x in cal["air_joints_mean"]])
    for j in cal.get("air_joints") or []:
        if j and len(j) >= 5:
            seeds.append([float(x) for x in j[:5]])
    _AIR_SEEDS = seeds
    if seeds:
        print(f"[track] 공중 관절 시드 {len(seeds)}개 로드")


def estimate_z(r_px):
    r = max(float(r_px), 1.0)
    if _SIZE_Z is not None:
        z_ball = _SIZE_Z["a"] + _SIZE_Z["b"] / r
        z_ee = float(np.clip(z_ball + Z_CLEAR * 0.5, Z_MIN, Z_MAX))
        return z_ee, float(np.clip(z_ball, 0.0, Z_MAX))
    h_virt = 0.20 * Z_SCALE_GAIN
    z_ball = Z_TABLE + h_virt * (1.0 - (R_REF_PX / r))
    z_ee = z_ball + Z_CLEAR
    return float(np.clip(z_ee, Z_MIN, Z_MAX)), float(np.clip(z_ball, 0.0, Z_MAX))


def want_down(z_ee):
    return z_ee <= Z_DOWN_THRESH


def draw_hud(bgr, text, color, y=22):
    for c, tk in (((0, 0, 0), 3), (color, 1)):
        cv2.putText(bgr, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, tk)


def solve_track(xyz, down, prefer_extend):
    """추적용 IK. 높을 때는 03_map 공중 시드 + extend 시드 중 elbow·wrist 펴진 해."""
    xyz = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    if down:
        ang, err = arm.ik.solve(xyz, seed_deg=list(_SEED), down=True, iters=IK_ITERS)
        return ang, err

    seeds = []
    if prefer_extend and _AIR_SEEDS:
        seeds.extend(_AIR_SEEDS)
    seeds.extend(_EXTEND_SEEDS if prefer_extend else [list(_SEED), [0, -30, 45, 0, 0]])
    best_ang, best_err, best_score = None, np.inf, np.inf
    for seed in seeds:
        ang, err = arm.ik.solve(xyz, seed_deg=seed, down=False, iters=IK_ITERS)
        if err > arm.REACH_TOL:
            if err < best_err:
                best_ang, best_err = ang, err
            continue
        if prefer_extend:
            score = 4.0 * abs(float(ang[2])) + abs(float(ang[3]))
        else:
            score = err
        if err <= arm.REACH_TOL and (best_ang is None or score < best_score - 1e-6
                                      or (abs(score - best_score) < 1e-6 and err < best_err)):
            best_ang, best_err, best_score = ang, err, score
            if prefer_extend and abs(float(ang[2])) < 8 and err < 1e-3:
                break
        elif err < best_err:
            best_ang, best_err = ang, err
    return best_ang, best_err


class FrameGrabber(threading.Thread):
    """최신 프레임만 유지. copy=False 뷰를 잠금 하에 교체, get 때만 복사."""

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

    def get(self, copy=True):
        with self._lock:
            if self._rgb is None:
                return None, 0
            return (self._rgb.copy() if copy else self._rgb), self._fid

    def stop(self):
        self._stop.set()


class ArmWorker(threading.Thread):
    """각도 명령을 최신값만 실행 (IK 재계산 없음)."""

    def __init__(self, drv=None):
        super().__init__(daemon=True, name="arm-worker")
        self._drv = drv
        self._cv = threading.Condition()
        self._cmd = None  # ("angles", deg5) | ("safe", None)
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            with self._cv:
                while self._cmd is None and not self._stop.is_set():
                    self._cv.wait(timeout=0.05)
                if self._stop.is_set():
                    break
                cmd, payload = self._cmd
                self._cmd = None
            try:
                if cmd == "angles":
                    if self._drv is None:
                        continue
                    pos = {i + 1: STS3215Driver.degrees_to_position(float(payload[i]))
                           for i in range(5)}
                    # 그리퍼(6)는 SAFE 기본값 유지
                    pos[6] = SAFE_POSE[6]
                    self._drv.set_all_positions(pos)
                elif cmd == "safe":
                    self._go_safe()
            except Exception as e:
                print("팔 명령 오류:", e)

    def _go_safe(self):
        if not REAL or self._drv is None:
            return
        print("기본 자세(SAFE_POSE)로 이동")
        self._drv.set_all_positions(dict(SAFE_POSE))

    def request_angles(self, deg5):
        with self._cv:
            self._cmd = ("angles", list(deg5))
            self._cv.notify()

    def request_safe(self):
        with self._cv:
            if self._cmd and self._cmd[0] == "safe":
                return
            self._cmd = ("safe", None)
            self._cv.notify()

    def stop(self):
        self._stop.set()
        with self._cv:
            self._cv.notify()


def main():
    load_map_calib()
    H = np.load(os.path.join("data", "H.npy"))
    drv = None
    if REAL:
        drv = arm._backend(True).drv
        if not drv.ping(1):
            raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인 후 다시 실행.")
        for sid in (1, 2, 3, 4, 5):
            drv.set_torque(sid, True)
            drv.set_acceleration(sid, ACC)
            drv.set_speed(sid, SPEED)

    worker = ArmWorker(drv)
    worker.start()

    last_xyz = None
    r_ema = None
    in_safe = False
    last_shown_fid = -1
    frame_i = 0
    print(f"트래킹 (REAL={REAL}, size→Z, high→extend j3/j4). q 종료.")

    with CameraReader(copy=False) as cam:
        grabber = FrameGrabber(cam)
        grabber.start()
        try:
            while True:
                # 검출용은 복사본(스레드 경합 방지)
                rgb, fid = grabber.get(copy=True)
                if rgb is None or fid == last_shown_fid:
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    time.sleep(0.001)
                    continue
                last_shown_fid = fid
                frame_i += 1

                det, area = detect_red(rgb)
                if det:
                    u, v, r = det
                    r_ema = r if r_ema is None else (R_EMA_ALPHA * r + (1 - R_EMA_ALPHA) * r_ema)
                    z_ee, z_ball = estimate_z(r_ema)
                    down = want_down(z_ee)
                    high = z_ee >= Z_HIGH_THRESH

                    x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
                    xyz = [float(x), float(y), float(z_ee)]

                    if last_xyz is None:
                        moved = True
                    else:
                        dxy = ((xyz[0] - last_xyz[0]) ** 2 + (xyz[1] - last_xyz[1]) ** 2) ** 0.5
                        dz = abs(xyz[2] - last_xyz[2])
                        moved = dxy > MARGIN_XY or dz > MARGIN_Z

                    ang = None
                    err = 0.0
                    if moved:
                        ang, err = solve_track(xyz, down=down, prefer_extend=high)
                        tol = 0.12 if down else arm.REACH_TOL
                        if ang is None or err > tol:
                            if frame_i % SHOW_EVERY == 0:
                                draw_hud(rgb, f"OOR {err*1000:.0f}mm → SAFE", (0, 200, 255), y=70)
                            if not in_safe:
                                print(f"도달 밖 → SAFE ({err*1000:.0f}mm) {xyz}")
                                worker.request_safe()
                                in_safe = True
                                last_xyz = None
                        else:
                            worker.request_angles(ang)
                            last_xyz = xyz
                            in_safe = False

                    if frame_i % SHOW_EVERY == 0:
                        cv2.circle(rgb, (u, v), int(r), (0, 0, 255), 2)
                        e3 = abs(float(ang[2])) if ang is not None else float("nan")
                        e4 = abs(float(ang[3])) if ang is not None else float("nan")
                        draw_hud(rgb,
                                 f"r={r_ema:.1f} z={z_ee*1000:.0f}mm "
                                 f"{'EXT' if high else ('DN' if down else 'ok')} "
                                 f"|j3|={e3:.0f} |j4|={e4:.0f}",
                                 (0, 255, 0), y=22)
                else:
                    if frame_i % SHOW_EVERY == 0:
                        draw_hud(rgb,
                                 f"NO BALL: max {area:.0f}px" if area
                                 else "NO BALL: red 0",
                                 (0, 200, 255))

                if frame_i % SHOW_EVERY == 0:
                    # 표시만 축소 — imshow 비용↓
                    vis = cv2.resize(rgb, None, fx=0.75, fy=0.75,
                                     interpolation=cv2.INTER_AREA)
                    cv2.imshow("track (q=quit)", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            grabber.stop()
            grabber.join(timeout=1.0)
            worker.stop()
            worker.join(timeout=2.0)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
