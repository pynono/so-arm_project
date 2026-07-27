# -*- coding: utf-8 -*-
"""03_map.py — 8점 캘리브: 픽셀↔로봇 XY + 크기→높이 + 관절각.

각 샘플에서
  1) 공 픽셀 (u,v) 와 반지름 r  (검출 c / 클릭)
  2) 팔끝을 공에 대고 FK → (x,y,z) 와 관절각 5개
를 모은다.

저장:
  data/H.npy          — 픽셀→로봇 XY 호모그래피 (findHomography, N≥4)
  data/map_calib.json — 샘플·크기→Z 피팅·공중(고도) 관절각 참고값
                        (하늘에 뜬 공용 각도는 여기서 채워 05_track 이 사용)

포트 /dev/ttyACM0 단독.
키: 클릭 또는 c=확정 · Esc=취소
"""
import json
import os
import signal
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false")

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab.driver_sdk import STS3215Driver
from soarm_lab.fk_core import FKSo101

_VISION = os.path.dirname(os.path.abspath(__file__))
if _VISION not in sys.path:
    sys.path.insert(0, _VISION)

_cv2_qt = os.path.join(os.path.dirname(cv2.__file__), "qt", "plugins")
if os.path.isdir(_cv2_qt):
    os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", _cv2_qt)

N = 8
PORT = "/dev/ttyACM0"
OUT_H = os.path.join("data", "H.npy")
OUT_CALIB = os.path.join("data", "map_calib.json")

# HSV·MIN_AREA 는 detect_objects 공유 (핑크 단일구간 → 이중 빨강)
from detect_objects import MIN_AREA, color_mask  # noqa: E402

Z_AIR_THRESH = 0.12   # 이 높이(m) 이상 샘플은 '공중' — 관절각을 air_joints 로 보관


def detect_red(bgr):
    """가장 큰 빨간 blob (ball/box 구분 없이 캘리브용)."""
    mask = color_mask(bgr, "red")
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, area
    (u, v), r = cv2.minEnclosingCircle(c)
    return (int(u), int(v), float(r)), area


def get_pixel(cam):
    """(u, v, r_px). 클릭 확정이면 그 순간의 검출 r 사용(없으면 None)."""
    win = "map: click or c=confirm / Esc=cancel"
    clicked = {}
    arm_keys_at = time.monotonic() + 2.0
    last_det = None

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and time.monotonic() >= arm_keys_at:
            clicked["uv"] = (x, y)

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 720)
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            if time.monotonic() >= arm_keys_at:
                try:
                    if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                        print("창 닫힘 — 취소")
                        raise KeyboardInterrupt
                except cv2.error:
                    raise KeyboardInterrupt
            det, area = detect_red(rgb)
            if det is not None:
                last_det = det
                u, v, r = det
                cv2.circle(rgb, (u, v), int(r), (0, 0, 255), 2)
                cv2.drawMarker(rgb, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(rgb, f"r={r:.1f}px  click/c", (u - 70, v - int(r) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                msg = (f"NO BALL: max blob {area:.0f}px < MIN_AREA {MIN_AREA}"
                       if area else "NO BALL: red pixels 0")
                for c_, tk in (((0, 0, 0), 3), ((0, 200, 255), 1)):
                    cv2.putText(rgb, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_, tk)
            cv2.imshow(win, rgb)
            k = cv2.waitKey(1) & 0xFF
            if time.monotonic() < arm_keys_at:
                continue
            if "uv" in clicked:
                u, v = clicked["uv"]
                r = float(last_det[2]) if last_det is not None else None
                print(f"  확정(클릭) ({u},{v}) r={r}")
                return (u, v, r)
            if k in (ord("c"), ord("C")) and det is not None:
                print(f"  확정(c) ({det[0]},{det[1]}) r={det[2]:.1f}")
                return (det[0], det[1], float(det[2]))
            if k == 27:
                print("Esc — 취소")
                raise KeyboardInterrupt
    finally:
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass


def robot_sample(drv, fk):
    """팔끝 FK (x,y,z) + 관절각(도)5."""
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, False)
    win = "map: tip on ball → c/Enter"
    img = np.zeros((160, 560, 3), dtype=np.uint8)
    cv2.putText(img, "Tip on ball (table or air), then c/Enter", (12, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    print("  팔끝을 공에 대고 창에서 c/Enter > ")
    cv2.namedWindow(win)
    try:
        while True:
            cv2.imshow(win, img)
            k = cv2.waitKey(50) & 0xFF
            if k in (13, 10, ord("c"), ord("C")):
                break
            if k == 27:
                raise KeyboardInterrupt
    finally:
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass
    pos = drv.get_all_positions()
    if any(pos.get(i + 1) is None for i in range(5)):
        raise SystemExit("서보 응답 없음 — 로봇 전원/케이블 확인.")
    deg = [STS3215Driver.position_to_degrees(pos.get(i + 1)) or 0.0 for i in range(5)]
    p, _ = fk.fk_deg(deg)
    xyz = (float(p[0]), float(p[1]), float(p[2]))
    return xyz, [float(d) for d in deg]


def fit_size_to_z(radii, zs):
    """z ≈ a + b / r  (큰 공 → 작은 1/r → 보통 더 높은 Z).

    유효 (r,z) 쌍으로 최소자승. 실패 시 None.
    """
    rs, zz = [], []
    for r, z in zip(radii, zs):
        if r is None or r < 1.0:
            continue
        rs.append(1.0 / float(r))
        zz.append(float(z))
    if len(rs) < 2:
        return None
    A = np.column_stack([np.ones(len(rs)), np.array(rs)])
    coef, _, _, _ = np.linalg.lstsq(A, np.array(zz), rcond=None)
    a, b = float(coef[0]), float(coef[1])
    pred = a + b * np.array(rs)
    rmse = float(np.sqrt(np.mean((pred - np.array(zz)) ** 2)))
    return {"model": "z=a+b/r", "a": a, "b": b, "rmse_m": rmse,
            "n": len(rs), "r_ref_px": float(np.median([1 / x for x in rs]))}


def main():
    drv = STS3215Driver(port=PORT)
    drv.connect()
    fk = FKSo101()
    samples = []
    try:
        with CameraReader() as cam:
            for i in range(N):
                print(f"[{i + 1}/{N}] 공 위치(책상/공중) 놓고 클릭 또는 c. "
                      f"공중이면 팔을 그에 맞게 대고 확정.")
                u, v, r = get_pixel(cam)
                xyz, deg = robot_sample(drv, fk)
                air = xyz[2] >= Z_AIR_THRESH
                print(f"  픽셀 ({u},{v}) r={r} -> xyz ({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f})"
                      f"  joints={np.round(deg,1).tolist()}  [{'AIR' if air else 'table'}]")
                samples.append({
                    "uv": [u, v],
                    "r_px": None if r is None else float(r),
                    "xyz": list(xyz),
                    "joints_deg": deg,
                    "air": bool(air),
                })
    except KeyboardInterrupt:
        print("취소 — 저장하지 않음")
        return
    finally:
        cv2.destroyAllWindows()

    if len(samples) < 4:
        raise SystemExit("샘플 4개 미만 — 저장 안 함")

    pixels = np.float32([s["uv"] for s in samples])
    robots_xy = np.float32([[s["xyz"][0], s["xyz"][1]] for s in samples])
    H, mask = cv2.findHomography(pixels, robots_xy, method=0)  # 최소자승
    if H is None:
        raise SystemExit("호모그래피 실패")

    worst = 0.0
    for s in samples:
        u, v = s["uv"]
        x, y = s["xyz"][0], s["xyz"][1]
        px, py = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
        worst = max(worst, float(((px - x) ** 2 + (py - y) ** 2) ** 0.5))
    print(f"재투영 최대오차 {worst * 1000:.1f}mm",
          "(양호)" if worst < 0.015 else "(큼 — 점을 더 넓게/고르게)")

    size_z = fit_size_to_z(
        [s["r_px"] for s in samples],
        [s["xyz"][2] for s in samples],
    )
    if size_z:
        print(f"크기→Z: z={size_z['a']:.4f}+{size_z['b']:.4f}/r  "
              f"RMSE={size_z['rmse_m']*1000:.1f}mm  (n={size_z['n']})")
    else:
        print("크기→Z 피팅 실패 — r 유효 샘플 부족")

    air_joints = [s["joints_deg"] for s in samples if s["air"]]
    table_joints = [s["joints_deg"] for s in samples if not s["air"]]
    print(f"공중 샘플 {len(air_joints)}개 / 테이블 {len(table_joints)}개")
    if air_joints:
        # 공중용 기본 시드: 평균 관절각 (05_track 이 고도일 때 warm-start)
        air_mean = np.mean(np.array(air_joints), axis=0).tolist()
        print("  air_joints_mean:", np.round(air_mean, 1).tolist())
    else:
        air_mean = None
        print("  (공중 샘플 없음 — 나중에 공 들고 재측정하거나 "
              "map_calib.json 의 air_joints_mean 을 수동 기입)")

    os.makedirs("data", exist_ok=True)
    np.save(OUT_H, H)
    calib = {
        "n": len(samples),
        "z_air_thresh_m": Z_AIR_THRESH,
        "samples": samples,
        "size_z": size_z,
        "air_joints_mean": air_mean,
        "air_joints": air_joints,          # 개별 공중 자세 (수동 편집 가능)
        "reproj_max_m": worst,
        "note": "공중 각도는 air_joints / air_joints_mean 을 편집하거나 "
                "공중 샘플을 넣어 다시 03_map 을 돌리면 됨.",
    }
    with open(OUT_CALIB, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2)
    print("저장:", OUT_H)
    print("저장:", OUT_CALIB)


if __name__ == "__main__":
    def _on_sigint(signum, frame):
        print(f"\n[signal] SIGINT({signum}) — 취소")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)
    main()
