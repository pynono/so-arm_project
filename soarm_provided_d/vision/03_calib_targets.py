# -*- coding: utf-8 -*-
"""03_calib_targets.py — 공/박스 분리 캘리브 (BALL 8점 + BOX 4점).

Phase 1 — BALL [i/8]
  빨간/파란 공을 책상에 넓게 두고 샘플 수집.
  픽셀(u,v,+r) 확정 → 팔끝 FK → findHomography → data/H.npy

Phase 2 — BOX [i/4]
  빨간 박스 2점 + 파란 박스 2점 (중심/모서리/place).
  픽셀↔로봇 연관을 JSON boxes 섹션에 별도 저장 (H 재계산 안 함).

노랑 제외. 공/박스는 면적 임계로 구분 (detect_objects 재사용).

저장:
  data/H.npy              — ball-8 호모그래피
  data/map_calib.json     — balls / boxes / size_z …
  data/targets_calib.json — 동일 payload (별칭)

키: 클릭 또는 c=픽셀 확정 · 팔끝 대고 c/Enter · Esc=취소
포트: /dev/ttyACM0
"""
from __future__ import annotations

import json
import os
import signal
import sys
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
from soarm_lab.driver_sdk import STS3215Driver
from soarm_lab.fk_core import FKSo101

_cv2_qt = os.path.join(os.path.dirname(cv2.__file__), "qt", "plugins")
if os.path.isdir(_cv2_qt):
    os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", _cv2_qt)

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
_VISION = Path(__file__).resolve().parent
if str(_VISION) not in sys.path:
    sys.path.insert(0, str(_VISION))

PORT = "/dev/ttyACM0"
OUT_H = os.path.join("data", "H.npy")
OUT_MAP = os.path.join("data", "map_calib.json")
OUT_TARGETS = os.path.join("data", "targets_calib.json")

N_BALL = 8
N_BOX = 4
Z_AIR_THRESH = 0.12

BOX_SLOTS = [
    ("red_box_1", "red", "빨간박스 #1 중심/모서리"),
    ("red_box_2", "red", "빨간박스 #2 place/모서리"),
    ("blue_box_1", "blue", "파란박스 #1 중심/모서리"),
    ("blue_box_2", "blue", "파란박스 #2 place/모서리"),
]

# Inline defaults (overridden if detect_objects present)
RED1_LO, RED1_HI = (0, 100, 80), (10, 255, 255)
RED2_LO, RED2_HI = (160, 100, 80), (179, 255, 255)
BLUE_LO, BLUE_HI = (95, 120, 60), (130, 255, 255)
MIN_AREA = 150
BOX_MIN_AREA = 2500
DRAW = {"red": (0, 0, 255), "blue": (255, 0, 0)}

_DETECT = None
try:
    import detect_objects as _DETECT  # type: ignore
except Exception:
    try:
        from vision import detect_objects as _DETECT  # type: ignore
    except Exception:
        _DETECT = None

if _DETECT is not None:
    for name in ("BOX_MIN_AREA", "MIN_AREA", "RED1_LO", "RED1_HI",
                 "RED2_LO", "RED2_HI", "BLUE_LO", "BLUE_HI", "DRAW"):
        if hasattr(_DETECT, name):
            globals()[name] = getattr(_DETECT, name)


def color_mask(bgr, color: str):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask = (cv2.inRange(hsv, RED1_LO, RED1_HI) |
                cv2.inRange(hsv, RED2_LO, RED2_HI))
    else:
        mask = cv2.inRange(hsv, BLUE_LO, BLUE_HI)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)


def _blob_from_contour(c, color: str):
    area = float(cv2.contourArea(c))
    if area < MIN_AREA:
        return None
    (u, v), r = cv2.minEnclosingCircle(c)
    x, y, w, h = cv2.boundingRect(c)
    kind = "box" if area >= BOX_MIN_AREA else "ball"
    return {
        "color": color,
        "kind": kind,
        "uv": (int(u), int(v)),
        "r": float(r),
        "area": area,
        "bbox": [int(x), int(y), int(w), int(h)],
    }


def detect_inline(bgr, want_kind=None, want_color=None):
    best, max_area = None, 0.0
    for color in ("red", "blue"):
        if want_color and color != want_color:
            continue
        cnts, _ = cv2.findContours(color_mask(bgr, color),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            blob = _blob_from_contour(c, color)
            if blob is None:
                continue
            max_area = max(max_area, blob["area"])
            if want_kind and blob["kind"] != want_kind:
                continue
            if best is None or blob["area"] > best["area"]:
                best = blob
    return best, max_area


def detect_best(bgr, want_kind=None, want_color=None):
    """Prefer detect_objects.detect_colored_objects; else inline."""
    if _DETECT is not None and hasattr(_DETECT, "detect_colored_objects"):
        objs = _DETECT.detect_colored_objects(bgr) or []
        max_area = 0.0
        best = None
        for o in objs:
            area = float(o.get("area", 0) or 0)
            max_area = max(max_area, area)
            color = o.get("color")
            kind = o.get("kind")
            if want_color and color != want_color:
                continue
            if want_kind and kind != want_kind:
                continue
            uv = o.get("uv")
            if uv is None and "u" in o:
                uv = (o["u"], o["v"])
            if uv is None:
                continue
            r = float(o.get("r", o.get("r_px", 0)) or 0)
            cand = {
                "color": color,
                "kind": kind,
                "uv": (int(uv[0]), int(uv[1])),
                "r": r,
                "area": area,
                "bbox": o.get("bbox"),
            }
            if best is None or area > best["area"]:
                best = cand
        return best, max_area
    return detect_inline(bgr, want_kind=want_kind, want_color=want_color)


def _hud(img, lines, color=(0, 255, 255)):
    y = 28
    for line in lines:
        for c_, tk in (((0, 0, 0), 3), (color, 1)):
            cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c_, tk)
        y += 28


def get_pixel(cam, phase, idx, total, want_kind, want_color=None, hint=""):
    tag = "BALL" if want_kind == "ball" else "BOX"
    # Avoid '/' and '[]' in Qt window titles (NULL window handler on some builds).
    win = f"calib {tag} {idx} of {total}"
    clicked = {}
    arm_at = time.monotonic() + 1.5
    last_det = None

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and time.monotonic() >= arm_at:
            clicked["uv"] = (x, y)

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.waitKey(1)
    try:
        cv2.resizeWindow(win, 960, 720)
    except cv2.error:
        pass
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            if time.monotonic() >= arm_at:
                try:
                    if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                        raise KeyboardInterrupt
                except cv2.error:
                    raise KeyboardInterrupt

            det, area = detect_best(rgb, want_kind=want_kind, want_color=want_color)
            if det is not None:
                last_det = det
                u, v = det["uv"]
                r = max(8, int(det["r"]))
                col = DRAW.get(det["color"], (0, 255, 255))
                if want_kind == "box" and det.get("bbox"):
                    x, y, w, h = det["bbox"]
                    cv2.rectangle(rgb, (x, y), (x + w, y + h), col, 2)
                else:
                    cv2.circle(rgb, (u, v), r, col, 2)
                cv2.drawMarker(rgb, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(
                    rgb,
                    f"{det['color']}_{det['kind']} r={det['r']:.0f} a={det['area']:.0f}",
                    (u - 70, v - r - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
                )
            else:
                need = want_color or "red/blue"
                msg = (f"NO {need} {want_kind}: maxArea={area:.0f} "
                       f"(ball<{BOX_MIN_AREA:.0f}<=box)")
                _hud(rgb, [msg], (0, 200, 255))

            _hud(rgb, [
                f"[{idx}/{total}] {tag}  phase={phase}",
                hint or f"place {want_kind} · click or c",
                "click / c=confirm · Esc=cancel",
            ])
            cv2.imshow(win, rgb)
            k = cv2.waitKey(1) & 0xFF
            if time.monotonic() < arm_at:
                continue
            if "uv" in clicked:
                u, v = clicked["uv"]
                r = float(last_det["r"]) if last_det else None
                a = float(last_det["area"]) if last_det else None
                bbox = last_det.get("bbox") if last_det else None
                color = last_det["color"] if last_det else (want_color or "unknown")
                print(f"  확정(클릭) ({u},{v}) r={r} color={color}")
                return u, v, r, a, bbox, color
            if k in (ord("c"), ord("C")) and last_det is not None:
                u, v = last_det["uv"]
                print(f"  확정(c) ({u},{v}) r={last_det['r']:.1f} "
                      f"{last_det['color']}_{last_det['kind']}")
                return (u, v, float(last_det["r"]), float(last_det["area"]),
                        last_det.get("bbox"), last_det["color"])
            if k == 27:
                raise KeyboardInterrupt
    finally:
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass


def robot_sample(drv, fk, prompt: str):
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, False)
    win = "calib: tip → c/Enter"
    img = np.zeros((160, 640, 3), dtype=np.uint8)
    cv2.putText(img, prompt[:58], (12, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    print(f"  {prompt}")
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
    return (float(p[0]), float(p[1]), float(p[2])), [float(d) for d in deg]


def fit_size_to_z(radii, zs):
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


def compute_H(samples):
    if len(samples) < 4:
        return None, None
    pixels = np.float32([s["uv"] for s in samples])
    robots = np.float32([[s["xyz"][0], s["xyz"][1]] for s in samples])
    H, _ = cv2.findHomography(pixels, robots, method=0)
    if H is None:
        return None, None
    worst = 0.0
    for s in samples:
        u, v = s["uv"]
        x, y = s["xyz"][0], s["xyz"][1]
        px, py = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
        worst = max(worst, float(((px - x) ** 2 + (py - y) ** 2) ** 0.5))
    return H, worst


def main():
    os.makedirs("data", exist_ok=True)
    src = "detect_objects.py" if _DETECT is not None else "inline dual-red HSV"
    print(f"검출 소스: {src}  BOX_MIN_AREA={BOX_MIN_AREA}  MIN_AREA={MIN_AREA}")
    print("=== Phase 1: BALL map 8점 (책상에 넓게) → H.npy ===")
    print("=== Phase 2: BOX map 4점 (빨강2 + 파랑2) → JSON boxes ===")
    print("키: 클릭/c=픽셀 · 팔끝 후 c/Enter · Esc=취소\n")

    drv = STS3215Driver(port=PORT)
    drv.connect()
    fk = FKSo101()
    ball_samples = []
    box_samples = []

    try:
        with CameraReader() as cam:
            for i in range(1, N_BALL + 1):
                print(f"[{i}/{N_BALL}] BALL — 공을 놓고 클릭/c 후 팔끝 티칭")
                u, v, r, area, bbox, color = get_pixel(
                    cam, "ball", i, N_BALL, want_kind="ball",
                    hint="spread balls on table for better H",
                )
                xyz, deg = robot_sample(
                    drv, fk, f"[{i}/{N_BALL}] BALL tip on ball → c/Enter")
                air = xyz[2] >= Z_AIR_THRESH
                print(f"  픽셀 ({u},{v}) {color} r={r} -> xyz "
                      f"({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f}) "
                      f"[{'AIR' if air else 'table'}]")
                ball_samples.append({
                    "uv": [u, v],
                    "r_px": None if r is None else float(r),
                    "area": area,
                    "bbox": bbox,
                    "color": color,
                    "kind": "ball",
                    "xyz": list(xyz),
                    "joints_deg": deg,
                    "air": bool(air),
                })

            print("\n=== Phase 1 완료 → Phase 2: BOX ===\n")

            for i, (label, color, tip) in enumerate(BOX_SLOTS, start=1):
                print(f"[{i}/{N_BOX}] BOX {label} — {tip}")
                u, v, r, area, bbox, det_color = get_pixel(
                    cam, "box", i, N_BOX, want_kind="box",
                    want_color=color, hint=tip,
                )
                xyz, deg = robot_sample(
                    drv, fk, f"[{i}/{N_BOX}] BOX tip on {label} → c/Enter")
                print(f"  픽셀 ({u},{v}) {det_color} a={area} -> xyz "
                      f"({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f})")
                box_samples.append({
                    "label": label,
                    "uv": [u, v],
                    "r_px": None if r is None else float(r),
                    "area": area,
                    "bbox": bbox,
                    "color": det_color or color,
                    "kind": "box",
                    "xyz": list(xyz),
                    "joints_deg": deg,
                })
    except KeyboardInterrupt:
        print("취소 — 저장하지 않음")
        return
    finally:
        cv2.destroyAllWindows()

    if len(ball_samples) < 4:
        raise SystemExit("공 샘플 4개 미만 — H 저장 안 함")

    H, worst = compute_H(ball_samples)
    if H is None:
        raise SystemExit("호모그래피 실패")
    print(f"H 재투영 최대오차 {worst * 1000:.1f}mm",
          "(양호)" if worst < 0.015 else "(큼 — 점을 더 넓게/고르게)")

    size_z = fit_size_to_z(
        [s["r_px"] for s in ball_samples],
        [s["xyz"][2] for s in ball_samples],
    )
    if size_z:
        print(f"크기→Z: z={size_z['a']:.4f}+{size_z['b']:.4f}/r  "
              f"RMSE={size_z['rmse_m']*1000:.1f}mm")

    air_joints = [s["joints_deg"] for s in ball_samples if s["air"]]
    air_mean = (np.mean(np.array(air_joints), axis=0).tolist()
                if air_joints else None)

    boxes_by_color = {"red": [], "blue": []}
    for s in box_samples:
        boxes_by_color.setdefault(s["color"], []).append(s)

    np.save(OUT_H, H)
    payload = {
        "n_ball": len(ball_samples),
        "n_box": len(box_samples),
        "z_air_thresh_m": Z_AIR_THRESH,
        "box_min_area": float(BOX_MIN_AREA),
        "min_area": float(MIN_AREA),
        "hsv": {
            "red1": [list(RED1_LO), list(RED1_HI)],
            "red2": [list(RED2_LO), list(RED2_HI)],
            "blue": [list(BLUE_LO), list(BLUE_HI)],
        },
        "balls": ball_samples,
        "boxes": box_samples,
        "boxes_by_color": boxes_by_color,
        "samples": ball_samples,
        "size_z": size_z,
        "air_joints_mean": air_mean,
        "air_joints": air_joints,
        "reproj_max_m": worst,
        "note": (
            "Phase1 ball-8 → H.npy; Phase2 box-4 별도 저장. "
            "노랑 제외. 공/박스는 area>=BOX_MIN_AREA 로 구분."
        ),
    }
    for path in (OUT_MAP, OUT_TARGETS):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        print("저장:", path)
    print("저장:", OUT_H)
    print(f"balls={len(ball_samples)}  boxes={len(box_samples)}  "
          f"labels={[s['label'] for s in box_samples]}")


if __name__ == "__main__":
    def _on_sigint(signum, frame):
        print(f"\n[signal] SIGINT({signum}) — 취소")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)
    main()
