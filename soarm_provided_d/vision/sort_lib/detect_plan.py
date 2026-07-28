# -*- coding: utf-8 -*-
"""Camera grabber, overlay helpers, SCAN ALL / redetect planning."""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np
from detect_objects import annotate_objects, detect_colored_objects
from hp60c_camera import CameraReader

from .constants import (
    DETECT_HOLD,
    REDETECT_SETTLE_STD,
    REDETECT_TIMEOUT,
    RedetectFailure,
    SCAN_KEYS,
    SCAN_TIMEOUT,
    SETTLE_N,
)


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


def xy_std(buf) -> tuple[float, float, float]:
    """Return (std_x, std_y, max_std) for xy sample buffer; zeros if empty."""
    if not buf:
        return 0.0, 0.0, 0.0
    arr = np.array(buf, dtype=float)
    sx, sy = float(np.std(arr[:, 0])), float(np.std(arr[:, 1]))
    return sx, sy, max(sx, sy)


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


def pump(rgb, win, stage, lines):
    draw_status(rgb, stage, lines)
    cv2.imshow(win, rgb)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        raise KeyboardInterrupt


def best_of(objs, color: str, kind: str):
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
    hit = {k: 0 for k in SCAN_KEYS}
    buf: dict[tuple, list] = {k: [] for k in SCAN_KEYS}
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

        for key in SCAN_KEYS:
            color, kind = key
            o = best_of(objs, color, kind)
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
        for color, kind in SCAN_KEYS:
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
        pump(rgb, win, stage, lines)

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
        pump(rgb, win, stage, [
            "ALL CACHED",
            f"red ball→box",
            f"blue ball→box",
            "starting ACT…",
        ])
        time.sleep(0.3)
        return plan

    raise TimeoutError(f"{stage} 타임아웃 — 전체 검출 실패")


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
        o = best_of(objs, color, "ball")
        if o is None:
            hit = 0
            sx, sy, smax = xy_std(buf)
            pump(rgb, win, stage, [
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
        sx, sy, smax = xy_std(buf)
        pump(rgb, win, stage, [
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

    sx, sy, smax = xy_std(buf)
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
