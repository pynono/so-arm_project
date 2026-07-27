# -*- coding: utf-8 -*-
"""Shared red/blue HSV detection + ball vs box by contour area.

HSV defaults (classic red wrap + blue). Yellow is intentionally omitted for now.

BOX_MIN_AREA tuning (vision/shots/, 640×480):
  - balls: contour area ≈ 300–1300 px² (live_now≈332, shot_000≈1292)
  - larger red/blue blobs: ≥1910 px² (phone UI / hand+object / likely boxes)
  → BOX_MIN_AREA = 1600: area >= threshold → kind "box", else "ball".
    Re-tune if camera resolution or object distance changes.

손/피부 오검출 억제:
  - 빨강 S 하한을 조금 올림 (피부는 보통 채도가 낮음)
  - 윤곽 circularity / solidity / aspect 로 불규칙한 손 형태 제외
"""
from __future__ import annotations

from typing import Any

import cv2
import numpy as np

# Classic dual-range red + blue (user defaults; red Smin 100→120 피부 완화)
RED1_LO, RED1_HI = (0, 120, 80), (10, 255, 255)    # red wrap low
RED2_LO, RED2_HI = (160, 120, 80), (179, 255, 255)  # red wrap high
BLUE_LO, BLUE_HI = (95, 120, 60), (130, 255, 255)

MIN_AREA = 150
# Contour area >= this → box; else ball (see module docstring).
BOX_MIN_AREA = 1600

# Shape gates (hand rejection)
MIN_SOLIDITY = 0.82          # 손가락 틈 → convex hull 대비 면적↓
MIN_BALL_CIRCULARITY = 0.55  # 4πA/P² — 손은 보통 이보다 낮음
MIN_BOX_EXTENT = 0.40        # A/(w*h) — 박스는 bbox를 꽤 채움
MAX_ASPECT = 2.8             # 너무 길쭉하면 손/팔로 간주

DRAW = {"red": (0, 0, 255), "blue": (255, 0, 0)}
# Ball = thin circle + cyan label tip; box = thick rectangle + green accent
KIND_STYLE = {
    "ball": {"thickness": 2, "label_bgr": (0, 255, 255)},
    "box": {"thickness": 3, "label_bgr": (0, 255, 0)},
}

COLOR_RANGES = {
    "red": ((RED1_LO, RED1_HI), (RED2_LO, RED2_HI)),
    "blue": ((BLUE_LO, BLUE_HI),),
}


def color_mask(bgr: np.ndarray, color: str) -> np.ndarray:
    """HSV mask for named color (red = two ranges OR'd)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in COLOR_RANGES[color]:
        part = cv2.inRange(hsv, lo, hi)
        mask = part if mask is None else (mask | part)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)


def classify_kind(area: float) -> str:
    return "box" if area >= BOX_MIN_AREA else "ball"


def _shape_metrics(c, area: float):
    peri = float(cv2.arcLength(c, True))
    circ = (4.0 * np.pi * area / (peri * peri)) if peri > 1e-3 else 0.0
    hull = cv2.convexHull(c)
    hull_a = float(cv2.contourArea(hull))
    solid = (area / hull_a) if hull_a > 1e-3 else 0.0
    x, y, w, h = cv2.boundingRect(c)
    extent = (area / float(w * h)) if w > 0 and h > 0 else 0.0
    aspect = (w / float(h)) if h > 0 else 0.0
    if aspect < 1.0 and aspect > 0:
        aspect = 1.0 / aspect
    return circ, solid, extent, aspect, (int(x), int(y), int(w), int(h))


def accept_contour(c, area: float, kind: str) -> bool:
    """손·팔 등 불규칙 윤곽 제외. kind는 area로 미리 잡은 ball/box."""
    circ, solid, extent, aspect, _ = _shape_metrics(c, area)
    if solid < MIN_SOLIDITY:
        return False
    if aspect > MAX_ASPECT:
        return False
    if kind == "ball":
        return circ >= MIN_BALL_CIRCULARITY
    # box: 사각형에 가깝게 bbox를 채울 것 (손은 extent·solidity 둘 다 낮은 편)
    return extent >= MIN_BOX_EXTENT and solid >= MIN_SOLIDITY


def detect_colored_objects(bgr: np.ndarray) -> list[dict[str, Any]]:
    """Detect red/blue blobs; classify each as ball or box by area.

    Returns list of dicts:
      color: "red"|"blue"
      kind:  "ball"|"box"
      uv:    (u, v) pixel center
      r:     min enclosing circle radius (px)
      area:  contour area (px²)
      bbox:  (x, y, w, h)
    """
    out: list[dict[str, Any]] = []
    for color in ("red", "blue"):
        mask = color_mask(bgr, color)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = float(cv2.contourArea(c))
            if area < MIN_AREA:
                continue
            kind = classify_kind(area)
            if not accept_contour(c, area, kind):
                continue
            (u, v), r = cv2.minEnclosingCircle(c)
            x, y, w, h = cv2.boundingRect(c)
            out.append({
                "color": color,
                "kind": kind,
                "uv": (int(u), int(v)),
                "r": float(r),
                "area": area,
                "bbox": (int(x), int(y), int(w), int(h)),
            })
    out.sort(key=lambda o: o["area"], reverse=True)
    return out


def annotate_objects(bgr: np.ndarray, objects: list[dict[str, Any]] | None = None) -> np.ndarray:
    """Draw ball (circle) vs box (rectangle) with color_kind labels."""
    if objects is None:
        objects = detect_colored_objects(bgr)
    for obj in objects:
        color, kind = obj["color"], obj["kind"]
        u, v = obj["uv"]
        r = int(round(obj["r"]))
        x, y, w, h = obj["bbox"]
        col = DRAW[color]
        thick = KIND_STYLE[kind]["thickness"]
        label = f"{color}_{kind}"
        if kind == "box":
            cv2.rectangle(bgr, (x, y), (x + w, y + h), col, thick)
        else:
            cv2.circle(bgr, (u, v), max(r, 1), col, thick)
        cv2.drawMarker(bgr, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(
            bgr, label, (u - 40, max(18, v - r - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2,
        )
    return bgr
