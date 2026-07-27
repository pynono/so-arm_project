# -*- coding: utf-8 -*-
import sys
import cv2
"""02_detect.py — HSV로 빨강/파랑 공 검출 → 공의 픽셀(u,v).

카메라:        python 02_detect.py
저장 사진으로: python 02_detect.py shots/shot_000.png ...   (카메라 없이 개발)
키: q=종료

RGB는 조명이 바뀌면 세 값이 다 흔들리지만, HSV는 색상(H)이 색을 조명에 강하게
분리해준다. 빨강은 색상환 0·180 양끝이라 Hmin>Hmax 로 감싸면 두 구간을 합친다(OR).
"""

# ── TODO ①: 빨강 HSV 임계값 ───────────────────────────────────────────────
# 일반적인 빨간색 공의 HSV 범위를 적용했습니다.
# 만약 환경(조명)에 따라 잘 안 잡힌다면 hsv_tuner.py에서 찾은 값으로 수정하세요.
RED_LO = (0, 128, 119)
RED_HI = (179, 224, 255)
# ──────────────────────────────────────────────────────────────────────────

# 파랑은 제공값 — 시간이 남으면 hsv_tuner 로 직접 다시 구해 바꿔 본다(심화)
BLUE_LO, BLUE_HI = (95, 120, 60), (130, 255, 255)

MIN_AREA = 150
DRAW = {"red": (0, 0, 255), "blue": (255, 0, 0)}


def color_mask(bgr, lo, hi):
    """HSV 범위 [lo, hi] → 흑백 마스크(공=흰색).
    Hmin>Hmax 면 색상환 양끝을 감싼 것으로 보고 [lo~179] + [0~hi] 를 합친다(빨강용)."""
    # ── TODO ②: BGR → HSV 변환 ────────────────────────────────────────────
    # 카메라 이미지(BGR)를 HSV 색공간으로 변환합니다.
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # ──────────────────────────────────────────────────────────────────────
    if hsv is None:
        raise SystemExit("TODO ② 먼저: color_mask() 의 BGR→HSV 변환을 구현하세요.")

    if lo[0] <= hi[0]:                               # 보통 범위
        mask = cv2.inRange(hsv, lo, hi)
    else:                                            # 양끝을 감싸는 범위(빨강)
        mask = (cv2.inRange(hsv, (lo[0], lo[1], lo[2]), (179, hi[1], hi[2])) |
                cv2.inRange(hsv, (0, lo[1], lo[2]), (hi[0], hi[1], hi[2])))

    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)      # 잔점 제거


def detect_ball(bgr, lo, hi):
    """색 마스크에서 가장 큰 덩어리 → 중심 픽셀(u,v)·반지름 r. 없으면 None."""
    cnts, _ = cv2.findContours(color_mask(bgr, lo, hi),
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    (u, v), r = cv2.minEnclosingCircle(c)
    return int(u), int(v), int(r)


def annotate(bgr):
    for color, lo, hi in (("red", RED_LO, RED_HI), ("blue", BLUE_LO, BLUE_HI)):
        det = detect_ball(bgr, lo, hi)
        if det:
            u, v, r = det
            cv2.circle(bgr, (u, v), r, DRAW[color], 2)
            cv2.drawMarker(bgr, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(bgr, color, (u - r, v - r - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, DRAW[color], 2)
    return bgr


def main():
    if RED_LO is None or RED_HI is None:
        raise SystemExit(
            "TODO ① 먼저: hsv_tuner.py 로 구한 값을 파일 상단 RED_LO / RED_HI 에 채우세요.")

    if len(sys.argv) > 1:                       # 저장 사진으로(카메라 없이)
        for path in sys.argv[1:]:
            img = cv2.imread(path)
            if img is None:
                print("못 읽음:", path)
                continue
            cv2.imshow(path, annotate(img))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    from hp60c_camera import CameraReader       # 카메라 모드
    print("빨강/파랑 공 검출 — q 종료")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            cv2.imshow("detect (q=quit)", annotate(rgb))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
