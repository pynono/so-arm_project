# -*- coding: utf-8 -*-
"""hsv_tuner.py — HSV 트랙바로 공 색 범위(숫자 6개)를 뽑는 도구 (정답지).

슬라이더로 H/S/V 의 최소·최대를 움직이며 마스크(흰=선택된 색)를 실시간으로 본다.
공만 하얗게 남는 값을 찾으면 그 6개 숫자가 결과물 → 02_detect 등의 임계값에 넣는다.
빨강은 색상환 양끝(0·180)에 걸쳐서, Hmin > Hmax 로 두면 양끝을 자동으로 OR 한다.
  예) 빨강: Hmin=160, Hmax=10 → [160~179] + [0~10] 을 함께 잡음.

카메라:        python hsv_tuner.py
저장 사진으로: python hsv_tuner.py shots/shot_000.png
키: p=현재 값 출력 · q=종료
"""
import sys

import cv2

WIN = "hsv tuner (p=print, q=quit)"
BARS = [("Hmin", 0, 179), ("Hmax", 10, 179),
        ("Smin", 100, 255), ("Smax", 255, 255),
        ("Vmin", 80, 255), ("Vmax", 255, 255)]


def _noop(_):
    pass


def make_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    p = {n: cv2.getTrackbarPos(n, WIN) for n, _, _ in BARS}
    lo = (p["Hmin"], p["Smin"], p["Vmin"])
    hi = (p["Hmax"], p["Smax"], p["Vmax"])
    if lo[0] <= hi[0]:                               # 보통 범위
        mask = cv2.inRange(hsv, lo, hi)
    else:                                            # 빨강처럼 색상환 양끝을 감싸는 경우
        mask = (cv2.inRange(hsv, (lo[0], lo[1], lo[2]), (179, hi[1], hi[2])) |
                cv2.inRange(hsv, (0, lo[1], lo[2]), (hi[0], hi[1], hi[2])))
    return mask, lo, hi


def main():
    src_img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    cv2.namedWindow(WIN)
    for name, val, mx in BARS:
        cv2.createTrackbar(name, WIN, val, mx, _noop)

    cam = None
    if src_img is None:
        from hp60c_camera import CameraReader
        cam = CameraReader()
    last = 0
    print("슬라이더로 공만 하얗게 남기세요. p=값 출력, q=종료")
    while True:
        if src_img is not None:
            bgr = src_img.copy()
        else:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            bgr = rgb
        mask, lo, hi = make_mask(bgr)
        masked = cv2.bitwise_and(bgr, bgr, mask=mask)
        cv2.putText(masked, f"LO={lo}  HI={hi}", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(WIN, cv2.hconcat([bgr, masked]))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("p"):
            print(f"LO={lo}  HI={hi}")
        elif k == ord("q"):
            break
    if cam is not None:
        cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
