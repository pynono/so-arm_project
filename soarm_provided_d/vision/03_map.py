# -*- coding: utf-8 -*-
"""03_map.py — 4점 호모그래피: 픽셀 <-> 로봇 좌표.

작업면 네 곳에 빨간 공을 두고, 각 지점에서 카메라가 자동검출한 공 픽셀과 로봇 xy 를
짝지어 모은다. 픽셀은 빨간공을 HSV로 자동검출해 화면에 표시하고 Space 로 확정한다
(검출이 안 되면 마우스 클릭으로 대체, q=취소). 로봇 xy 는 토크를 끄고 팔끝을 공에
댄 뒤 관절각을 FK 로 읽어 얻는다. 결과 H 는 data/H.npy 로 저장되어 04_click_move /
05_track 이 불러 쓴다.
포트 /dev/ttyACM0 단독(컨트롤러 앱·다른 프로세스는 종료).
"""
import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab.driver_sdk import STS3215Driver
from soarm_lab.fk_core import FKSo101

N = 4
PORT = "/dev/ttyACM0"
OUT = os.path.join("data", "H.npy")

# 빨간공 HSV 검출용(02_detect / 05_track 과 동일 임계값; 자체완결 위해 인라인)
RED1_LO, RED1_HI = (0, 128, 119), (179, 224, 255)
RED2_LO, RED2_HI = (160, 100, 80), (179, 255, 255)
MIN_AREA = 150


def detect_red(bgr):
    """빨간공 → ((u,v,r), 최대덩어리 면적). 못 찾으면 (None, 면적).
    면적을 같이 주는 이유: 못 찾았을 때 '왜'를 화면에 띄우기 위해서."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, RED1_LO, RED1_HI) | cv2.inRange(
        hsv, RED2_LO, RED2_HI)
    cnts, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, area
    (u, v), r = cv2.minEnclosingCircle(c)
    return (int(u), int(v), int(r)), area


def get_pixel(cam):
    """빨간공을 자동검출해 표시하고 Space 로 그 위치 확정. 검출 없으면 클릭. q=취소."""
    win = "map: Space=auto / click=manual / q=cancel"
    clicked = {}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["uv"] = (x, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            det, area = detect_red(rgb)
            if det is not None:
                u, v, r = det
                cv2.circle(rgb, (u, v), r, (0, 0, 255), 2)      # 검출된 실제 반지름
                cv2.drawMarker(rgb, (u, v), (0, 255, 255),
                               cv2.MARKER_CROSS, 16, 2)
                cv2.putText(rgb, "Space=confirm", (u - 45, v - r - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:   # 왜 못 찾았는지 표시 — 조용히 Space 가 안 먹는 상황 방지
                msg = (f"NO BALL: max blob {area:.0f}px < MIN_AREA {MIN_AREA}"
                       if area else "NO BALL: red pixels 0")
                for c_, tk in (((0, 0, 0), 3), ((0, 200, 255), 1)):
                    cv2.putText(rgb, msg, (8, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_, tk)
            cv2.imshow(win, rgb)
            k = cv2.waitKey(1) & 0xFF
            if "uv" in clicked:                       # 수동: 클릭한 픽셀
                return clicked["uv"]
            if k == ord(" ") and det is not None:     # 자동: 검출된 픽셀 확정
                return det[:2]
            if k == ord("q"):
                raise KeyboardInterrupt
    finally:
        cv2.destroyWindow(win)


def robot_xy(drv, fk):
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, False)
    input("  팔끝을 공에 대고 Enter > ")
    pos = drv.get_all_positions()
    if any(pos.get(i + 1) is None for i in range(5)):
        raise SystemExit("서보 응답 없음 — 로봇 전원/케이블 확인. "
                         "(이대로 계속하면 모든 점이 같은 좌표로 읽혀 H가 엉터리가 된다)")
    deg = [STS3215Driver.position_to_degrees(
        pos.get(i + 1)) or 0.0 for i in range(5)]
    p, _ = fk.fk_deg(deg)
    return float(p[0]), float(p[1])


def main():
    drv = STS3215Driver(port=PORT)
    drv.connect()
    fk = FKSo101()
    pixels, robots = [], []
    try:
        with CameraReader() as cam:
            for i in range(N):
                print(f"[{i + 1}/{N}] 공을 새 위치에 놓고(작업면 넓게), Space로 확정(또는 클릭).")
                uv = get_pixel(cam)
                xy = robot_xy(drv, fk)
                print(f"  픽셀 {uv} -> 로봇 ({xy[0]:+.3f}, {xy[1]:+.3f})")
                pixels.append(uv)
                robots.append(xy)
    except KeyboardInterrupt:
        print("취소 — 저장하지 않음")
        return
    finally:
        cv2.destroyAllWindows()

    H = cv2.getPerspectiveTransform(np.float32(pixels), np.float32(robots))

    worst = 0.0
    for (u, v), (x, y) in zip(pixels, robots):
        px, py = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
        worst = max(worst, ((px - x) ** 2 + (py - y) ** 2) ** 0.5)
    print(f"재투영 최대오차 {worst * 1000:.1f}mm",
          "(양호)" if worst < 0.01 else "(큼 — 4점을 더 넓게 재측정)")

    os.makedirs("data", exist_ok=True)
    np.save(OUT, H)
    print("저장:", OUT)


if __name__ == "__main__":
    main()
