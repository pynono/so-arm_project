# -*- coding: utf-8 -*-
"""05_track.py — 빨간 공을 로봇이 따라간다 (2D 추적).

검출 → H로 로봇좌표 → hover 높이로 이동, 매 프레임 반복. 마진(데드밴드)으로
미세 떨림엔 명령을 생략해 팔을 보호한다. 먼저 03_map.py 로 data/H.npy 를 만든다.

⚠️ 팔이 공을 계속 따라 움직인다. 작업면에 손을 넣지 말 것.
    처음엔 REAL=False(시뮬)로 궤적 확인 후 REAL=True.
키: q=종료
"""
import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

RED1_LO, RED1_HI = (0, 100, 80), (10, 255, 255)
RED2_LO, RED2_HI = (160, 100, 80), (179, 255, 255)
MIN_AREA = 150
Z_HOVER = 0.12          # 손끝 유지 높이(m) — 안 집고 위에서만 따라감
MARGIN = 0.01           # 이 거리(m) 이하로 움직이면 명령 생략(데드밴드)
SPEED = 600             # 실물 서보 속도(작을수록 느림)
ACC = 20                # 실물 가감속
REAL = True             # 시뮬 확인은 False, 실물은 True


def detect_red(bgr):
    """빨간 공 → ((u, v, r), 최대덩어리 면적). 못 찾으면 (None, 면적).

    면적을 같이 돌려주는 이유: 못 찾았을 때 '왜'를 화면에 띄우기 위해서다.
    면적이 MIN_AREA 언저리면 조명·그림자에 따라 검출이 깜빡인다 —
    그때는 MIN_AREA 를 낮추거나 hsv_tuner 로 S/V 하한을 내린다."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, RED1_LO, RED1_HI) | cv2.inRange(hsv, RED2_LO, RED2_HI)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, area
    (u, v), r = cv2.minEnclosingCircle(c)
    return (int(u), int(v), int(r)), area


def draw_hud(bgr, text, color):
    for c, tk in (((0, 0, 0), 3), (color, 1)):
        cv2.putText(bgr, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, tk)


def main():
    H = np.load(os.path.join("data", "H.npy"))
    if REAL:
        drv = arm._backend(True).drv
        if not drv.ping(1):
            raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인 후 다시 실행.")
        for sid in (1, 2, 3, 4, 5):
            drv.set_torque(sid, True)           # 티칭 등으로 꺼진 토크를 다시 켠다
            drv.set_acceleration(sid, ACC)
            drv.set_speed(sid, SPEED)

    last_xy = None
    print(f"트래킹 (REAL={REAL}). 작업면에 손 넣지 말 것. q 종료.")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            det, area = detect_red(rgb)
            if det:
                u, v, r = det
                cv2.circle(rgb, (u, v), r, (0, 0, 255), 2)      # 검출된 실제 반지름
                draw_hud(rgb, f"ball area={area:.0f}", (0, 255, 0))
                x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
                moved = (last_xy is None or
                         ((x - last_xy[0]) ** 2 + (y - last_xy[1]) ** 2) ** 0.5 > MARGIN)
                if moved:
                    try:
                        arm.go([float(x), float(y), Z_HOVER], real=REAL, down=False)
                        last_xy = (x, y)
                    except Exception as e:
                        print("스킵(도달 밖):", e)
            else:
                # 왜 못 찾았는지 화면에 — 조용한 실패를 없앤다
                draw_hud(rgb, f"NO BALL: max blob {area:.0f}px < MIN_AREA {MIN_AREA}"
                              if area else "NO BALL: red pixels 0 (HSV 재측정)",
                         (0, 200, 255))
            cv2.imshow("track (q=quit)", rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
