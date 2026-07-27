# -*- coding: utf-8 -*-
"""04_click_move.py — 화면을 클릭하면 그 지점으로 로봇이 이동 (고정 높이).

클릭 픽셀 → H로 로봇 (x,y) → arm.go 한 번(연속 아님 → 안전).
먼저 03_map.py 로 data/H.npy 를 만들어야 한다.

핵심은 5줄:  H = np.load(...)  →  클릭(u,v)  →  perspectiveTransform  →  arm.go
나머지는 카메라 창·마우스·속도설정.

⚠️ 클릭하면 팔이 그 지점으로 움직인다. 작업면에 손을 넣지 말 것.
    처음엔 REAL=False(시뮬)로 확인 후 REAL=True.
키: 좌클릭=이동 · q=종료
"""
import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

Z_FIXED = 0.12          # 유지할 손끝 높이(m)
SPEED = 600             # 실물 서보 속도(1~4095, 작을수록 느림 · 0=최대)
ACC = 20                # 실물 가감속(0~254, 작을수록 부드럽게)
REAL = True             # 시뮬 확인은 False, 실물은 True


def pixel_to_robot(u, v, H):
    """클릭 픽셀 (u, v) → 호모그래피 H 로 로봇 좌표 (x, y) 를 반환한다."""
    # ── TODO ①: 픽셀 → 로봇 좌표 변환 ─────────────────────────────────────
    # 할 일 : 픽셀 한 점 (u, v) 를 H 로 변환해 (x, y) 를 return 한다.
    # 키워드: cv2.perspectiveTransform, np.float32([[[u, v]]]),  결과[0, 0]
    # 참고  : H 는 03_map 이 만든 "픽셀→로봇" 3x3 행렬. 점 하나도 [[[u,v]]]
    #         3겹 배열로 감싸서 넣어야 한다(함수가 점들의 배열을 기대).
    x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
    return float(x), float(y)
    # ──────────────────────────────────────────────────────────────────────


# ── 심화 TODO ②(도전): 클릭 지점과 팔이 '어긋나 보이는' 이유 찾기 ──────────
# 증상  : 클릭하면 팔이 가긴 가는데, 화면으로 보면 커서와 꽤 떨어져 보인다.
# 할 일 : 원인을 조사해 설명하고, 줄일 수 있는 만큼 줄여 본다.
# 키워드: 원근 착시(손끝은 책상 위 Z_FIXED 높이에 떠 있다), 카메라 기울기,
#         티칭 접촉 오차(공 꼭대기 vs 옆구리), IK 허용오차(Arm.REACH_TOL),
#         5번째 점 검증(새 위치에 공을 놓고 클릭 → 그 바로 위에 오는가)
# 힌트  : 화면만 보면 속는다 — 위에서 수직으로 내려다보며 다시 판단해 보라.
# ──────────────────────────────────────────────────────────────────────────


def main():
    H = np.load(os.path.join("data", "H.npy"))
    if REAL:                                    # 실물 서보 속도를 미리 낮춤(안 그러면 최대속도로 홱)
        drv = arm._backend(True).drv
        if not drv.ping(1):
            raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인 후 다시 실행.")
        for sid in (1, 2, 3, 4, 5):
            drv.set_torque(sid, True)           # 03(티칭)이 꺼둔 토크를 다시 켠다
            drv.set_acceleration(sid, ACC)
            drv.set_speed(sid, SPEED)

    click = {}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click["uv"] = (x, y)

    win = "click-move (click=go / q=quit)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    print(f"클릭한 곳으로 이동 (REAL={REAL}). 작업면에 손 넣지 말 것. q 종료.")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            if "uv" in click:
                u, v = click.pop("uv")
                xy = pixel_to_robot(u, v, H)
                if xy is None:
                    print("TODO ① 먼저: pixel_to_robot() 을 구현하세요.")
                else:
                    x, y = xy
                    print(f"클릭 ({u},{v}) -> 로봇 ({x:+.3f}, {y:+.3f})")
                    cv2.drawMarker(rgb, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
                    try:
                        arm.go([float(x), float(y), Z_FIXED], real=REAL, down=False)
                    except Exception as e:
                        print("스킵(도달 밖):", e)
            cv2.imshow(win, rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
