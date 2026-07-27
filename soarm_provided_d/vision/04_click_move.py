import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

Z_FIXED = 0.12          # 유지할 손끝 높이(m)
SPEED = 600             # 실물 서보 속도(1~4095, 작을수록 느림 · 0=최대)
ACC = 20                # 실물 가감속(0~254, 작을수록 부드럽게)
REAL = True             # 시뮬 확인은 False, 실물은 True


def main():
    H = np.load(os.path.join("data", "H.npy"))
    # 실물 서보 속도를 미리 낮춤(안 그러면 최대속도로 홱)
    if REAL:
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
                x, y = cv2.perspectiveTransform(
                    np.float32([[[u, v]]]), H)[0, 0]
                print(f"클릭 ({u},{v}) -> 로봇 ({x:+.3f}, {y:+.3f})")
                cv2.drawMarker(rgb, (u, v), (0, 255, 255),
                               cv2.MARKER_CROSS, 18, 2)
                try:
                    arm.go([float(x), float(y), Z_FIXED],
                           real=REAL, down=False)
                except Exception as e:
                    print("스킵(도달 밖):", e)
            cv2.imshow(win, rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if name == "main":
    main()
