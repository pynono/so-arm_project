#!/usr/bin/env python3
"""HP60C 카메라를 여는 가장 간단한 예제 — RGB만 표시.

사전 조건:
    1) bridge/build.sh 로 shm_bridge 빌드
    2) scripts/start_bridge.sh 실행 (별도 터미널)
실행:
    python examples/01_open_camera.py
종료:
    창에 포커스를 두고 q
"""

import sys

import cv2

from hp60c_camera import CameraReader


def main() -> int:
    print("[01_open_camera] /dev/shm/hp60c_frames 에 연결 중...")
    with CameraReader() as cam:
        print("[01_open_camera] 첫 프레임 대기...")
        rgb, _, fid = cam.read_blocking(timeout=5.0)
        if rgb is None:
            print("프레임을 받지 못했습니다. shm_bridge 가 실행 중인지 확인하세요.", file=sys.stderr)
            return 1
        print(f"[01_open_camera] 프레임 #{fid}, shape={rgb.shape}")

        last_id = 0
        while True:
            rgb, _, last_id = cam.read_blocking(last_frame_id=last_id, timeout=1.0)
            if rgb is None:
                continue
            cv2.imshow("HP60C RGB", rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
