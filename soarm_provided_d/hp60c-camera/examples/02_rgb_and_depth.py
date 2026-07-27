#!/usr/bin/env python3
"""HP60C RGB + Depth 동시 표시.

Depth(uint16 mm)를 컬러맵으로 시각화한다. 마우스를 올린 픽셀의 깊이가
콘솔에 출력된다.
"""

import sys

import cv2
import numpy as np

from hp60c_camera import CameraReader


_last_xy = (0, 0)


def _on_mouse(event, x, y, flags, _):
    global _last_xy
    if event == cv2.EVENT_MOUSEMOVE:
        _last_xy = (x, y)


def colorize_depth(depth_mm: np.ndarray, near: int = 200, far: int = 2000) -> np.ndarray:
    """uint16 mm 뎁스를 BGR 컬러맵으로."""
    d = depth_mm.astype(np.float32)
    d = np.clip((d - near) / max(far - near, 1), 0.0, 1.0)
    d8 = (d * 255).astype(np.uint8)
    color = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
    color[depth_mm == 0] = 0  # invalid 픽셀 검게
    return color


def main() -> int:
    with CameraReader() as cam:
        print("첫 프레임 대기...")
        rgb, depth, fid = cam.read_blocking(timeout=5.0)
        if rgb is None and depth is None:
            print("프레임 없음. shm_bridge 가 실행 중인가요?", file=sys.stderr)
            return 1
        print(f"frame #{fid}  rgb={None if rgb is None else rgb.shape}  depth={None if depth is None else depth.shape}")

        cv2.namedWindow("HP60C RGB")
        cv2.namedWindow("HP60C Depth")
        cv2.setMouseCallback("HP60C Depth", _on_mouse)

        last_id = 0
        while True:
            rgb, depth, last_id = cam.read_blocking(last_frame_id=last_id, timeout=1.0)
            if rgb is None and depth is None:
                continue

            if rgb is not None:
                cv2.imshow("HP60C RGB", rgb)

            if depth is not None:
                vis = colorize_depth(depth)
                x, y = _last_xy
                if 0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]:
                    mm = int(depth[y, x])
                    cv2.putText(
                        vis, f"({x},{y}) {mm} mm", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA,
                    )
                cv2.imshow("HP60C Depth", vis)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
