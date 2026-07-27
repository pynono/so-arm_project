#!/usr/bin/env python3
"""HP60C RGB + Depth 한 창에 나란히 표시.

shm_bridge 가 떠있어야 한다. 마우스를 올린 픽셀의 깊이(mm)와 FPS 가 화면에 표시.
종료: 창 포커스에서 q 또는 ESC.
"""

import sys
import time

import cv2
import numpy as np

from hp60c_camera import CameraReader


WINDOW = "HP60C RGB | Depth"
DEPTH_NEAR_MM = 200
DEPTH_FAR_MM = 2000


_mouse_xy = (-1, -1)


def _on_mouse(event, x, y, flags, _):
    global _mouse_xy
    if event == cv2.EVENT_MOUSEMOVE:
        _mouse_xy = (x, y)


def colorize_depth(depth_mm: np.ndarray) -> np.ndarray:
    d = depth_mm.astype(np.float32)
    d = np.clip((d - DEPTH_NEAR_MM) / max(DEPTH_FAR_MM - DEPTH_NEAR_MM, 1), 0.0, 1.0)
    color = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_JET)
    color[depth_mm == 0] = 0
    return color


def main() -> int:
    with CameraReader() as cam:
        print("[03] /dev/shm/hp60c_frames 연결, 첫 프레임 대기...")
        rgb, depth, fid = cam.read_blocking(timeout=5.0)
        if rgb is None or depth is None:
            print("프레임 없음. shm_bridge 실행 여부 확인.", file=sys.stderr)
            return 1
        print(f"[03] OK — RGB {rgb.shape}, Depth {depth.shape}, frame #{fid}")

        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, _on_mouse)

        last_id = 0
        t_prev = time.monotonic()
        fps = 0.0
        while True:
            rgb, depth, last_id = cam.read_blocking(last_frame_id=last_id, timeout=1.0)
            if rgb is None or depth is None:
                continue

            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

            depth_vis = colorize_depth(depth)

            # depth 와 rgb 해상도가 다를 수 있으니 depth 크기에 맞춰 맞춘다
            if depth_vis.shape[:2] != rgb.shape[:2]:
                depth_vis = cv2.resize(depth_vis, (rgb.shape[1], rgb.shape[0]))

            mx, my = _mouse_xy
            cx, cy = rgb.shape[1] // 2, rgb.shape[0] // 2
            center_mm = int(depth[cy, cx])

            # 마우스 좌표 보정: 합쳐진 창 기준 → 패널 안 좌표로
            panel_w = rgb.shape[1]
            mouse_in_left = 0 <= mx < panel_w
            mouse_in_right = panel_w <= mx < 2 * panel_w
            local_x = mx if mouse_in_left else (mx - panel_w if mouse_in_right else -1)
            mouse_mm = -1
            if 0 <= local_x < depth.shape[1] and 0 <= my < depth.shape[0]:
                mouse_mm = int(depth[my, local_x])

            # 중앙 십자선 + readout
            cv2.drawMarker(rgb, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 16, 1)
            cv2.drawMarker(depth_vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)

            combined = np.hstack([rgb, depth_vis])

            txt = [
                f"FPS: {fps:5.1f}   frame #{last_id}",
                f"center({cx},{cy}): {center_mm} mm",
            ]
            if mouse_mm >= 0:
                txt.append(f"mouse({local_x},{my}): {mouse_mm} mm")
            for i, line in enumerate(txt):
                y = 25 + 24 * i
                cv2.putText(combined, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(combined, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow(WINDOW, combined)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):  # q or ESC
                break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
