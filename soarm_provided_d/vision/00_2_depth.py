from hp60c_camera import CameraReader
import cv2
import numpy as np
with CameraReader() as cam:
    last = 0
    while True:
        rgb, depth, last = cam.read_blocking(last)
        if rgb is None:
            continue
        cv2.imshow('rgb', rgb)

        d = np.clip((depth - 400)/120, 0, 1)
        vis = cv2.applyColorMap((d*255).astype("uint8"), cv2.COLORMAP_JET)
        vis[depth == 0] = 0
        cv2.imshow('depth', vis)
        cy, cx = depth.shape[0]//2, depth.shape[1]//2
        print("중앙:", depth[cy, cx], "mm|최소:",
              depth[depth > 0].min(), "최대:", depth.max())
        if cv2.waitKey(1) == ord('q'):
            break
