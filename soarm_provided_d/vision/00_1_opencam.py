from hp60c_camera import CameraReader
import cv2
with CameraReader() as cam:
    last = 0
    while True:
        rgb, depth, last = cam.read_blocking(last)
        if rgb is None:
            continue
        cv2.imshow('rgb', rgb)
        if cv2.waitKey(1) == ord('q'):
            break
