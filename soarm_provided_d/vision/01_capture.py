# -*- coding: utf-8 -*-
"""01_capture.py — 공 사진 저장 (HSV 개발용 데이터 모으기).

브리지를 켠 뒤 실행. 화면을 보며 s 로 저장하면 shots/ 에 쌓인다.
빨강·파랑·조명별로 몇 장씩 찍어두면 다음(02_detect)에서 HSV 임계값 맞추기 쉽다.
키: s=저장 · q=종료
"""
import os

import cv2
from hp60c_camera import CameraReader

OUT = "shots"


def main():
    os.makedirs(OUT, exist_ok=True)
    n = len([f for f in os.listdir(OUT) if f.endswith(".png")])
    print("s=저장, q=종료")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            cv2.imshow("capture (s=save, q=quit)", rgb)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("s"):
                path = os.path.join(OUT, f"shot_{n:03d}.png")
                cv2.imwrite(path, rgb)
                print("저장:", path)
                n += 1
            elif k == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()