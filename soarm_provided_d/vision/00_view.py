# -*- coding: utf-8 -*-
"""00_view.py — 카메라 화면만 보기. q=종료"""
import os
from pathlib import Path

# OpenCV Qt GUI: use X11; point Qt at cv2 bundled (or symlinked) fonts dir.
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
_cv2_fonts = Path(os.environ.get("VIRTUAL_ENV", "/home/rookie/D053/.venv")) / (
    "lib/python3.13/site-packages/cv2/qt/fonts"
)
if _cv2_fonts.is_dir() or _cv2_fonts.is_symlink():
    os.environ.setdefault("QT_QPA_FONTDIR", str(_cv2_fonts.resolve()))

import cv2
from hp60c_camera import CameraReader


def main():
    print("카메라 뷰 · q=종료")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last, timeout=1.0)
            if rgb is None:
                continue
            cv2.imshow("camera (q=quit)", rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
