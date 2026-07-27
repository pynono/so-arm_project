# -*- coding: utf-8 -*-
"""02_detect.py — HSV로 빨강/파랑 공·상자 검출 → 픽셀(u,v).

카메라:        python vision/02_detect.py
저장 사진으로: python vision/02_detect.py vision/shots/shot_000.png ...
키: q=종료

검출 로직·임계값은 detect_objects.py 공유 모듈을 쓴다.
  - 빨강: 이중 구간 HSV (색상환 wrap)
  - 파랑: 단일 HSV
  - 노랑: 사용 안 함
  - 같은 색이라도 contour area >= BOX_MIN_AREA → box, 아니면 ball
  - 손/피부: 빨강 S 상향 + circularity/solidity 로 제외
"""
import sys
from pathlib import Path

import cv2

# 프로젝트 루트 / vision 둘 다에서 import 되게
_VISION = Path(__file__).resolve().parent
if str(_VISION) not in sys.path:
    sys.path.insert(0, str(_VISION))

from detect_objects import (  # noqa: E402
    BOX_MIN_AREA,
    MIN_AREA,
    annotate_objects,
    detect_colored_objects,
)


def annotate(bgr):
    objs = detect_colored_objects(bgr)
    annotate_objects(bgr, objs)
    # HUD: threshold reminder
    cv2.putText(
        bgr,
        f"BOX_MIN_AREA={BOX_MIN_AREA}  MIN_AREA={MIN_AREA}  "
        f"n={len(objs)}  ball=circle box=rect",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        1,
    )
    return bgr


def main():
    if len(sys.argv) > 1:                       # 저장 사진으로(카메라 없이)
        for path in sys.argv[1:]:
            img = cv2.imread(path)
            if img is None:
                print("못 읽음:", path)
                continue
            objs = detect_colored_objects(img)
            print(path, "→", [(o["color"], o["kind"], int(o["area"])) for o in objs])
            cv2.imshow(path, annotate(img))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    from hp60c_camera import CameraReader       # 카메라 모드
    print(
        f"빨강/파랑 ball|box 검출 (BOX_MIN_AREA={BOX_MIN_AREA}) — q 종료"
    )
    print("라벨: red_ball / red_box / blue_ball / blue_box  "
          f"(면적>={BOX_MIN_AREA} → box)")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            cv2.imshow("detect (q=quit)", annotate(rgb))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
