"""HP60C / ASC60C 뎁스카메라용 공유메모리 reader.

shm_bridge(C++)가 카메라에서 받은 RGB/Depth 프레임을
/dev/shm/hp60c_frames 에 써주면, 이 패키지로 어떤 Python 코드에서든 읽을 수 있다.

사용 예:
    from hp60c_camera import CameraReader

    cam = CameraReader()
    rgb, depth, frame_id = cam.read()
"""

from .reader import CameraReader, ShmHeader, SHM_PATH, SHM_MAGIC

__all__ = ["CameraReader", "ShmHeader", "SHM_PATH", "SHM_MAGIC"]
__version__ = "0.1.0"
