"""HP60C 공유메모리 reader.

shm_bridge가 쓴 /dev/shm/hp60c_frames 을 mmap으로 열어
RGB(BGR uint8)와 Depth(uint16, mm) 프레임을 numpy 배열로 돌려준다.
"""

from __future__ import annotations

import mmap
import os
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

SHM_PATH = "/dev/shm/hp60c_frames"
SHM_MAGIC = 0x48503630  # "HP60"

HEADER_SIZE = 64
MAX_RGB_SIZE = 1920 * 1080 * 3
MAX_DEPTH_SIZE = 640 * 480 * 2

# Header layout (struct.unpack_from 오프셋 기준):
#   0  : uint32 magic
#   4  : uint32 rgb_w, rgb_h, rgb_size
#   16 : uint32 depth_w, depth_h, depth_size
#   28 : uint32 _pad
#   32 : uint64 frame_id
#   40 : uint64 timestamp_us
#   48 : uint32 rgb_ready
#   52 : uint32 depth_ready


@dataclass
class ShmHeader:
    magic: int
    rgb_w: int
    rgb_h: int
    rgb_size: int
    depth_w: int
    depth_h: int
    depth_size: int
    frame_id: int
    timestamp_us: int
    rgb_ready: bool
    depth_ready: bool


class CameraReader:
    """HP60C shm_bridge 가 쓴 공유메모리를 읽는 클라이언트.

    Parameters
    ----------
    path : str
        공유메모리 경로. 기본값은 /dev/shm/hp60c_frames.
    wait_timeout : float | None
        shm 파일이 아직 안 보일 때 최대 몇 초까지 대기할지. None 이면 즉시 실패.
    copy : bool
        True 면 numpy 배열을 복사해서 돌려줌(권장). False 면 mmap을 그대로 view.
    """

    def __init__(
        self,
        path: str = SHM_PATH,
        wait_timeout: Optional[float] = 5.0,
        copy: bool = True,
    ) -> None:
        self.path = path
        self.copy = copy
        self._wait_for_shm(wait_timeout)
        self._fd = os.open(path, os.O_RDONLY)
        self._buf = mmap.mmap(self._fd, 0, access=mmap.ACCESS_READ)

    # ─ context manager ────────────────────────────────────────────────────
    def __enter__(self) -> "CameraReader":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ─ public API ─────────────────────────────────────────────────────────
    def header(self) -> ShmHeader:
        """현재 헤더 스냅샷."""
        magic = struct.unpack_from("<I", self._buf, 0)[0]
        rgb_w, rgb_h, rgb_size = struct.unpack_from("<III", self._buf, 4)
        depth_w, depth_h, depth_size = struct.unpack_from("<III", self._buf, 16)
        frame_id = struct.unpack_from("<Q", self._buf, 32)[0]
        timestamp_us = struct.unpack_from("<Q", self._buf, 40)[0]
        rgb_ready = struct.unpack_from("<I", self._buf, 48)[0]
        depth_ready = struct.unpack_from("<I", self._buf, 52)[0]
        return ShmHeader(
            magic=magic,
            rgb_w=rgb_w, rgb_h=rgb_h, rgb_size=rgb_size,
            depth_w=depth_w, depth_h=depth_h, depth_size=depth_size,
            frame_id=frame_id, timestamp_us=timestamp_us,
            rgb_ready=bool(rgb_ready), depth_ready=bool(depth_ready),
        )

    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        """가장 최근 프레임을 (rgb, depth, frame_id) 로 반환.

        bridge가 아직 첫 프레임을 안 썼으면 (None, None, 0) 을 돌려준다.
        rgb 는 BGR uint8 (h, w, 3), depth 는 uint16 mm (h, w).
        """
        h = self.header()
        if h.magic != SHM_MAGIC:
            return None, None, 0

        rgb: Optional[np.ndarray] = None
        depth: Optional[np.ndarray] = None

        if h.rgb_ready and h.rgb_w > 0 and h.rgb_h > 0:
            n = h.rgb_w * h.rgb_h * 3
            view = np.frombuffer(
                self._buf, dtype=np.uint8, count=n, offset=HEADER_SIZE
            ).reshape(h.rgb_h, h.rgb_w, 3)
            rgb = view.copy() if self.copy else view

        if h.depth_ready and h.depth_w > 0 and h.depth_h > 0:
            n = h.depth_w * h.depth_h
            offset = HEADER_SIZE + MAX_RGB_SIZE
            view = np.frombuffer(
                self._buf, dtype=np.uint16, count=n, offset=offset
            ).reshape(h.depth_h, h.depth_w)
            depth = view.copy() if self.copy else view

        return rgb, depth, h.frame_id

    def read_blocking(
        self, last_frame_id: int = 0, timeout: float = 1.0, poll_hz: float = 200.0
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        """frame_id 가 ``last_frame_id`` 보다 커질 때까지 폴링한다.

        timeout 초 안에 새 프레임이 안 오면 (None, None, last_frame_id) 반환.
        """
        deadline = time.monotonic() + timeout
        period = 1.0 / poll_hz
        while time.monotonic() < deadline:
            rgb, depth, fid = self.read()
            if fid > last_frame_id and (rgb is not None or depth is not None):
                return rgb, depth, fid
            time.sleep(period)
        return None, None, last_frame_id

    def close(self) -> None:
        if getattr(self, "_buf", None) is not None:
            self._buf.close()
            self._buf = None
        if getattr(self, "_fd", None) is not None:
            os.close(self._fd)
            self._fd = None

    # ─ helpers ────────────────────────────────────────────────────────────
    def _wait_for_shm(self, timeout: Optional[float]) -> None:
        if os.path.exists(self.path):
            return
        if timeout is None:
            raise FileNotFoundError(
                f"{self.path} 가 없습니다. shm_bridge 를 먼저 실행하세요."
            )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if os.path.exists(self.path):
                return
            time.sleep(0.1)
        raise FileNotFoundError(
            f"{self.path} 가 {timeout}s 안에 나타나지 않았습니다. shm_bridge 가 실행 중인지 확인하세요."
        )
