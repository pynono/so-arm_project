# -*- coding: utf-8 -*-
"""Shared constants / config for the sort pipeline."""
from __future__ import annotations

from pathlib import Path

_VISION = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[2]

H_PATH = ROOT / "data" / "H.npy"
CALIB_PATH = ROOT / "data" / "map_calib.json"

SPEED = 500
ACC = 25
PORT = "/dev/ttyACM0"

Z_HOVER = 0.13          # default mid; pick uses distance-adaptive hover
Z_HOVER_NEAR = 0.11     # near reach: lower hover (less stretch)
Z_HOVER_FAR = 0.17      # far reach: higher hover so j2/j3/j4 stay softer
Z_PICK = 0.04
Z_TEST_LIFT = Z_PICK + 0.025  # ~2.5cm test lift after close (verify grasp)
Z_CARRY = 0.14
Z_PLACE = 0.06
# Radial reach (m) for near↔far blending of hover / seed / soft-down
R_NEAR = 0.16
R_FAR = 0.32
DOWN_TILT_FAR_DEG = 25.0  # soft-down outward tilt at R_FAR (0 at R_NEAR)
SETTLE_N = 8
DETECT_HOLD = 5       # 연속 N프레임 보이면 DETECT 통과
SCAN_TIMEOUT = 45.0
REDETECT_TIMEOUT = 12.0
SEED = (0, 30, -45, 0, 0)

# Taught approach j5 + shared open j6 (live raw from /dev/ttyACM0)
PICK_J5 = 1021
J6_OPEN = 2350          # approach open + place/drop open
PICK_J6 = J6_OPEN       # MOVE_TO_BALL approach + GRIP open
DROP_J6 = J6_OPEN       # PLACE open after lower
# Place/drop wrist: hold current live j5 (raw from /dev/ttyACM0) — do not use IK j5
DROP_J5 = 1019          # live present position id=5 (~-90.5°)
PICK_APPROACH_SECS = 3.8

# Box approach: slow carry to box hover; grip stays CLOSED until PLACE opens
BOX_APPROACH_SECS = 3.8

# Grip: open = J6_OPEN; close for grasp
GRIP_OPEN = J6_OPEN
GRIP_CLOSE = 1750
GRIP_LOAD_THRESH = 100       # get_load(6) >= this → grasped (signed, no abs)
GRIP_LOAD_SAMPLES = 5        # samples over ~1s after test lift
GRIP_LOAD_SAMPLE_DT = 0.2    # seconds between load samples (5 × 0.2 = 1.0s)
GRIP_LOAD_MAJORITY = 3       # need >= this many samples with load >= thresh
GRIP_MAX_RETRIES = 3         # full pick cycles: SAFE→redetect→MOVE→GRIP; then re-SCAN
GRIP_CLOSE_SETTLE_SECS = 1.0   # wait until close complete
GRIP_POST_CLOSE_STABILIZE = 0.25  # short settle after close confirm, before test lift
GRIP_DOWN_SECS = 2.4           # hover → pick z
GRIP_TEST_LIFT_SECS = 0.55     # pick → test lift (~2.5cm) before load verify
GRIP_LIFT_SECS = 1.4           # test → carry after grasp OK
JOINT_ARRIVE_TOL = 40          # raw units: present vs goal
JOINT_ARRIVE_POLL = 0.05
JOINT_ARRIVE_EXTRA = 2.0       # extra seconds beyond motion secs for arrive poll
REDETECT_SETTLE_STD = 0.008    # max xy std (m) for settle OK
MEASURE_LOAD_N = 30            # samples per --measure-load scenario
MEASURE_LOAD_DT = 0.1

SAFE_POSE = {1: 2047, 2: 813, 3: 3192, 4: 1039, 5: 1137, 6: GRIP_OPEN}

# Scan targets: (color, kind) — balls required; boxes may fall back to taught
SCAN_KEYS = (("red", "ball"), ("blue", "ball"), ("red", "box"), ("blue", "box"))
COLOR_ORDER = ("red", "blue")


class GripFailure(RuntimeError):
    """Raised when pick/grip fails after GRIP_MAX_RETRIES redetect cycles.

    Caller (run_sort_with_recover) goes SAFE → full re-SCAN → resume pending.
    """

    def __init__(self, color: str, msg: str):
        super().__init__(msg)
        self.color = color


class RedetectFailure(GripFailure):
    """Recoverable: ball re-detect timed out / did not settle.

    Treated like GripFailure so callers go SAFE → retry / re-SCAN.
    """
