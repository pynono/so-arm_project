"""
STS3215 servo driver for SO-ARM101.

Feetech STS3215 half-duplex UART protocol.

Packet format:
  TX: [0xFF][0xFF][ID][Length][Instruction][Param...][Checksum]
  RX: [0xFF][0xFF][ID][Length][Error][Param...][Checksum]
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import serial


# Instructions
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# Register addresses (Feetech STS3215)
ADDR_HOMING_OFFSET = 31       # EEPROM; sign-magnitude, sign bit at index 11
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_LOCK = 55                # 0 = EEPROM writable, 1 = write-protected
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66

# Homing_Offset is stored sign-magnitude with the sign in bit 11, so the
# magnitude maxes out at 2047. On Feetech servos the relationship is
#   Present_Position = Actual_Position - Homing_Offset
# i.e. writing a positive offset makes the reported position smaller.
HOMING_SIGN_BIT = 11
HOMING_MAX_MAGNITUDE = (1 << HOMING_SIGN_BIT) - 1   # 2047

# SO-ARM101 joint map
JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
JOINT_IDS = [1, 2, 3, 4, 5, 6]
NUM_JOINTS = 6

POS_MIN = 0
POS_MAX = 4095
POS_CENTER = 2048
POS_RANGE = 4096          # counts per full servo revolution (POS_MAX + 1)

# Per-joint soft limits for the UI slider + server-side clamp.
#
#   kind=angle    → UI shows (pos, degrees from center)
#   kind=percent  → UI shows (pos, NN% open) where min=0%, max=100%
#
#   rad_min, rad_max (optional) — override the default "servo rotation =
#     joint rotation" assumption with a linear map from (min, max) servo
#     counts to (rad_min, rad_max) URDF joint radians. The gripper needs
#     this because the servo drives a linkage: 360° of servo = ~110° of
#     jaw.
#
#   invert (optional, bool) — flip the rendered rotation direction for
#     this joint. Use this if the 3D twin rotates the opposite way from
#     the physical joint when you move the slider (caused by the servo's
#     reported axis pointing opposite to what the URDF expects for that
#     joint on your particular arm).
JOINT_LIMITS: dict[int, dict] = {
    1: {"min": POS_MIN, "max": POS_MAX, "kind": "angle"},
    2: {"min": POS_MIN, "max": POS_MAX, "kind": "angle"},
    3: {"min": POS_MIN, "max": POS_MAX, "kind": "angle"},
    4: {"min": POS_MIN, "max": POS_MAX, "kind": "angle"},
    5: {"min": POS_MIN, "max": POS_MAX, "kind": "angle"},
    6: {
        "min": 984, "max": 2318, "kind": "percent",
        "rad_min": -0.175, "rad_max": 1.745,   # URDF gripper joint range
    },
}


# Per-robot calibration overrides. Each physical arm has slightly different
# servo installation angles + mechanical end-stops, so the JOINT_LIMITS
# defaults above only fit the very first arm we measured. The wizard at
# scripts/calibrate.py writes data/calibration.json with this robot's
# measured min/max (and optionally rad_min/rad_max/invert) per joint;
# we load that file at import time and merge it into JOINT_LIMITS.
#
# File format (all fields optional, only the ones present override):
#   {
#     "robot": "team1",                  # informational
#     "joints": {
#       "1": {"min": 320,  "max": 3920},
#       "2": {"min": 800,  "max": 3300},
#       ...
#       "6": {"min": 2080, "max": 2540, "rad_min": -0.175, "rad_max": 1.745}
#     }
#   }
#
# min/max bound the joint's travel with min < max, so the slider range and
# server clamp stay sane. They are NOT always plain raw counts: on an arm
# whose travel straddles the 0/4095 seam the window is stored unwrapped and
# max exceeds POS_MAX (see unwrap_position() below). For percent-mapped
# joints (the gripper) the calibration wizard adds "flip": true when this
# arm's servo counts DOWN from the 0% (closed) end to the 100% (open) end —
# i.e. min is physically the OPEN end. The UI percent readout and the twin's
# raw→radians map reverse their interpolation when flip is set, so the
# gripper never renders inverted.
_ROOT = Path(__file__).resolve().parent.parent
CALIBRATION_FILE = _ROOT / "data" / "calibration.json"
KINEMATICS_FILE = _ROOT / "model" / "kinematics.json"

# Which arm the loaded calibration describes. Recorded into saved poses so a
# file that lands on the wrong robot can say where it came from.
CALIBRATION_LABEL = "uncalibrated"


def _load_urdf_rad_limits(path: Path = KINEMATICS_FILE) -> dict[int, tuple]:
    """servo id → (lower, upper) URDF joint radians, via kinematics.json's
    drive_order. Empty dict if the model can't be read — callers then keep
    whatever rad limits JOINT_LIMITS already declares."""
    try:
        kin = json.loads(path.read_text())
        by_name = {j["name"]: j for j in kin["joints"]}
        out = {}
        for jid, name in enumerate(kin["drive_order"], start=1):
            j = by_name.get(name, {})
            if "lower" in j and "upper" in j:
                out[jid] = (float(j["lower"]), float(j["upper"]))
        return out
    except Exception as exc:
        print(f"[driver] URDF limits unavailable ({path.name}): {exc}")
        return {}


def _load_calibration_overrides(path: Path = CALIBRATION_FILE) -> None:
    """Merge per-robot calibration from disk into the module-level
    JOINT_LIMITS dict. Silent no-op if the file doesn't exist (we fall
    back to the JOINT_LIMITS defaults, which is the original behaviour)."""
    if not path.exists():
        return
    try:
        cal = json.loads(path.read_text())
    except Exception as exc:
        print(f"[driver] calibration load failed ({path.name}): {exc}")
        return
    rad_limits = _load_urdf_rad_limits()
    calibrated = []
    for jid_str, fields in (cal.get("joints") or {}).items():
        try:
            jid = int(jid_str)
        except ValueError:
            continue
        if jid not in JOINT_LIMITS or not isinstance(fields, dict):
            continue
        JOINT_LIMITS[jid].update(fields)

        # A measured window is what makes the URDF rad range usable: it says
        # which raw counts this arm's end-stops actually sit at, so min..max
        # can be interpolated onto lower..upper. Without it we'd be mapping
        # the servo's full 0..4095 onto a joint that only travels ~200°, which
        # is worse than the centre-2048 assumption we'd be replacing — hence
        # only joints the wizard actually measured get rad limits attached.
        if "min" in fields and "max" in fields and jid in rad_limits:
            JOINT_LIMITS[jid].setdefault("rad_min", rad_limits[jid][0])
            JOINT_LIMITS[jid].setdefault("rad_max", rad_limits[jid][1])
        calibrated.append(jid)
    global CALIBRATION_LABEL
    CALIBRATION_LABEL = cal.get("robot") or "unnamed"
    print(f"[driver] loaded calibration for '{CALIBRATION_LABEL}' from "
          f"{path.name} (joints {sorted(calibrated)})")


_load_calibration_overrides()


# ── Wrapped calibration windows ───────────────────────────────────────────────
#
# The STS3215 reports an absolute 0..4095 count over one full revolution, and
# the horn can only be seated on discrete spline teeth. So on some arms a
# joint's travel straddles the 0/4095 seam: the gripper might read 3915 when
# closed, count up to 4095, roll over to 0, and reach 1302 fully open. Its
# travel is 1483 counts, but the two endpoints alone (3915, 1302) can't tell
# you whether the arm took the short way round the seam or the long way
# through the middle — sorting them into min/max silently assumes the latter,
# which renders the joint mirrored and pinned to its extremes.
#
# calibrate.py resolves the ambiguity by sampling continuously along the
# travel, so it can record an *unwrapped* window whose max may exceed POS_MAX
# (that gripper is stored as min=3915, max=5398). Every consumer of a raw
# count then maps it back into that window with unwrap_position() before
# comparing against min/max, and wrap_position() converts back to a real
# servo count before it goes on the bus.
#
# Joints whose window doesn't wrap keep max <= POS_MAX, and unwrap_position()
# is a no-op for them — the original behaviour, untouched.


def unwrap_position(pos: int, lim: dict) -> int:
    """Map a raw servo count into this joint's calibrated window.

    For a wrapped window (max > POS_MAX) the returned value can exceed
    POS_MAX — that's the point: it makes the window contiguous so plain
    min/max comparisons work. Idempotent, so it's safe to apply more than
    once along a call chain."""
    if not lim or lim.get("max", POS_MAX) <= POS_MAX:
        return pos
    lo = lim.get("min", POS_MIN)
    return lo + (pos - lo) % POS_RANGE


def wrap_position(pos: int) -> int:
    """Unwrapped window coordinate → real servo count (0..4095)."""
    return pos % POS_RANGE


def window_fraction(pos: Optional[int], lim: dict) -> Optional[float]:
    """Raw servo count → where it sits in the joint's travel, 0..1.

    This is the arm-independent way to name a position: 0 is "this joint's
    one end-stop", 1 is "the other", whatever raw counts those happen to be on
    a given arm. Saved poses and recordings store these, so a pose taught on
    one robot lands in the same physical place on the next.

    A count can sit in the arc the joint can't reach (a stop pressed a few
    counts past where it was measured); snap to the circularly nearest end,
    matching _clamp_position() and the frontend's windowFraction()."""
    if pos is None or not lim:
        return None
    lo, hi = lim.get("min"), lim.get("max")
    if lo is None or hi is None:
        return None
    p = unwrap_position(int(pos), lim)
    if p > hi:
        p = hi if (p - hi) <= (lo + POS_RANGE - p) else lo
    t = max(0.0, min(1.0, (p - lo) / ((hi - lo) or 1)))
    return 1.0 - t if lim.get("flip") else t


def position_from_fraction(t: float, lim: dict) -> int:
    """0..1 within the joint's travel → real servo count for THIS arm."""
    lo = lim.get("min", POS_MIN)
    hi = lim.get("max", POS_MAX)
    t = max(0.0, min(1.0, float(t)))
    if lim.get("flip"):
        t = 1.0 - t
    return wrap_position(int(round(lo + t * (hi - lo))))


@dataclass
class JointStatus:
    id: int
    name: str
    online: bool = False
    position: Optional[int] = None
    angle_deg: Optional[float] = None
    temperature: Optional[int] = None
    voltage: Optional[float] = None
    load: Optional[int] = None
    torque_enabled: Optional[bool] = None

    def to_dict(self) -> dict:
        return asdict(self)


class STS3215Driver:
    """Thread-safe driver for Feetech STS3215 servos on a half-duplex bus."""

    def __init__(self, port: str = "/dev/ttyACM0",
                 baudrate: int = 1_000_000, timeout: float = 0.02):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                exclusive=True,   # 다른 프로세스와 포트 공유 금지
            )
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            return True
        except Exception as exc:
            print(f"[driver] connection error: {exc}")
            self.serial = None
            return False

    def disconnect(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None

    def is_connected(self) -> bool:
        return self.serial is not None and self.serial.is_open

    # ── Low-level protocol ────────────────────────────────────────────────────

    @staticmethod
    def _checksum(packet: list[int]) -> int:
        return (~sum(packet[2:])) & 0xFF

    def _build_packet(self, servo_id: int, instruction: int,
                      params: Optional[list[int]] = None) -> bytes:
        params = params or []
        length = len(params) + 2
        pkt = [0xFF, 0xFF, servo_id, length, instruction, *params]
        pkt.append(self._checksum(pkt))
        return bytes(pkt)

    def _transact(self, servo_id: int, instruction: int,
                  params: Optional[list[int]] = None,
                  response_len: int = 0) -> Optional[dict]:
        with self._lock:
            if not self.is_connected():
                return None
            packet = self._build_packet(servo_id, instruction, params)
            try:
                self.serial.reset_input_buffer()
                self.serial.write(packet)
                self.serial.flush()
                if response_len > 0:
                    expected = 6 + response_len
                    response = self.serial.read(expected)
                    if len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF:
                        error = response[4]
                        data = list(response[5:-1]) if len(response) > 6 else []
                        return {"id": response[2], "error": error, "data": data}
                return None
            except Exception as exc:
                print(f"[driver] comm error: {exc}")
                return None

    # ── Register access ───────────────────────────────────────────────────────

    def ping(self, servo_id: int) -> bool:
        with self._lock:
            if not self.is_connected():
                return False
            packet = self._build_packet(servo_id, INST_PING)
            try:
                self.serial.reset_input_buffer()
                self.serial.write(packet)
                self.serial.flush()
                response = self.serial.read(6)
                return len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF
            except Exception:
                return False

    def _read(self, servo_id: int, addr: int, length: int) -> Optional[list[int]]:
        result = self._transact(servo_id, INST_READ, [addr, length], length)
        if result and result["error"] == 0 and len(result["data"]) >= length:
            return result["data"]
        return None

    def _write(self, servo_id: int, addr: int, data: list[int]) -> bool:
        params = [addr, *data]
        with self._lock:
            if not self.is_connected():
                return False
            packet = self._build_packet(servo_id, INST_WRITE, params)
            try:
                self.serial.reset_input_buffer()
                self.serial.write(packet)
                self.serial.flush()
                response = self.serial.read(6)
                if len(response) >= 6:
                    return response[4] == 0
                return True
            except Exception as exc:
                print(f"[driver] write error: {exc}")
                return False

    def _read_u16(self, servo_id: int, addr: int) -> Optional[int]:
        data = self._read(servo_id, addr, 2)
        if data and len(data) >= 2:
            return data[0] | (data[1] << 8)
        return None

    def _write_u16(self, servo_id: int, addr: int, value: int) -> bool:
        value = max(0, min(65535, int(value)))
        return self._write(servo_id, addr, [value & 0xFF, (value >> 8) & 0xFF])

    def _write_u8(self, servo_id: int, addr: int, value: int) -> bool:
        value = max(0, min(255, int(value)))
        return self._write(servo_id, addr, [value])

    # ── High-level API ────────────────────────────────────────────────────────

    def get_position(self, servo_id: int) -> Optional[int]:
        return self._read_u16(servo_id, ADDR_PRESENT_POSITION)

    def set_position(self, servo_id: int, position: int) -> bool:
        """Write only goal_position. Speed/acceleration are set separately
        and cached by the controller to avoid resetting the servo's motion
        planner every command (main source of jitter at high update rates).
        """
        position = max(POS_MIN, min(POS_MAX, int(position)))
        return self._write_u16(servo_id, ADDR_GOAL_POSITION, position)

    def set_speed(self, servo_id: int, speed: int) -> bool:
        speed = max(0, min(4095, int(speed)))
        return self._write_u16(servo_id, ADDR_GOAL_SPEED, speed)

    def set_torque(self, servo_id: int, enable: bool) -> bool:
        return self._write_u8(servo_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)

    def get_torque(self, servo_id: int) -> Optional[bool]:
        data = self._read(servo_id, ADDR_TORQUE_ENABLE, 1)
        return data[0] == 1 if data else None

    def get_temperature(self, servo_id: int) -> Optional[int]:
        data = self._read(servo_id, ADDR_PRESENT_TEMPERATURE, 1)
        return data[0] if data else None

    def get_voltage(self, servo_id: int) -> Optional[float]:
        data = self._read(servo_id, ADDR_PRESENT_VOLTAGE, 1)
        return data[0] / 10.0 if data else None

    def get_load(self, servo_id: int) -> Optional[int]:
        val = self._read_u16(servo_id, ADDR_PRESENT_LOAD)
        if val is None:
            return None
        load = val & 0x3FF
        return -load if (val & 0x0400) else load

    def set_acceleration(self, servo_id: int, acc: int) -> bool:
        return self._write_u8(servo_id, ADDR_ACCELERATION, max(0, min(254, int(acc))))

    # ── Homing offset (EEPROM) ────────────────────────────────────────────────
    #
    # Centres a joint's range so it never straddles the servo's 0/4095 seam.
    # This is the piece that makes commanding work: the seam is a hard wall to
    # the position controller (it moves linearly from goal to present and can't
    # cross 4095↔0), so a range spanning it can't be driven end to end. Moving
    # the seam out of the range with a homing offset is the fix; the software
    # unwrap is only a fallback for arms that haven't been centred yet.

    def set_eeprom_lock(self, servo_id: int, locked: bool) -> bool:
        """EEPROM is write-protected unless this is 0. Torque must be off to
        write EEPROM anyway; unlock right before writing, lock right after."""
        return self._write_u8(servo_id, ADDR_LOCK, 1 if locked else 0)

    def get_homing_offset(self, servo_id: int) -> Optional[int]:
        raw = self._read_u16(servo_id, ADDR_HOMING_OFFSET)
        if raw is None:
            return None
        magnitude = raw & HOMING_MAX_MAGNITUDE
        return -magnitude if (raw >> HOMING_SIGN_BIT) & 1 else magnitude

    def set_homing_offset(self, servo_id: int, offset: int) -> bool:
        """Write a sign-magnitude homing offset, clamped to the register's
        ±2047. Unlocks EEPROM around the write and locks it back."""
        offset = max(-HOMING_MAX_MAGNITUDE, min(HOMING_MAX_MAGNITUDE, int(offset)))
        encoded = (1 << HOMING_SIGN_BIT) | abs(offset) if offset < 0 else abs(offset)
        self.set_eeprom_lock(servo_id, False)
        ok = self._write_u16(servo_id, ADDR_HOMING_OFFSET, encoded)
        self.set_eeprom_lock(servo_id, True)
        return ok

    def center_half_turn(self, servo_id: int) -> Optional[int]:
        """Make the joint's current physical pose read POS_CENTER.

        Zero the existing offset first so we read the true actual position,
        then write offset = actual - POS_CENTER (Present = Actual - Offset).
        Returns the offset written, or None on a read failure."""
        self.set_homing_offset(servo_id, 0)
        time.sleep(0.02)
        actual = self.get_position(servo_id)
        if actual is None:
            return None
        offset = actual - POS_CENTER
        # actual∈[0,4095] ⇒ offset∈[-2048,2047]; the one out-of-range endpoint
        # costs a single count, harmless for a centre pose that's mid-range.
        offset = max(-HOMING_MAX_MAGNITUDE, min(HOMING_MAX_MAGNITUDE, offset))
        return offset if self.set_homing_offset(servo_id, offset) else None

    # ── Bulk operations ───────────────────────────────────────────────────────

    def get_all_positions(self) -> dict[int, Optional[int]]:
        return {sid: self.get_position(sid) for sid in JOINT_IDS}

    def set_all_positions(self, positions: dict[int, int]) -> None:
        for sid, pos in positions.items():
            if pos is not None:
                self.set_position(sid, pos)

    def set_all_torque(self, enable: bool) -> None:
        for sid in JOINT_IDS:
            self.set_torque(sid, enable)

    def get_all_status(self) -> dict[int, JointStatus]:
        out: dict[int, JointStatus] = {}
        for sid in JOINT_IDS:
            online = self.ping(sid)
            if online:
                pos = self.get_position(sid)
                s = JointStatus(
                    id=sid,
                    name=JOINT_NAMES[sid - 1],
                    online=True,
                    position=pos,
                    angle_deg=self.position_to_degrees(pos) if pos is not None else None,
                    temperature=self.get_temperature(sid),
                    voltage=self.get_voltage(sid),
                    load=self.get_load(sid),
                    torque_enabled=self.get_torque(sid),
                )
            else:
                s = JointStatus(id=sid, name=JOINT_NAMES[sid - 1], online=False)
            out[sid] = s
        return out

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def position_to_degrees(pos: Optional[int]) -> Optional[float]:
        if pos is None:
            return None
        return round(((pos - POS_CENTER) / 4095.0) * 360.0, 2)

    @staticmethod
    def degrees_to_position(deg: float) -> int:
        return int(POS_CENTER + (deg / 360.0) * 4095)
