"""
High-level robot controller.

Wraps the STS3215Driver and adds:
  - background status polling
  - pose save/load
  - recording & playback
  - per-subscriber broadcast queue for WebSocket streaming
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from sdk.driver_sdk import (
    CALIBRATION_LABEL,
    JOINT_IDS, JOINT_LIMITS, JOINT_NAMES,
    POS_CENTER, POS_MAX, POS_MIN, POS_RANGE,
    STS3215Driver,
    position_from_fraction, unwrap_position, window_fraction, wrap_position,
)

# Marks a poses/recordings payload as holding 0..1 fractions of each joint's
# travel rather than one arm's raw counts. Absent = the older raw format.
POSE_FORMAT = "normalized"

# Moderate acceleration — lets the servo ramp smoothly between goal-position
# updates instead of slamming. Range 0-254 where 0 = max. ~32 gives roughly
# 150ms to reach goal speed; any less and rapid position updates look jittery.
DEFAULT_ACCEL = 32
DEFAULT_SPEED = 200


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "data"
RECORDINGS_DIR = DATA_DIR / "recordings"
POSES_FILE = DATA_DIR / "poses.json"

class ClientRegistry:
    """Per-client moderation registry — what the classroom controller uses
    in place of a single-driver lock.

    Every client (browser tab, SDK session) is identified by an opaque
    `cid` (browser localStorage UUID, or `sdk-...` for the Python client)
    sent in the `X-Client-Id` header. The first time we see a cid we
    auto-register it with `permitted=True`; the main-PC operator can
    revoke or re-grant per-cid via the admin endpoints. Revoked clients
    get 423 on every write.

    No driver/holder concept — multiple clients can write concurrently.
    Concurrency is handled socially.
    """

    # Clients we haven't heard from in this many seconds get auto-pruned
    # next time the registry is read. Keeps the team panel from filling up
    # with stale rows after students rerun their script several times.
    STALE_AFTER_SEC = 120.0

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clients: dict[str, dict] = {}

    def touch(self, cid: str, name: str, ip: str) -> bool:
        """Record a sighting. Returns True iff this is a brand-new cid
        (so the route layer can broadcast a registry update)."""
        with self._lock:
            now = time.time()
            entry = self._clients.get(cid)
            if entry is None:
                self._clients[cid] = {
                    "name": (name or "anonymous"),
                    "ip": ip or "",
                    "permitted": True,
                    "first_seen": now,
                    "last_seen": now,
                }
                return True
            if name and name != entry["name"]:
                entry["name"] = name
            if ip and ip != entry.get("ip"):
                entry["ip"] = ip
            entry["last_seen"] = now
            return False

    def is_permitted(self, cid: str) -> bool:
        with self._lock:
            entry = self._clients.get(cid)
            return True if entry is None else entry["permitted"]

    def set_permitted(self, cid: str, value: bool) -> bool:
        with self._lock:
            entry = self._clients.get(cid)
            if entry is None:
                return False
            entry["permitted"] = bool(value)
            return True

    def forget(self, cid: str) -> bool:
        with self._lock:
            return self._clients.pop(cid, None) is not None

    def _prune_locked(self) -> list[str]:
        """Drop clients whose last_seen is older than STALE_AFTER_SEC. Must
        be called with self._lock held. Returns the cids that were pruned
        so the caller can decide whether to broadcast."""
        cutoff = time.time() - self.STALE_AFTER_SEC
        stale = [cid for cid, info in self._clients.items()
                 if info.get("last_seen", 0) < cutoff]
        for cid in stale:
            self._clients.pop(cid, None)
        return stale

    def list_all(self) -> list[dict]:
        with self._lock:
            self._prune_locked()
            return [{"cid": cid, **info} for cid, info in self._clients.items()]

    def prune_stale(self) -> list[str]:
        """Public prune — returns pruned cids. Useful before broadcasting."""
        with self._lock:
            return self._prune_locked()

    def get(self, cid: str) -> Optional[dict]:
        with self._lock:
            entry = self._clients.get(cid)
            return {"cid": cid, **entry} if entry else None


class RobotController:
    """Owns the driver and exposes async-friendly operations."""

    def __init__(self, port: str = "/dev/ttyACM0", team: str = ""):
        self.driver = STS3215Driver(port=port)
        self.team = team
        self.clients = ClientRegistry()

        DATA_DIR.mkdir(parents=True, exist_ok=True)
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

        self._status_cache: dict[str, Any] = self._empty_status()
        self._status_lock = threading.Lock()

        self._poll_thread: Optional[threading.Thread] = None
        self._poll_stop = threading.Event()

        self._recording = False
        self._record_frames: list[dict] = []
        self._record_thread: Optional[threading.Thread] = None

        self._playing = False
        self._play_thread: Optional[threading.Thread] = None

        self._subscribers: set[asyncio.Queue] = set()
        self._main_loop: Optional[asyncio.AbstractEventLoop] = None

        # Per-joint cache so we only rewrite goal_speed when it actually
        # changes. Rewriting every tick forces the servo to restart its
        # motion profile and is the main cause of high-frequency jitter.
        self._last_speed: dict[int, int] = {}

        self.saved_poses: dict[str, dict] = self._load_poses_file()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Register the main asyncio loop so background threads can push updates."""
        self._main_loop = loop

    def shutdown(self) -> None:
        self._poll_stop.set()
        self._recording = False
        self._playing = False
        if self.driver.is_connected():
            try:
                self.driver.set_all_torque(False)
            except Exception:
                pass
            self.driver.disconnect()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self, port: Optional[str] = None) -> dict:
        if port:
            self.driver.port = port
        if self.driver.is_connected():
            return {"ok": True, "connected": True, "port": self.driver.port}
        ok = self.driver.connect()
        if ok:
            self._apply_motion_defaults()
            self._start_polling()
        return {"ok": ok, "connected": ok, "port": self.driver.port}

    def _apply_motion_defaults(self) -> None:
        """On connect, seed each joint with a moderate acceleration and our
        default goal_speed. After this we only rewrite these registers if
        they actually change."""
        for sid in JOINT_IDS:
            try:
                self.driver.set_acceleration(sid, DEFAULT_ACCEL)
                self.driver.set_speed(sid, DEFAULT_SPEED)
                self._last_speed[sid] = DEFAULT_SPEED
            except Exception as exc:
                print(f"[controller] init joint {sid}: {exc}")

    def disconnect(self) -> dict:
        self._recording = False
        self._playing = False
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None
        self.driver.disconnect()
        with self._status_lock:
            self._status_cache = self._empty_status()
        self._broadcast({"type": "status", "data": self._status_cache})
        return {"ok": True, "connected": False}

    def is_connected(self) -> bool:
        return self.driver.is_connected()

    # ── Polling / broadcast ───────────────────────────────────────────────────

    def _start_polling(self) -> None:
        if self._poll_thread and self._poll_thread.is_alive():
            return
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        fast_tick = 0.12   # positions ~8 Hz
        slow_tick = 2.0    # full status every 2 s
        prune_tick = 30.0  # prune stale clients every 30 s
        last_slow = 0.0
        last_prune = 0.0

        while not self._poll_stop.is_set():
            t0 = time.time()
            try:
                positions = self.driver.get_all_positions()
                snapshot = {
                    "connected": self.driver.is_connected(),
                    "port": self.driver.port,
                    "positions": {str(sid): positions.get(sid) for sid in JOINT_IDS},
                    "angles_deg": {
                        str(sid): STS3215Driver.position_to_degrees(positions.get(sid))
                        for sid in JOINT_IDS
                    },
                }
                with self._status_lock:
                    self._status_cache.update(snapshot)

                if t0 - last_slow > slow_tick:
                    full = self.driver.get_all_status()
                    joints = {str(sid): s.to_dict() for sid, s in full.items()}
                    with self._status_lock:
                        self._status_cache["joints"] = joints
                    last_slow = t0

                self._broadcast({"type": "status", "data": self._status_cache})

                # Sweep stale clients periodically so the team panel stays
                # accurate even if nobody triggers /api/clients manually.
                if t0 - last_prune > prune_tick:
                    if self.clients.prune_stale():
                        self.broadcast_clients()
                    last_prune = t0

            except Exception as exc:
                print(f"[controller] poll error: {exc}")

            elapsed = time.time() - t0
            time.sleep(max(0.0, fast_tick - elapsed))

    def _broadcast(self, message: dict) -> None:
        if self._main_loop is None or not self._subscribers:
            return
        snapshot = dict(message)
        for q in list(self._subscribers):
            try:
                self._main_loop.call_soon_threadsafe(q.put_nowait, snapshot)
            except Exception:
                pass

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=64)
        self._subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subscribers.discard(q)

    def get_status(self) -> dict:
        with self._status_lock:
            return json.loads(json.dumps(self._status_cache))

    def broadcast_clients(self) -> None:
        """Push the full client list to every WebSocket subscriber. Called
        when a new client is first seen or the admin toggles permission."""
        self._broadcast({"type": "clients", "data": self.clients.list_all()})

    @staticmethod
    def _empty_status() -> dict:
        return {
            "connected": False,
            "port": "",
            "positions": {str(sid): None for sid in JOINT_IDS},
            "angles_deg": {str(sid): None for sid in JOINT_IDS},
            "joints": {
                str(sid): {
                    "id": sid,
                    "name": JOINT_NAMES[sid - 1],
                    "online": False,
                }
                for sid in JOINT_IDS
            },
            "recording": False,
            "playing": False,
        }

    # ── Joint commands ────────────────────────────────────────────────────────

    def _clamp_position(self, joint_id: int, position: int) -> int:
        """Clamp to the joint's calibrated travel and return a real servo count.

        Accepts either a raw count or an already-unwrapped window coordinate —
        unwrap_position() is idempotent, so callers don't have to care which
        they hold. That matters because positions reach here from the UI (which
        works unwrapped), from saved poses, and from recordings (both raw)."""
        lim = JOINT_LIMITS.get(joint_id)
        if not lim:
            return max(POS_MIN, min(POS_MAX, int(position)))
        if lim["max"] <= POS_MAX:
            return max(lim["min"], min(lim["max"], int(position)))

        # Wrapped window: unwrapping puts pos in [min, min+4096), so the only
        # way out of range is past max — into the arc the joint can't reach.
        # Snap to whichever end is nearer *round the circle*, not numerically:
        # a count just below min is a hair past the min stop, not a full sweep
        # away from max.
        pos = unwrap_position(int(position), lim)
        if pos > lim["max"]:
            past_max = pos - lim["max"]
            below_min = (lim["min"] + POS_RANGE) - pos
            pos = lim["max"] if past_max <= below_min else lim["min"]
        return wrap_position(pos)

    def _maybe_update_speed(self, joint_id: int, speed: int) -> None:
        """Write goal_speed only if it changed — critical for smooth motion."""
        if speed <= 0:
            return
        if self._last_speed.get(joint_id) != speed:
            self.driver.set_speed(joint_id, speed)
            self._last_speed[joint_id] = speed

    def set_joint_position(self, joint_id: int, position: int,
                           speed: int = DEFAULT_SPEED) -> bool:
        self._maybe_update_speed(joint_id, speed)
        pos = self._clamp_position(joint_id, position)
        return self.driver.set_position(joint_id, pos)

    def set_all_positions(self, positions: dict[int, int],
                          speed: int = DEFAULT_SPEED) -> None:
        for sid in positions:
            self._maybe_update_speed(sid, speed)
        clamped = {sid: self._clamp_position(sid, p)
                   for sid, p in positions.items() if p is not None}
        self.driver.set_all_positions(clamped)

    def set_torque(self, joint_id: int, enable: bool) -> bool:
        return self.driver.set_torque(joint_id, enable)

    def set_all_torque(self, enable: bool) -> None:
        self.driver.set_all_torque(enable)

    def center_all(self, speed: int = DEFAULT_SPEED) -> None:
        self.driver.set_all_torque(True)
        for sid in JOINT_IDS:
            self._maybe_update_speed(sid, speed)
            self.driver.set_position(sid, POS_CENTER)

    # ── Poses ─────────────────────────────────────────────────────────────────

    def _load_poses_file(self) -> dict[str, dict]:
        if POSES_FILE.exists():
            try:
                return json.loads(POSES_FILE.read_text())
            except Exception:
                return {}
        return {}

    def _save_poses_file(self) -> None:
        POSES_FILE.write_text(json.dumps(self.saved_poses, indent=2))

    def _encode_positions(self, positions: dict) -> dict:
        """Raw counts → 0..1 fractions of each joint's travel."""
        return {str(sid): window_fraction(positions.get(sid), JOINT_LIMITS.get(sid))
                for sid in JOINT_IDS}

    def _decode_positions(self, positions: dict, normalized: bool) -> dict[int, int]:
        """Stored payload → raw counts for THIS arm.

        `normalized` is false for files written before poses were arm-independent;
        those hold the teaching arm's raw counts and can only be replayed as-is."""
        out = {}
        for key, val in (positions or {}).items():
            if val is None:
                continue
            sid = int(key)
            lim = JOINT_LIMITS.get(sid)
            if lim is None:
                continue
            out[sid] = position_from_fraction(val, lim) if normalized else int(val)
        return out

    def save_pose(self, name: str) -> dict:
        # Stored as fractions of each joint's travel, not raw counts: raw only
        # means something on the arm that produced it, and these files outlive
        # the arm — they get copied between the classroom's robots.
        positions = self.driver.get_all_positions()
        entry = {
            "positions": self._encode_positions(positions),
            "format": POSE_FORMAT,
            "robot": CALIBRATION_LABEL,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.saved_poses[name] = entry
        self._save_poses_file()
        return {"ok": True, "name": name, "pose": entry}

    def delete_pose(self, name: str) -> dict:
        if name in self.saved_poses:
            del self.saved_poses[name]
            self._save_poses_file()
            return {"ok": True}
        return {"ok": False, "error": "not found"}

    def list_poses(self) -> dict:
        # Resolve to this arm's raw counts for display — the stored fractions
        # would mean nothing in the UI's raw column.
        out = {}
        for name, entry in self.saved_poses.items():
            raw = self._decode_positions(entry.get("positions"),
                                         entry.get("format") == POSE_FORMAT)
            out[name] = {**entry,
                         "positions": {str(sid): raw.get(sid) for sid in JOINT_IDS}}
        return out

    def goto_pose(self, name: str, speed: int = 200) -> dict:
        pose = self.saved_poses.get(name)
        if not pose:
            return {"ok": False, "error": "not found"}
        positions = self._decode_positions(pose.get("positions"),
                                           pose.get("format") == POSE_FORMAT)
        self.driver.set_all_torque(True)
        self.set_all_positions(positions, speed)
        return {"ok": True}

    # ── Recording ─────────────────────────────────────────────────────────────

    def start_recording(self, interval_ms: int = 100) -> dict:
        if self._recording:
            return {"ok": False, "error": "already recording"}
        self._record_frames = []
        self._recording = True

        def loop() -> None:
            t0 = time.time()
            interval = max(20, int(interval_ms)) / 1000.0
            while self._recording:
                positions = self.driver.get_all_positions()
                self._record_frames.append({
                    "time": round(time.time() - t0, 3),
                    "positions": {str(sid): positions.get(sid) for sid in JOINT_IDS},
                })
                self._broadcast({
                    "type": "recording",
                    "data": {"frames": len(self._record_frames)},
                })
                time.sleep(interval)

        self._record_thread = threading.Thread(target=loop, daemon=True)
        self._record_thread.start()
        with self._status_lock:
            self._status_cache["recording"] = True
        return {"ok": True}

    def stop_recording(self) -> dict:
        self._recording = False
        if self._record_thread:
            self._record_thread.join(timeout=1.0)
            self._record_thread = None
        with self._status_lock:
            self._status_cache["recording"] = False
        return {"ok": True, "frames": len(self._record_frames)}

    def clear_recording(self) -> dict:
        self._record_frames = []
        return {"ok": True}

    def get_recording(self) -> list[dict]:
        return list(self._record_frames)

    def load_recording(self, frames: list[dict]) -> dict:
        self._record_frames = list(frames or [])
        return {"ok": True, "frames": len(self._record_frames)}

    def save_recording_file(self, name: str) -> dict:
        # Frames convert at the file boundary only. In memory they stay raw, so
        # the play loop, the WS broadcast and /recording/frames are untouched;
        # a take only has to be arm-independent once it outlives this session.
        if not name:
            return {"ok": False, "error": "empty name"}
        safe = "".join(c for c in name if c.isalnum() or c in "-_ ")
        path = RECORDINGS_DIR / f"{safe or 'recording'}.json"
        payload = {
            "format": POSE_FORMAT,
            "robot": CALIBRATION_LABEL,
            "frames": [
                {"time": f.get("time"),
                 "positions": self._encode_positions(
                     {int(k): v for k, v in (f.get("positions") or {}).items()})}
                for f in self._record_frames
            ],
        }
        path.write_text(json.dumps(payload, indent=2))
        return {"ok": True, "path": str(path.relative_to(PROJECT_ROOT))}

    def load_recording_file(self, name: str) -> dict:
        path = RECORDINGS_DIR / f"{name}"
        if not path.suffix:
            path = path.with_suffix(".json")
        if not path.exists():
            return {"ok": False, "error": "not found"}
        try:
            self._record_frames = self._decode_recording(json.loads(path.read_text()))
            return {"ok": True, "frames": len(self._record_frames)}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def _decode_recording(self, data) -> list[dict]:
        """Recording file → raw frames for THIS arm. A bare list is the older
        raw-count format, kept readable so existing takes still play."""
        if isinstance(data, list):
            return data
        normalized = data.get("format") == POSE_FORMAT
        out = []
        for f in data.get("frames") or []:
            raw = self._decode_positions(f.get("positions"), normalized)
            out.append({"time": f.get("time"),
                        "positions": {str(sid): raw.get(sid) for sid in JOINT_IDS}})
        return out

    def list_recordings(self) -> list[dict]:
        # Name + frame count + length, so the UI can list takes without
        # opening each one first.
        items = []
        for p in sorted(RECORDINGS_DIR.glob("*.json")):
            frames, seconds, robot = 0, 0.0, None
            try:
                data = json.loads(p.read_text())
                # A bare list is the older raw-count format; newer takes wrap
                # their frames so they can name the arm they were taught on.
                frame_list = data if isinstance(data, list) else (data.get("frames") or [])
                robot = None if isinstance(data, list) else data.get("robot")
                frames = len(frame_list)
                if frames:
                    seconds = float(frame_list[-1].get("time", 0.0))
            except Exception:
                pass
            items.append({"name": p.stem, "frames": frames,
                          "seconds": round(seconds, 1), "robot": robot})
        return items

    def delete_recording(self, name: str) -> dict:
        path = RECORDINGS_DIR / f"{name}.json"
        if path.exists():
            path.unlink()
            return {"ok": True}
        return {"ok": False, "error": "not found"}

    # ── Playback ──────────────────────────────────────────────────────────────

    def start_playback(self, speed: float = 1.0, loop_play: bool = False) -> dict:
        if self._playing:
            return {"ok": False, "error": "already playing"}
        if not self._record_frames:
            return {"ok": False, "error": "empty recording"}
        speed = max(0.1, min(5.0, float(speed)))
        self._playing = True
        self.driver.set_all_torque(True)

        def loop_body() -> None:
            while self._playing:
                frames = list(self._record_frames)
                for i, frame in enumerate(frames):
                    if not self._playing:
                        break
                    positions = {int(k): v for k, v in frame["positions"].items()
                                 if v is not None}
                    # During playback keep whatever goal_speed was last set
                    # (the 0 here means "don't rewrite speed every frame").
                    self.set_all_positions(positions, 0)
                    if i + 1 < len(frames):
                        dt = frames[i + 1]["time"] - frame["time"]
                        time.sleep(max(0.0, dt / speed))
                if not loop_play:
                    break
            self._playing = False
            with self._status_lock:
                self._status_cache["playing"] = False
            self._broadcast({"type": "playback", "data": {"playing": False}})

        self._play_thread = threading.Thread(target=loop_body, daemon=True)
        self._play_thread.start()
        with self._status_lock:
            self._status_cache["playing"] = True
        return {"ok": True}

    def stop_playback(self) -> dict:
        self._playing = False
        return {"ok": True}
