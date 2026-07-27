"""
SO-ARM101 Python client for the classroom controller.

Designed for the multi-team Wi-Fi setup: each team's main PC runs the
controller with --team teamN, students hit it from their own laptops.

Quick start:
    from soarm import Robot

    bot = Robot("team1", name="Alice")        # → http://team1.local:8000
    bot.connect()                              # opens the USB serial port

    bot.move(2, 30)                            # joint 2 to +30°
    bot.move_all({1: 0, 2: 0, 3: 0})           # several joints at once
    bot.go_pose("home")                        # named pose saved in the UI
    bot.center()                               # all joints to mechanical center

    print(bot.angles())                        # current angles (deg)

Anyone with the team URL can drive by default. The instructor (the main
PC operator) can revoke a client from the local admin panel; revoked
clients get PermissionDeniedError on every write until restored.

The only third-party dependency is ``requests`` (everyone has it).
"""

from __future__ import annotations

import math
import os
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import requests


# Per-user persistent client id. We cache one uuid in ~/.soarm/cid and reuse
# it on every Python script the same user runs, so the team-members panel
# doesn't fill up with a fresh "anonymous" row every time `check.py` is
# rerun. The id is opaque (uuid4), not identifying.
def _persistent_client_id() -> str:
    override = os.environ.get("SOARM_CLIENT_ID")
    if override:
        return override
    path = Path.home() / ".soarm" / "cid"
    try:
        if path.exists():
            existing = path.read_text().strip()
            if existing:
                return existing
        path.parent.mkdir(parents=True, exist_ok=True)
        new_id = f"sdk-{uuid.uuid4().hex[:12]}"
        path.write_text(new_id + "\n")
        return new_id
    except OSError:
        # Read-only home, missing perms, etc. — fall back to a per-process id
        # (worse UX, but never block the user).
        return f"sdk-{uuid.uuid4().hex[:12]}"


# Position units used by the Feetech STS3215 servo:
#   0 .. 4095 → 360°. 2048 is mechanical center (= 0°).
POS_CENTER = 2048
POS_PER_DEG = 4095 / 360.0


def deg_to_pos(deg: float) -> int:
    """Convert ±180° to the servo's 0..4095 raw position."""
    return int(round(POS_CENTER + deg * POS_PER_DEG))


def pos_to_deg(pos: int) -> float:
    return round((pos - POS_CENTER) / POS_PER_DEG, 1)


# Joint name → id, so students can write `bot.move("elbow", 30)` instead
# of memorising IDs. Both forms work everywhere a joint id is accepted.
JOINT_BY_NAME = {
    "base":     1,
    "shoulder": 2,
    "elbow":    3,
    "wrist_pitch": 4, "wrist": 4,
    "wrist_roll":  5, "roll":  5,
    "gripper":  6,
}


def _resolve_joint(joint: "int | str") -> int:
    if isinstance(joint, int):
        return joint
    key = str(joint).lower().strip()
    if key not in JOINT_BY_NAME:
        raise ValueError(
            f"Unknown joint '{joint}'. Use 1..6 or one of "
            f"{sorted(set(JOINT_BY_NAME))}.")
    return JOINT_BY_NAME[key]


class ControllerError(RuntimeError):
    """Generic error returned by the controller HTTP API."""


class PermissionDeniedError(ControllerError):
    """The instructor has revoked this client's permission to drive.
    Ask them to restore it from the admin panel on the main PC."""


# Backwards-compatibility alias for code written against the previous
# class name. Prefer PermissionDeniedError in new code.
ControlLockError = PermissionDeniedError


@dataclass
class Status:
    connected: bool
    port: str
    positions: dict[int, Optional[int]]
    angles_deg: dict[int, Optional[float]]


class Robot:
    """Thin client for the SO-ARM101 web controller.

    Args:
        team:    "team1" or full URL like "http://192.168.0.42:8000".
                 If a bare team name is given we resolve it to
                 http://{team}.local:8000.
        name:    Display name shown to other teammates as the active driver.
        port:    HTTP port on the team PC (default 8000).
        timeout: HTTP timeout in seconds.
    """

    def __init__(self, team: str, *, name: str = "anonymous",
                 port: int = 8000, timeout: float = 5.0):
        self.team = team
        self.name = name or "anonymous"
        self.timeout = timeout
        # Persistent client id: same across Python script invocations on this
        # user account (cached in ~/.soarm/cid), so the controller's team
        # panel doesn't accumulate a fresh row each time a script reruns.
        # Override with $SOARM_CLIENT_ID if you ever need a one-off id.
        self.client_id = _persistent_client_id()

        if team.startswith(("http://", "https://")):
            self.base_url = team.rstrip("/")
        else:
            self.base_url = f"http://{team}.local:{port}"

        self._sess = requests.Session()
        self._sess.headers.update({
            "x-client-id": self.client_id,
            "x-client-name": self.name,
        })

    # ── Internal HTTP helpers ──────────────────────────────────────────────
    def _url(self, path: str) -> str:
        return f"{self.base_url}/api{path}"

    def _request(self, method: str, path: str, **kw) -> dict:
        try:
            r = self._sess.request(method, self._url(path),
                                   timeout=self.timeout, **kw)
        except requests.exceptions.ConnectionError as e:
            raise ControllerError(
                f"Cannot reach {self.base_url}. "
                f"Is the team PC on and the controller running? ({e})") from e

        if r.status_code == 423:
            try:
                detail = r.json().get("detail", {}) or {}
            except ValueError:
                detail = {}
            raise PermissionDeniedError(
                detail.get("error") or "your control was revoked by the instructor")
        if not r.ok:
            raise ControllerError(f"{method} {path} → {r.status_code}: {r.text[:200]}")
        try:
            return r.json()
        except ValueError:
            return {}

    # ── Connection / control lock ──────────────────────────────────────────
    def info(self) -> dict:
        """Static metadata (team label, joint names, kinematics)."""
        return self._request("GET", "/info")

    def connect(self, port: Optional[str] = None) -> dict:
        """Open the USB serial link on the team PC."""
        return self._request("POST", "/connect", json={"port": port})

    def disconnect(self) -> dict:
        return self._request("POST", "/disconnect")

    def me(self) -> dict:
        """What the server thinks of this client (cid, name, ip, permitted, is_admin)."""
        return self._request("GET", "/me")

    # ── Status ─────────────────────────────────────────────────────────────
    def status(self) -> Status:
        s = self._request("GET", "/status")
        positions = {int(k): v for k, v in (s.get("positions") or {}).items()}
        angles = {int(k): v for k, v in (s.get("angles_deg") or {}).items()}
        return Status(
            connected=bool(s.get("connected")),
            port=s.get("port") or "",
            positions=positions,
            angles_deg=angles,
        )

    def angles(self) -> dict[int, Optional[float]]:
        """Current joint angles in degrees, keyed by joint id."""
        return self.status().angles_deg

    # ── Motion ─────────────────────────────────────────────────────────────
    def move(self, joint: "int | str", angle_deg: float, speed: int = 200) -> dict:
        """Move a single joint to ``angle_deg`` (0° = mechanical center).

        For the gripper (joint 6) the angle is interpreted as % open
        if you stick to its mapped range; the server clamps to safe limits.
        """
        jid = _resolve_joint(joint)
        return self._request("POST", "/move", json={
            "joint_id": jid, "position": deg_to_pos(angle_deg), "speed": speed,
        })

    def move_raw(self, joint: "int | str", position: int, speed: int = 200) -> dict:
        """Move a joint to a raw 0..4095 servo position (advanced)."""
        jid = _resolve_joint(joint)
        return self._request("POST", "/move", json={
            "joint_id": jid, "position": int(position), "speed": speed,
        })

    def move_all(self, angles_deg: "dict[int|str, float]",
                 speed: int = 200) -> dict:
        """Move several joints in one go.

        Example: ``bot.move_all({"shoulder": 30, "elbow": -20})``
        """
        positions = {str(_resolve_joint(k)): deg_to_pos(v)
                     for k, v in angles_deg.items()}
        return self._request("POST", "/move_all", json={
            "positions": positions, "speed": speed,
        })

    def center(self, speed: int = 200) -> dict:
        """All joints to mechanical center (= each joint at 0°)."""
        return self._request("POST", "/center", json={"speed": speed})

    def torque(self, enable: bool, joint: "int | str | None" = None) -> dict:
        """Enable (lock) or disable (free / teach) servo torque.

        With no ``joint`` it applies to all six.
        """
        body = {"enable": bool(enable)}
        if joint is not None:
            body["joint_id"] = _resolve_joint(joint)
        return self._request("POST", "/torque", json=body)

    lock = lambda self, j=None: self.torque(True, j)            # noqa: E731
    unlock = lambda self, j=None: self.torque(False, j)         # noqa: E731

    # ── Poses ──────────────────────────────────────────────────────────────
    def poses(self) -> dict:
        return self._request("GET", "/poses")

    def save_pose(self, name: str) -> dict:
        return self._request("POST", "/poses/save", json={"name": name})

    def go_pose(self, name: str, speed: int = 200) -> dict:
        return self._request("POST", "/poses/go",
                             json={"name": name, "speed": speed})

    def delete_pose(self, name: str) -> dict:
        return self._request("DELETE", f"/poses/{name}")

    # ── Convenience ────────────────────────────────────────────────────────
    def wait_until_settled(self, tolerance_deg: float = 1.5,
                           timeout: float = 5.0) -> bool:
        """Poll ``/status`` until two consecutive samples are within
        ``tolerance_deg`` of each other on every joint, or ``timeout``
        elapses. Useful after a move() before the next one in a sequence.

        Returns True if the arm settled, False if the timeout was hit.
        """
        deadline = time.time() + timeout
        prev: dict[int, Optional[float]] = {}
        while time.time() < deadline:
            now = self.angles()
            if prev and all(
                a is not None and b is not None
                and abs(a - b) <= tolerance_deg
                for a, b in zip(prev.values(), now.values())
            ):
                return True
            prev = now
            time.sleep(0.15)
        return False

    def __enter__(self) -> "Robot":
        self.connect()
        return self

    def __exit__(self, *_exc) -> None:
        # Nothing to release — no holder concept in the moderation model.
        pass
