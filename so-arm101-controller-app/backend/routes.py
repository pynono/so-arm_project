"""FastAPI routes for the SO-ARM101 controller."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Depends, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field

from .controller import RobotController
from sdk.driver_sdk import JOINT_IDS, JOINT_LIMITS, JOINT_NAMES


MODEL_DIR = Path(__file__).resolve().parent.parent / "model"


# ── Request models ───────────────────────────────────────────────────────────


class ConnectReq(BaseModel):
    port: Optional[str] = None


class MoveReq(BaseModel):
    joint_id: int = Field(..., ge=1, le=6)
    position: int = Field(..., ge=0, le=4095)
    speed: int = Field(200, ge=0, le=2000)


class MoveAllReq(BaseModel):
    positions: dict[str, int]
    speed: int = Field(200, ge=0, le=2000)


class TorqueReq(BaseModel):
    joint_id: Optional[int] = None
    enable: bool = True


class CenterReq(BaseModel):
    speed: int = Field(200, ge=0, le=2000)


class PoseSaveReq(BaseModel):
    name: str


class PoseGoReq(BaseModel):
    name: str
    speed: int = Field(200, ge=0, le=2000)


class RecordStartReq(BaseModel):
    interval_ms: int = Field(100, ge=20, le=2000)


class PlaybackReq(BaseModel):
    speed: float = Field(1.0, ge=0.1, le=5.0)
    loop: bool = False


class RecordFileReq(BaseModel):
    name: str


class RecordLoadReq(BaseModel):
    frames: list[dict]


class PermitReq(BaseModel):
    permitted: bool


# ── Router factory ───────────────────────────────────────────────────────────


# Loopback hosts the admin endpoints accept. The main-PC operator must
# reach the controller via http://localhost:8000 or http://127.0.0.1:8000
# to see the team-members panel; students on other laptops will not.
LOCAL_HOSTS = {"127.0.0.1", "::1", "localhost"}


def _client_identity(request: Request) -> tuple[str, str, str]:
    """Pull cid / name / IP. cid falls back to ip:<host> for header-less
    callers (curl, tests) so each peer still has a stable identity."""
    ip = request.client.host if request.client else ""
    cid = request.headers.get("x-client-id") or f"ip:{ip or 'anon'}"
    name = request.headers.get("x-client-name") or "anonymous"
    return cid, name, ip


def _is_local(request: Request) -> bool:
    host = request.client.host if request.client else ""
    return host in LOCAL_HOSTS


def create_router(controller: RobotController) -> APIRouter:
    router = APIRouter(prefix="/api", tags=["soarm"])

    def require_permitted(request: Request) -> tuple[str, str, str]:
        """Write-endpoint dep. Touch already happened in the HTTP
        middleware (see app.py), so we just check permission."""
        cid, name, ip = _client_identity(request)
        if controller.clients.is_permitted(cid):
            return cid, name, ip
        raise HTTPException(
            status_code=423,
            detail={
                "error": "your control was revoked by the instructor",
                "client": controller.clients.get(cid),
            },
        )

    def require_admin(request: Request) -> None:
        if not _is_local(request):
            raise HTTPException(
                status_code=403,
                detail="admin endpoints are only reachable from the main PC "
                       "(http://localhost:8000 or http://127.0.0.1:8000)",
            )

    @router.get("/info")
    def info(request: Request) -> dict:
        kin = json.loads((MODEL_DIR / "kinematics.json").read_text())
        return {
            "robot": "SO-ARM101",
            "team": controller.team or "",
            "is_admin": _is_local(request),
            "joints": [{"id": sid, "name": name}
                       for sid, name in zip(JOINT_IDS, JOINT_NAMES)],
            "joint_limits": {str(k): v for k, v in JOINT_LIMITS.items()},
            "kinematics": kin,
        }

    @router.get("/status")
    def status() -> dict:
        return controller.get_status()

    @router.post("/connect")
    def connect(req: ConnectReq) -> dict:
        return controller.connect(req.port)

    @router.post("/disconnect")
    def disconnect() -> dict:
        return controller.disconnect()

    @router.post("/move")
    def move(req: MoveReq, _=Depends(require_permitted)) -> dict:
        ok = controller.set_joint_position(req.joint_id, req.position, req.speed)
        return {"ok": ok}

    @router.post("/move_all")
    def move_all(req: MoveAllReq, _=Depends(require_permitted)) -> dict:
        positions = {int(k): v for k, v in req.positions.items()}
        controller.set_all_positions(positions, req.speed)
        return {"ok": True}

    @router.post("/center")
    def center(req: CenterReq, _=Depends(require_permitted)) -> dict:
        controller.center_all(req.speed)
        return {"ok": True}

    @router.post("/torque")
    def torque(req: TorqueReq, _=Depends(require_permitted)) -> dict:
        if req.joint_id is None:
            controller.set_all_torque(req.enable)
        else:
            controller.set_torque(req.joint_id, req.enable)
        return {"ok": True}

    # ── Poses ──
    @router.get("/poses")
    def poses() -> dict:
        return controller.list_poses()

    @router.post("/poses/save")
    def pose_save(req: PoseSaveReq) -> dict:
        return controller.save_pose(req.name)

    @router.post("/poses/go")
    def pose_go(req: PoseGoReq, _=Depends(require_permitted)) -> dict:
        return controller.goto_pose(req.name, req.speed)

    @router.delete("/poses/{name}")
    def pose_delete(name: str) -> dict:
        return controller.delete_pose(name)

    # ── Recording ──
    @router.post("/record/start")
    def rec_start(req: RecordStartReq) -> dict:
        return controller.start_recording(req.interval_ms)

    @router.post("/record/stop")
    def rec_stop() -> dict:
        return controller.stop_recording()

    @router.post("/record/clear")
    def rec_clear() -> dict:
        return controller.clear_recording()

    @router.get("/record")
    def rec_get() -> dict:
        return {"frames": controller.get_recording()}

    @router.post("/record/load")
    def rec_load(req: RecordLoadReq) -> dict:
        return controller.load_recording(req.frames)

    @router.post("/record/save")
    def rec_save(req: RecordFileReq) -> dict:
        return controller.save_recording_file(req.name)

    @router.post("/record/open")
    def rec_open(req: RecordFileReq) -> dict:
        return controller.load_recording_file(req.name)

    @router.get("/record/list")
    def rec_list() -> dict:
        return {"items": controller.list_recordings()}

    @router.delete("/record/{name}")
    def rec_delete(name: str) -> dict:
        return controller.delete_recording(name)

    # ── Playback ──
    @router.post("/playback/start")
    def pb_start(req: PlaybackReq, _=Depends(require_permitted)) -> dict:
        return controller.start_playback(req.speed, req.loop)

    @router.post("/playback/stop")
    def pb_stop() -> dict:
        return controller.stop_playback()

    # ── Self / clients ──
    @router.get("/me")
    def me(request: Request) -> dict:
        cid, name, ip = _client_identity(request)
        return {
            "cid": cid, "name": name, "ip": ip,
            "is_admin": _is_local(request),
            "permitted": controller.clients.is_permitted(cid),
        }

    @router.get("/clients")
    def clients_list(_admin: None = Depends(require_admin)) -> dict:
        return {"clients": controller.clients.list_all()}

    @router.post("/clients/{cid}/permission")
    def clients_set_permission(cid: str, req: PermitReq,
                               _admin: None = Depends(require_admin)) -> dict:
        if not controller.clients.set_permitted(cid, req.permitted):
            raise HTTPException(404, detail="unknown client")
        controller.broadcast_clients()
        return {"ok": True, "client": controller.clients.get(cid)}

    @router.delete("/clients/{cid}")
    def clients_forget(cid: str,
                       _admin: None = Depends(require_admin)) -> dict:
        forgotten = controller.clients.forget(cid)
        if forgotten:
            controller.broadcast_clients()
        return {"ok": forgotten}

    # ── WebSocket ──
    # Plain WebSocket subscriber. Critically, NO Depends here — putting
    # any dependency that takes `Request: Request` on a websocket route
    # crashes with "missing argument: request". WS clients only consume
    # broadcasts; identification happens out-of-band via HTTP requests.
    @router.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        queue = controller.subscribe()
        try:
            await ws.send_json({"type": "status", "data": controller.get_status()})
            await ws.send_json({"type": "clients",
                                "data": controller.clients.list_all()})
            while True:
                message = await queue.get()
                await ws.send_json(message)
        except WebSocketDisconnect:
            pass
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            print(f"[ws] error: {exc}")
        finally:
            controller.unsubscribe(queue)

    return router
