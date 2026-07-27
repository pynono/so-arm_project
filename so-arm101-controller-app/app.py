#!/usr/bin/env python3
"""
SO-ARM101 Web Controller — FastAPI entry point.

Serves:
  - Static frontend at /
  - STL meshes at /meshes
  - JSON API at /api
  - WebSocket at /api/ws
"""

from __future__ import annotations

import argparse
import asyncio
import os
import socket
import sys
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from backend.controller import RobotController
from backend.routes import create_router


ROOT = Path(__file__).resolve().parent
FRONTEND_DIR = ROOT / "frontend"
MESHES_DIR = ROOT / "model" / "assets"
MODEL_DIR = ROOT / "model"


def create_app(port_hint: str = "/dev/ttyACM0", team: str = "") -> FastAPI:
    controller = RobotController(port=port_hint, team=team)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        controller.bind_loop(asyncio.get_running_loop())
        yield
        controller.shutdown()

    app = FastAPI(title="SO-ARM101 Controller", lifespan=lifespan)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Prevent the browser from caching HTML/JS/CSS — students should always
    # get fresh code after a git pull, and stale imports have caused trouble
    # before (import map pointing at an old CDN, old app.js running, ...).
    @app.middleware("http")
    async def no_cache(request, call_next):
        response = await call_next(request)
        path = request.url.path
        if path == "/" or path.endswith((".html", ".js", ".css")):
            response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
            response.headers["Pragma"] = "no-cache"
            response.headers["Expires"] = "0"
        return response

    # Register every API caller into the client registry on every HTTP hit.
    # Lives here as middleware (not as a router-level dep) so it does NOT
    # fire on the WebSocket route — WS connections lack a Request object
    # and would crash a Depends() that asks for one.
    @app.middleware("http")
    async def register_client(request, call_next):
        path = request.url.path
        if path.startswith("/api/") or path == "/api":
            cid = request.headers.get("x-client-id")
            ip = request.client.host if request.client else ""
            if cid:
                name = request.headers.get("x-client-name") or "anonymous"
                if controller.clients.touch(cid, name, ip):
                    controller.broadcast_clients()
        return await call_next(request)

    app.include_router(create_router(controller))

    # Serve STL mesh files
    if MESHES_DIR.exists():
        app.mount("/meshes", StaticFiles(directory=str(MESHES_DIR)), name="meshes")

    # Serve kinematics JSON directly
    @app.get("/model/kinematics.json")
    def kinematics():
        return FileResponse(str(MODEL_DIR / "kinematics.json"),
                            media_type="application/json")

    # Serve frontend static bundle (mounted last so API routes win)
    if FRONTEND_DIR.exists():
        app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True),
                  name="frontend")

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-ARM101 web controller")
    # Default 0.0.0.0 so that team members on the same Wi-Fi can reach this
    # controller from their own laptops. Pass --host 127.0.0.1 to restrict
    # to the local machine.
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--serial", default=os.environ.get("SOARM_PORT", "/dev/ttyACM0"),
                        help="Default serial port (can be changed in UI)")
    parser.add_argument("--team", default=os.environ.get("SOARM_TEAM", ""),
                        help="Team label shown in the UI (e.g. team1)")
    parser.add_argument("--reload", action="store_true", help="Enable dev auto-reload")
    args = parser.parse_args()

    os.environ["SOARM_PORT"] = args.serial
    if args.team:
        os.environ["SOARM_TEAM"] = args.team

    _print_access_banner(args.host, args.port, args.team or os.environ.get("SOARM_TEAM", ""))

    try:
        import uvicorn  # noqa
    except ImportError:
        print("uvicorn is not installed. Run ./scripts/install.sh first.",
              file=sys.stderr)
        sys.exit(1)

    import uvicorn
    if args.reload:
        uvicorn.run("app:build_for_reload", factory=True,
                    host=args.host, port=args.port, reload=True)
    else:
        uvicorn.run(app_instance, host=args.host, port=args.port)


def _print_access_banner(host: str, port: int, team: str) -> None:
    """Print every URL students/laptops on the same Wi-Fi can use to reach
    this controller. Helps the operator copy-paste the right address."""
    label = f"  [Team: {team}]" if team else ""
    lines = [f"\n  SO-ARM101 controller{label}",
             "  ──────────────────────────────"]
    if host in ("0.0.0.0", "::"):
        hostname = socket.gethostname()
        lines.append(f"  Local:   http://127.0.0.1:{port}")
        lines.append(f"  mDNS:    http://{hostname}.local:{port}")
        try:
            # Trick to find the LAN IP without sending traffic.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                lan_ip = s.getsockname()[0]
            lines.append(f"  LAN:     http://{lan_ip}:{port}")
        except OSError:
            pass
    else:
        lines.append(f"  URL:     http://{host}:{port}")
    print("\n".join(lines) + "\n", flush=True)


def build_for_reload() -> FastAPI:
    """Factory used when --reload is set (uvicorn re-imports this module)."""
    return create_app(
        port_hint=os.environ.get("SOARM_PORT", "/dev/ttyACM0"),
        team=os.environ.get("SOARM_TEAM", ""),
    )


app_instance: FastAPI = create_app(
    port_hint=os.environ.get("SOARM_PORT", "/dev/ttyACM0"),
    team=os.environ.get("SOARM_TEAM", ""),
)


if __name__ == "__main__":
    main()
