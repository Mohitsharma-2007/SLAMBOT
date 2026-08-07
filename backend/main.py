"""SLAM Bot backend — FastAPI app entry point.

Run with:  uvicorn main:app --host 0.0.0.0 --port 8000

This one process is, per §4: a WebSocket server for both MCUs, a WebSocket
server for the browser, a ROS2 node (in a background thread), the process that
shells out to arduino-cli, and the OpenRouter client.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
from typing import Any

from fastapi import APIRouter, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

import tuning
import ws_frontend
import ws_robot
from ai_advisor import ADVISOR, summarize_map
from config import SETTINGS
from flasher import FLASHER, FlashBusy
from ros_bridge import BRIDGE
from state import STATE, now_ms

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
)
logger = logging.getLogger("slam_bot")

api = APIRouter(prefix="/api")


# ---------------------------------------------------------------------------
# REST API
# ---------------------------------------------------------------------------
@api.get("/status")
async def get_status() -> dict[str, Any]:
    async with STATE.lock:
        return STATE.status_snapshot()


@api.get("/tuning")
async def get_tuning() -> dict[str, Any]:
    async with STATE.lock:
        snapshot = STATE.tuning_snapshot()
    snapshot["schema"] = tuning.schema()
    snapshot["sections"] = tuning.sections()
    return snapshot


@api.get("/scan")
async def get_scan() -> dict[str, Any]:
    async with STATE.lock:
        return STATE.scan.snapshot()


@api.get("/map")
async def get_map() -> dict[str, Any]:
    async with STATE.lock:
        if STATE.map.is_empty:
            raise HTTPException(
                status_code=404,
                detail="no map yet — is slam_toolbox running and receiving /scan?",
            )
        return STATE.map.snapshot()


@api.get("/logs")
async def get_logs(
    limit: int = 300, sources: str | None = None, levels: str | None = None
) -> dict[str, Any]:
    src = [s.strip() for s in sources.split(",")] if sources else None
    lvl = [l.strip() for l in levels.split(",")] if levels else None
    async with STATE.lock:
        return {
            "entries": STATE.logs.history(limit=limit, sources=src, levels=lvl),
            "counts": STATE.logs.counts(),
        }


@api.get("/session")
async def get_session() -> dict[str, Any]:
    async with STATE.lock:
        return STATE.session_summary()


# -- flash center (§6.4) ----------------------------------------------------
@api.get("/flash/probe")
async def flash_probe() -> dict[str, Any]:
    info = await FLASHER.probe()
    # The spec asks for this warning to be explicit in the UI.
    info["note"] = (
        "Arduino flashing runs on the backend host. The board must be plugged "
        "into the machine running this server, not into your browsing device, "
        "if they differ. NodeMCU flashing happens in your browser over "
        "WebSerial and needs no backend."
    )
    return info


@api.get("/flash/ports")
async def flash_ports() -> dict[str, Any]:
    return {"ports": await FLASHER.list_ports()}


@api.get("/flash/artifact/{target}")
async def flash_artifact(target: str):
    """Serve the last compiled binary — this is the .bin esptool-js flashes."""
    artifact = FLASHER.artifact_for(target)
    if artifact is None or not artifact.is_file():
        raise HTTPException(
            status_code=404,
            detail=f"no build artifact for {target!r} — compile it first",
        )
    return FileResponse(
        artifact, media_type="application/octet-stream", filename=artifact.name
    )


# -- AI (§9) ---------------------------------------------------------------
@api.get("/ai/status")
async def ai_status() -> dict[str, Any]:
    return {
        "configured": ADVISOR.configured,
        "model": ADVISOR.model,
        "candidates": SETTINGS.openrouter_candidates,
        "free_model_count": len(ADVISOR.free_models),
        "last_error": ADVISOR.last_error,
    }


@api.post("/ai/resolve-model")
async def ai_resolve_model() -> dict[str, Any]:
    model = await ADVISOR.resolve_model()
    async with STATE.lock:
        STATE.ai_model = model
        STATE.ai_last_error = ADVISOR.last_error
        STATE.logs.emit("ai", "info", f"resolved AI model: {model}")
    return {"model": model, "last_error": ADVISOR.last_error}


@api.post("/ai/analyze-session")
async def ai_analyze_session() -> dict[str, Any]:
    async with STATE.lock:
        summary = STATE.session_summary()
        STATE.logs.emit("ai", "info", "analyzing last session")
    result = await ADVISOR.analyze_session(summary)
    async with STATE.lock:
        STATE.ai_last_error = ADVISOR.last_error
        if result.get("ok"):
            STATE.logs.emit(
                "ai",
                "info",
                f"session analysis returned {len(result.get('suggestions', []))} "
                "suggestion(s)",
            )
        else:
            STATE.logs.emit("ai", "error", f"session analysis failed: {result.get('error')}")
    result["session_summary"] = summary
    return result


@api.post("/ai/describe-map")
async def ai_describe_map() -> dict[str, Any]:
    async with STATE.lock:
        if STATE.map.is_empty:
            raise HTTPException(status_code=404, detail="no map to describe yet")
        width, height = STATE.map.width, STATE.map.height
        resolution = STATE.map.resolution
        rle = list(STATE.map.rle)

    # Flood-fill on a large grid is CPU-bound; keep it off the event loop.
    summary = await asyncio.to_thread(summarize_map, width, height, resolution, rle)
    result = await ADVISOR.describe_map(summary)
    result["map_summary"] = summary
    async with STATE.lock:
        STATE.ai_last_error = ADVISOR.last_error
        STATE.logs.emit(
            "ai",
            "info" if result.get("ok") else "error",
            "map description generated"
            if result.get("ok")
            else f"map description failed: {result.get('error')}",
        )
    return result


@api.get("/ai/map-summary")
async def ai_map_summary() -> dict[str, Any]:
    """The local numeric analysis on its own — no API call, no key needed."""
    async with STATE.lock:
        if STATE.map.is_empty:
            raise HTTPException(status_code=404, detail="no map yet")
        width, height = STATE.map.width, STATE.map.height
        resolution = STATE.map.resolution
        rle = list(STATE.map.rle)
    return await asyncio.to_thread(summarize_map, width, height, resolution, rle)


@api.post("/ai/chat")
async def ai_chat(payload: dict[str, Any]) -> dict[str, Any]:
    message = str(payload.get("message", "")).strip()
    if not message:
        raise HTTPException(status_code=400, detail="'message' is required")
    if len(message) > 4000:
        raise HTTPException(status_code=400, detail="message too long (max 4000 chars)")

    async with STATE.lock:
        context = {
            "running": STATE.running,
            "control_mode": STATE.control_mode,
            "devices": {
                "arduino": STATE.arduino.connected,
                "nodemcu": STATE.nodemcu.connected,
            },
            "odom": STATE.odom.snapshot(),
            "scan": {
                "samples": len(STATE.scan.angles_deg),
                "min_distance_mm": STATE.scan.min_distance_mm,
            },
            "session": STATE.session_summary(),
            "ros_available": STATE.ros_available,
        }
        STATE.logs.emit("ai", "info", f"chat: {message[:80]}")
    result = await ADVISOR.chat(message, context)
    async with STATE.lock:
        STATE.ai_last_error = ADVISOR.last_error
        if not result.get("ok"):
            STATE.logs.emit("ai", "error", f"chat failed: {result.get('error')}")
    return result


# ---------------------------------------------------------------------------
# Flash WebSocket — streams compile/upload console output live (§6.4)
# ---------------------------------------------------------------------------
flash_router = APIRouter()


@flash_router.websocket("/ws/flash")
async def ws_flash(ws: WebSocket) -> None:
    await ws.accept()
    send_lock = asyncio.Lock()

    async def send(payload: dict[str, Any]) -> None:
        async with send_lock:
            with contextlib.suppress(Exception):
                await ws.send_text(json.dumps(payload, separators=(",", ":")))

    loop = asyncio.get_running_loop()

    def emit(stream: str, line: str) -> None:
        # Called from the subprocess drain task; hop back onto the loop.
        asyncio.run_coroutine_threadsafe(
            send({"type": "output", "stream": stream, "line": line}), loop
        )

    try:
        await send({"type": "ready", "probe": await FLASHER.probe()})
        while True:
            raw = await ws.receive_json()
            if not isinstance(raw, dict):
                continue
            if raw.get("type") != "flash":
                await send({"type": "error", "msg": f"unknown command {raw.get('type')!r}"})
                continue

            target = str(raw.get("target", ""))
            port = raw.get("port")
            upload = bool(raw.get("upload", True))

            async with STATE.lock:
                if STATE.running:
                    await send(
                        {
                            "type": "error",
                            "msg": "refused: press Stop first. Flashing resets the "
                            "board and would leave motors unattended mid-drive.",
                        }
                    )
                    continue
                STATE.logs.emit("system", "info", f"flash requested: {target}")

            await send({"type": "started", "target": target, "upload": upload})
            try:
                result = await FLASHER.compile_and_upload(
                    target, str(port) if port else None, emit, upload=upload
                )
            except FlashBusy as exc:
                await send({"type": "error", "msg": str(exc)})
                continue

            await send(
                {
                    "type": "finished",
                    "ok": result.ok,
                    "detail": result.detail,
                    "artifact": result.artifact.name if result.artifact else None,
                    "artifact_url": (
                        f"/api/flash/artifact/{target}" if result.artifact else None
                    ),
                }
            )
            async with STATE.lock:
                STATE.logs.emit(
                    "system",
                    "info" if result.ok else "error",
                    f"flash {target}: {result.detail}",
                )
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.info("flash ws error: %s", exc)


# ---------------------------------------------------------------------------
# App assembly
# ---------------------------------------------------------------------------
BACKGROUND_TASKS: list[asyncio.Task[Any]] = []


async def _startup() -> None:
    loop = asyncio.get_running_loop()

    async with STATE.lock:
        STATE.logs.emit("system", "info", "backend starting")

    # ROS2 is optional; the backend is fully usable without it.
    if SETTINGS.enable_ros:
        available = await asyncio.to_thread(BRIDGE.start, loop)
    else:
        available = False
        logger.info("ROS2 disabled via SLAM_ENABLE_ROS=0")

    async with STATE.lock:
        STATE.ros_available = available
        if available:
            STATE.ros_nodes_seen.add("slam_bot_bridge")
            STATE.logs.emit("system", "info", "ROS2 bridge active")
        else:
            STATE.logs.emit(
                "system",
                "warn",
                "running without ROS2 — live scan, logs, tuning and flashing "
                "work; /map and Nav2 goals do not",
            )

    # §9: resolve a free model at startup rather than hardcoding one.
    model = await ADVISOR.resolve_model()
    async with STATE.lock:
        STATE.ai_model = model
        STATE.ai_model_candidates = list(SETTINGS.openrouter_candidates)
        STATE.ai_last_error = ADVISOR.last_error
        if model:
            STATE.logs.emit("ai", "info", f"AI model resolved: {model}")
        else:
            STATE.logs.emit(
                "ai", "warn", f"AI unavailable: {ADVISOR.last_error}"
            )

    BACKGROUND_TASKS.extend(
        [
            asyncio.create_task(ws_robot.control_loop(), name="control-loop"),
            asyncio.create_task(ws_robot.presence_loop(), name="presence-loop"),
            asyncio.create_task(ws_frontend.status_loop(), name="status-loop"),
            asyncio.create_task(ws_frontend.map_loop(), name="map-loop"),
        ]
    )
    logger.info("backend ready on %s:%s", SETTINGS.host, SETTINGS.port)


async def _shutdown() -> None:
    # Best effort: stop the robot before the process goes away. Even if this
    # fails, firmware's 2 s watchdog (§11.2) halts it independently.
    with contextlib.suppress(Exception):
        await ws_robot.push_control("stop")

    for task in BACKGROUND_TASKS:
        task.cancel()
    for task in BACKGROUND_TASKS:
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await task
    BACKGROUND_TASKS.clear()

    await asyncio.to_thread(BRIDGE.shutdown)
    logger.info("backend stopped")


@contextlib.asynccontextmanager
async def lifespan(app: FastAPI):
    await _startup()
    try:
        yield
    finally:
        await _shutdown()


app = FastAPI(
    title="SLAM Bot Backend",
    version="1.0.0",
    description=(
        "Bridge between the robot's two MCUs, ROS2/Nav2, and the web app. "
        "See SLAM_Bot_Full_Spec.md."
    ),
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=SETTINGS.cors_origins,
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api)
app.include_router(flash_router)
app.include_router(ws_robot.router)
app.include_router(ws_frontend.router)


@app.get("/healthz")
async def healthz() -> dict[str, Any]:
    async with STATE.lock:
        return {
            "ok": True,
            "t_ms": now_ms(),
            "running": STATE.running,
            "arduino": STATE.arduino.connected,
            "nodemcu": STATE.nodemcu.connected,
            "ros": STATE.ros_available,
        }


# Serve the built frontend if it exists, so a single process serves everything
# in production. During development the Vite dev server proxies to us instead.
if SETTINGS.serve_webapp and SETTINGS.webapp_dist.is_dir():
    assets = SETTINGS.webapp_dist / "assets"
    if assets.is_dir():
        app.mount("/assets", StaticFiles(directory=str(assets)), name="assets")

    @app.get("/")
    async def index():
        return FileResponse(SETTINGS.webapp_dist / "index.html")

    @app.get("/{path:path}")
    async def spa_fallback(path: str):
        # Client-side routing: unknown non-API paths return index.html.
        candidate = SETTINGS.webapp_dist / path
        if candidate.is_file():
            return FileResponse(candidate)
        return FileResponse(SETTINGS.webapp_dist / "index.html")

else:

    @app.get("/")
    async def index_placeholder() -> JSONResponse:
        return JSONResponse(
            {
                "service": "SLAM Bot backend",
                "webapp": "not built — run `npm run build` in webapp/, or use "
                "the Vite dev server at http://localhost:5173",
                "docs": "/docs",
                "health": "/healthz",
            }
        )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host=SETTINGS.host,
        port=SETTINGS.port,
        reload=False,
        log_level="info",
    )
