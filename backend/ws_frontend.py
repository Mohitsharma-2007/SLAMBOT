"""Browser-facing WebSocket endpoint: /ws/app.

One socket carries every topic the web app needs (status, scan, map, log,
tuning, plan, flash, ai) and every command it sends (control, drive, tuning
update/save/reset, nav goal, AI requests). Topic-tagged frames:

    server -> browser   {"topic": "...", "data": {...}}
    browser -> server   {"type": "...", ...}

Commands are validated here before they can reach a motor. §9.3's rule — never
let a model's output reach the motors without a bounds-checked intermediate step
— is enforced by routing everything through the same clamped command path used
by the manual joystick.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

import tuning
import ws_robot
from config import SETTINGS
from hub import HUB, Client
from state import STATE, now_ms

logger = logging.getLogger("slam_bot.ws_frontend")

router = APIRouter()


@router.websocket("/ws/app")
async def ws_app(ws: WebSocket) -> None:
    await ws.accept()
    client = HUB.add(ws)
    writer = asyncio.create_task(HUB.writer(client))
    log_queue = STATE.logs.subscribe()
    log_pump = asyncio.create_task(_pump_logs(client, log_queue))

    # Immediately hand the new tab a full picture so it renders without waiting
    # for the next periodic tick.
    async with STATE.lock:
        HUB.send_to(client, "hello", {"server_time_ms": now_ms()})
        HUB.send_to(client, "schema", {"params": tuning.schema(),
                                       "sections": tuning.sections()})
        HUB.send_to(client, "status", STATE.status_snapshot())
        HUB.send_to(client, "tuning", STATE.tuning_snapshot())
        HUB.send_to(client, "log_history", STATE.logs.history(limit=400))
        if STATE.scan.angles_deg:
            HUB.send_to(client, "scan", STATE.scan.snapshot())
        if not STATE.map.is_empty:
            HUB.send_to(client, "map", STATE.map.snapshot())
        HUB.send_to(
            client,
            "trail",
            {"points": [[x, y] for x, y in STATE.pose_trail]},
        )

    try:
        while True:
            raw = await ws.receive_json()
            if isinstance(raw, dict):
                await _handle_command(client, raw)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.info("frontend client %s error: %s", client.id, exc)
    finally:
        HUB.remove(client)
        STATE.logs.unsubscribe(log_queue)
        for task in (writer, log_pump):
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task


async def _pump_logs(client: Client, queue: asyncio.Queue[dict[str, Any]]) -> None:
    try:
        while client.alive:
            entry = await queue.get()
            HUB.send_to(client, "log", entry)
    except asyncio.CancelledError:
        raise


# ---------------------------------------------------------------------------
# Command handling
# ---------------------------------------------------------------------------
async def _reply(client: Client, ok: bool, message: str, **extra: Any) -> None:
    HUB.send_to(client, "reply", {"ok": ok, "msg": message, **extra})


async def _handle_command(client: Client, msg: dict[str, Any]) -> None:
    kind = str(msg.get("type", ""))

    if kind == "control":
        await _handle_control(client, msg)

    elif kind == "drive":
        await _handle_drive(client, msg)

    elif kind == "tuning_update":
        await _handle_tuning_update(client, msg)

    elif kind == "tuning_save":
        await _handle_tuning_save(client)

    elif kind == "tuning_reset":
        await _handle_tuning_reset(client)

    elif kind == "set_mode":
        mode = str(msg.get("mode", "manual"))
        if mode not in ("manual", "nav2"):
            await _reply(client, False, f"unknown control mode: {mode}")
            return
        async with STATE.lock:
            STATE.control_mode = mode  # type: ignore[assignment]
            STATE.manual_linear_mm_s = 0.0
            STATE.manual_angular_mdeg_s = 0.0
            STATE.logs.emit("system", "info", f"control mode -> {mode}")
        await _broadcast_status()
        await _reply(client, True, f"control mode set to {mode}")

    elif kind == "nav_cmd_vel":
        # From a standalone bridge node forwarding Nav2's /cmd_vel. Goes through
        # the same run-state gate and the same clamps as manual drive — §9.3's
        # bounds-checked-intermediate rule applies to autonomy too.
        await _handle_nav_cmd_vel(msg)

    elif kind == "nav_goal":
        await _handle_nav_goal(client, msg)

    elif kind == "clear_trail":
        async with STATE.lock:
            STATE.pose_trail.clear()
            STATE.last_trail_point = None
        HUB.broadcast("trail", {"points": []})
        await _reply(client, True, "trail cleared")

    elif kind == "clear_map":
        from state import MapState
        async with STATE.lock:
            STATE.map = MapState()
            STATE.pose_trail.clear()
            STATE.last_trail_point = None
            STATE.plan = []
        HUB.broadcast("map", STATE.map.snapshot())
        HUB.broadcast("trail", {"points": []})
        HUB.broadcast("plan", {"points": []})
        
        # Clear local/embedded ROS bridge SLAM Toolbox map
        from ros_bridge import BRIDGE
        ros_cleared = False
        if BRIDGE.available:
            ros_cleared = BRIDGE.clear_map()
            
        # Broadcast clear command to any connected web relays
        await broadcast_to_relays({"type": "clear_map"})
        
        msg_detail = "Map and pose trail cleared locally"
        if ros_cleared:
            msg_detail += " and SLAM Toolbox map reset"
        await _reply(client, True, msg_detail)

    elif kind == "reset_session":
        async with STATE.lock:
            STATE.collision_events.clear()
            STATE.stuck_events = 0
            STATE.watchdog_trips = 0
            STATE.distance_travelled_mm = 0.0
            STATE.logs.emit("system", "info", "session counters reset")
        await _broadcast_status()
        await _reply(client, True, "session counters reset")

    elif kind == "ping":
        HUB.send_to(client, "pong", {"t_ms": now_ms()})

    else:
        await _reply(client, False, f"unknown command: {kind!r}")


async def _handle_control(client: Client, msg: dict[str, Any]) -> None:
    action = str(msg.get("action", ""))
    if action not in ("start", "stop", "estop"):
        await _reply(client, False, f"unknown control action: {action!r}")
        return

    if action == "start":
        async with STATE.lock:
            if not STATE.arduino.connected:
                STATE.logs.emit("system", "warn", "Start refused: Arduino not connected")
                await _reply(client, False, "Arduino is not connected")
                return
        sent = await ws_robot.push_control("start")
        async with STATE.lock:
            if sent:
                STATE.set_running(True)
                STATE.logs.emit("system", "info", "RUN state: started")
            else:
                STATE.logs.emit("system", "error", "Start failed: link write error")
        await _broadcast_status()
        await _reply(client, sent, "started" if sent else "could not reach Arduino")
        return

    # stop / estop — always update backend state even if the write fails, since
    # firmware's 2 s watchdog will independently stop the robot (§11.2).
    sent = await ws_robot.push_control("stop" if action == "stop" else "estop")
    async with STATE.lock:
        STATE.set_running(False, estop=(action == "estop"))
        STATE.manual_linear_mm_s = 0.0
        STATE.manual_angular_mdeg_s = 0.0
        label = "EMERGENCY STOP" if action == "estop" else "RUN state: stopped"
        STATE.logs.emit("system", "error" if action == "estop" else "info", label)
        if not sent:
            STATE.logs.emit(
                "system",
                "warn",
                "stop could not be delivered — firmware watchdog will halt in <2 s",
            )
    if STATE.control_mode == "nav2":
        from ros_bridge import BRIDGE

        BRIDGE.cancel_navigation()
        # Also reach the standalone bridge in WSL, which is the process that
        # actually holds the Nav2 goal handle in the normal deployment.
        HUB.broadcast("nav_cancel", {"reason": action})
    await _broadcast_status()
    await _reply(client, True, "stopped")


async def _handle_drive(client: Client, msg: dict[str, Any]) -> None:
    """Manual joystick / arrow-key drive from the Dashboard.

    Values are clamped to the live tuning limits here as well as in firmware —
    defence in depth, and it means the UI's readback shows what will actually
    happen rather than what was requested.
    """
    try:
        linear = float(msg.get("linear_mm_s", 0.0))
        angular = float(msg.get("angular_mdeg_s", 0.0))
    except (TypeError, ValueError):
        await _reply(client, False, "drive: linear_mm_s/angular_mdeg_s must be numbers")
        return

    async with STATE.lock:
        if not STATE.running:
            await _reply(client, False, "ignored: bot is stopped — press Start first")
            return

        if STATE.control_mode != "manual":
            STATE.control_mode = "manual"
            STATE.logs.emit("system", "info", "control mode switched to manual for drivepad")

        lin_cap = float(STATE.tuning["max_linear_speed_mm_s"])
        ang_cap = float(STATE.tuning["max_angular_speed_mdeg_s"])
        STATE.manual_linear_mm_s = max(-lin_cap, min(lin_cap, linear))
        STATE.manual_angular_mdeg_s = max(-ang_cap, min(ang_cap, angular))
        STATE.manual_until_ms = now_ms() + SETTINGS.manual_command_ttl_ms
        applied = (STATE.manual_linear_mm_s, STATE.manual_angular_mdeg_s)

    # Send immediately rather than waiting for the next control-loop tick, so
    # keyboard driving feels responsive.
    await ws_robot.push_cmd_vel(*applied)
    HUB.send_to(
        client,
        "drive_echo",
        {"linear_mm_s": applied[0], "angular_mdeg_s": applied[1]},
    )


async def _handle_tuning_update(client: Client, msg: dict[str, Any]) -> None:
    updates = msg.get("values")
    if not isinstance(updates, dict) or not updates:
        await _reply(client, False, "tuning_update: 'values' object required")
        return

    accepted, errors = tuning.coerce_updates(updates)
    if not accepted:
        await _reply(client, False, "; ".join(errors) or "nothing to apply")
        return

    async with STATE.lock:
        changed = {k: v for k, v in accepted.items() if STATE.tuning.get(k) != v}
        STATE.tuning.update(accepted)

    if changed:
        result = await ws_robot.push_tuning(changed)
        async with STATE.lock:
            names = ", ".join(f"{k}={v}" for k, v in sorted(changed.items()))
            STATE.logs.emit("system", "info", f"tuning applied: {names}")
            for device, ok in result.items():
                if not ok and tuning.split_by_target(changed).get(device):
                    STATE.logs.emit(
                        "system",
                        "warn",
                        f"tuning could not reach {device} (not connected)",
                    )
    await _broadcast_tuning()
    await _reply(
        client,
        True,
        f"applied {len(changed)} value(s)" + (f"; {'; '.join(errors)}" if errors else ""),
    )


async def _handle_tuning_save(client: Client) -> None:
    result = await ws_robot.push_tuning_save()
    async with STATE.lock:
        # ROS-only parameters have no MCU flash to persist to; treat the
        # backend's own value as their saved state.
        routed = tuning.split_by_target(STATE.tuning)
        for name in STATE.tuning:
            param = tuning.BY_NAME[name]
            if param.targets == ("ros",):
                STATE.tuning_saved[name] = STATE.tuning[name]
        for device, ok in result.items():
            if ok:
                continue
            if routed.get(device):
                STATE.logs.emit(
                    "system", "warn", f"save could not reach {device} (not connected)"
                )
        STATE.logs.emit("system", "info", "tuning_save sent to connected devices")
    await _broadcast_tuning()
    await _reply(client, any(result.values()), "save requested")


async def _handle_tuning_reset(client: Client) -> None:
    await ws_robot.push_tuning_reset()
    async with STATE.lock:
        STATE.tuning = dict(tuning.DEFAULTS)
        STATE.tuning_saved = dict(tuning.DEFAULTS)
        STATE.tuning_acked = {"arduino": {}, "nodemcu": {}}
        STATE.logs.emit("system", "info", "tuning reset to defaults on all targets")
        values = dict(STATE.tuning)
    await ws_robot.push_tuning(values)
    await _broadcast_tuning()
    await _reply(client, True, "reset to defaults")


async def _handle_nav_cmd_vel(msg: dict[str, Any]) -> None:
    """Velocity from a standalone bridge relaying Nav2's /cmd_vel."""
    try:
        linear = float(msg.get("linear_mm_s", 0.0))
        angular = float(msg.get("angular_mdeg_s", 0.0))
    except (TypeError, ValueError):
        return

    async with STATE.lock:
        if not STATE.running or STATE.control_mode != "nav2":
            return
        lin_cap = float(STATE.tuning["max_linear_speed_mm_s"])
        ang_cap = float(STATE.tuning["max_angular_speed_mdeg_s"])
        applied = (
            max(-lin_cap, min(lin_cap, linear)),
            max(-ang_cap, min(ang_cap, angular)),
        )
        STATE.nav_linear_mm_s = applied[0]
        STATE.nav_angular_mdeg_s = applied[1]
        STATE.nav_until_ms = now_ms() + 500  # expire after 500ms
        
    await ws_robot.push_cmd_vel(*applied)


async def _handle_nav_goal(client: Client, msg: dict[str, Any]) -> None:
    try:
        x_m = float(msg.get("x_m"))
        y_m = float(msg.get("y_m"))
        theta_rad = float(msg.get("theta_rad", 0.0))
    except (TypeError, ValueError):
        await _reply(client, False, "nav_goal: numeric x_m, y_m required")
        return

    # Bounds check before anything reaches the planner. A goal 10 km away is a
    # UI bug or a bad click, not a navigation request.
    if not (-100.0 <= x_m <= 100.0 and -100.0 <= y_m <= 100.0):
        await _reply(client, False, "nav_goal rejected: outside ±100 m sanity bounds")
        return

    async with STATE.lock:
        if not STATE.running:
            await _reply(client, False, "nav_goal ignored: bot is stopped")
            return
        if STATE.control_mode != "nav2":
            await _reply(client, False, "nav_goal ignored: switch to nav2 mode first")
            return

    from ros_bridge import BRIDGE

    if BRIDGE.available:
        ok, detail = BRIDGE.send_goal(x_m, y_m, theta_rad)
    else:
        # The usual deployment: ROS2 lives in WSL and the backend runs on
        # Windows without rclpy, so the standalone bridge owns the ROS graph.
        # It is itself a /ws/app client, so broadcasting reaches it. Without
        # this branch a clicked goal was silently swallowed by the no-op
        # embedded bridge and the robot never moved.
        HUB.broadcast(
            "nav_goal", {"x_m": x_m, "y_m": y_m, "theta_rad": theta_rad}
        )
        ok, detail = True, "goal forwarded to the ROS bridge"

    async with STATE.lock:
        STATE.logs.emit(
            "nav",
            "info" if ok else "error",
            f"goal ({x_m:.2f}, {y_m:.2f}) -> {detail}",
        )
    await _reply(client, ok, detail)


CONNECTED_RELAYS: set[WebSocket] = set()

async def broadcast_to_relays(msg: dict[str, Any]) -> None:
    for ws in list(CONNECTED_RELAYS):
        try:
            await ws.send_json(msg)
        except Exception:
            pass

# ---------------------------------------------------------------------------
# /ws/relay — ingest from a standalone slam_bot_web_relay node
# ---------------------------------------------------------------------------
@router.websocket("/ws/relay")
async def ws_relay(ws: WebSocket) -> None:
    """Accept ROS topic data from a separately-running web relay node.

    Only used when ROS2 runs in its own process (``standalone_bridge:=true``).
    In the default embedded setup the backend's own rclpy node subscribes to
    /map and /plan directly and this endpoint stays idle.
    """
    await ws.accept()
    CONNECTED_RELAYS.add(ws)
    async with STATE.lock:
        STATE.ros_available = True
        STATE.ros_nodes_seen.add("slam_toolbox")
        STATE.logs.emit("system", "info", "ROS web relay connected")
        status = STATE.status_snapshot()
    HUB.broadcast("status", status)

    try:
        while True:
            frame = await ws.receive_json()
            if not isinstance(frame, dict):
                continue
            topic = frame.get("topic")
            data = frame.get("data")
            if not isinstance(data, dict):
                continue

            if topic == "ros_map":
                async with STATE.lock:
                    STATE.map.width = int(data.get("width", 0))
                    STATE.map.height = int(data.get("height", 0))
                    STATE.map.resolution = float(data.get("resolution", 0.0))
                    STATE.map.origin_x = float(data.get("origin_x", 0.0))
                    STATE.map.origin_y = float(data.get("origin_y", 0.0))
                    STATE.map.rle = [int(v) for v in data.get("rle", [])]
                    STATE.map.stamp_ms = now_ms()
                    STATE.ros_available = True
                    STATE.ros_nodes_seen.add("slam_toolbox")
                    map_snapshot = STATE.map.snapshot()
                HUB.broadcast("map", map_snapshot)

            elif topic == "ros_plan":
                points = [
                    (float(p[0]), float(p[1]))
                    for p in data.get("points", [])
                    if isinstance(p, (list, tuple)) and len(p) >= 2
                ]
                async with STATE.lock:
                    STATE.plan = points
                    STATE.ros_nodes_seen.add("planner_server")
                HUB.broadcast("plan", {"points": [[x, y] for x, y in points]})

            elif topic == "ros_scan":
                # The MCU's own scan is authoritative for the polar plot; a
                # relayed /scan is only used if the NodeMCU is not connected
                # directly (e.g. someone is replaying a bag).
                async with STATE.lock:
                    if STATE.nodemcu.connected:
                        continue
                    STATE.scan.angles_deg = [
                        float(a) for a in data.get("angles_deg", [])
                    ]
                    STATE.scan.dists_mm = [float(d) for d in data.get("dists_mm", [])]
                    STATE.scan.quality = []
                    STATE.scan.seq += 1
                    STATE.scan.t_ms = now_ms()
                    STATE.scan.min_distance_mm = (
                        min(STATE.scan.dists_mm) if STATE.scan.dists_mm else None
                    )
                    payload = STATE.scan.snapshot()
                HUB.broadcast("scan", payload)

    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.info("relay link error: %s", exc)
    finally:
        CONNECTED_RELAYS.discard(ws)
        async with STATE.lock:
            if not CONNECTED_RELAYS:
                STATE.ros_available = False
            STATE.logs.emit("system", "warn", "ROS web relay disconnected")
            status = STATE.status_snapshot()
        HUB.broadcast("status", status)


# ---------------------------------------------------------------------------
# Periodic broadcasts
# ---------------------------------------------------------------------------
async def _broadcast_status() -> None:
    async with STATE.lock:
        payload = STATE.status_snapshot()
    HUB.broadcast("status", payload)


async def _broadcast_tuning() -> None:
    async with STATE.lock:
        payload = STATE.tuning_snapshot()
    HUB.broadcast("tuning", payload)


async def status_loop() -> None:
    """Push status + pose trail to every tab at ``status_hz``."""
    period = 1.0 / max(0.5, SETTINGS.status_hz)
    tick = 0
    while True:
        try:
            await asyncio.sleep(period)
            if HUB.client_count == 0:
                continue
            tick += 1
            async with STATE.lock:
                status = STATE.status_snapshot()
                trail = [[x, y] for x, y in STATE.pose_trail]
            HUB.broadcast("status", status)
            # The trail grows slowly; a lower rate keeps frames small.
            if tick % 5 == 0:
                HUB.broadcast("trail", {"points": trail})
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("status loop iteration failed")


async def map_loop() -> None:
    """Push the occupancy grid at ``map_hz`` when it has changed."""
    period = 1.0 / max(0.2, SETTINGS.map_hz)
    last_stamp = -1
    while True:
        try:
            await asyncio.sleep(period)
            if HUB.client_count == 0:
                continue
            async with STATE.lock:
                if STATE.map.is_empty or STATE.map.stamp_ms == last_stamp:
                    continue
                last_stamp = STATE.map.stamp_ms
                payload = STATE.map.snapshot()
                plan = list(STATE.plan)
            HUB.broadcast("map", payload)
            if plan:
                HUB.broadcast("plan", {"points": [[x, y] for x, y in plan]})
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("map loop iteration failed")
