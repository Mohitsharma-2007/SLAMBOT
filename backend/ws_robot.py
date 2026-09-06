"""WebSocket endpoints for the two MCUs: /ws/motion and /ws/lidar.

Both MCUs are WebSocket *clients* of this backend (§4). This module owns:

  * their connection lifecycle and presence tracking,
  * the downlink: cmd_vel, control (start/stop), tuning_update, tuning_save,
    proximity — all the §11.3 schemas,
  * the uplink: odom from the Arduino, scan from the NodeMCU, log frames from
    either,
  * the control loop that decides what cmd_vel the Arduino actually receives,
    merging manual drive and Nav2 output under the run-state gate,
  * the heartbeat that keeps the firmware's 2 s watchdog fed while a session is
    legitimately idle.

Design note on the collision path: the LIDAR is on the NodeMCU but
collision-stop is enforced on the Arduino (§11.2 wants firmware-level safety).
The backend is the only thing that sees both, so it forwards each revolution's
nearest return to the Arduino as a ``proximity`` message. The Arduino also
enforces its own watchdog, so a backend crash still stops the robot.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import time
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

import tuning
from config import SETTINGS
from hub import HUB
from state import STATE, DeviceState, ScanState, now_ms

logger = logging.getLogger("slam_bot.ws_robot")

router = APIRouter()


class RobotLink:
    """A live WebSocket to one MCU, with a serialised send path."""

    def __init__(self, ws: WebSocket, device: DeviceState) -> None:
        self.ws = ws
        self.device = device
        self._send_lock = asyncio.Lock()
        self.alive = True

    async def send(self, message: dict[str, Any]) -> bool:
        """Send one JSON message. Returns False if the link is gone."""
        if not self.alive:
            return False
        payload = json.dumps(message, separators=(",", ":"))
        try:
            async with self._send_lock:
                await self.ws.send_text(payload)
            return True
        except Exception as exc:
            self.alive = False
            logger.info("send to %s failed: %s", self.device.name, exc)
            return False


class RobotLinks:
    """Holds the (at most one each) Arduino and NodeMCU links."""

    def __init__(self) -> None:
        self.arduino: RobotLink | None = None
        self.nodemcu: RobotLink | None = None

    async def send_arduino(self, message: dict[str, Any]) -> bool:
        link = self.arduino
        if link is None:
            return False
        return await link.send(message)

    async def send_nodemcu(self, message: dict[str, Any]) -> bool:
        link = self.nodemcu
        if link is None:
            return False
        return await link.send(message)


LINKS = RobotLinks()

# Timestamp of the last empty-scan diagnostic, so the 5.5 Hz scan stream cannot
# flood the log ring. Single-element list rather than a module global purely to
# keep the mutation local to the handler that owns it.
_LAST_SCAN_DIAG_MS = [0]


# ---------------------------------------------------------------------------
# Downlink helpers — used by ws_frontend, ros_bridge and the REST routes
# ---------------------------------------------------------------------------
async def push_control(action: str) -> bool:
    """Send §11.3 control start/stop to the Arduino."""
    return await LINKS.send_arduino({"type": "control", "action": action})


async def push_cmd_vel(linear_mm_s: float, angular_mdeg_s: float) -> bool:
    return await LINKS.send_arduino(
        {
            "type": "cmd_vel",
            "linear_mm_s": round(float(linear_mm_s), 2),
            "angular_mdeg_s": round(float(angular_mdeg_s), 1),
        }
    )


async def push_tuning(values: dict[str, Any]) -> dict[str, bool]:
    """Route accepted tuning values to whichever devices own them (§11.3).

    Each MCU receives only the fields it owns, under the firmware's own key
    names, and only the fields that changed — exactly the partial-patch
    semantics §11.3 specifies.
    """
    routed = tuning.split_by_target(values)
    result: dict[str, bool] = {"arduino": True, "nodemcu": True}

    if routed["arduino"]:
        payload: dict[str, Any] = {"type": "tuning_update"}
        for name, value in routed["arduino"].items():
            payload[tuning.to_firmware_key(name)] = value
        result["arduino"] = await LINKS.send_arduino(payload)

    if routed["nodemcu"]:
        payload = {"type": "tuning_update"}
        for name, value in routed["nodemcu"].items():
            payload[tuning.to_firmware_key(name)] = value
        result["nodemcu"] = await LINKS.send_nodemcu(payload)

    if routed["ros"]:
        # Applied by the ROS bridge if it is up; harmless no-op otherwise.
        from ros_bridge import BRIDGE

        BRIDGE.apply_tuning(routed["ros"])

    return result


async def push_tuning_save() -> dict[str, bool]:
    """§11.4 — persist to EEPROM / LittleFS on both MCUs."""
    return {
        "arduino": await LINKS.send_arduino({"type": "tuning_save"}),
        "nodemcu": await LINKS.send_nodemcu({"type": "tuning_save"}),
    }


async def push_tuning_reset() -> dict[str, bool]:
    return {
        "arduino": await LINKS.send_arduino({"type": "tuning_reset"}),
        "nodemcu": await LINKS.send_nodemcu({"type": "tuning_reset"}),
    }


# ---------------------------------------------------------------------------
# /ws/motion — Arduino Uno R4 WiFi
# ---------------------------------------------------------------------------
@router.websocket("/ws/motion")
async def ws_motion(ws: WebSocket) -> None:
    await ws.accept()
    link = RobotLink(ws, STATE.arduino)

    previous = LINKS.arduino
    LINKS.arduino = link
    if previous is not None:
        previous.alive = False
        with contextlib.suppress(Exception):
            await previous.ws.close(code=1012, reason="superseded by new connection")

    peer = ws.client.host if ws.client else None
    async with STATE.lock:
        STATE.arduino.connected = True
        STATE.arduino.ip = peer
        STATE.arduino.connected_at_ms = now_ms()
        STATE.arduino.last_message_ms = now_ms()
        STATE.arduino.messages_received = 0
        # Firmware boots stopped (§11.2); reflect that rather than assuming the
        # backend's previous run-state still holds on a fresh MCU boot.
        STATE.set_running(False)
        STATE.logs.emit("motion", "info", f"Arduino connected from {peer}")

    try:
        while True:
            raw = await ws.receive_text()
            await _handle_arduino_message(raw)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.info("arduino link error: %s", exc)
    finally:
        link.alive = False
        if LINKS.arduino is link:
            LINKS.arduino = None
            async with STATE.lock:
                STATE.arduino.connected = False
                # Losing the motion link means we can no longer command a stop, so
                # the backend's own view must go to Stopped too. The firmware's 2 s
                # watchdog independently does the same thing on its side (§11.2).
                STATE.set_running(False)
                STATE.logs.emit("motion", "warn", "Arduino disconnected")


async def _handle_arduino_message(raw: str) -> None:
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        async with STATE.lock:
            STATE.logs.emit("motion", "warn", f"non-JSON from Arduino: {raw[:120]!r}")
        return
    if not isinstance(msg, dict):
        return

    kind = msg.get("type")
    odom_payload: dict[str, Any] | None = None

    async with STATE.lock:
        STATE.arduino.connected = True
        STATE.arduino.last_message_ms = now_ms()
        STATE.arduino.messages_received += 1

        if kind == "odom":
            odom = STATE.odom
            odom.t_ms = int(msg.get("t_ms", now_ms()))
            odom.x_mm = float(msg.get("x_mm", 0.0))
            odom.y_mm = float(msg.get("y_mm", 0.0))
            odom.theta_rad = float(msg.get("theta_rad", 0.0))
            odom.linear_mm_s = float(msg.get("linear_mm_s", 0.0))
            odom.angular_mdeg_s = float(msg.get("angular_mdeg_s", 0.0))
            odom.ticks_l = int(msg.get("ticks_l", 0))
            odom.ticks_r = int(msg.get("ticks_r", 0))
            odom.duty_l = int(msg.get("duty_l", 0))
            odom.duty_r = int(msg.get("duty_r", 0))
            was_collision = odom.collision
            odom.collision = bool(msg.get("collision", False))
            # Absent battery_v means no divider fitted — stay None (§6.1).
            if "battery_v" in msg:
                odom.battery_v = float(msg["battery_v"])
            if "rssi" in msg:
                STATE.arduino.rssi = int(msg["rssi"])

            firmware_running = bool(msg.get("running", False))
            if STATE.running and not firmware_running:
                # Firmware stopped itself — watchdog, e-stop or collision.
                STATE.set_running(False)
                STATE.watchdog_trips += 1
                STATE.logs.emit(
                    "motion", "warn", "firmware reports stopped — run-state synced"
                )
            odom.running = firmware_running

            STATE.note_pose(odom.x_mm, odom.y_mm)

            if odom.collision and not was_collision:
                distance = STATE.scan.min_distance_mm
                event = STATE.note_collision(
                    distance if distance is not None else -1.0
                )
                STATE.logs.emit(
                    "motion",
                    "error",
                    f"collision stop triggered at {event.distance_mm:.0f} mm",
                )

            # Feed odometry into ROS so slam_toolbox has a motion prior.
            from ros_bridge import BRIDGE

            BRIDGE.publish_odom(
                x_m=odom.x_mm / 1000.0,
                y_m=odom.y_mm / 1000.0,
                theta_rad=odom.theta_rad,
                linear_m_s=odom.linear_mm_s / 1000.0,
                angular_rad_s=(odom.angular_mdeg_s / 1000.0) * 0.017453292519943295,
            )
            odom_payload = odom.snapshot()

        elif kind == "hello":
            STATE.arduino.firmware = str(msg.get("fw", "unknown"))
            STATE.arduino.ip = str(msg.get("ip") or STATE.arduino.ip or "")
            reported = msg.get("config") or {}
            if isinstance(reported, dict):
                _record_ack("arduino", reported)
            STATE.logs.emit(
                "motion",
                "info",
                f"Arduino firmware {STATE.arduino.firmware} announced itself",
            )

        elif kind == "log":
            STATE.logs.emit(
                str(msg.get("source", "motion")),
                str(msg.get("level", "info")),
                str(msg.get("msg", "")),
            )

        elif kind == "ack":
            what = str(msg.get("for", "?"))
            ok = bool(msg.get("ok", False))
            if what == "tuning_update":
                _record_ack("arduino", tuning.split_by_target(STATE.tuning)["arduino"])
            elif what == "tuning_save" and ok:
                for name in tuning.split_by_target(STATE.tuning)["arduino"]:
                    STATE.tuning_saved[name] = STATE.tuning[name]
                STATE.logs.emit("motion", "info", "Arduino saved config to EEPROM")

    # Outside the lock. The standalone ROS bridge is a /ws/app client, so this
    # is how odometry reaches slam_toolbox at the Arduino's full 20 Hz. Relying
    # on the 5 Hz status loop instead starved the scan matcher of a motion prior
    # and left odom->base_link TF too sparse for the scan timestamps to
    # interpolate against, which is a direct cause of a smeared map.
    if odom_payload is not None:
        HUB.broadcast("odom", odom_payload)

    # Only on `hello` — a re-sync means "this device just booted, push it the
    # operator's values". Doing it on every ack would echo a full tuning frame
    # back at the MCU for each one it acknowledges.
    if kind == "hello":
        await _resync_device_tuning("arduino")


# ---------------------------------------------------------------------------
# /ws/lidar — NodeMCU (ESP8266)
# ---------------------------------------------------------------------------
@router.websocket("/ws/lidar")
async def ws_lidar(ws: WebSocket) -> None:
    await ws.accept()
    link = RobotLink(ws, STATE.nodemcu)

    previous = LINKS.nodemcu
    LINKS.nodemcu = link
    if previous is not None:
        previous.alive = False
        with contextlib.suppress(Exception):
            await previous.ws.close(code=1012, reason="superseded by new connection")

    peer = ws.client.host if ws.client else None
    async with STATE.lock:
        STATE.nodemcu.connected = True
        STATE.nodemcu.ip = peer
        STATE.nodemcu.connected_at_ms = now_ms()
        STATE.nodemcu.last_message_ms = now_ms()
        STATE.nodemcu.messages_received = 0
        STATE.logs.emit("lidar", "info", f"NodeMCU connected from {peer}")

    try:
        while True:
            raw = await ws.receive_text()
            await _handle_nodemcu_message(raw)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.info("nodemcu link error: %s", exc)
    finally:
        link.alive = False
        if LINKS.nodemcu is link:
            LINKS.nodemcu = None
            async with STATE.lock:
                STATE.nodemcu.connected = False
                STATE.logs.emit("lidar", "warn", "NodeMCU disconnected")


async def _handle_nodemcu_message(raw: str) -> None:
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        async with STATE.lock:
            STATE.logs.emit("lidar", "warn", f"non-JSON from NodeMCU: {raw[:120]!r}")
        return
    if not isinstance(msg, dict):
        return

    kind = msg.get("type")
    forward_proximity: float | None = None
    scan_payload: dict[str, Any] | None = None

    async with STATE.lock:
        STATE.nodemcu.connected = True
        STATE.nodemcu.last_message_ms = now_ms()
        STATE.nodemcu.messages_received += 1

        if kind == "scan":
            scan: ScanState = STATE.scan
            angles = msg.get("angles_deg") or []
            dists = msg.get("dists_mm") or []
            quality = msg.get("quality") or []
            # Defensive: a truncated frame must not desynchronise the arrays.
            n = min(len(angles), len(dists))
            scan.seq = int(msg.get("seq", scan.seq + 1))
            scan.t_ms = int(msg.get("t_ms", now_ms()))
            scan.rev_ms = int(msg.get("rev_ms", 0))
            scan.angles_deg = [float(a) for a in angles[:n]]
            scan.dists_mm = [float(d) for d in dists[:n]]
            scan.quality = [int(q) for q in quality[:n]] if quality else []
            scan.dropped = int(msg.get("dropped", 0))

            reported_min = msg.get("min_distance_mm")
            if reported_min is not None:
                scan.min_distance_mm = float(reported_min)
            elif scan.dists_mm:
                scan.min_distance_mm = min(scan.dists_mm)
            else:
                scan.min_distance_mm = None

            forward_proximity = scan.min_distance_mm
            scan_payload = scan.snapshot()

            if scan.dropped:
                STATE.logs.emit(
                    "lidar",
                    "warn",
                    f"scan {scan.seq}: dropped {scan.dropped} samples (buffer full)",
                )

            # An empty revolution carries a breakdown of why. Surface it, but
            # rate-limited: at ~5.5 Hz an unconditional log would bury every
            # other message within seconds — the Arduino truncation bug filled
            # the ring buffer with ~2000 identical warnings in exactly that way.
            diag = msg.get("diag")
            if isinstance(diag, dict) and n == 0:
                last = _LAST_SCAN_DIAG_MS[0]
                if now_ms() - last > 5000:
                    _LAST_SCAN_DIAG_MS[0] = now_ms()
                    packets = int(diag.get("packets", 0))
                    if packets == 0:
                        detail = (
                            "no valid packets parsed "
                            f"(bad_frame={diag.get('bad_frame', 0)}) — LIDAR "
                            "not spinning, TX/RX not crossed, or baud mismatch"
                        )
                    else:
                        detail = (
                            f"{packets} packets parsed but every sample "
                            f"filtered: quality={diag.get('rej_quality', 0)} "
                            f"zero={diag.get('rej_zero', 0)} "
                            f"near={diag.get('rej_near', 0)} "
                            f"far={diag.get('rej_far', 0)} "
                            f"angle={diag.get('rej_angle', 0)} "
                            f"(best quality seen {diag.get('max_quality', 0)}, "
                            f"min_quality={diag.get('min_quality_cfg', '?')})"
                        )
                    STATE.logs.emit("lidar", "error", f"scan empty: {detail}")

        elif kind == "hello":
            STATE.nodemcu.firmware = str(msg.get("fw", "unknown"))
            STATE.nodemcu.ip = str(msg.get("ip") or STATE.nodemcu.ip or "")
            reported = msg.get("config") or {}
            if isinstance(reported, dict):
                _record_ack("nodemcu", reported)
            STATE.logs.emit(
                "lidar",
                "info",
                f"NodeMCU firmware {STATE.nodemcu.firmware} announced itself",
            )

        elif kind == "log":
            STATE.logs.emit(
                str(msg.get("source", "lidar")),
                str(msg.get("level", "info")),
                str(msg.get("msg", "")),
            )

        elif kind == "ack":
            what = str(msg.get("for", "?"))
            if what == "tuning_save" and bool(msg.get("ok", False)):
                for name in tuning.split_by_target(STATE.tuning)["nodemcu"]:
                    STATE.tuning_saved[name] = STATE.tuning[name]
                STATE.logs.emit("lidar", "info", "NodeMCU saved config to LittleFS")

    # Outside the lock: publishing to ROS and pushing to the Arduino can block.
    if scan_payload is not None:
        from ros_bridge import BRIDGE

        BRIDGE.publish_scan(scan_payload)
        HUB.broadcast("scan", scan_payload)

    if forward_proximity is not None:
        await LINKS.send_arduino(
            {"type": "proximity", "min_distance_mm": round(forward_proximity)}
        )

    if kind == "hello":
        await _resync_device_tuning("nodemcu")


# ---------------------------------------------------------------------------
# Tuning sync
# ---------------------------------------------------------------------------
def _record_ack(device: str, reported: dict[str, Any]) -> None:
    """Record what a device says it currently holds, in registry key terms."""
    reverse = {tuning.to_firmware_key(p.name): p.name for p in tuning.PARAMS}
    for key, value in reported.items():
        name = reverse.get(key, key)
        if name in tuning.BY_NAME:
            STATE.tuning_acked.setdefault(device, {})[name] = value


async def _resync_device_tuning(device: str) -> None:
    """Push the backend's authoritative tuning to a device that just appeared.

    A freshly booted MCU either loaded its own saved config or fell back to
    firmware defaults; either way the backend's values are the ones the operator
    last chose in the UI, so they win. Without this, a mid-session MCU reboot
    would silently revert tuning.
    """
    async with STATE.lock:
        owned = tuning.split_by_target(STATE.tuning)
        values = owned.get(device, {})
    if not values:
        return

    payload: dict[str, Any] = {"type": "tuning_update"}
    for name, value in values.items():
        payload[tuning.to_firmware_key(name)] = value

    if device == "arduino":
        await LINKS.send_arduino(payload)
    else:
        await LINKS.send_nodemcu(payload)

    async with STATE.lock:
        STATE.logs.emit(
            "system", "info", f"re-synced {len(values)} tuning values to {device}"
        )


# ---------------------------------------------------------------------------
# Control loop + heartbeat
# ---------------------------------------------------------------------------
async def control_loop() -> None:
    """Decide and send the Arduino's cmd_vel, and keep its watchdog fed.

    Runs at ``heartbeat_hz``. Sends *something* on every tick while the Arduino
    is connected, because §11.2's firmware watchdog counts any message as
    liveness — going quiet during an idle session would trip it spuriously.
    """
    period = 1.0 / max(0.5, SETTINGS.heartbeat_hz)
    while True:
        try:
            await asyncio.sleep(period)
            if LINKS.arduino is None:
                continue

            async with STATE.lock:
                running = STATE.running
                mode = STATE.control_mode
                manual_fresh = now_ms() < STATE.manual_until_ms
                manual_lin = STATE.manual_linear_mm_s if manual_fresh else 0.0
                manual_ang = STATE.manual_angular_mdeg_s if manual_fresh else 0.0
                if not manual_fresh and (
                    STATE.manual_linear_mm_s or STATE.manual_angular_mdeg_s
                ):
                    # Expired manual command — clear it so the next tick is a
                    # clean zero rather than a stale velocity.
                    STATE.manual_linear_mm_s = 0.0
                    STATE.manual_angular_mdeg_s = 0.0

            if not running:
                # Not a cmd_vel: while stopped, the heartbeat alone keeps the
                # link alive. Firmware ignores cmd_vel when stopped anyway, and
                # sending zeros would be indistinguishable from a live command.
                await LINKS.send_arduino({"type": "ping"})
                continue

            if mode == "nav2":
                from ros_bridge import BRIDGE
                if BRIDGE.available:
                    nav_lin, nav_ang = BRIDGE.latest_cmd_vel()
                    linear_mm_s = nav_lin * 1000.0
                    angular_mdeg_s = nav_ang * 57295.77951308232  # rad/s -> mdeg/s
                else:
                    async with STATE.lock:
                        nav_fresh = now_ms() < STATE.nav_until_ms
                        linear_mm_s = STATE.nav_linear_mm_s if nav_fresh else 0.0
                        angular_mdeg_s = STATE.nav_angular_mdeg_s if nav_fresh else 0.0
            else:
                linear_mm_s = manual_lin
                angular_mdeg_s = manual_ang

            await push_cmd_vel(linear_mm_s, angular_mdeg_s)

        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("control loop iteration failed")


async def presence_loop() -> None:
    """Mark a device disconnected if it goes quiet, and log stuck-wheel events."""
    while True:
        try:
            await asyncio.sleep(0.5)
            changed = False
            async with STATE.lock:
                stale = SETTINGS.device_stale_ms
                for dev_name, device in (("arduino", STATE.arduino), ("nodemcu", STATE.nodemcu)):
                    if not device.connected:
                        continue
                    if device.last_message_ms is None or (now_ms() - device.last_message_ms > stale):
                        device.connected = False
                        changed = True
                        STATE.logs.emit(
                            "system",
                            "warn",
                            f"{device.name} offline (no telemetry for >{stale} ms)",
                        )
                        if dev_name == "arduino":
                            STATE.set_running(False)
                            link = LINKS.arduino
                            LINKS.arduino = None
                            if link is not None:
                                link.alive = False
                        elif dev_name == "nodemcu":
                            link = LINKS.nodemcu
                            LINKS.nodemcu = None
                            if link is not None:
                                link.alive = False

                # Stuck detection: commanded to move, drive applied, no motion.
                odom = STATE.odom
                commanded = abs(STATE.manual_linear_mm_s) > 20 or (
                    STATE.control_mode == "nav2" and STATE.running
                )
                driving = abs(odom.duty_l) > 40 or abs(odom.duty_r) > 40
                moving = abs(odom.linear_mm_s) > 10 or abs(odom.angular_mdeg_s) > 2000
                if STATE.running and commanded and driving and not moving:
                    STATE.stuck_events += 1
                    if STATE.stuck_events % 5 == 1:
                        STATE.logs.emit(
                            "motion",
                            "warn",
                            "wheels driven but no motion — possible stall",
                        )
                status = STATE.status_snapshot()
            if changed:
                HUB.broadcast("status", status)
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("presence loop iteration failed")


# ---------------------------------------------------------------------------
# Phone & Phone Relay WebSocket support
# ---------------------------------------------------------------------------
ACTIVE_PHONE_WEBSOCKET: WebSocket | None = None
CONNECTED_PHONE_RELAYS: set[WebSocket] = set()

# Shared buffer to accumulate accelerometer, gyro, and orientation components
phone_imu_buffer = {
    "acc_x": 0.0, "acc_y": 0.0, "acc_z": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0
}

@router.websocket("/ws/phone")
async def ws_phone(ws: WebSocket) -> None:
    global ACTIVE_PHONE_WEBSOCKET
    await ws.accept()
    ACTIVE_PHONE_WEBSOCKET = ws
    logger.info("Android Phone connected to backend WebSocket")
    
    # Send initial map state if available
    try:
        from map_manager import MAP_MANAGER
        map_snapshot = MAP_MANAGER.get_snapshot()
        if map_snapshot:
            await ws.send_text(json.dumps({"type": "map", **map_snapshot}))
    except Exception:
        pass
    
    try:
        while True:
            raw = await ws.receive_text()
            try:
                frame = json.loads(raw)
            except Exception:
                continue
            
            dataType = frame.get("type")
            relay_payload = None

            # 1. Action Commands from Phone UI
            if dataType == "command":
                action = frame.get("action")
                if action == "estop":
                    async with STATE.lock:
                        STATE.set_running(False)
                        STATE.logs.emit("phone", "error", "E-STOP triggered from Android App")
                elif action == "auto_explore":
                    async with STATE.lock:
                        STATE.logs.emit("phone", "info", "Autonomous Frontier Exploration commanded from Android")
                continue

            # 2. Sub-sensor updates (Flat or Nested)
            if dataType == "accelerometer":
                phone_imu_buffer["acc_x"] = frame.get("ax", 0.0)
                phone_imu_buffer["acc_y"] = frame.get("ay", 0.0)
                phone_imu_buffer["acc_z"] = frame.get("az", 0.0)
                relay_payload = {"type": "phone_imu", "data": phone_imu_buffer}
            elif dataType == "gyroscope":
                phone_imu_buffer["gyro_x"] = frame.get("gx", 0.0)
                phone_imu_buffer["gyro_y"] = frame.get("gy", 0.0)
                phone_imu_buffer["gyro_z"] = frame.get("gz", 0.0)
                relay_payload = {"type": "phone_imu", "data": phone_imu_buffer}
            elif dataType == "orientation":
                phone_imu_buffer["yaw"] = frame.get("yaw", 0.0)
                phone_imu_buffer["pitch"] = frame.get("pitch", 0.0)
                phone_imu_buffer["roll"] = frame.get("roll", 0.0)
                relay_payload = {"type": "phone_imu", "data": phone_imu_buffer}
            elif dataType == "gps":
                relay_payload = {"type": "phone_gps", "data": frame}
            elif dataType == "camera":
                relay_payload = {"type": "phone_camera", "data": frame}
            
            if relay_payload is not None:
                # Forward to all connected ROS2 phone relay nodes
                msg_str = json.dumps(relay_payload)
                for relay in list(CONNECTED_PHONE_RELAYS):
                    try:
                        await relay.send_text(msg_str)
                    except Exception:
                        CONNECTED_PHONE_RELAYS.discard(relay)
                        
    except WebSocketDisconnect:
        pass
    finally:
        if ACTIVE_PHONE_WEBSOCKET == ws:
            ACTIVE_PHONE_WEBSOCKET = None
        logger.info("Android Phone disconnected from backend WebSocket")

@router.websocket("/ws/phone_relay")
async def ws_phone_relay(ws: WebSocket) -> None:
    await ws.accept()
    CONNECTED_PHONE_RELAYS.add(ws)
    logger.info("ROS2 Phone Relay client connected to backend")
    
    try:
        while True:
            raw = await ws.receive_text()
            try:
                frame = json.loads(raw)
            except Exception:
                continue
            
            # If the ROS2 stack issues eyes animation state changes
            if frame.get("type") == "robot_state":
                state = frame.get("state", "idle")
                if ACTIVE_PHONE_WEBSOCKET is not None:
                    try:
                        await ACTIVE_PHONE_WEBSOCKET.send_text(json.dumps({"robot_state": state}))
                    except Exception:
                        pass
    except WebSocketDisconnect:
        pass
    finally:
        CONNECTED_PHONE_RELAYS.discard(ws)
        logger.info("ROS2 Phone Relay client disconnected")

