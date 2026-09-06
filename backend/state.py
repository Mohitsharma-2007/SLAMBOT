"""Central backend state — run-state, device presence, telemetry, log bus.

Why one module: §6.1 requires the Start/Stop run-state to survive a browser
refresh, which means it cannot live in a React component; it lives here and the
browser only ever renders it. Everything that both the robot-facing and
browser-facing WebSocket layers need to see is in this object, guarded by a
single asyncio lock.

Nothing in here imports FastAPI or rclpy — it stays importable from tests and
from the ROS bridge thread.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Iterable, Literal

from tuning import DEFAULTS, PARAMS

LogSource = Literal["lidar", "motion", "slam", "nav", "ai", "system"]
LogLevel = Literal["info", "warn", "error"]

LOG_SOURCES: tuple[str, ...] = ("lidar", "motion", "slam", "nav", "ai", "system")
LOG_LEVELS: tuple[str, ...] = ("info", "warn", "error")

MAX_LOG_HISTORY = 2000
MAX_TRAIL_POINTS = 3000
MAX_COLLISION_EVENTS = 200


def now_ms() -> int:
    return int(time.time() * 1000)


@dataclass
class DeviceState:
    """Connection + liveness for one MCU."""

    name: str
    connected: bool = False
    ip: str | None = None
    firmware: str | None = None
    connected_at_ms: int | None = None
    last_message_ms: int | None = None
    messages_received: int = 0
    rssi: int | None = None

    def snapshot(self) -> dict[str, Any]:
        stale_ms: int | None = None
        is_conn = self.connected
        if self.last_message_ms is not None:
            stale_ms = now_ms() - self.last_message_ms
            if stale_ms > 3000:
                is_conn = False
        else:
            is_conn = False
        return {
            "name": self.name,
            "connected": is_conn,
            "ip": self.ip,
            "firmware": self.firmware,
            "connected_at_ms": self.connected_at_ms,
            "last_message_ms": self.last_message_ms,
            "stale_ms": stale_ms,
            "messages_received": self.messages_received,
            "rssi": self.rssi,
        }


@dataclass
class Odometry:
    """Latest odometry as reported by the Arduino."""

    t_ms: int = 0
    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_rad: float = 0.0
    linear_mm_s: float = 0.0
    angular_mdeg_s: float = 0.0
    ticks_l: int = 0
    ticks_r: int = 0
    duty_l: int = 0
    duty_r: int = 0
    running: bool = False
    collision: bool = False
    # Only populated when a real resistor divider is fitted (§6.1: never faked).
    battery_v: float | None = None

    def snapshot(self) -> dict[str, Any]:
        out = {
            "t_ms": self.t_ms,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "theta_rad": self.theta_rad,
            "theta_deg": math.degrees(self.theta_rad),
            "linear_mm_s": self.linear_mm_s,
            "angular_mdeg_s": self.angular_mdeg_s,
            "ticks_l": self.ticks_l,
            "ticks_r": self.ticks_r,
            "duty_l": self.duty_l,
            "duty_r": self.duty_r,
            "running": self.running,
            "collision": self.collision,
        }
        if self.battery_v is not None:
            out["battery_v"] = self.battery_v
        return out


@dataclass
class ScanState:
    """Most recent LIDAR revolution, kept as parallel arrays for cheap JSON."""

    seq: int = 0
    t_ms: int = 0
    rev_ms: int = 0
    angles_deg: list[float] = field(default_factory=list)
    dists_mm: list[float] = field(default_factory=list)
    quality: list[int] = field(default_factory=list)
    min_distance_mm: float | None = None
    dropped: int = 0

    def snapshot(self) -> dict[str, Any]:
        return {
            "seq": self.seq,
            "t_ms": self.t_ms,
            "rev_ms": self.rev_ms,
            "n": len(self.angles_deg),
            "angles_deg": self.angles_deg,
            "dists_mm": self.dists_mm,
            "quality": self.quality,
            "min_distance_mm": self.min_distance_mm,
            "dropped": self.dropped,
        }


@dataclass
class MapState:
    """Latest occupancy grid received from slam_toolbox via the ROS bridge."""

    stamp_ms: int = 0
    width: int = 0
    height: int = 0
    resolution: float = 0.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    # Run-length encoded occupancy: flat [value, count, value, count, ...].
    # A 4000x4000 grid is 16M cells; RLE keeps the WebSocket frame sane because
    # unexplored space (-1) dominates and compresses to almost nothing.
    rle: list[int] = field(default_factory=list)

    def snapshot(self) -> dict[str, Any]:
        return {
            "stamp_ms": self.stamp_ms,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "rle": self.rle,
        }

    @property
    def is_empty(self) -> bool:
        return self.width == 0 or self.height == 0


def rle_encode(cells: Iterable[int]) -> list[int]:
    """Run-length encode an occupancy grid into a flat [value, count, ...]."""
    out: list[int] = []
    prev: int | None = None
    count = 0
    for cell in cells:
        value = int(cell)
        if value == prev:
            count += 1
        else:
            if prev is not None:
                out.extend((prev, count))
            prev = value
            count = 1
    if prev is not None:
        out.extend((prev, count))
    return out


@dataclass
class CollisionEvent:
    t_ms: int
    distance_mm: float
    pose_x_mm: float
    pose_y_mm: float
    theta_rad: float

    def snapshot(self) -> dict[str, Any]:
        return {
            "t_ms": self.t_ms,
            "distance_mm": self.distance_mm,
            "pose_x_mm": self.pose_x_mm,
            "pose_y_mm": self.pose_y_mm,
            "theta_rad": self.theta_rad,
        }


class LogBus:
    """Fan-out log bus with bounded history for late-joining browser clients."""

    def __init__(self, history: int = MAX_LOG_HISTORY) -> None:
        self._history: deque[dict[str, Any]] = deque(maxlen=history)
        self._subscribers: set[asyncio.Queue[dict[str, Any]]] = set()
        self._seq = 0

    def emit(
        self,
        source: str,
        level: str,
        message: str,
        **extra: Any,
    ) -> dict[str, Any]:
        if source not in LOG_SOURCES:
            source = "system"
        if level not in LOG_LEVELS:
            level = "info"
        self._seq += 1
        entry: dict[str, Any] = {
            "seq": self._seq,
            "t_ms": now_ms(),
            "source": source,
            "level": level,
            "msg": message,
        }
        if extra:
            entry["extra"] = extra
        self._history.append(entry)

        for queue in list(self._subscribers):
            try:
                queue.put_nowait(entry)
            except asyncio.QueueFull:
                # A browser tab that stopped draining must not block the robot
                # loop; drop its oldest entry and keep the newest.
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()
                with contextlib.suppress(asyncio.QueueFull):
                    queue.put_nowait(entry)
        return entry

    def subscribe(self, maxsize: int = 500) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=maxsize)
        self._subscribers.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        self._subscribers.discard(queue)

    def history(
        self,
        limit: int = 300,
        sources: Iterable[str] | None = None,
        levels: Iterable[str] | None = None,
    ) -> list[dict[str, Any]]:
        src = set(sources) if sources else None
        lvl = set(levels) if levels else None
        items = [
            e
            for e in self._history
            if (src is None or e["source"] in src)
            and (lvl is None or e["level"] in lvl)
        ]
        return items[-limit:]

    def counts(self) -> dict[str, int]:
        out = {level: 0 for level in LOG_LEVELS}
        for entry in self._history:
            out[entry["level"]] = out.get(entry["level"], 0) + 1
        return out


class AppState:
    """Everything the backend knows, in one lock-guarded object."""

    def __init__(self) -> None:
        self.lock = asyncio.Lock()
        self.logs = LogBus()
        self.started_at_ms = now_ms()

        # §6.1 / §11.2 — run-state lives here, not in the browser.
        self.running = False
        self.estop_latched = False
        self.run_started_at_ms: int | None = None

        self.arduino = DeviceState("arduino_uno_r4")
        self.nodemcu = DeviceState("nodemcu_lidar")

        self.odom = Odometry()
        self.scan = ScanState()
        self.map = MapState()

        self.pose_trail: deque[tuple[float, float]] = deque(maxlen=MAX_TRAIL_POINTS)
        self.plan: list[tuple[float, float]] = []

        # Tuning: the backend's authoritative desired values, plus what each
        # device has actually acknowledged, so the UI can show "unsaved" and
        # "not yet applied" honestly (§11.4).
        self.tuning: dict[str, Any] = dict(DEFAULTS)
        self.tuning_saved: dict[str, Any] = dict(DEFAULTS)
        self.tuning_acked: dict[str, dict[str, Any]] = {"arduino": {}, "nodemcu": {}}

        # Manual drive command from the Dashboard, in the firmware's units.
        self.manual_linear_mm_s = 0.0
        self.manual_angular_mdeg_s = 0.0
        self.manual_until_ms = 0
        
        # Nav2 command from standalone bridge, in the firmware's units.
        self.nav_linear_mm_s = 0.0
        self.nav_angular_mdeg_s = 0.0
        self.nav_until_ms = 0
        
        self.control_mode: Literal["manual", "nav2"] = "manual"

        # Session stats feeding the AI advisor's summary (§9.1).
        self.collision_events: deque[CollisionEvent] = deque(
            maxlen=MAX_COLLISION_EVENTS
        )
        self.stuck_events = 0
        self.watchdog_trips = 0
        self.distance_travelled_mm = 0.0
        self.last_trail_point: tuple[float, float] | None = None

        self.ros_available = False
        self.ros_nodes_seen: set[str] = set()

        # Populated by ai_advisor at startup — §9 forbids a hardcoded model id.
        self.ai_model: str | None = None
        self.ai_model_candidates: list[str] = []
        self.ai_last_error: str | None = None

    # -- run state ---------------------------------------------------------
    def set_running(self, running: bool, *, estop: bool = False) -> None:
        self.running = running
        if running:
            self.estop_latched = False
            if self.run_started_at_ms is None:
                self.run_started_at_ms = now_ms()
        else:
            self.run_started_at_ms = None
            self.manual_linear_mm_s = 0.0
            self.manual_angular_mdeg_s = 0.0
            if estop:
                self.estop_latched = True

    # -- telemetry ---------------------------------------------------------
    def note_pose(self, x_mm: float, y_mm: float) -> None:
        point = (x_mm, y_mm)
        if self.last_trail_point is not None:
            dx = x_mm - self.last_trail_point[0]
            dy = y_mm - self.last_trail_point[1]
            step = math.hypot(dx, dy)
            if step < 5.0:
                return  # ignore sub-5 mm jitter so the trail stays readable
            self.distance_travelled_mm += step
        self.last_trail_point = point
        self.pose_trail.append(point)

    def note_collision(self, distance_mm: float) -> CollisionEvent:
        event = CollisionEvent(
            t_ms=now_ms(),
            distance_mm=distance_mm,
            pose_x_mm=self.odom.x_mm,
            pose_y_mm=self.odom.y_mm,
            theta_rad=self.odom.theta_rad,
        )
        self.collision_events.append(event)
        return event

    # -- snapshots ---------------------------------------------------------
    def status_snapshot(self) -> dict[str, Any]:
        """The Dashboard's whole payload — everything except scan/map bulk."""
        return {
            "t_ms": now_ms(),
            "uptime_ms": now_ms() - self.started_at_ms,
            "running": self.running,
            "estop_latched": self.estop_latched,
            "run_started_at_ms": self.run_started_at_ms,
            "control_mode": self.control_mode,
            "devices": {
                "arduino": self.arduino.snapshot(),
                "nodemcu": self.nodemcu.snapshot(),
            },
            "odom": self.odom.snapshot(),
            "scan_meta": {
                "seq": self.scan.seq,
                "t_ms": self.scan.t_ms,
                "rev_ms": self.scan.rev_ms,
                "n": len(self.scan.angles_deg),
                "min_distance_mm": self.scan.min_distance_mm,
            },
            "map_meta": {
                "width": self.map.width,
                "height": self.map.height,
                "resolution": self.map.resolution,
                "stamp_ms": self.map.stamp_ms,
            },
            "ros": {
                "available": self.ros_available,
                "nodes": sorted(self.ros_nodes_seen),
            },
            "session": {
                "collisions": len(self.collision_events),
                "stuck_events": self.stuck_events,
                "watchdog_trips": self.watchdog_trips,
                "distance_travelled_mm": self.distance_travelled_mm,
            },
            "ai": {
                "model": self.ai_model,
                "candidates": self.ai_model_candidates,
                "last_error": self.ai_last_error,
            },
            "log_counts": self.logs.counts(),
        }

    def tuning_snapshot(self) -> dict[str, Any]:
        dirty = sorted(
            name
            for name in self.tuning
            if self.tuning[name] != self.tuning_saved.get(name)
        )
        return {
            "values": dict(self.tuning),
            "saved": dict(self.tuning_saved),
            "dirty": dirty,
            "acked": {k: dict(v) for k, v in self.tuning_acked.items()},
            "defaults": dict(DEFAULTS),
        }

    def session_summary(self) -> dict[str, Any]:
        """Text-safe session summary for the AI advisor (§9.1: summarise, never
        ship raw sensor streams to the API)."""
        run_ms = 0
        if self.run_started_at_ms is not None:
            run_ms = now_ms() - self.run_started_at_ms

        coverage = None
        if not self.map.is_empty:
            known = 0
            total = 0
            for i in range(0, len(self.map.rle) - 1, 2):
                value, count = self.map.rle[i], self.map.rle[i + 1]
                total += count
                if value >= 0:
                    known += count
            if total:
                coverage = round(100.0 * known / total, 1)

        recent = [e.snapshot() for e in list(self.collision_events)[-10:]]
        counts = self.logs.counts()
        return {
            "run_duration_s": round(run_ms / 1000.0, 1),
            "distance_travelled_mm": round(self.distance_travelled_mm, 1),
            "collision_count": len(self.collision_events),
            "recent_collisions": recent,
            "stuck_events": self.stuck_events,
            "watchdog_trips": self.watchdog_trips,
            "map_coverage_pct": coverage,
            "map_dimensions": {
                "width_cells": self.map.width,
                "height_cells": self.map.height,
                "resolution_m": self.map.resolution,
            },
            "log_counts": counts,
            "current_tuning": {
                p.name: self.tuning.get(p.name) for p in PARAMS
            },
            "devices_connected": {
                "arduino": self.arduino.connected,
                "nodemcu": self.nodemcu.connected,
            },
        }


# Single process-wide instance. Imported by every other backend module.
STATE = AppState()
