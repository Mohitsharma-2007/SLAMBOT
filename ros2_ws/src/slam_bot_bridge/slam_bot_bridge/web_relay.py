"""slam_bot_web_relay — forwards ROS topics to the backend for the browser (§7).

Subscribes to /scan, /map, /odom and /plan and pushes them to the FastAPI
backend over a WebSocket, which fans them out to browser clients.

As with bridge_node, this is only needed when ROS2 and the backend run as
separate processes. In the default embedded setup the backend's own rclpy node
already subscribes to /map and /plan directly, so this relay is redundant —
launch files leave it off unless ``standalone_bridge:=true``.

The occupancy grid is run-length encoded before sending: a 4000x4000 grid is 16
million cells, but unexplored space dominates and collapses to almost nothing
under RLE.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


def rle_encode(cells: Iterable[int]) -> list[int]:
    """Flat [value, count, value, count, ...] encoding."""
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


class WebRelay(Node):
    def __init__(self) -> None:
        super().__init__("slam_bot_web_relay")

        self.declare_parameter("backend_url", "ws://127.0.0.1:8000/ws/relay")
        # Rate limits: /scan and /map arrive faster than a browser can paint.
        self.declare_parameter("scan_max_hz", 8.0)
        self.declare_parameter("map_max_hz", 1.0)
        self.declare_parameter("odom_max_hz", 10.0)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, sensor_qos)
        self.create_subscription(Path, "plan", self._on_plan, 10)

        self._ws: Any = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_sent: dict[str, float] = {}

        self._thread = threading.Thread(
            target=self._websocket_loop, name="relay-ws", daemon=True
        )
        self._thread.start()
        self.get_logger().info("web relay started")

    # -- rate limiting -----------------------------------------------------
    def _should_send(self, topic: str, max_hz: float) -> bool:
        if max_hz <= 0:
            return True
        now = time.monotonic()
        last = self._last_sent.get(topic, 0.0)
        if now - last < 1.0 / max_hz:
            return False
        self._last_sent[topic] = now
        return True

    # -- subscriptions -----------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        if not self._should_send("scan", float(self.get_parameter("scan_max_hz").value)):
            return
        # Convert back to (angle, distance) pairs, dropping inf/nan beams so the
        # browser's polar plot only receives real returns.
        angles: list[float] = []
        dists: list[float] = []
        for i, distance in enumerate(msg.ranges):
            if distance != distance or distance in (float("inf"), float("-inf")):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle_rad = msg.angle_min + i * msg.angle_increment
            angles.append(round(angle_rad * 57.29577951308232, 2))
            dists.append(round(distance * 1000.0, 1))
        self._send(
            "ros_scan",
            {
                "angles_deg": angles,
                "dists_mm": dists,
                "frame_id": msg.header.frame_id,
                "n": len(angles),
            },
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        if not self._should_send("map", float(self.get_parameter("map_max_hz").value)):
            return
        self._send(
            "ros_map",
            {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "rle": rle_encode(msg.data),
            },
        )

    def _on_odom(self, msg: Odometry) -> None:
        if not self._should_send("odom", float(self.get_parameter("odom_max_hz").value)):
            return
        self._send(
            "ros_odom",
            {
                "x_m": msg.pose.pose.position.x,
                "y_m": msg.pose.pose.position.y,
                "linear_m_s": msg.twist.twist.linear.x,
                "angular_rad_s": msg.twist.twist.angular.z,
            },
        )

    def _on_plan(self, msg: Path) -> None:
        self._send(
            "ros_plan",
            {
                "points": [
                    [round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                    for p in msg.poses
                ]
            },
        )

    # -- transport ---------------------------------------------------------
    def _send(self, topic: str, data: dict[str, Any]) -> None:
        with self._lock:
            ws = self._ws
            if ws is None:
                return
            try:
                ws.send(json.dumps({"topic": topic, "data": data}, separators=(",", ":")))
            except Exception as exc:
                self.get_logger().debug(f"relay send failed: {exc}")

    def _websocket_loop(self) -> None:
        try:
            from websockets.sync.client import connect
        except ImportError:
            self.get_logger().error(
                "the 'websockets' package is required for the web relay: "
                "pip install websockets"
            )
            return

        url = str(self.get_parameter("backend_url").value)
        while not self._stop.is_set():
            try:
                with connect(url, open_timeout=5, close_timeout=2) as ws:
                    with self._lock:
                        self._ws = ws
                    self.get_logger().info(f"relay connected to {url}")
                    # Hold the connection open; this relay is send-only, so just
                    # block on reads to detect closure.
                    for _ in ws:
                        if self._stop.is_set():
                            break
            except Exception as exc:
                self.get_logger().warning(f"relay link down ({exc}); retrying in 2 s")
            finally:
                with self._lock:
                    self._ws = None
            if not self._stop.is_set():
                time.sleep(2.0)

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
