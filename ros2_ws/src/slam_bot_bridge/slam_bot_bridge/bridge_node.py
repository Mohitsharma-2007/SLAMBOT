"""Standalone MCU <-> ROS2 bridge node.

Two ways to run the bridge exist, and they are alternatives, not both:

  A. Embedded (default, §4/§5): the FastAPI backend runs the bridge in-process
     via ``backend/ros_bridge.py``. One process holds the MCU sockets and the
     ROS node, so there is no extra hop. Use this unless you have a reason not
     to.

  B. Standalone (this node): the bridge is its own process and connects to the
     backend as a *WebSocket client* on ``/ws/app``, consuming the same scan and
     odom topics the browser sees and republishing them onto ROS. Useful when
     ROS2 lives on a different machine from the backend, or when you want the
     ROS graph to survive a backend restart.

Running both at once would publish /scan twice. Launch files default to (A);
pass ``standalone_bridge:=true`` to use this instead.

Topics (§7):  publishes /scan, /odom, /tf, /tf_static   subscribes /cmd_vel
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def scan_to_ranges(
    angles_deg: list[float],
    dists_mm: list[float],
    bins: int,
    range_min_m: float,
    range_max_m: float,
) -> list[float]:
    """Bin irregular RPLIDAR samples into fixed-width LaserScan beams.

    Keeps the nearest return per bin: for obstacle avoidance, under-reporting
    distance is the safe direction to err. Empty bins are inf, which costmaps
    read as "no information" rather than "clear".
    """
    ranges = [float("inf")] * bins
    bin_width = 360.0 / bins
    for angle, dist_mm in zip(angles_deg, dists_mm):
        dist_m = dist_mm / 1000.0
        if dist_m < range_min_m or dist_m > range_max_m:
            continue
        index = int((angle % 360.0) / bin_width)
        if index >= bins:
            index = bins - 1
        if dist_m < ranges[index]:
            ranges[index] = dist_m
    return ranges


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("slam_bot_bridge")

        self.declare_parameter("backend_url", "ws://127.0.0.1:8000/ws/app")
        self.declare_parameter("scan_bins", 360)
        self.declare_parameter("scan_range_min", 0.15)
        self.declare_parameter("scan_range_max", 6.0)
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("laser_offset_x", 0.0)
        self.declare_parameter("laser_offset_y", 0.0)
        self.declare_parameter("laser_offset_z", 0.0)
        self.declare_parameter("laser_offset_yaw", 0.0)
        self.declare_parameter("publish_odom_tf", True)
        # Odometry geometry — §10 says measure these. The bridge only uses them
        # for logging/validation here since the MCU integrates pose itself.
        self.declare_parameter("encoder_cpr", 700.0)
        self.declare_parameter("wheel_diameter", 0.043)
        self.declare_parameter("wheel_base", 0.150)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_pub = self.create_publisher(LaserScan, "scan", sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "odom", sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        self._ws: Any = None
        self._ws_lock = threading.Lock()
        self._stop = threading.Event()

        self._publish_static_tf()

        self._thread = threading.Thread(
            target=self._websocket_loop, name="backend-ws", daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            f"standalone bridge started, connecting to "
            f"{self.get_parameter('backend_url').value}"
        )

    # -- static TF (§7) ----------------------------------------------------
    def _publish_static_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("base_frame").value
        t.child_frame_id = self.get_parameter("laser_frame").value
        t.transform.translation.x = float(self.get_parameter("laser_offset_x").value)
        t.transform.translation.y = float(self.get_parameter("laser_offset_y").value)
        t.transform.translation.z = float(self.get_parameter("laser_offset_z").value)
        t.transform.rotation = yaw_to_quaternion(
            float(self.get_parameter("laser_offset_yaw").value)
        )
        self.static_tf.sendTransform(t)

    # -- cmd_vel -> backend ------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        """Forward Nav2's velocity command to the backend, which owns the MCU.

        The backend still gates this behind the Start/Stop run-state, so a
        stopped robot ignores whatever Nav2 emits (§11.2).
        """
        payload = {
            "type": "nav_cmd_vel",
            "linear_mm_s": msg.linear.x * 1000.0,
            "angular_mdeg_s": math.degrees(msg.angular.z) * 1000.0,
        }
        self._send(payload)

    def _send(self, payload: dict[str, Any]) -> None:
        with self._ws_lock:
            ws = self._ws
            if ws is None:
                return
            try:
                ws.send(json.dumps(payload))
            except Exception as exc:
                self.get_logger().debug(f"send failed: {exc}")

    # -- backend WebSocket client -----------------------------------------
    def _websocket_loop(self) -> None:
        """Reconnecting client loop, run in its own thread."""
        try:
            from websockets.sync.client import connect
        except ImportError:
            self.get_logger().error(
                "the 'websockets' package is required for the standalone bridge: "
                "pip install websockets"
            )
            return

        url = str(self.get_parameter("backend_url").value)
        while not self._stop.is_set():
            try:
                with connect(url, open_timeout=5, close_timeout=2) as ws:
                    with self._ws_lock:
                        self._ws = ws
                    self.get_logger().info(f"connected to backend at {url}")
                    for raw in ws:
                        if self._stop.is_set():
                            break
                        self._handle_frame(raw)
            except Exception as exc:
                self.get_logger().warning(f"backend link down ({exc}); retrying in 2 s")
            finally:
                with self._ws_lock:
                    self._ws = None
            if not self._stop.is_set():
                time.sleep(2.0)

    def _handle_frame(self, raw: str | bytes) -> None:
        try:
            frame = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(frame, dict):
            return

        topic = frame.get("topic")
        data = frame.get("data")
        if not isinstance(data, dict):
            return

        if topic == "scan":
            self._publish_scan(data)
        elif topic == "status":
            odom = data.get("odom")
            if isinstance(odom, dict):
                self._publish_odom(odom)

    def _publish_scan(self, data: dict[str, Any]) -> None:
        bins = int(self.get_parameter("scan_bins").value)
        rmin = float(self.get_parameter("scan_range_min").value)
        rmax = float(self.get_parameter("scan_range_max").value)

        angles = data.get("angles_deg") or []
        dists = data.get("dists_mm") or []
        if not angles or not dists:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("laser_frame").value)
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (bins - 1) / bins
        msg.angle_increment = 2.0 * math.pi / bins
        rev_ms = float(data.get("rev_ms") or 0.0)
        msg.scan_time = (rev_ms / 1000.0) if rev_ms > 0 else 0.18
        msg.time_increment = msg.scan_time / bins
        msg.range_min = rmin
        msg.range_max = rmax
        msg.ranges = scan_to_ranges(
            [float(a) for a in angles],
            [float(d) for d in dists],
            bins,
            rmin,
            rmax,
        )
        self.scan_pub.publish(msg)

    def _publish_odom(self, data: dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)

        x_m = float(data.get("x_mm", 0.0)) / 1000.0
        y_m = float(data.get("y_mm", 0.0)) / 1000.0
        theta = float(data.get("theta_rad", 0.0))
        linear = float(data.get("linear_mm_s", 0.0)) / 1000.0
        angular = math.radians(float(data.get("angular_mdeg_s", 0.0)) / 1000.0)
        quat = yaw_to_quaternion(theta)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        msg.pose.pose.orientation = quat
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        # Honest covariance for wheel odometry on a small hobby chassis.
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        if bool(self.get_parameter("publish_odom_tf").value):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = x_m
            t.transform.translation.y = y_m
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
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
