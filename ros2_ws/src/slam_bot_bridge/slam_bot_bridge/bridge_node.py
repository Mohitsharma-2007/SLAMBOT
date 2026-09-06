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

It also owns the autonomy entry point. The browser's "click a point on the map"
goal arrives here as a ``nav_goal`` frame and is sent to Nav2's NavigateToPose
action server, because in the Windows-backend + WSL-ROS split this process is
the only one that has both rclpy and a route to the browser. Nav2's resulting
/cmd_vel comes back through ``_on_cmd_vel`` and out to the MCU. That round trip
is what makes the robot drive itself.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
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

    Handedness (this is load-bearing): the RPLIDAR A1 reports its angle
    increasing CLOCKWISE viewed from above, while sensor_msgs/LaserScan is read
    counter-clockwise about +Z (REP-103, and this node publishes a positive
    angle_increment). Binning the raw angle therefore publishes a mirror image
    of the room. Straight-line driving still looks fine — a mirrored world is
    self-consistent — but every rotation is seen by the scan matcher as turning
    the *opposite* way to odometry, so the pose graph is fed a contradiction on
    every turn and the map smears and double-walls. Negating here, once, at the
    single point where sensor degrees become beam indices, is the whole fix.
    """
    ranges = [float("inf")] * bins
    bin_width = 360.0 / bins
    for angle, dist_mm in zip(angles_deg, dists_mm):
        dist_m = dist_mm / 1000.0
        if dist_m < range_min_m or dist_m > range_max_m:
            continue
        index = int(((360.0 - angle) % 360.0) / bin_width)
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

        # Autonomy: Nav2's goal endpoint. Created lazily on first use so the
        # bridge still starts when Nav2 is not running (mapping-only sessions).
        self._nav_client: ActionClient | None = None
        self._goal_handle: Any = None

        # millis() on each MCU is boot-relative and unsynchronised with the ROS
        # clock, so we learn a per-device offset instead of stamping on arrival.
        self._clock_offset_ns: dict[str, int] = {}

        self._ws: Any = None
        self._ws_lock = threading.Lock()
        self._stop = threading.Event()
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta = 0.0
        self._tf_timer = self.create_timer(0.05, self._broadcast_odom_tf)

        self._publish_static_tf()

        self._thread = threading.Thread(
            target=self._websocket_loop, name="backend-ws", daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            f"standalone bridge started, connecting to "
            f"{self.get_parameter('backend_url').value}"
        )

    def _broadcast_odom_tf(self) -> None:
        if not bool(self.get_parameter("publish_odom_tf").value):
            return
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = self._current_x
        t.transform.translation.y = self._current_y
        t.transform.rotation = yaw_to_quaternion(self._current_theta)
        self.tf_broadcaster.sendTransform(t)

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

    def _log_to_backend(self, level: str, message: str) -> None:
        """Surface a message on the web app's Logs page under the 'nav' source."""
        self._send(
            {"type": "log", "source": "nav", "level": level, "msg": message}
        )

    # -- timestamps --------------------------------------------------------
    def _stamp_from_mcu(self, source: str, t_ms: Any):
        """Map an MCU ``millis()`` reading onto the ROS clock.

        Stamping on arrival makes every message look like it was captured the
        instant it finished crossing WiFi, so scans and odometry — which travel
        different paths at different rates — get timestamps whose ordering has
        nothing to do with when the events actually happened. slam_toolbox then
        matches a scan against the wrong pose, which smears walls.

        millis() is boot-relative, so the offset to the ROS clock is unknown but
        constant. The least-delayed frame seen is the best estimate of it, so we
        track the running minimum of (now - t_ms). A backwards jump means the
        MCU rebooted or millis() wrapped, so re-latch rather than staying pinned
        to a stale offset.
        """
        now_ns = self.get_clock().now().nanoseconds
        try:
            device_ns = int(float(t_ms) * 1_000_000)
        except (TypeError, ValueError):
            return self.get_clock().now().to_msg()

        candidate = now_ns - device_ns
        previous = self._clock_offset_ns.get(source)
        if previous is None or abs(candidate - previous) > 5_000_000_000:
            # First frame, or the device clock restarted.
            self._clock_offset_ns[source] = candidate
        elif candidate < previous:
            self._clock_offset_ns[source] = candidate
        else:
            # Bleed upward very slowly so millis() drift cannot pin us to an
            # offset learned from one unusually fast frame hours ago.
            self._clock_offset_ns[source] = previous + 1_000_000

        stamp_ns = device_ns + self._clock_offset_ns[source]
        # Never hand out a future stamp: tf2 refuses to extrapolate forward.
        if stamp_ns > now_ns:
            stamp_ns = now_ns
        from builtin_interfaces.msg import Time as TimeMsg

        return TimeMsg(sec=int(stamp_ns // 1_000_000_000),
                       nanosec=int(stamp_ns % 1_000_000_000))

    # -- autonomy: goals from the browser ----------------------------------
    def _handle_nav_goal(self, data: dict[str, Any]) -> None:
        """Send a clicked map point to Nav2 as a NavigateToPose goal."""
        try:
            x_m = float(data.get("x_m"))
            y_m = float(data.get("y_m"))
            theta = float(data.get("theta_rad", 0.0) or 0.0)
        except (TypeError, ValueError):
            self._log_to_backend("error", "nav_goal ignored: non-numeric x/y")
            return

        if self._nav_client is None:
            self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self._log_to_backend(
                "error",
                "Nav2 navigate_to_pose action server not available — is the "
                "Nav2 stack running and activated?",
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # Leave the stamp at zero: "the latest available transform", which is
        # what you want for a goal, rather than a pose pinned to an instant that
        # may already have fallen out of the TF buffer.
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x_m
        goal.pose.pose.position.y = y_m
        goal.pose.pose.orientation = yaw_to_quaternion(theta)

        self.get_logger().info(f"sending Nav2 goal ({x_m:.2f}, {y_m:.2f})")
        self._log_to_backend("info", f"goal sent to Nav2: ({x_m:.2f}, {y_m:.2f}) m")
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._log_to_backend("error", f"Nav2 rejected the goal request: {exc}")
            return
        if not handle.accepted:
            self._log_to_backend("warn", "Nav2 rejected the goal (unreachable?)")
            return
        self._goal_handle = handle
        self._log_to_backend("info", "Nav2 accepted the goal — navigating")
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_nav_feedback(self, feedback: Any) -> None:
        try:
            remaining = float(feedback.feedback.distance_remaining)
        except Exception:
            return
        self._send({"type": "nav_feedback", "distance_remaining_m": round(remaining, 3)})

    def _on_goal_result(self, future: Any) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self._log_to_backend("error", f"Nav2 goal result unavailable: {exc}")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._log_to_backend("info", "Nav2 goal reached")
        elif status == GoalStatus.STATUS_CANCELED:
            self._log_to_backend("warn", "Nav2 goal cancelled")
        else:
            self._log_to_backend("error", f"Nav2 goal failed (status {status})")
        self._goal_handle = None
        self._send({"type": "nav_done", "status": int(status)})

    def _cancel_goal(self) -> None:
        handle = self._goal_handle
        self._goal_handle = None
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
            self._log_to_backend("info", "Nav2 goal cancelled by operator")
        except Exception as exc:
            self.get_logger().debug(f"cancel failed: {exc}")

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

        # Commands the backend addresses to us directly carry a "type"; topic
        # broadcasts carry a "topic". Handle commands first.
        kind = frame.get("type")
        if kind == "nav_goal":
            self._handle_nav_goal(frame.get("data") or frame)
            return
        if kind in ("nav_cancel", "estop"):
            self._cancel_goal()
            return

        topic = frame.get("topic")
        data = frame.get("data")
        if not isinstance(data, dict):
            return

        if topic == "nav_goal":
            self._handle_nav_goal(data)
        elif topic == "nav_cancel":
            self._cancel_goal()
        elif topic == "scan":
            self._publish_scan(data)
        elif topic == "odom":
            # Dedicated high-rate odometry feed (see the backend's odom_loop).
            self._publish_odom(data)
        elif topic == "status":
            if "odom" in data and isinstance(data["odom"], dict):
                self._publish_odom(data["odom"])

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
        self._current_x = x_m
        self._current_y = y_m
        self._current_theta = theta
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
