"""ROS2 bridge — an rclpy node driven from a background thread.

This module is imported unconditionally but ROS2 is *optional*. On a machine
without rclpy (e.g. the Windows dev box this repo is developed on) every method
here becomes a cheap no-op and the rest of the backend runs unchanged: the
Dashboard, LIDAR polar plot, Logs, Flash Center and Tuning Panel all work
without ROS. Only the occupancy grid and Nav2 goals require it.

Threading model
---------------
rclpy wants its own executor thread; FastAPI owns the asyncio loop. So:

  * The node spins in a daemon thread with a SingleThreadedExecutor.
  * asyncio -> ROS calls (publish_scan, publish_odom, send_goal) are plain
    method calls that only touch thread-safe rclpy publishers, or hand work to
    the executor thread via a lock-guarded field.
  * ROS -> asyncio hand-off (map, plan, cmd_vel) uses
    ``run_coroutine_threadsafe`` against the loop captured at startup, so state
    mutation still happens under the asyncio lock and never races the WebSocket
    handlers.

Topics (per §7):
  publishes    /scan (LaserScan), /odom (Odometry), /tf, /tf_static
  subscribes   /cmd_vel, /map, /plan
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Any

logger = logging.getLogger("slam_bot.ros")

try:  # ROS2 is optional — see module docstring.
    import rclpy
    from geometry_msgs.msg import Quaternion, Twist, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    RCLPY_AVAILABLE = True
except Exception as _import_error:  # pragma: no cover - depends on environment
    RCLPY_AVAILABLE = False
    _RCLPY_IMPORT_ERROR = _import_error
    Node = object  # type: ignore[assignment,misc]


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation about Z only."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Scan conversion — shared with the no-ROS path so it stays testable
# ---------------------------------------------------------------------------
def scan_to_ranges(
    angles_deg: list[float],
    dists_mm: list[float],
    bins: int = 360,
    range_min_m: float = 0.15,
    range_max_m: float = 6.0,
) -> list[float]:
    """Bin sparse (angle, distance) samples into a fixed-size ranges array.

    A LaserScan needs evenly spaced beams, but the RPLIDAR delivers samples at
    irregular angles that shift every revolution. Binning into fixed 1-degree
    buckets and
    keeping the *nearest* return per bucket is the conservative choice: for
    obstacle avoidance, under-reporting distance is safe and over-reporting is
    not. Empty buckets become inf, which slam_toolbox and the costmaps read as
    "no return" rather than "clear".
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


class _BridgeNode(Node):  # type: ignore[misc]
    """The actual rclpy node. Only constructed when rclpy imported cleanly."""

    def __init__(self, bridge: "RosBridge") -> None:
        super().__init__("slam_bot_bridge")
        self._bridge = bridge

        self.declare_parameter("scan_bins", 360)
        self.declare_parameter("scan_range_min", 0.15)
        self.declare_parameter("scan_range_max", 6.0)
        self.declare_parameter("encoder_cpr", 700.0)
        self.declare_parameter("wheel_diameter", 0.043)
        self.declare_parameter("wheel_base", 0.150)
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        # §7: static TF base_link -> laser, default 0,0,0 unless measured.
        self.declare_parameter("laser_offset_x", 0.0)
        self.declare_parameter("laser_offset_y", 0.0)
        self.declare_parameter("laser_offset_z", 0.0)
        self.declare_parameter("laser_offset_yaw", 0.0)
        self.declare_parameter("publish_odom_tf", True)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.scan_pub = self.create_publisher(LaserScan, "scan", sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "odom", sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
        self.create_subscription(Path, "plan", self._on_plan, 10)

        self._publish_static_tf()
        self.get_logger().info("slam_bot_bridge node up")

    # -- static TF ---------------------------------------------------------
    def _publish_static_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("base_frame").value
        t.child_frame_id = self.get_parameter("laser_frame").value
        t.transform.translation.x = float(self.get_parameter("laser_offset_x").value)
        t.transform.translation.y = float(self.get_parameter("laser_offset_y").value)
        t.transform.translation.z = float(self.get_parameter("laser_offset_z").value)
        qx, qy, qz, qw = _yaw_to_quaternion(
            float(self.get_parameter("laser_offset_yaw").value)
        )
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.static_tf.sendTransform(t)

    # -- subscriptions -----------------------------------------------------
    def _on_cmd_vel(self, msg: "Twist") -> None:
        self._bridge._store_cmd_vel(msg.linear.x, msg.angular.z)

    def _on_map(self, msg: "OccupancyGrid") -> None:
        self._bridge._store_map(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=msg.data,
        )

    def _on_plan(self, msg: "Path") -> None:
        points = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._bridge._store_plan(points)

    # -- publishing --------------------------------------------------------
    def publish_scan(self, snapshot: dict[str, Any]) -> None:
        bins = int(self.get_parameter("scan_bins").value)
        rmin = float(self.get_parameter("scan_range_min").value)
        rmax = float(self.get_parameter("scan_range_max").value)

        ranges = scan_to_ranges(
            snapshot.get("angles_deg", []),
            snapshot.get("dists_mm", []),
            bins=bins,
            range_min_m=rmin,
            range_max_m=rmax,
        )

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("laser_frame").value
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (bins - 1) / bins
        msg.angle_increment = 2.0 * math.pi / bins
        rev_ms = float(snapshot.get("rev_ms") or 0.0)
        msg.scan_time = (rev_ms / 1000.0) if rev_ms > 0 else 0.18
        msg.time_increment = msg.scan_time / bins
        msg.range_min = rmin
        msg.range_max = rmax
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def publish_odom(
        self,
        x_m: float,
        y_m: float,
        theta_rad: float,
        linear_m_s: float,
        angular_rad_s: float,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        qx, qy, qz, qw = _yaw_to_quaternion(theta_rad)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.twist.twist.linear.x = linear_m_s
        msg.twist.twist.angular.z = angular_rad_s
        # Wheel odometry on N20s with a plastic chassis is not precise; give
        # slam_toolbox an honest covariance rather than an optimistic one.
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
            t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self.tf_broadcaster.sendTransform(t)


class RosBridge:
    """Facade the rest of the backend talks to. Safe to call with no ROS2."""

    def __init__(self) -> None:
        self._node: Any = None
        self._executor: Any = None
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._nav_client: Any = None
        self._goal_handle: Any = None
        self._lock = threading.Lock()
        self._cmd_vel: tuple[float, float] = (0.0, 0.0)
        self._cmd_vel_t = 0.0
        self.available = False

    # -- lifecycle ---------------------------------------------------------
    def start(self, loop: asyncio.AbstractEventLoop) -> bool:
        """Bring up the node in a background thread. Returns availability."""
        self._loop = loop
        if not RCLPY_AVAILABLE:
            logger.warning(
                "rclpy unavailable (%s) — /map and Nav2 goals are disabled; "
                "everything else works",
                _RCLPY_IMPORT_ERROR,
            )
            return False
        try:
            rclpy.init(args=None)
            self._node = _BridgeNode(self)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._spin, name="ros-executor", daemon=True
            )
            self._thread.start()
            self.available = True
            logger.info("ROS2 bridge started")
            return True
        except Exception:
            logger.exception("ROS2 bridge failed to start — continuing without it")
            self.available = False
            return False

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            logger.exception("ROS executor stopped")

    def shutdown(self) -> None:
        if not self.available:
            return
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            logger.debug("ROS shutdown raised", exc_info=True)
        finally:
            self.available = False

    # -- asyncio -> ROS ----------------------------------------------------
    def publish_scan(self, snapshot: dict[str, Any]) -> None:
        if not self.available or self._node is None:
            return
        try:
            self._node.publish_scan(snapshot)
        except Exception:
            logger.debug("publish_scan failed", exc_info=True)

    def publish_odom(
        self,
        x_m: float,
        y_m: float,
        theta_rad: float,
        linear_m_s: float,
        angular_rad_s: float,
    ) -> None:
        if not self.available or self._node is None:
            return
        try:
            self._node.publish_odom(x_m, y_m, theta_rad, linear_m_s, angular_rad_s)
        except Exception:
            logger.debug("publish_odom failed", exc_info=True)

    def apply_tuning(self, values: dict[str, Any]) -> None:
        """Set ROS parameters for §10 values that Nav2 / the bridge own."""
        if not self.available or self._node is None:
            return
        import tuning as tuning_registry
        from rclpy.parameter import Parameter

        triples = tuning_registry.ros_overrides(values)
        own_name = "slam_bot_bridge"
        for node_name, param_name, value in triples:
            if node_name == own_name:
                try:
                    self._node.set_parameters(
                        [Parameter(param_name, Parameter.Type.DOUBLE, float(value))]
                    )
                except Exception:
                    logger.debug("local param %s failed", param_name, exc_info=True)
            else:
                # Remote Nav2 nodes: fire-and-forget async param set. Nav2
                # accepts dynamic updates for velocity limits and inflation
                # radius, which is exactly what §10 exposes.
                self._set_remote_param(node_name, param_name, value)

    def _set_remote_param(self, node_name: str, param_name: str, value: float) -> None:
        try:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue

            client = self._node.create_client(
                SetParameters, f"/{node_name}/set_parameters"
            )
            if not client.wait_for_service(timeout_sec=0.2):
                logger.debug("no param service for %s", node_name)
                return
            request = SetParameters.Request()
            request.parameters = [
                ParamMsg(
                    name=param_name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                    ),
                )
            ]
            client.call_async(request)
        except Exception:
            logger.debug("remote param set failed for %s", node_name, exc_info=True)

    def latest_cmd_vel(self, max_age_s: float = 0.5) -> tuple[float, float]:
        """Nav2's most recent /cmd_vel, or zeros if it has gone stale.

        Staleness matters: if the controller server dies mid-run, the last
        command must not be repeated forever at the heartbeat rate.
        """
        with self._lock:
            linear, angular = self._cmd_vel
            age = time.monotonic() - self._cmd_vel_t
        if age > max_age_s:
            return (0.0, 0.0)
        return (linear, angular)

    def send_goal(self, x_m: float, y_m: float, theta_rad: float) -> tuple[bool, str]:
        if not self.available or self._node is None:
            return False, "ROS2 is not available on this backend"
        try:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient

            if self._nav_client is None:
                self._nav_client = ActionClient(
                    self._node, NavigateToPose, "navigate_to_pose"
                )
            if not self._nav_client.wait_for_server(timeout_sec=2.0):
                return False, "Nav2 navigate_to_pose action server not available"

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x_m
            goal.pose.pose.position.y = y_m
            qx, qy, qz, qw = _yaw_to_quaternion(theta_rad)
            goal.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            future = self._nav_client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)
            return True, "goal sent to Nav2"
        except Exception as exc:
            logger.exception("send_goal failed")
            return False, f"goal failed: {exc}"

    def _on_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
        except Exception:
            self._emit_log("nav", "error", "Nav2 rejected the goal request")
            return
        if not handle.accepted:
            self._emit_log("nav", "warn", "Nav2 rejected the goal")
            return
        self._goal_handle = handle
        self._emit_log("nav", "info", "Nav2 accepted the goal")
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future: Any) -> None:
        try:
            status = future.result().status
        except Exception:
            self._emit_log("nav", "error", "Nav2 goal result unavailable")
            return
        # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
        self._emit_log(
            "nav",
            "info" if status == 4 else "warn",
            f"Nav2 goal finished with status {status}",
        )
        self._goal_handle = None

    def cancel_navigation(self) -> None:
        handle = self._goal_handle
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
            self._emit_log("nav", "info", "Nav2 goal cancelled")
        except Exception:
            logger.debug("cancel failed", exc_info=True)
        finally:
            self._goal_handle = None

    # -- ROS -> asyncio ----------------------------------------------------
    def _store_cmd_vel(self, linear_m_s: float, angular_rad_s: float) -> None:
        with self._lock:
            self._cmd_vel = (float(linear_m_s), float(angular_rad_s))
            self._cmd_vel_t = time.monotonic()

    def _store_map(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: Any,
    ) -> None:
        from state import STATE, rle_encode, now_ms

        rle = rle_encode(data)

        async def apply() -> None:
            async with STATE.lock:
                STATE.map.width = int(width)
                STATE.map.height = int(height)
                STATE.map.resolution = float(resolution)
                STATE.map.origin_x = float(origin_x)
                STATE.map.origin_y = float(origin_y)
                STATE.map.rle = rle
                STATE.map.stamp_ms = now_ms()
                STATE.ros_nodes_seen.add("slam_toolbox")

        self._dispatch(apply())

    def _store_plan(self, points: list[tuple[float, float]]) -> None:
        from state import STATE

        async def apply() -> None:
            async with STATE.lock:
                STATE.plan = points
                STATE.ros_nodes_seen.add("planner_server")

        self._dispatch(apply())

    def _emit_log(self, source: str, level: str, message: str) -> None:
        from state import STATE

        async def apply() -> None:
            async with STATE.lock:
                STATE.logs.emit(source, level, message)

        self._dispatch(apply())

    def _dispatch(self, coro: Any) -> None:
        """Run a coroutine on the asyncio loop from the ROS executor thread."""
        loop = self._loop
        if loop is None or loop.is_closed():
            coro.close()
            return
        try:
            asyncio.run_coroutine_threadsafe(coro, loop)
        except Exception:
            coro.close()
            logger.debug("dispatch to asyncio loop failed", exc_info=True)


BRIDGE = RosBridge()
