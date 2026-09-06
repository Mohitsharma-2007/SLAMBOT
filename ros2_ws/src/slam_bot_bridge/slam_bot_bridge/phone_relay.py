import json
import math
import threading
import time
import base64
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix, CompressedImage
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (rad) to geometry_msgs/Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy
    )

class PhoneRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("phone_relay")

        self.declare_parameter("backend_url", "ws://127.0.0.1:8000/ws/phone_relay")
        self.declare_parameter("phone_frame", "phone_link")

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "phone/imu", sensor_qos)
        self.gps_pub = self.create_publisher(NavSatFix, "phone/gps", sensor_qos)
        self.cam_pub = self.create_publisher(CompressedImage, "phone/camera/compressed", sensor_qos)

        self._ws: Any = None
        self._ws_lock = threading.Lock()
        self._stop = threading.Event()

        self._thread = threading.Thread(
            target=self._websocket_loop, name="phone-ws", daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            f"Phone relay node started, connecting to "
            f"{self.get_parameter('backend_url').value}"
        )

    def _websocket_loop(self) -> None:
        try:
            from websockets.sync.client import connect
        except ImportError:
            self.get_logger().error(
                "The 'websockets' package is required for the phone relay node: "
                "pip install websockets"
            )
            return

        url = str(self.get_parameter("backend_url").value)
        while not self._stop.is_set():
            try:
                with connect(url, open_timeout=5, close_timeout=2) as ws:
                    with self._ws_lock:
                        self._ws = ws
                    self.get_logger().info(f"Connected to phone socket at {url}")
                    for raw in ws:
                        if self._stop.is_set():
                            break
                        self._handle_frame(raw)
            except Exception as exc:
                self.get_logger().warning(f"Phone socket link down ({exc}); retrying in 2 s")
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

        dataType = frame.get("type")
        data = frame.get("data")
        if not isinstance(data, dict):
            return

        stamp = self.get_clock().now().to_msg()
        phone_frame = str(self.get_parameter("phone_frame").value)

        if dataType == "phone_imu":
            self._publish_imu(data, stamp, phone_frame)
        elif dataType == "phone_gps":
            self._publish_gps(data, stamp, phone_frame)
        elif dataType == "phone_camera":
            self._publish_camera(data, stamp, phone_frame)

    def _publish_imu(self, data: dict[str, Any], stamp: Any, frame_id: str) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        # Acceleration (m/s^2)
        msg.linear_acceleration.x = float(data.get("acc_x", 0.0))
        msg.linear_acceleration.y = float(data.get("acc_y", 0.0))
        msg.linear_acceleration.z = float(data.get("acc_z", 0.0))

        # Angular Velocity (rad/s)
        msg.angular_velocity.x = float(data.get("gyro_x", 0.0))
        msg.angular_velocity.y = float(data.get("gyro_y", 0.0))
        msg.angular_velocity.z = float(data.get("gyro_z", 0.0))

        # Orientation quaternion from phone orientation (yaw/pitch/roll)
        roll = float(data.get("roll", 0.0))
        pitch = float(data.get("pitch", 0.0))
        yaw = float(data.get("yaw", 0.0))
        msg.orientation = euler_to_quaternion(roll, pitch, yaw)

        # Covariances
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        msg.orientation_covariance[0] = 0.05
        msg.orientation_covariance[4] = 0.05
        msg.orientation_covariance[8] = 0.05

        self.imu_pub.publish(msg)

    def _publish_gps(self, data: dict[str, Any], stamp: Any, frame_id: str) -> None:
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.latitude = float(data.get("lat", 0.0))
        msg.longitude = float(data.get("lon", 0.0))
        msg.altitude = float(data.get("alt", 0.0))

        msg.position_covariance[0] = 1.0
        msg.position_covariance[4] = 1.0
        msg.position_covariance[8] = 4.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(msg)

    def _publish_camera(self, data: dict[str, Any], stamp: Any, frame_id: str) -> None:
        img_b64 = data.get("image_base64")
        if not img_b64:
            return

        try:
            img_bytes = base64.b64decode(img_b64)
        except Exception:
            return

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.format = "jpeg"
        msg.data = img_bytes

        self.cam_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PhoneRelayNode()
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
