import math
import numpy as np
from typing import Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))

class AutonomousExploreNode(Node):
    def __init__(self) -> None:
        super().__init__("slam_bot_explore")

        # Parameters
        self.declare_parameter("min_frontier_size", 5) # Minimum number of contiguous cells to count as a frontier
        self.declare_parameter("safety_distance_m", 0.3) # Avoid frontiers closer than this to obstacles
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self._on_map_received, 10
        )

        # Nav2 Goal Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # TF listener for current position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.latest_map: OccupancyGrid | None = None
        self.current_goal: NavigateToPose.Goal | None = None
        self.goal_handle: Any = None
        self.exploration_active = True
        
        # Timer to drive the exploration loop state machine
        self.loop_timer = self.create_timer(2.0, self._exploration_loop)

        self.get_logger().info("Autonomous Frontier Exploration Node initialized.")

    def _on_map_received(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _get_robot_pose(self) -> Tuple[float, float, float] | None:
        """Retrieve current robot x, y, yaw from TF tree."""
        try:
            target_frame = self.get_parameter("map_frame").value
            source_frame = self.get_parameter("robot_base_frame").value
            
            # Look up transformation
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Convert quaternion to yaw angle
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except TransformException as e:
            self.get_logger().debug(f"Could not look up robot pose: {e}")
            return None

    def _exploration_loop(self) -> None:
        if not self.exploration_active or self.latest_map is None:
            return

        # If we currently have an active navigation task, do not send a new one
        if self.goal_handle is not None:
            return

        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().warn("Waiting for transform map -> base_link to locate robot...")
            return

        rx, ry, _ = pose
        self.get_logger().info("Searching for map frontiers...")
        
        frontiers = self._detect_frontiers()
        if not frontiers:
            self.get_logger().info("Mapping Complete! No unmapped frontiers found in the arena.")
            self.exploration_active = False
            return

        # Filter out frontiers that are too close to known obstacles
        valid_frontiers = self._filter_safety_margins(frontiers)
        if not valid_frontiers:
            self.get_logger().info("Frontiers found but they are in unsafe zones. Recalculating...")
            return

        # Find the closest frontier centroid
        target = self._find_best_frontier(valid_frontiers, rx, ry)
        if target is None:
            return

        tx, ty = target
        self.get_logger().info(f"Dispatched new autonomous exploration target: ({tx:.2f}, {ty:.2f}) m")
        self._send_goal(tx, ty)

    def _detect_frontiers(self) -> List[Tuple[float, float]]:
        """Detect all frontier centroids from the occupancy grid map."""
        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = np.array(grid.data).reshape((height, width))

        frontiers = []
        visited = np.zeros_like(data, dtype=bool)

        # Iterate through the grid map
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # A frontier cell is free space (0) having at least one unknown neighbor (-1)
                if data[y, x] == 0:
                    if (data[y+1, x] == -1 or data[y-1, x] == -1 or 
                        data[y, x+1] == -1 or data[y, x-1] == -1):
                        
                        # Find centroid of this contiguous frontier group via flood fill BFS
                        if not visited[y, x]:
                            centroid = self._flood_fill_frontier(x, y, data, visited)
                            if centroid:
                                # Convert grid indices to map world coordinates
                                tx = centroid[0] * resolution + origin_x
                                ty = centroid[1] * resolution + origin_y
                                frontiers.append((tx, ty))

        return frontiers

    def _flood_fill_frontier(self, sx: int, sy: int, data: np.ndarray, visited: np.ndarray) -> Tuple[float, float] | None:
        """Group connected frontier cells together and returns the group's centroid."""
        queue = [(sx, sy)]
        visited[sy, sx] = True
        
        cells = []
        min_size = self.get_parameter("min_frontier_size").value

        while queue:
            cx, cy = queue.pop(0)
            cells.append((cx, cy))

            # Check 4-way neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < data.shape[1] and 0 <= ny < data.shape[0]:
                    if not visited[ny, nx] and data[ny, nx] == 0:
                        # Neighbor has to touch unknown too
                        if (data[ny+1, nx] == -1 or data[ny-1, nx] == -1 or 
                            data[ny, nx+1] == -1 or data[ny, nx-1] == -1):
                            visited[ny, nx] = True
                            queue.append((nx, ny))

        if len(cells) >= min_size:
            avg_x = sum(c[0] for c in cells) / len(cells)
            avg_y = sum(c[1] for c in cells) / len(cells)
            return avg_x, avg_y
        
        return None

    def _filter_safety_margins(self, frontiers: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Filter out frontiers that are too close to walls/obstacles."""
        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = np.array(grid.data).reshape((height, width))

        safety_m = self.get_parameter("safety_distance_m").value
        safety_cells = int(safety_m / resolution)

        valid = []
        for fx, fy in frontiers:
            # Convert world coord back to grid indices
            gx = int((fx - origin_x) / resolution)
            gy = int((fy - origin_y) / resolution)

            # Check bounding box safety zone around the frontier centroid
            is_safe = True
            for dy in range(-safety_cells, safety_cells + 1):
                for dx in range(-safety_cells, safety_cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if data[ny, nx] > 50: # Cell is occupied (obstacle)
                            is_safe = False
                            break
                if not is_safe:
                    break

            if is_safe:
                valid.append((fx, fy))

        return valid

    def _find_best_frontier(self, frontiers: List[Tuple[float, float]], rx: float, ry: float) -> Tuple[float, float] | None:
        """Find the frontier centroid closest to the robot's current position."""
        best_dist = float("inf")
        best_frontier = None

        for fx, fy in frontiers:
            dist = math.hypot(fx - rx, fy - ry)
            if dist < best_dist:
                best_dist = dist
                best_frontier = (fx, fy)

        return best_frontier

    def _send_goal(self, x: float, y: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 server unavailable. Unable to dispatch goal.")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.get_parameter("map_frame").value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = yaw_to_quaternion(0.0)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn("Exploration goal rejected by Nav2.")
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self._on_goal_result)
        except Exception as e:
            self.get_logger().error(f"Error sending goal: {e}")

    def _on_goal_result(self, future: Any) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Frontier target reached. Recalculating next location...")
        else:
            self.get_logger().warn(f"Frontier target failed or aborted (status code: {status}). Finding alternative...")
        
        self.goal_handle = None

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutonomousExploreNode()
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
