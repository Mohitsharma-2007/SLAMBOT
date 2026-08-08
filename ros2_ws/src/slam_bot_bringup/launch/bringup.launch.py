"""Full ROS2 bringup for the SLAM Bot — step 4 of §12.

    ros2 launch slam_bot_bringup bringup.launch.py

Starts slam_toolbox (async mapping) and the Nav2 stack, plus the static
base_link -> laser transform. Assumes the FastAPI backend is already running and
publishing /scan and /odom via its embedded bridge (§4).

Arguments
---------
    use_nav2:=false            mapping only, no autonomous navigation
    use_slam:=false            Nav2 only (e.g. with AMCL against a saved map)
    standalone_bridge:=true    also start slam_bot_bridge's own node + web relay
                               instead of relying on the backend's embedded one
    laser_offset_x/y/z/yaw     lidar mount offset (§7 — default 0,0,0)
    autostart:=false           bring Nav2 up but leave it unconfigured

The static TF default of 0,0,0 follows §7: measure your actual lidar mount
offset and pass it, rather than assuming the lidar sits exactly at base_link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_nav2 = LaunchConfiguration("use_nav2")
    use_slam = LaunchConfiguration("use_slam")
    standalone_bridge = LaunchConfiguration("standalone_bridge")
    autostart = LaunchConfiguration("autostart")
    backend_url = LaunchConfiguration("backend_url")
    laser_offset_x = LaunchConfiguration("laser_offset_x")
    laser_offset_y = LaunchConfiguration("laser_offset_y")
    laser_offset_z = LaunchConfiguration("laser_offset_z")
    laser_offset_yaw = LaunchConfiguration("laser_offset_yaw")

    slam_params = PathJoinSubstitution(
        [FindPackageShare("slam_bot_bringup"), "config", "slam_toolbox_params.yaml"]
    )
    nav2_params = PathJoinSubstitution(
        [FindPackageShare("slam_bot_nav"), "config", "nav2_params.yaml"]
    )

    declarations = [
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_slam", default_value="true"),
        DeclareLaunchArgument(
            "standalone_bridge",
            default_value="false",
            description=(
                "Start the bridge as its own node. Leave false when the FastAPI "
                "backend runs the embedded bridge — running both publishes /scan "
                "twice."
            ),
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "backend_url", default_value="ws://127.0.0.1:8000/ws/app"
        ),
        DeclareLaunchArgument(
            "relay_url", default_value="ws://127.0.0.1:8000/ws/relay"
        ),
        DeclareLaunchArgument("laser_offset_x", default_value="0.0"),
        DeclareLaunchArgument("laser_offset_y", default_value="0.0"),
        DeclareLaunchArgument("laser_offset_z", default_value="0.0"),
        DeclareLaunchArgument("laser_offset_yaw", default_value="0.0"),
    ]

    relay_url = LaunchConfiguration("relay_url")

    # --- static TF base_link -> laser (§7) ---------------------------------
    # Published unconditionally: slam_toolbox cannot transform scans without it,
    # and the embedded backend bridge only publishes it when it is itself up.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        arguments=[
            "--x", laser_offset_x,
            "--y", laser_offset_y,
            "--z", laser_offset_z,
            "--yaw", laser_offset_yaw,
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
        output="screen",
    )

    # --- optional standalone bridge ---------------------------------------
    bridge_group = GroupAction(
        condition=IfCondition(standalone_bridge),
        actions=[
            LogInfo(msg="Starting standalone bridge + web relay"),
            Node(
                package="slam_bot_bridge",
                executable="bridge_node",
                name="slam_bot_bridge",
                output="screen",
                parameters=[
                    {
                        "backend_url": backend_url,
                        "laser_offset_x": laser_offset_x,
                        "laser_offset_y": laser_offset_y,
                        "laser_offset_z": laser_offset_z,
                        "laser_offset_yaw": laser_offset_yaw,
                        # The static TF above already covers base_link->laser.
                        "publish_odom_tf": True,
                    }
                ],
            ),
            Node(
                package="slam_bot_bridge",
                executable="web_relay",
                name="slam_bot_web_relay",
                output="screen",
                parameters=[{"backend_url": relay_url}],
            ),
        ],
    )

    # --- slam_toolbox (§8) -------------------------------------------------
    slam = Node(
        condition=IfCondition(use_slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": False}],
    )

    # --- Nav2 -------------------------------------------------------------
    nav2_nodes = [
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_smoother", "smoother_server", "smoother_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
    ]

    # The velocity chain must be wired end to end or it feeds back on itself:
    #
    #   controller_server --/cmd_vel_nav--> velocity_smoother
    #     --/cmd_vel_smoothed--> collision_monitor --/cmd_vel--> bridge -> MCU
    #
    # Only the last hop is named /cmd_vel, which is what the bridge subscribes
    # to (§7). Without these remaps controller_server and collision_monitor both
    # publish /cmd_vel and the smoother's input is also its own output.
    velocity_chain_remaps = {
        "controller_server": [("/cmd_vel", "/cmd_vel_nav")],
        "velocity_smoother": [
            ("/cmd_vel", "/cmd_vel_nav"),
            ("/cmd_vel_smoothed", "/cmd_vel_smoothed"),
        ],
    }

    nav2_group = GroupAction(
        condition=IfCondition(use_nav2),
        actions=[
            LogInfo(msg="Starting Nav2 (SmacPlanner2D + DWB)"),
            *[
                Node(
                    package=pkg,
                    executable=exe,
                    name=name,
                    output="screen",
                    parameters=[nav2_params],
                    remappings=velocity_chain_remaps.get(name, []),
                )
                for pkg, exe, name in nav2_nodes
            ],
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": autostart,
                        "node_names": [
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                            "collision_monitor",
                        ],
                    }
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            *declarations,
            LogInfo(
                msg=(
                    "SLAM Bot bringup. Make sure the FastAPI backend is already "
                    "running (uvicorn main:app) so /scan and /odom are being "
                    "published."
                )
            ),
            static_tf,
            bridge_group,
            slam,
            nav2_group,
        ]
    )
