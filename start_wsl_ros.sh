#!/bin/bash
# SLAM Bot — WSL ROS2 Launcher
# Usage: bash start_wsl_ros.sh

echo "=================================================="
echo "   SLAM BOT — Starting ROS2 & Nav2 (WSL)         "
echo "=================================================="

# Source ROS2 installation
if [ -f /opt/ros/humble/setup.bash ]; then
    echo ">> Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
    echo ">> Sourcing ROS2 Foxy..."
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    echo ">> Sourcing ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    echo ">> Sourcing ROS2 Iron..."
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/rolling/setup.bash ]; then
    echo ">> Sourcing ROS2 Rolling..."
    source /opt/ros/rolling/setup.bash
else
    echo "ERROR: ROS2 setup.bash not found in /opt/ros!"
    exit 1
fi

# Build workspace if install directory doesn't exist
if [ ! -d "ros2_ws/install" ]; then
    echo ">> Building ros2_ws with colcon..."
    (cd ros2_ws && colcon build)
fi

# Source workspace
if [ -f "ros2_ws/install/setup.bash" ]; then
    echo ">> Sourcing ros2_ws overlay..."
    source ros2_ws/install/setup.bash
fi

# Auto-detect Windows host IP from WSL2
WIN_IP=$(ip route show default | awk '{print $3}')
if [ -z "$WIN_IP" ]; then
    WIN_IP="127.0.0.1"
fi

echo ">> Connecting ROS2 Bridge to Windows Host IP: $WIN_IP..."

BACKEND_URL="ws://${WIN_IP}:8000/ws/app"
RELAY_URL="ws://${WIN_IP}:8000/ws/relay"

echo ">> Launching SLAM Toolbox + Nav2 + Standalone Bridge..."
ros2 launch slam_bot_bringup bringup.launch.py standalone_bridge:=true backend_url:=$BACKEND_URL relay_url:=$RELAY_URL
