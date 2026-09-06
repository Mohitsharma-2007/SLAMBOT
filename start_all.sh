#!/bin/bash
# SLAM Bot — Single Command Start Script for WSL
# Usage: bash start_all.sh

echo "=================================================="
echo "   SLAM BOT — Starting Backend + ROS2 + WebApp    "
echo "=================================================="

# Source ROS2 if available
if [ -f /opt/ros/humble/setup.bash ]; then
    echo ">> Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
    export SLAM_ENABLE_ROS=1
elif [ -f /opt/ros/foxy/setup.bash ]; then
    echo ">> Sourcing ROS2 Foxy..."
    source /opt/ros/foxy/setup.bash
    export SLAM_ENABLE_ROS=1
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    echo ">> Sourcing ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
    export SLAM_ENABLE_ROS=1
elif [ -f /opt/ros/iron/setup.bash ]; then
    echo ">> Sourcing ROS2 Iron..."
    source /opt/ros/iron/setup.bash
    export SLAM_ENABLE_ROS=1
elif [ -f /opt/ros/rolling/setup.bash ]; then
    echo ">> Sourcing ROS2 Rolling..."
    source /opt/ros/rolling/setup.bash
    export SLAM_ENABLE_ROS=1
else
    echo ">> ROS2 setup.bash not found in /opt/ros — running in no-ROS mode"
fi

# Ensure Python requirements are installed in WSL
if ! python3 -c "import fastapi" 2>/dev/null; then
    echo ">> Installing backend requirements in WSL..."
    python3 -m pip install -r backend/requirements.txt
fi

# Clear any previous process using port 8000 or 5173
fuser -k 8000/tcp 2>/dev/null || true
fuser -k 5173/tcp 2>/dev/null || true

# Start FastAPI backend in background
echo ">> Starting Python FastAPI Backend on port 8000..."
python3 backend/main.py &
BACKEND_PID=$!

# Trapping Ctrl+C to kill background processes cleanly
trap "echo '>> Stopping SLAM Bot...'; kill $BACKEND_PID 2>/dev/null; fuser -k 8000/tcp 2>/dev/null; exit 0" INT TERM

sleep 2

# Start Vite Frontend
echo ">> Starting WebApp Vite Dev Server on port 5173..."
cd webapp && npm run dev
