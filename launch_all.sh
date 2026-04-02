#!/bin/bash
# Launch the full UR simulation stack:
# 1. Gazebo simulation (headless)
# 2. rosbridge WebSocket server (port 9090)
# 3. Web dashboard (port 8080)
#
# Usage: ./launch_all.sh [ur_type]
#   ur_type: ur3, ur5, ur5e (default), ur10e, ur16e, ur20, ur30

set -e

UR_TYPE="${1:-ur5e}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo "=== UR Robot Simulator ==="
echo "Robot type: $UR_TYPE"
echo "Workspace: $WS_DIR"
echo ""

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

# Kill any existing processes
echo "[1/3] Cleaning up old processes..."
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "rosbridge_websocket" 2>/dev/null || true
sleep 2

# Start Gazebo simulation (headless)
echo "[2/3] Starting Gazebo simulation (headless)..."
ros2 launch ur_simulation_gz ur_sim_control.launch.py \
    ur_type:="$UR_TYPE" \
    gazebo_gui:=false \
    launch_rviz:=false &
SIM_PID=$!

# Wait for simulation to be ready
echo "    Waiting for /joint_states topic..."
for i in $(seq 1 30); do
    if ros2 topic info /joint_states 2>/dev/null | grep -q "Publisher count: 1"; then
        echo "    Simulation ready."
        break
    fi
    sleep 1
done

# Start rosbridge WebSocket server
echo "[3/3] Starting rosbridge WebSocket (port 9090)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
BRIDGE_PID=$!
sleep 2

# Start web server
echo ""
echo "=== Starting web dashboard on port 8080 ==="
echo ""
echo "Open in browser: http://localhost:8080"
echo "rosbridge WebSocket: ws://localhost:9090"
echo ""
echo "Press Ctrl+C to stop all services."
echo ""

cd "$WS_DIR/src/ur_web_dashboard"
python3 server.py 8080 &
WEB_PID=$!

# Cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $WEB_PID 2>/dev/null
    kill $BRIDGE_PID 2>/dev/null
    kill $SIM_PID 2>/dev/null
    pkill -f "ign gazebo" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "rosbridge_websocket" 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

wait
