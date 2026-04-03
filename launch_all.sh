#!/bin/bash
# Launch the full UR simulation stack:
# 1. Gazebo simulation (headless)
# 2. rosbridge WebSocket server
# 3. Web dashboard
#

# Usage: ./launch_all.sh [--control_mode position|effort] [config_file]
#   --control_mode: position (default) or effort
#   config_file: path to config YAML (default: config/config.yaml)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# Default values
CONTROL_MODE="position"
CONFIG_FILE=""
CONTROLLERS_FILE=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --control_mode)
            CONTROL_MODE="$2"
            shift 2
            ;;
        --controllers_file)
            CONTROLLERS_FILE="$2"
            shift 2
            ;;
        *)
            CONFIG_FILE="$1"
            shift
            ;;
    esac
done

CONFIG_FILE="${CONFIG_FILE:-$WS_DIR/config/config.yaml}"

# Generate config from template if it doesn't exist
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Config not found: $CONFIG_FILE"
    echo "Generating from template..."
    cp "$WS_DIR/config/config.template.yaml" "$CONFIG_FILE"
    echo "Created $CONFIG_FILE — edit it to customize settings."
fi

# Parse YAML config (simple key: value parser)
parse_yaml() {
    local key="$1"
    grep "^${key}:" "$CONFIG_FILE" | sed "s/^${key}:[[:space:]]*//" | sed 's/[[:space:]]*#.*//' | tr -d '"'
}

UR_TYPE="$(parse_yaml ur_type)"
GAZEBO_GUI="$(parse_yaml gazebo_gui)"
LAUNCH_RVIZ="$(parse_yaml launch_rviz)"
WORLD_FILE_CFG="$(parse_yaml world_file)"
ROSBRIDGE_PORT="$(parse_yaml rosbridge_port)"
DASHBOARD_PORT="$(parse_yaml dashboard_port)"
ROSBRIDGE_URL="$(parse_yaml rosbridge_url)"

# Defaults
UR_TYPE="${UR_TYPE:-ur5e}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
DASHBOARD_PORT="${DASHBOARD_PORT:-8000}"

# Resolve world file path
if [[ "$WORLD_FILE_CFG" == /* ]]; then
    WORLD_FILE="$WORLD_FILE_CFG"
else
    WORLD_FILE="$WS_DIR/src/ur_web_dashboard/worlds/${WORLD_FILE_CFG:-no_ground_collision.sdf}"
fi

echo "=== UR Robot Simulator ==="
echo "Config:       $CONFIG_FILE"
echo "Robot type:   $UR_TYPE"
echo "Control mode: $CONTROL_MODE"
echo "Gazebo GUI:   $GAZEBO_GUI"
echo "World:        $WORLD_FILE"
echo "Ports:        dashboard=$DASHBOARD_PORT, rosbridge=$ROSBRIDGE_PORT"
echo ""

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

# Kill any existing processes
echo "[1/4] Cleaning up old processes..."
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "rosbridge_websocket" 2>/dev/null || true
sleep 2

# Generate static URDFs for the 3D viewer (if missing)
URDF_DIR="$WS_DIR/src/ur_web_dashboard/urdf"
XACRO_FILE="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
if [ ! -f "$URDF_DIR/$UR_TYPE.urdf" ]; then
    echo "[2/4] Generating static URDFs for 3D viewer..."
    mkdir -p "$URDF_DIR"
    for ut in ur3 ur3e ur5 ur5e ur7e ur8long ur10 ur10e ur12e ur15 ur16e ur18 ur20 ur30; do
        xacro "$XACRO_FILE" ur_type:="$ut" name:=ur 2>/dev/null \
            | sed 's|package://ur_description/|../ur_description/|g' \
            > "$URDF_DIR/${ut}.urdf"
    done
    echo "    Generated URDFs in $URDF_DIR/"
else
    echo "[2/4] Static URDFs already exist, skipping generation."
fi


# Start Gazebo simulation with selected control mode
echo "[3/4] Starting Gazebo simulation..."
if [ "$CONTROL_MODE" = "effort" ]; then
    EFFORT_ARGS=(
        ur_type:="$UR_TYPE"
        gazebo_gui:="$GAZEBO_GUI"
        launch_rviz:="$LAUNCH_RVIZ"
        world_file:="$WORLD_FILE"
    )
    if [ -n "$CONTROLLERS_FILE" ]; then
        EFFORT_ARGS+=(controllers_file:="$CONTROLLERS_FILE")
    fi
    ros2 launch ur_sim_config ur_sim_effort.launch.py "${EFFORT_ARGS[@]}" &
else
    ros2 launch ur_simulation_gz ur_sim_control.launch.py \
        ur_type:="$UR_TYPE" \
        gazebo_gui:="$GAZEBO_GUI" \
        launch_rviz:="$LAUNCH_RVIZ" \
        world_file:="$WORLD_FILE" &
fi
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
echo "[4/4] Starting rosbridge WebSocket (port $ROSBRIDGE_PORT)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$ROSBRIDGE_PORT" &
BRIDGE_PID=$!
sleep 2

# Start web server
echo ""
echo "=== Starting web dashboard on port $DASHBOARD_PORT ==="
echo ""
echo "Open in browser: http://localhost:$DASHBOARD_PORT"
echo "rosbridge WebSocket: ws://localhost:$ROSBRIDGE_PORT"
echo ""
echo "Press Ctrl+C to stop all services."
echo ""

cd "$WS_DIR/src/ur_web_dashboard"
python3 server.py "$DASHBOARD_PORT" &
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
