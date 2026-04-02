#!/bin/bash
set -e

echo "=== Step 1: Install Gazebo Fortress ==="
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gz

echo ""
echo "=== Step 2: Initialize rosdep (if needed) ==="
sudo rosdep init 2>/dev/null || true
# Skip rosdep update if raw.githubusercontent.com is unreachable (use cached data)
rosdep update 2>/dev/null || echo "WARNING: rosdep update failed (network issue), using cached data"

echo ""
echo "=== Step 3: Install all workspace dependencies ==="
cd /home/yizhongzhang/Documents/ur_sim
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y

echo ""
echo "=== Step 4: Build the workspace ==="
colcon build --symlink-install

echo ""
echo "=== Done! ==="
echo "To test, run:"
echo "  source /home/yizhongzhang/Documents/ur_sim/install/setup.bash"
echo "  ros2 launch ur_simulation_gz ur_sim_control.launch.py"
