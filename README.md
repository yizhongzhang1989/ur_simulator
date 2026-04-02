# UR Robot Simulator

A ROS 2 simulation environment for Universal Robots (UR3, UR5e, UR10e, UR16e, UR20, etc.) with a web-based monitoring dashboard. Designed for evaluating control systems in a virtual environment before deploying to real hardware.

## Features

- **Gazebo Ignition Fortress** physics simulation (headless supported)
- **ros2_control** integration with `joint_trajectory_controller`
- **Web dashboard** for real-time monitoring and joint jogging
- **One-command launch** of the full stack

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress (`ros-humble-ros-gz`)

## Quick Start

### 1. Clone (with submodules)

```bash
git clone --recurse-submodules https://github.com/yizhongzhang1989/ur_simulator.git
cd ur_simulator
```

### 2. Install dependencies

```bash
sudo apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ur-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-ur-moveit-config \
  ros-humble-rosbridge-suite

source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
```

### 3. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 4. Launch everything

```bash
./launch_all.sh          # default: UR5e
./launch_all.sh ur10e    # or specify robot type
```

This starts:
- Gazebo simulation (headless) on ROS 2
- rosbridge WebSocket server on port **9090**
- Web dashboard on port **8080**

Open `http://localhost:8080` in a browser.

### Manual launch (step by step)

```bash
# Terminal 1: Simulation
source install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=false launch_rviz:=false

# Terminal 2: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 3: Web dashboard
cd src/ur_web_dashboard && python3 -m http.server 8080
```

## Web Dashboard

| Feature | Description |
|---|---|
| Joint states | Real-time position (°), velocity, effort |
| TCP pose | End-effector X/Y/Z + Roll/Pitch/Yaw |
| Robot type | Auto-detected from URDF |
| Controllers | Active/inactive status |
| Sim clock | Simulation time display |
| Joint jog | Per-joint sliders with trajectory send |

## Architecture

```
Browser <--:8080--> python3 http.server (static files)
Browser <--:9090--> rosbridge_websocket <--ROS 2--> Gazebo Ignition
```

## Repository Structure

```
├── src/
│   ├── ur_simulation_gz/          # Git submodule (Universal Robots)
│   └── ur_web_dashboard/          # Web dashboard (index.html)
├── launch_all.sh                  # One-command launcher
├── install_deps.sh                # Dependency installer
└── ai_setup_guideline.md          # AI setup guide
```

## License

- `ur_simulation_gz`: BSD-3-Clause (Universal Robots)
- Web dashboard and scripts: MIT
