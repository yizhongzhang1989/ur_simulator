# UR Robot Simulator

A ROS 2 simulation environment for Universal Robots (UR3, UR5e, UR10e, UR16e, UR20, etc.) with a web-based monitoring dashboard. Designed for evaluating control systems in a virtual environment before deploying to real hardware.

## Features

- **Gazebo Ignition Fortress** physics simulation (headless supported)
- **ros2_control** integration with `joint_trajectory_controller`
- **3D visualization** of the robot in the browser (Three.js + urdf-loader)
- **Web dashboard** for real-time monitoring and joint jogging
- **TCP pose** computed via forward kinematics from the 3D model
- **Auto-detect robot type** from the running simulation
- **Self-contained** — all JS libraries vendored locally (no CDN required)
- **Configurable** via YAML config file
- **One-command launch** of the full stack
- **No ground collision** — robot can reach below z=0 (table-mounted scenario)

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
./launch_all.sh
```

On first run, `config/config.yaml` is auto-generated from the template and static
URDFs are generated for the 3D viewer. Edit config to customize settings.

This starts:
- Gazebo simulation (headless by default) on ROS 2
- rosbridge WebSocket server (port **9090**)
- Web dashboard (port **8080**)

Open `http://localhost:8080` in a browser.

## Configuration

Edit `config/config.yaml` (auto-generated from `config/config.template.yaml` on first launch):

```yaml
ur_type: ur5e                  # ur3, ur3e, ur5, ur5e, ur7e, ur8long, ur10, ur10e, ur12e, ur15, ur16e, ur18, ur20, ur30
gazebo_gui: false              # Show Gazebo GUI (requires X display)
launch_rviz: false             # Launch RViz (requires X display)
world_file: no_ground_collision.sdf  # Gazebo world file
rosbridge_port: 9090           # rosbridge WebSocket port
dashboard_port: 8080           # Web dashboard HTTP port
```

Usage: `./launch_all.sh` (default config) or `./launch_all.sh path/to/config.yaml`

### Manual launch (step by step)

```bash
# Terminal 1: Simulation
source install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=false launch_rviz:=false

# Terminal 2: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 3: Web dashboard (use server.py to serve meshes correctly)
cd src/ur_web_dashboard && python3 server.py 8080
```

## Web Dashboard

| Feature | Description |
|---|---|
| 3D viewer | Interactive Three.js robot model with orbit controls |
| Joint states | Real-time position (°), velocity, effort |
| TCP pose | End-effector X/Y/Z + Roll/Pitch/Yaw (computed from 3D FK) |
| Robot type | Auto-detected from URDF (shown in header) |
| Controllers | Active/inactive status |
| Sim clock | Simulation time display |
| Joint jog | Per-joint sliders with trajectory send |
| Default pose | Initializes to URDF default joint positions |

## Architecture

```
Browser <--:8080--> server.py (static files + mesh routing)
Browser <--:9090--> rosbridge_websocket <--ROS 2--> Gazebo Ignition
```

## Repository Structure

```
├── config/
│   ├── config.template.yaml       # Configuration template (tracked)
│   └── config.yaml                # User config (gitignored, auto-generated)
├── src/
│   ├── ur_simulation_gz/          # Git submodule: UR Gazebo simulation
│   ├── ur_description/            # Git submodule: UR URDF + meshes
│   └── ur_web_dashboard/          # Web dashboard
│       ├── index.html             # Single-page dashboard app
│       ├── server.py              # HTTP server (dashboard + mesh routing)
│       ├── lib/                   # Vendored JS libraries (Three.js, urdf-loader)
│       ├── urdf/                  # Generated static URDFs (gitignored, auto-generated)
│       └── worlds/                # Custom Gazebo world files
│           └── no_ground_collision.sdf
├── launch_all.sh                  # One-command launcher (reads config.yaml)
├── install_deps.sh                # Dependency installer
├── ai_setup_guideline.md          # AI setup guide
└── README.md                      # This file
```

## AI-Assisted Setup

This repo includes `ai_setup_guideline.md` — a structured guide for AI agents
(GitHub Copilot, ChatGPT, etc.) to set up the full environment from a fresh clone.
Point your AI to that file and it will handle dependency installation, building,
and verification.

## License

- `ur_simulation_gz`: BSD-3-Clause (Universal Robots)
- `ur_description`: BSD-3-Clause (Universal Robots)
- Web dashboard and scripts: MIT
