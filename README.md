# UR Robot Simulator

A ROS 2 simulation environment for Universal Robots (UR3, UR5e, UR10e, UR16e, UR20, etc.) with a web-based monitoring dashboard. Designed for evaluating control systems in a virtual environment before deploying to real hardware.

## Features

- **Gazebo Ignition Fortress** physics simulation (headless supported)
- **MuJoCo** physics simulation (alternative backend, headless supported)
- **ros2_control** integration with `joint_trajectory_controller`
- **Effort (torque) mode** with Pinocchio gravity compensation for external controllers (e.g. CRISP)
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
- Pinocchio (`ros-humble-pinocchio`) — required for effort mode gravity compensation

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
  ros-humble-pinocchio \
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
- Web dashboard (port **8000**)

Open `http://localhost:8000` in a browser.

## Configuration

Edit `config/config.yaml` (auto-generated from `config/config.template.yaml` on first launch):

```yaml
ur_type: ur5e                  # ur3, ur3e, ur5, ur5e, ur7e, ur8long, ur10, ur10e, ur12e, ur15, ur16e, ur18, ur20, ur30
gazebo_gui: false              # Show Gazebo GUI (requires X display)
launch_rviz: false             # Launch RViz (requires X display)
world_file: no_ground_collision.sdf  # Gazebo world file
rosbridge_port: 9090           # rosbridge WebSocket port
dashboard_port: 8000           # Web dashboard HTTP port
```


## Launch Modes: Position vs Effort

You can select the robot control mode and physics engine at launch time:

```bash
# Position mode with Gazebo (default)
./launch_all.sh

# Effort mode with Gazebo
./launch_all.sh --control_mode effort

# Position mode with MuJoCo
./launch_all.sh --simulator mujoco

# Effort mode with MuJoCo
./launch_all.sh --simulator mujoco --control_mode effort

# You can also specify a config file:
./launch_all.sh --simulator mujoco --control_mode effort path/to/config.yaml

# You can also provide a custom controllers YAML (for external controllers like CRISP):
./launch_all.sh --control_mode effort --controllers_file /path/to/custom_controllers.yaml
```

If no flags are given, Gazebo with position mode is used by default.

---

Usage: `./launch_all.sh [--simulator gazebo|mujoco] [--control_mode position|effort] [--controllers_file path/to/controllers.yaml] [path/to/config.yaml]`


## Control Modes Explained

### Position Mode (default)

- Launches the simulation with robust position control using the `joint_trajectory_controller`.
- Effort (torque) commands are **not** available.
- Use this mode for most applications, including web dashboard jogging and trajectory following.

### Effort Mode (for CRISP/external controllers)

- Launches the simulation with **effort (torque) command interfaces** enabled on all joints.
- Required for external torque-based controllers (e.g. CRISP).
- **Built-in gravity compensation** — just like a real UR robot, gravity is compensated
  at the simulation level using **Pinocchio** rigid body dynamics.
  When zero external torque is commanded, the robot holds its position.
- A `gravity_compensation` node runs automatically, providing:
  - Pinocchio-based gravity torque computation (accurate for all UR models)
  - PID position hold with velocity filtering (mimics the real UR internal servo)
  - Trajectory execution via `/joint_trajectory_controller/joint_trajectory` topic
    (same topic as position mode — web dashboard jogging works in both modes)
  - Effort clamping to safe joint limits (150 Nm base, 28 Nm wrist)
  - Joint friction and damping in physics (simulates reduction gear friction)
- External controllers publish torques on `/external_effort_commands` (6-element `Float64MultiArray`).
  These are **added** on top of gravity compensation, matching real UR behavior.
- The robot starts at the same home position as position mode
  (`shoulder_lift=-90°`, `wrist_1=-90°`, all others `0°`).

```bash
# Example: send external torques from another node
ros2 topic pub /external_effort_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---


## Manual launch (advanced)

```bash
# Position mode:
source install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=false launch_rviz:=false

# Effort mode:
source install/setup.bash
ros2 launch ur_sim_config ur_sim_effort.launch.py ur_type:=ur5e gazebo_gui:=false launch_rviz:=false

# rosbridge (separate terminal)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Web dashboard (separate terminal)
cd src/ur_web_dashboard && python3 server.py 8000
```

## Web Dashboard

| Feature | Description |
|---|---|
| 3D viewer | Interactive Three.js robot model with orbit controls, TCP axes |
| Joint states | Real-time position (°), velocity (rad/s), effort (Nm) |
| TCP pose | End-effector X/Y/Z + Roll/Pitch/Yaw (computed from 3D FK) |
| Robot type | Auto-detected from URDF (shown in header) |
| Controllers | Active/inactive status |
| Sim clock | Simulation time display |
| Joint jog | Per-joint sliders with trajectory send |
| Default pose | Initializes to URDF default joint positions |

## Architecture

```
Browser <--:8000--> server.py (static files + mesh routing)
Browser <--:9090--> rosbridge_websocket <--ROS 2--> Gazebo Ignition
```

## Repository Structure

```
├── config/
│   ├── config.template.yaml       # Configuration template (tracked)
│   └── config.yaml                # User config (gitignored, auto-generated)
├── src/
│   ├── ur_sim_config/             # Effort mode config, gravity compensation
│   │   ├── scripts/gravity_compensation.py  # Pinocchio gravity comp + PID node
│   │   ├── config/ur_effort_controllers.yaml
│   │   ├── launch/ur_sim_effort.launch.py
│   │   └── urdf/generate_effort_urdf.sh
│   ├── ur_simulation_gz/          # Git submodule: UR Gazebo simulation
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
- Web dashboard and scripts: MIT
