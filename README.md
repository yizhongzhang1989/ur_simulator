# UR Robot Simulator

[![CI](https://github.com/OWNER/REPO/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/ci.yml)

A ROS 2 simulation environment for Universal Robots (UR3, UR5e, UR10e, UR16e, UR20, etc.) with a web-based monitoring dashboard. Designed for evaluating control systems in a virtual environment before deploying to real hardware.

<!-- AI Quick Reference
Workspace root: the directory containing this file
ROS distro:     Humble (Ubuntu 22.04)
Build system:   colcon
Submodules:     ur_simulation_gz (Gazebo sim)
System package: ur_description (URDF + meshes, from ros-humble-ur-description)
Simulators:     MuJoCo (default), Gazebo Ignition Fortress
One-command:    ./launch_all.sh
Config:         config/config.yaml (auto-generated from config/config.template.yaml)
-->

## Features

- **MuJoCo** physics simulation (default backend, headless supported)
- **Gazebo Ignition Fortress** physics simulation (alternative backend, headless supported)
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
- Gazebo Fortress (`ros-humble-ros-gz`) — for Gazebo backend
- MuJoCo (`ros-humble-mujoco-ros2-control`) — for MuJoCo backend (default)
- Pinocchio (`ros-humble-pinocchio`) — required for effort mode gravity compensation
- Build tools: `colcon`, `git`, `xacro`

## Quick Start

### 1. Clone (with submodules)

```bash
git clone --recurse-submodules https://github.com/yizhongzhang1989/ur_simulator.git
cd ur_simulator
```

If the repo was cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

Verify: `src/ur_simulation_gz/` should be non-empty.

> **Note:** `ur_description` is NOT a submodule — it uses the system package
> `ros-humble-ur-description` (installed in Step 2).

### 2. Install dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ur-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-pinocchio \
  ros-humble-rosbridge-suite \
  ros-humble-mujoco-ros2-control

source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
```

> **Tip:** If `rosdep update` fails due to DNS issues (e.g. `raw.githubusercontent.com`
> resolves to `0.0.0.0`), skip it — the cached rosdep database is sufficient.

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
1. Generate `config/config.yaml` from template (first run only)
2. Generate static URDFs for the 3D web viewer (first run only)
3. MuJoCo simulation (headless by default)
4. rosbridge WebSocket server (port **9090**)
5. Web dashboard (port **8000**)

Open `http://localhost:8000` in a browser.

**Headless environment (no X display):** The default config already sets
`gazebo_gui: false` and `launch_rviz: false`. No changes needed.

### 5. Verify

In a separate terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states --once   # Should show 6 joint positions
ros2 control list_controllers          # Should list active controllers
```

Expected controllers by mode (all are *loaded*; columns show which are *active*
on startup — any other loaded controller is available for runtime switching
via `ros2 control switch_controllers`):

| Mode                     | Active                                                                    | Inactive (loaded, switchable)                                                                                              |
|--------------------------|---------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| MuJoCo (any)             | `joint_state_broadcaster`, `forward_effort_controller`                    | `joint_trajectory_controller`, `scaled_joint_trajectory_controller`, `forward_position_controller`, `forward_velocity_controller` |
| Gazebo position          | `joint_state_broadcaster`, `joint_trajectory_controller`                  | `scaled_joint_trajectory_controller`, `forward_position_controller`, `forward_velocity_controller`, `forward_effort_controller`    |
| Gazebo effort            | `joint_state_broadcaster`, `forward_effort_controller`                    | `joint_trajectory_controller`, `scaled_joint_trajectory_controller`, `forward_position_controller`, `forward_velocity_controller` |

> **Note:** MuJoCo always runs in effort mode internally (motor actuators + gravity
> compensation via Pinocchio). The `--control_mode` flag only affects which
> controllers are active in Gazebo.
>
> `scaled_joint_trajectory_controller` is loaded for parity with the real UR
> driver (MoveIt2 defaults to it on hardware). It has an empty
> `speed_scaling_interface_name` and falls back to plain JTC behavior until
> Round 3 wires a speed-scaling state interface.

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

## Launch Modes: Simulator & Control Mode

You can select the physics engine and control mode at launch time:

```bash
# Position mode with MuJoCo (default)
./launch_all.sh

# Effort mode with MuJoCo
./launch_all.sh --control_mode effort

# Position mode with Gazebo
./launch_all.sh --simulator gazebo

# Effort mode with Gazebo
./launch_all.sh --simulator gazebo --control_mode effort

# You can also specify a config file:
./launch_all.sh --control_mode effort path/to/config.yaml

# You can also provide a custom controllers YAML (for external controllers like CRISP):
./launch_all.sh --control_mode effort --controllers_file /path/to/custom_controllers.yaml
```

Usage: `./launch_all.sh [--simulator gazebo|mujoco] [--control_mode position|effort] [--controllers_file path/to/controllers.yaml] [path/to/config.yaml]`

If no flags are given, MuJoCo with position mode is used by default.

## Control Modes Explained

### Position Mode (default)

- Launches the simulation with robust position control using the `joint_trajectory_controller`.
- On **MuJoCo**, backed by MuJoCo built-in `<position>` actuators — gravity is
  handled implicitly by the physics engine, no gravity-compensation node is
  started.
- On **Gazebo**, backed by the Ignition position command interface.
- Effort (torque) commands are **not** active in this mode (but the effort
  controller is loaded; you can switch to it via `ros2 control switch_controllers`).
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

## Architecture

### MuJoCo Mode (default)
```
Browser (HTTP)       <--:8000--> server.py (static files + mesh routing)
Browser (WebSocket)  <--:9090--> rosbridge_websocket <--ROS 2--> mujoco_ros2_control
                                                               + gravity_compensation.py
```

### Gazebo Mode
```
Browser (HTTP)       <--:8000--> server.py (static files + mesh routing)
Browser (WebSocket)  <--:9090--> rosbridge_websocket <--ROS 2--> Gazebo Ignition
```

Both modes expose the same ROS 2 interface — the web dashboard works identically.

### ROS 2 Interfaces

| Interface | Type | Description |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | Joint positions, velocities, efforts |
| `/joint_trajectory_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | Send trajectory commands (works in all modes) |
| `/forward_effort_controller/commands` | std_msgs/Float64MultiArray | Direct effort commands (effort mode) |
| `/external_effort_commands` | std_msgs/Float64MultiArray | External torques added to gravity comp (effort mode) |
| `/joint_trajectory_controller/follow_joint_trajectory` | action | Trajectory with feedback |
| `/controller_manager/list_controllers` | service | List active controllers |
| `/clock` | rosgraph_msgs/Clock | Simulation time |

### Joint Names (in order)

`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
`wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`

### Default Joint Positions (from URDF)

```
shoulder_pan_joint:  0.0
shoulder_lift_joint: -1.57  (-90°)
elbow_joint:         0.0
wrist_1_joint:       -1.57  (-90°)
wrist_2_joint:       0.0
wrist_3_joint:       0.0
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

File: `src/ur_web_dashboard/index.html` (single-page, no build step)

`server.py` routes requests:
- `/` → `index.html`
- `/lib/` → vendored JS libraries
- `/urdf/` → generated static URDFs
- `/ur_description/` → UR mesh files (from system `ros-humble-ur-description` package)

Use `server.py` instead of `python3 -m http.server` — the latter can't serve meshes.

## Manual Launch (advanced)

```bash
# MuJoCo mode:
source install/setup.bash
ros2 launch ur_sim_config ur_sim_mujoco.launch.py ur_type:=ur5e

# Gazebo position mode:
source install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=false launch_rviz:=false

# Gazebo effort mode:
source install/setup.bash
ros2 launch ur_sim_config ur_sim_effort.launch.py ur_type:=ur5e gazebo_gui:=false launch_rviz:=false

# rosbridge (separate terminal)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Web dashboard (separate terminal)
cd src/ur_web_dashboard && python3 server.py 8000
```

## Key Dependencies

| apt Package | Purpose |
|---|---|
| `ros-humble-mujoco-ros2-control` | MuJoCo physics backend (default) |
| `ros-humble-ros-gz` | Gazebo Ignition Fortress + ROS bridge |
| `ros-humble-gz-ros2-control` | Gazebo hardware interface plugin |
| `ros-humble-ur-description` | UR robot URDF + meshes (required) |
| `ros-humble-ros2-control` | Controller manager framework |
| `ros-humble-ros2-controllers` | JointTrajectoryController, JointStateBroadcaster |
| `ros-humble-pinocchio` | Rigid body dynamics (gravity compensation) |
| `ros-humble-rosbridge-suite` | WebSocket bridge to ROS 2 |

## Repository Structure

```
├── config/
│   ├── config.template.yaml       # Configuration template (tracked)
│   └── config.yaml                # User config (gitignored, auto-generated)
├── src/
│   ├── ur_sim_config/             # Effort mode, gravity comp, MuJoCo support
│   │   ├── scripts/gravity_compensation.py  # Pinocchio gravity comp + PID
│   │   ├── scripts/generate_mujoco_model.sh # URDF→MJCF converter
│   │   ├── config/ur_effort_controllers.yaml
│   │   ├── launch/ur_sim_effort.launch.py   # Gazebo effort mode
│   │   ├── launch/ur_sim_mujoco.launch.py   # MuJoCo (any mode)
│   │   ├── mujoco/                          # Generated MJCF (auto, gitignored)
│   │   └── urdf/ur_sim.urdf.xacro           # Unified URDF xacro (mujoco|ignition)
│   ├── ur_simulation_gz/          # Git submodule: UR Gazebo sim
│   └── ur_web_dashboard/
│       ├── index.html             # Dashboard app
│       ├── server.py              # HTTP server
│       ├── lib/                   # Vendored JS (Three.js 0.170.0, urdf-loader 0.12.4)
│       ├── urdf/                  # Generated URDFs (gitignored, auto-generated)
│       └── worlds/
│           └── no_ground_collision.sdf
├── launch_all.sh                  # One-command launcher
└── README.md                      # This file
```

## Troubleshooting

| Problem | Solution |
|---|---|
| `rosdep update` fails | Skip it — cached data is sufficient |
| Gazebo GUI won't start | Set `gazebo_gui: false` in config (no X display) |
| "Service does not exist" errors on dashboard | Timing issue — rosbridge started before sim. Wait and refresh |
| URDF 404 on dashboard | URDFs not generated. Delete `src/ur_web_dashboard/urdf/` and rerun `./launch_all.sh` |
| Port already in use | Kill old processes: `pkill -9 -f "ign gazebo"; pkill -9 -f rosbridge; pkill -9 -f server.py` |
| Stale Gazebo preventing restart | `pkill -9 -f "ign gazebo"` then wait 2 seconds |
| MuJoCo: GLFW warning | Normal in headless mode (no display). Camera publishing disabled but sim works |
| MuJoCo: "Failed to initialize GLFW" | Headless environment — expected, no action needed |
| MuJoCo: MJCF not found | Delete `install/ur_sim_config/share/ur_sim_config/mujoco/` and rebuild |
| MuJoCo: joint not moving | Ensure MJCF was regenerated after script changes (`rm -rf install/.../mujoco/ur5e`) |

## License

- `ur_simulation_gz`: BSD-3-Clause (Universal Robots)
- Web dashboard and scripts: MIT
