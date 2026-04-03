# AI Setup Guideline — UR Robot Simulator

This file is intended for AI agents. When a user clones this repo and asks an AI
to set up the environment, the AI should follow these instructions.

## Quick Reference

```
Workspace root: the directory containing this file
ROS distro:     Humble (Ubuntu 22.04)
Build system:   colcon
Submodules:     ur_simulation_gz (Gazebo sim)
System package: ur_description (URDF + meshes, from ros-humble-ur-description)
One-command:    ./launch_all.sh
Config:         config/config.yaml (auto-generated from config/config.template.yaml)
```

## Setup Steps

Run these in order. All commands assume the working directory is the repo root.

### Step 1: Verify prerequisites

```bash
# Check OS
lsb_release -a   # Expect: Ubuntu 22.04

# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 --help       # Should work

# Check tools
which colcon git xacro   # All must exist
```

If ROS 2 Humble is not installed, install it first per
https://docs.ros.org/en/humble/Installation.html.

### Step 2: Initialize submodules

If the repo was cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

Verify: `src/ur_simulation_gz/` should be non-empty.

Note: `ur_description` is NOT a submodule — it uses the system package
`ros-humble-ur-description` (installed in Step 3).

### Step 3: Install apt dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ur-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rosbridge-suite
```

Then install any remaining rosdep keys:

```bash
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
```

**Tip:** If `rosdep update` fails due to DNS issues (e.g. `raw.githubusercontent.com`
resolves to `0.0.0.0`), skip it — the cached rosdep database is sufficient.

### Step 4: Build

Only `ur_simulation_gz` needs source building (~1 second):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Step 5: Launch

```bash
./launch_all.sh
```

This will:
1. Generate `config/config.yaml` from template (first run only)
2. Generate static URDFs for the 3D web viewer (first run only)
3. Start Gazebo simulation (headless by default)
4. Start rosbridge WebSocket server
5. Start the web dashboard

Open `http://localhost:8000` in a browser.

**Headless environment (no X display):** The default config already sets
`gazebo_gui: false` and `launch_rviz: false`. No changes needed.

### Step 6: Verify

In a separate terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states --once   # Should show 6 joint positions
ros2 control list_controllers          # Should list 2 active controllers
```

---

## Configuration

Edit `config/config.yaml` to change settings. Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `ur_type` | `ur5e` | Robot model: ur3, ur3e, ur5, ur5e, ur7e, ur8long, ur10, ur10e, ur12e, ur15, ur16e, ur18, ur20, ur30 |
| `gazebo_gui` | `false` | Show Gazebo GUI (requires X display) |
| `launch_rviz` | `false` | Launch RViz (requires X display) |
| `world_file` | `no_ground_collision.sdf` | Gazebo world file |
| `rosbridge_port` | `9090` | rosbridge WebSocket port |
| `dashboard_port` | `8000` | Web dashboard HTTP port |

---

## Architecture

```
Browser (HTTP)       <--:8000--> server.py (static files + mesh routing)
Browser (WebSocket)  <--:9090--> rosbridge_websocket <--ROS 2--> Gazebo Ignition
```

### ROS 2 Interfaces

| Interface | Type | Description |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | Joint positions, velocities, efforts |
| `/joint_trajectory_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | Send trajectory commands |
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

---

## Web Dashboard Details

File: `src/ur_web_dashboard/index.html` (single-page, no build step)

### Features

- 3D robot viewer (Three.js + urdf-loader, orbit controls)
- Real-time joint states table
- TCP pose via forward kinematics (tool0 frame)
- Auto-detect robot type from simulation
- Controller status polling
- Sim clock display
- Joint jog sliders with trajectory send
- Self-contained (all JS vendored in `lib/`, no CDN)

### File Serving (server.py)

`server.py` routes requests:
- `/` → `index.html`
- `/lib/` → vendored JS libraries
- `/urdf/` → generated static URDFs
- `/ur_description/` → UR mesh files (from system `ros-humble-ur-description` package)

Use `server.py` instead of `python3 -m http.server` — the latter can't serve meshes.

---

## Repository Structure

```
├── config/
│   ├── config.template.yaml       # Configuration template (tracked)
│   └── config.yaml                # User config (gitignored, auto-generated)
├── src/
│   ├── ur_simulation_gz/          # Git submodule: UR Gazebo sim
│   └── ur_web_dashboard/
│       ├── index.html             # Dashboard app
│       ├── server.py              # HTTP server
│       ├── lib/                   # Vendored JS (Three.js 0.170.0, urdf-loader 0.12.4)
│       ├── urdf/                  # Generated URDFs (gitignored, auto-generated)
│       └── worlds/
│           └── no_ground_collision.sdf
├── launch_all.sh                  # One-command launcher
├── install_deps.sh                # Dependency installer (legacy)
├── ai_setup_guideline.md          # This file
└── README.md
```

## Key Dependencies (all apt)

| apt Package | Purpose |
|---|---|
| `ros-humble-ros-gz` | Gazebo Ignition Fortress + ROS bridge |
| `ros-humble-gz-ros2-control` | Gazebo hardware interface plugin |
| `ros-humble-ur-description` | UR robot URDF + meshes (required) |
| `ros-humble-ros2-control` | Controller manager framework |
| `ros-humble-ros2-controllers` | JointTrajectoryController, JointStateBroadcaster |
| `ros-humble-rosbridge-suite` | WebSocket bridge to ROS 2 |

## Troubleshooting

| Problem | Solution |
|---|---|
| `rosdep update` fails | Skip it — cached data is sufficient |
| Gazebo GUI won't start | Set `gazebo_gui: false` in config (no X display) |
| "Service does not exist" errors on dashboard | Timing issue — rosbridge started before sim. Wait and refresh |
| URDF 404 on dashboard | URDFs not generated. Delete `src/ur_web_dashboard/urdf/` and rerun `./launch_all.sh` |
| Port already in use | Kill old processes: `pkill -9 -f "ign gazebo"; pkill -9 -f rosbridge; pkill -9 -f server.py` |
| Stale Gazebo preventing restart | `pkill -9 -f "ign gazebo"` then wait 2 seconds |
