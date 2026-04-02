# AI Setup Guideline — UR Robot Simulator

This file guides an AI agent through setting up and extending the UR robot simulation
environment. Follow each phase in order. Execute commands verbatim unless adaptation
is required for the host system.

## Prerequisites

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble (installed at `/opt/ros/humble/`)
- **Tools required:** `colcon`, `vcs`, `rosdep`, `git`
- **Workspace root:** `/home/yizhongzhang/Documents/ur_sim`

## Known Issues

- `raw.githubusercontent.com` may resolve to `0.0.0.0` due to DNS filtering.
  `rosdep update` will fail in this case. **Workaround:** skip `rosdep update` and
  rely on the existing cached rosdep database. It is sufficient for all packages used
  in this project.
- The VM has no X display (`$DISPLAY` is unset). Always launch Gazebo and RViz in
  headless mode: `gazebo_gui:=false launch_rviz:=false`.

---

## Phase 1 — Environment & Simulation Setup [COMPLETED]

All steps below have been executed and verified.

### 1.1 Install Gazebo + ROS bridge (apt binary)

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gz
```

### 1.2 Clone `ur_simulation_gz` (the only source-build package)

```bash
cd /home/yizhongzhang/Documents/ur_sim
mkdir -p src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz
```

### 1.3 Install remaining dependencies as apt binaries

Do NOT build these from source — binary packages are available and much faster:

```bash
sudo apt-get install -y \
  ros-humble-gz-ros2-control \
  ros-humble-ur-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-ur-moveit-config
```

Then install any remaining rosdep keys (skip `rosdep update` if DNS is blocked):

```bash
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
```

### 1.4 Build workspace

Only `ur_simulation_gz` needs building (~1 second):

```bash
cd /home/yizhongzhang/Documents/ur_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 1.5 Verify simulation

```bash
source install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=false launch_rviz:=false
```

**Expected result:** Gazebo Ignition Fortress starts in server mode. All 6 UR5e joints
load. `joint_state_broadcaster` and `joint_trajectory_controller` activate. No errors.

**Verification commands (in a second terminal):**

```bash
source /home/yizhongzhang/Documents/ur_sim/install/setup.bash
ros2 topic echo /joint_states --once          # Should show 6 joint positions
ros2 topic echo /clock --once                 # Should show advancing sim time
ros2 control list_controllers                 # Should list 2 active controllers
```

### Launch file reference

File: `src/ur_simulation_gz/ur_simulation_gz/launch/ur_sim_control.launch.py`

Key arguments:
| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur5e` | Robot model (ur3, ur5, ur5e, ur10e, ur16e, ur20, ur30, etc.) |
| `gazebo_gui` | `true` | Set `false` for headless (no X display required) |
| `launch_rviz` | `true` | Set `false` for headless |
| `start_joint_controller` | `true` | Auto-start the trajectory controller |
| `initial_joint_controller` | `joint_trajectory_controller` | Controller to start |
| `world_file` | `empty.sdf` | Gazebo world file |

---

## Phase 2 — Control Interface Layer [COMPLETED]

The simulation exposes standard ROS 2 control interfaces. Verified working.

### 2.1 Available ROS 2 interfaces

After launching the simulation, these interfaces are available:

```bash
# Joint states (sensor feedback) — 6 joints at ~500 Hz
ros2 topic echo /joint_states --once

# Trajectory control via topic (direct publish)
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{...}"

# Trajectory control via action (with feedback)
ros2 action info /joint_trajectory_controller/follow_joint_trajectory

# Controller manager
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

**Active controllers:**
- `joint_state_broadcaster` (JointStateBroadcaster) — active
- `joint_trajectory_controller` (JointTrajectoryController) — active

**Joint names (in order):**
`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
`wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`

### 2.2 Test trajectory execution

Verified — publish a trajectory and joints move to commanded positions:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [{positions: [0.5, -1.0, 0.5, -1.0, 0.5, 0.0],
            velocities: [], accelerations: [], effort: [],
            time_from_start: {sec: 3, nanosec: 0}}]
}"
```

### 2.3 Sim/real note

The sim uses `joint_trajectory_controller`. Real UR robots use
`scaled_joint_trajectory_controller`. For portable control code, either:
- Create a launch-time topic remapping, or
- Use a config parameter for the controller name

---

## Phase 3 — Web Dashboard [COMPLETED]

A browser-based dashboard for monitoring and controlling the simulated robot.

### 3.1 Install WebSocket bridge

```bash
sudo apt-get install -y ros-humble-rosbridge-suite
```

### 3.2 Launch rosbridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Exposes all ROS 2 topics/services/actions over WebSocket on port 9090.

### 3.3 Launch web dashboard

```bash
cd src/ur_web_dashboard && python3 -m http.server 8080
```

Open `http://<host>:8080` in a browser.

### 3.4 Dashboard features (implemented)

File: `src/ur_web_dashboard/index.html` (single-page, no build step)

- **Joint states table** — real-time positions (°), velocities, efforts
- **TCP pose** — X/Y/Z position + Roll/Pitch/Yaw from `/tf` (tool0 frame)
- **Controller status** — polls `/controller_manager/list_controllers` every 5s
- **Sim clock** — displays simulation time from `/clock`
- **Joint jog sliders** — per-joint sliders with degree readouts
- **Send trajectory** — publishes to `/joint_trajectory_controller/joint_trajectory`
- **Reset to current** — snaps sliders to current joint positions
- **Connection status** — auto-reconnects to rosbridge on disconnect
- **Log panel** — timestamped event log

Uses raw WebSocket to rosbridge (no roslibjs CDN dependency — works offline).

### 3.5 Architecture

```
Browser (WebSocket) <--:9090--> rosbridge_websocket <--ROS 2--> Gazebo sim
                    <--:8080--> python3 http.server (static files)
```

### 3.6 One-command launch

`./launch_all.sh [ur_type]` starts all three services (sim, rosbridge, dashboard).
Default robot is `ur5e`. Cleans up all processes on Ctrl+C.

---

## Phase 4 — Control System Integration [NOT STARTED]

### 4.1 Connect custom control system

The custom control system should publish to:
- `/joint_trajectory_controller/follow_joint_trajectory` (action)

And subscribe to:
- `/joint_states` (sensor_msgs/msg/JointState)
- `/tf` (geometry_msgs/msg/TransformStamped) for TCP pose

### 4.2 Validate

- Trajectory tracking accuracy
- Collision avoidance behavior
- Timing consistency (sim time vs. wall time)

---

## Phase 5 — Polish & Sim↔Real Parity [NOT STARTED]

- Add a launch argument `use_sim:=true|false` to switch between sim and real robot
- Tune Gazebo physics parameters (PID gains, friction, inertia)
- Add `ros2 bag` recording for post-hoc analysis
- Dockerize the full stack for reproducible deployment

---

## Key Dependencies (all installed via apt)

| Component | apt Package | Purpose |
|---|---|---|
| Gazebo + ROS bridge | `ros-humble-ros-gz` | Physics simulation |
| Gazebo ros2_control | `ros-humble-gz-ros2-control` | Hardware interface plugin |
| Robot URDF | `ros-humble-ur-description` | UR robot model |
| Control framework | `ros-humble-ros2-control` | Controller manager |
| Controllers | `ros-humble-ros2-controllers` | JTC, JSB, etc. |
| Motion planning | `ros-humble-moveit` | MoveIt 2 |
| MoveIt config | `ros-humble-ur-moveit-config` | UR-specific MoveIt config |
| Web bridge | `ros-humble-rosbridge-suite` | WebSocket↔ROS 2 |
| **Source build** | `ur_simulation_gz` | Launch files & configs |

## Workspace Structure

```
/home/yizhongzhang/Documents/ur_sim/
├── src/
│   ├── ur_simulation_gz/          # UR Gazebo sim (source-build)
│   │   └── ur_simulation_gz/
│   │       ├── launch/            # Launch files
│   │       ├── config/            # Controller YAML configs
│   │       ├── urdf/              # Xacro overrides
│   │       └── test/
│   └── ur_web_dashboard/          # Web dashboard (static files)
│       └── index.html             # Single-page dashboard app
├── install/                       # Colcon install space
├── build/                         # Colcon build artifacts
├── log/                           # Colcon build logs
├── install_deps.sh                # Automated install script
├── launch_all.sh                  # One-command full stack launcher
└── ai_setup_guideline.md          # This file
```

## Service Ports

| Service | Port | Protocol |
|---|---|---|
| Web dashboard | 8080 | HTTP |
| rosbridge | 9090 | WebSocket |
| Gazebo sim | — | ROS 2 (local) |
