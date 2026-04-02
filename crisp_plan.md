# CRISP Controller Integration Plan

Goal: Make the ur_sim platform support effort (torque) control interfaces so that
external torque-based controllers like CRISP can connect to the Gazebo simulation.
CRISP itself is NOT a submodule — it runs from a separate workspace.

## Status

- [x] 1. Create plan
- [x] 2. Add effort command interfaces to URDF (patch script)
- [x] 3. Create effort-mode controller config (with position/velocity/effort controllers)
- [x] 4. Create effort-mode launch support (`--control_mode effort` flag)
- [x] 5. Add forward_effort_controller for torque commands
- [x] 6. Test: verify effort interfaces exposed in Gazebo
- [ ] 7. Test: build CRISP externally and connect to sim
- [x] 8. Update config template and documentation
- [x] 9. Pinocchio-based gravity compensation (matches real UR behavior)
- [x] 10. Joint friction/damping in physics (simulates reduction gear)
- [x] 11. Trajectory execution in effort mode (web dashboard works in both modes)
- [x] 12. Additional controllers (forward_position, forward_velocity)

## Architecture

```
CRISP workspace (external)              ur_sim workspace
┌──────────────────────┐   ROS 2 topics   ┌─────────────────────────────────┐
│ crisp_controllers    │ ◄──────────────► │ Gazebo Ignition                  │
│ - cart_impedance     │                   │ + gz_ros2_control                │
│ - joint_impedance    │   /joint_states   │ + gravity_compensation node      │
│ - etc.               │   /external_      │   (Pinocchio gravity + PID hold) │
│                      │    effort_commands │ + forward_effort_controller      │
└──────────────────────┘                   └─────────────────────────────────┘
```

## Sim vs Real Gravity

| Setting | Real UR | Gazebo Sim (effort mode) |
|---|---|---|
| Internal gravity comp | YES (firmware) | YES (Pinocchio node) |
| Joint friction/damping | YES (gears) | YES (URDF dynamics) |
| CRISP `use_gravity_compensation` | `false` | `false` (sim handles it) |

The sim now compensates gravity at the simulation level, matching real UR behavior.
CRISP should set `use_gravity_compensation: false` — same as on real hardware.

## Control Mode

Selected at launch via CLI flag (not config file):
- `./launch_all.sh` — position mode (default)
- `./launch_all.sh --control_mode effort` — effort mode with gravity comp

## Available Controllers (effort mode)

| Controller | Type | State |
|---|---|---|
| joint_state_broadcaster | JointStateBroadcaster | active |
| forward_effort_controller | JointGroupEffortController | active |
| joint_trajectory_controller | JointTrajectoryController | inactive |
| forward_position_controller | JointGroupPositionController | inactive |
| forward_velocity_controller | JointGroupVelocityController | inactive |
