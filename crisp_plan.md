# CRISP Controller Integration Plan

Goal: Make the ur_sim platform support effort (torque) control interfaces so that
external torque-based controllers like CRISP can connect to the Gazebo simulation.
CRISP itself is NOT a submodule — it runs from a separate workspace.

## Status

- [x] 1. Create plan
- [x] 2. Create wrapper xacro with effort interface enabled
- [x] 3. Create effort-mode controller config
- [x] 4. Create effort-mode launch support in launch_all.sh
- [x] 5. Add forward_effort_controller for torque commands
- [x] 6. Test: verify effort interfaces exposed in Gazebo
- [ ] 7. Test: build CRISP externally and connect to sim
- [x] 8. Update config template and documentation

## Key Finding

The upstream `ur_description` xacro **excludes** `<command_interface name="effort"/>`
when `sim_ignition:=true`. We need a wrapper xacro that re-adds it.

## Architecture

```
CRISP workspace (external)              ur_sim workspace
┌──────────────────────┐   ROS 2 topics   ┌─────────────────┐
│ crisp_controllers    │ ◄──────────────► │ Gazebo Ignition  │
│ - gravity_comp       │   /joint_states   │ + gz_ros2_control│
│ - cart_impedance     │   effort commands │                  │
│ - joint_impedance    │                   │ effort interface │
└──────────────────────┘                   └─────────────────┘
```

## Sim vs Real Gravity

| Setting | Real UR | Gazebo Sim |
|---|---|---|
| UR internal gravity comp | YES (always on) | NO |
| CRISP `use_gravity_compensation` | `false` | `true` |

The sim config MUST enable CRISP's gravity compensation (Pinocchio RNEA) because
Gazebo's physics engine does not internally compensate gravity like the real UR.

## Control Mode Config

`config/config.yaml` will have a new `control_mode` parameter:
- `position` (default) — current behavior, joint_trajectory_controller
- `effort` — effort interfaces exposed, forward_effort_controller available
