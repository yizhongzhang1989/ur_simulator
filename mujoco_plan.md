# MuJoCo Integration Plan

Goal: Add MuJoCo as an alternative physics engine alongside Gazebo.
User selects via `--simulator gazebo|mujoco` flag in `launch_all.sh`.

## Status

- [ ] 1. Install `ros-humble-mujoco-ros2-control` dependencies
- [ ] 2. Create MJCF generation script (`generate_mujoco_model.sh`)
- [ ] 3. Create `mujoco_inputs.xml` with UR-specific MuJoCo settings
- [ ] 4. Create `ur_sim_mujoco.launch.py` (effort mode)
- [ ] 5. Create `ur_sim_mujoco_position.launch.py` (position mode)
- [ ] 6. Add `--simulator` flag to `launch_all.sh`
- [ ] 7. Test position mode (joint trajectory via web dashboard)
- [ ] 8. Test effort mode (gravity comp, trajectory, stability)
- [ ] 9. Update README, config, and documentation

## Key Differences: Gazebo vs MuJoCo

| Aspect | Gazebo (current) | MuJoCo (new) |
|---|---|---|
| Plugin | `ign_ros2_control/IgnitionSystem` | `mujoco_ros2_control/MujocoSystemInterface` |
| Robot model | URDF (with `<gazebo>` tags) | MJCF (converted from URDF) |
| Control node | Gazebo-internal controller_manager | Separate `mujoco_ros2_control/ros2_control_node` |
| Clock bridge | `ros_gz_bridge` for `/clock` | MuJoCo plugin publishes `/clock` natively |
| Spawning | `ros_gz_sim create` + Gazebo process | MuJoCo Simulate app (built into control node) |
| Headless | `-s` flag to `gz sim` | `<param name="headless">true</param>` in URDF |
| Effort interface | Patched URDF + `forward_effort_controller` | Motor actuators in MJCF + effort command interface |
| Gravity comp | `gravity_compensation.py` node | Same node (physics-agnostic, reads `/joint_states`) |

## Architecture

```
launch_all.sh --simulator mujoco --control_mode effort
       │
       ▼
ur_sim_mujoco.launch.py
       │
       ├── URDF→MJCF conversion (generate_mujoco_model.sh)
       │     └── mujoco_inputs.xml (actuators, friction, scene)
       │
       ├── mujoco_ros2_control/ros2_control_node
       │     └── loads MJCF model, runs MuJoCo physics
       │
       ├── robot_state_publisher (original URDF for TF/RViz)
       │
       ├── controller_manager spawners
       │     ├── joint_state_broadcaster (active)
       │     ├── forward_effort_controller (active)
       │     ├── joint_trajectory_controller (inactive)
       │     ├── forward_position_controller (inactive)
       │     └── forward_velocity_controller (inactive)
       │
       └── gravity_compensation.py (Pinocchio, reused as-is)
```

## Implementation Details

### Phase 1 — Install & MJCF Generation

1. **Dependencies**: `sudo apt install ros-humble-mujoco-ros2-control`
2. **generate_mujoco_model.sh**: Convert UR URDF to MJCF using
   `ros2 run mujoco_ros2_control make_mjcf_from_robot_description.py`
   with `--urdf`, `--mujoco_inputs`, `--save_only`, `--output`
3. **mujoco_inputs.xml**: Define UR-specific settings:
   - `<raw_inputs>`: motor actuators for all 6 joints (effort mode),
     or position actuators (position mode)
   - `<processed_inputs>`: `<modify_element>` to set `frictionloss`, `damping`
     on each joint (matching Gazebo friction values)
   - `<scene>`: ground plane, lighting, gravity

### Phase 2 — Launch Files

4. **ur_sim_mujoco.launch.py** (effort mode):
   - Generate or locate MJCF model
   - Start `mujoco_ros2_control/ros2_control_node` with:
     - `use_sim_time: True`
     - Controller YAML (reuse `ur_effort_controllers.yaml`)
   - `robot_state_publisher` with original URDF (for TF tree / web dashboard)
   - Spawn controllers (same as Gazebo effort mode)
   - Start `gravity_compensation.py` after effort controller ready
   - Support `headless` parameter

5. **ur_sim_mujoco_position.launch.py** (position mode):
   - Same structure but position actuators in MJCF
   - `joint_trajectory_controller` active
   - No gravity compensation node needed

### Phase 3 — launch_all.sh Integration

6. **New flag**: `--simulator gazebo|mujoco` (default: `gazebo`)
7. **Routing logic**:
   ```
   simulator=gazebo, control_mode=position → ur_sim_control.launch.py
   simulator=gazebo, control_mode=effort  → ur_sim_effort.launch.py
   simulator=mujoco, control_mode=position → ur_sim_mujoco_position.launch.py
   simulator=mujoco, control_mode=effort  → ur_sim_mujoco.launch.py
   ```
8. **Cleanup**: kill `ros2_control_node` for MuJoCo (instead of `ign gazebo`)

### Phase 4 — Testing & Documentation

9. Update README with MuJoCo usage examples
10. Update `config.template.yaml` if needed
11. Verify web dashboard works identically with both backends

## Key Technical Decisions

- **Controller YAML reuse**: Same YAML for both Gazebo and MuJoCo — the
  ros2_control abstraction means controllers are physics-engine-agnostic
- **Gravity comp reuse**: `gravity_compensation.py` only depends on
  `/joint_states` and publishes to `/forward_effort_controller/commands`,
  both physics-engine-independent
- **Web dashboard**: No changes — it only talks to rosbridge topics
- **MJCF storage**: Pre-generate on first launch, store in
  `ur_sim_config/mujoco/` (gitignored, like static URDFs)

## MuJoCo Actuator Mapping

For effort mode (motor actuators — native effort support):
```xml
<actuator>
  <motor name="shoulder_pan_joint" joint="shoulder_pan_joint" ctrlrange="-150 150"/>
  <motor name="shoulder_lift_joint" joint="shoulder_lift_joint" ctrlrange="-150 150"/>
  <motor name="elbow_joint" joint="elbow_joint" ctrlrange="-150 150"/>
  <motor name="wrist_1_joint" joint="wrist_1_joint" ctrlrange="-28 28"/>
  <motor name="wrist_2_joint" joint="wrist_2_joint" ctrlrange="-28 28"/>
  <motor name="wrist_3_joint" joint="wrist_3_joint" ctrlrange="-28 28"/>
</actuator>
```

For position mode (position actuators — native position support):
```xml
<actuator>
  <position name="shoulder_pan_joint" joint="shoulder_pan_joint" kp="5000" ctrlrange="-6.28 6.28"/>
  ...
</actuator>
```

## Risks & Mitigations

| Risk | Mitigation |
|---|---|
| URDF→MJCF conversion fails for UR models | Use `mujoco_inputs.xml` + `modify_element` to fix up joint properties. Maintain a pre-built MJCF as fallback |
| Mesh format issues (DAE→OBJ) | Pre-convert meshes and store in repo |
| Different physics behavior | Tune `frictionloss`, `damping` in MJCF to match Gazebo behavior |
| `mujoco_ros2_control` requires its own `ros2_control_node` | Already documented — use `package="mujoco_ros2_control"` in launch |

## Files to Create/Modify

**New files (~5):**
- `src/ur_sim_config/scripts/generate_mujoco_model.sh`
- `src/ur_sim_config/mujoco/mujoco_inputs.xml`
- `src/ur_sim_config/launch/ur_sim_mujoco.launch.py`
- `src/ur_sim_config/launch/ur_sim_mujoco_position.launch.py`

**Modified files (~4):**
- `launch_all.sh` (add `--simulator` flag)
- `src/ur_sim_config/package.xml` (add mujoco dependency)
- `src/ur_sim_config/CMakeLists.txt` (install new files)
- `README.md` (document MuJoCo usage)

**Reused as-is:**
- `gravity_compensation.py`
- `ur_effort_controllers.yaml`
- Web dashboard (index.html, server.py)
