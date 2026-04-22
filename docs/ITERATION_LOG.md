# Iteration Log

Append-only log of improvement rounds. Newest entry at top.

Format per entry:

```
## Round N — <title>                         <YYYY-MM-DD>  [status]
Commit: <hash or "uncommitted">
Smoke:  PASS | FAIL (<short reason>)
Targets: #<shortcoming-ids from SHORTCOMINGS.md>

Changes:
- <bullet>

Follow-ups (new issues surfaced):
- <bullet>

Notes:
<optional prose>
```

`status` is one of: `in-progress`, `complete`, `rolled-back`.

---

## Round 11 — MuJoCo position-actuator stability                2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (position 26/26, effort 27/27)
Targets: #15

Changes:
- `src/ur_sim_config/scripts/generate_mujoco_model.sh`:
  - Position actuator gains retuned (`kp=1500 kv=200` shoulders,
    `kp=1000 kv=120` elbow, `kp=300 kv=40` wrists) so damping ratio
    is close to critical instead of lightly underdamped.
  - Added `forcerange` on every `<position>` actuator sized from
    the per-`ur_type` effort limit — a single bad command can no
    longer push a joint past the torque envelope.
  - `ur_scene.xml` now sets `<option integrator="implicitfast"/>`,
    which is stable with stiff PD actuators at `timestep=0.001`
    (the implicit RK4 default diverged).
- `src/ur_web_dashboard/index.html`:
  - Added in-flight guard `switchInFlight` + optimistic update of
    `activeMotionController` on switch-success. Rapid clicks on
    two switch buttons no longer issue a stale-state second
    service call.
  - Early-return when the clicked button matches the already-active
    controller (no more no-op `switch_controller` round-trips).
  - Control-mode-gated buttons (fwd_vel / fwd_eff in position mode,
    position in effort mode) marked `data-gated-off` so the
    in-flight enable/disable cycle leaves them greyed.
  - Corrected incorrect `// BEST_EFFORT` comment (code was using
    `strictness=2` = STRICT all along).

Follow-ups (new issues surfaced):
- None — repro script now passes every switch and every send.

Notes:
- Repro (`/tmp/repro_fwdpos_bug.py`): JTC → fwd_pos, send
  `shoulder_pan += 0.3 rad`, switch back to JTC, to scaled-JTC,
  back to fwd_pos, another jog, back to scaled-JTC: all six
  transitions `ok=True`, arm tracks to within 3 mrad, no wrist
  divergence.
- Before the fix, the same script made `wrist_3_joint` run to
  361 rad in 2 s, the MuJoCo step loop stalled at `sec=65`, and
  every subsequent `switch_controller` call timed out.


Commit: uncommitted
Smoke:  PASS (effort 27/27, position 26/26)
Targets: #11

Changes:
- New `.github/workflows/ci.yml`:
  - Triggers on `push` / `pull_request` to `main` and
    `workflow_dispatch`.
  - Runs in `osrf/ros:humble-desktop-full` with `--user 0` so apt
    works. Matrix over `control_mode: [position, effort]`.
  - Installs `xacro`, `ros2-control*`, `ur-description`,
    `rosbridge-suite`, `ign-ros2-control`, `ros-gz*` via apt; best-
    effort install for `ros-humble-mujoco-ros2-control` (the humble
    apt index doesn't ship it, but the smoke test's backend
    assertion already flags its absence in position mode instead of
    crashing).
  - `rosdep install --skip-keys=mujoco_ros2_control`, then
    `colcon build --symlink-install`, then
    `scripts/smoke_test.sh --control_mode <matrix>`. Smoke log
    uploaded as an artifact on every run (incl. failures) via
    `actions/upload-artifact@v4`.
- `README.md`: CI badge added at the top (OWNER/REPO placeholders
  will be filled once the repo is pushed to a known remote; the
  badge is safe to render as "unknown" locally).
- Smoke test: new `ci.workflow_present` assertion verifies the YAML
  exists **and** references `smoke_test.sh`, so a well-meaning
  cleanup that deletes the workflow or rewrites it without running
  the smoke test fails the regression gate immediately.

Branch-protection recommendation (documented here, not enforced in
code): require `ci / smoke (effort)` and `ci / smoke (position)` to
be green before merging to `main`. See the workflow's `name` and
`matrix.control_mode` for the exact check names.

Follow-ups (new issues surfaced):
- `mujoco_ros2_control` isn't available via apt on Humble, so CI's
  effort-mode job will depend on any future packaging of that
  plugin. Until then the smoke test's `backend.mujoco_up` assertion
  will fail in CI for position mode but pass for effort mode
  (Ignition). If this blocks the CI requirement for both modes in
  practice, add a build-from-source step to the workflow.
- The README badge URL contains `OWNER/REPO` placeholders. Replace
  once the repo is pushed to a real GitHub namespace; this is a
  one-line doc fix, not a shortcoming.

Notes:
Round 10 closes out the 10-round plan. All rounds complete; smoke
currently at 27/27 (effort) / 26/26 (position).

---

## Round 9 — Dashboard upgrades                              2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (effort 26/26, position 25/25)
Targets: #12

Changes:
- `src/ur_web_dashboard/index.html`:
  - Subscribes to `/tcp_pose_broadcaster/pose`
    (`geometry_msgs/msg/PoseStamped`) and renders its `pose.position`
    + quaternion-derived RPY into the TCP panel. Falls back to the
    existing browser-side URDF FK if the broadcaster is silent (e.g.
    early startup), so the panel is never empty.
  - New "→ JTC / → fwd_pos / → fwd_eff" controller-switch buttons.
    Clicking one calls
    `/controller_manager/switch_controller` with the selected target
    in `activate_controllers` and the current active motion
    controller (if any, except the target) in
    `deactivate_controllers`. Strictness `BEST_EFFORT`, 2s timeout.
  - Refreshes the controller list 500ms after a switch request so
    users see the change without waiting for the 5s poll cycle.
- Smoke test:
  - `dashboard.switch_controller_service_present` verifies
    `/controller_manager/switch_controller` is advertised at
    runtime.
  - `dashboard.wired_to_upstream_topics_services` greps the
    dashboard source for both `switch_controller` and
    `tcp_pose_broadcaster/pose`, protecting against the R9 wiring
    being inadvertently reverted.

Follow-ups (new issues surfaced):
- The IO panel (`io_and_status_controller`) still displays only
  `program_running`; proper digital-I/O + analog-I/O visualization
  requires the full `ur_msgs/IOStates` publisher, which
  `sim_broadcasters.py` stubs but doesn't populate. Deferred — not a
  shortcoming in `SHORTCOMINGS.md`, add there first if it becomes
  real.
- No visual indicator for the currently-active motion controller on
  the jog panel; the switcher fires-and-forgets. Fine for a
  developer tool.

Notes:
The dashboard uses raw rosbridge JSON service calls rather than
roslibjs service proxies, matching the style already used for
`list_controllers`/`get_parameters`.

---

## Round 8 — Launcher & reliability                          2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (effort 24/24, position 23/23)
Targets: #10, #8

Changes:
- New `scripts/setup_rt.sh`: optional helper that writes
  `/etc/security/limits.d/99-ur-sim.conf` so the invoking user gets
  `rtprio 95` and unlimited `memlock`. Documented as purely optional —
  the simulator runs without it; setting it tightens the 500 Hz
  control-loop jitter.
- Smoke test hardening (carried over from R7 controller-wait retry):
  - `reliability.no_controllers_in_error` — every controller listed
    by `ros2 control list_controllers` must be in state `active` or
    `inactive`. Any other state (e.g. `unconfigured`, `finalized`,
    `erroring`) fails the check.
  - `reliability.setup_rt_script_present` — guards against accidental
    removal of the helper.

Follow-ups (new issues surfaced):
- The full launcher rewrite (pure-Python launch file owning the
  lifecycle of Gazebo/MuJoCo + rosbridge + dashboard) is not landed
  yet; `launch_all.sh` is kept because it's the stable entry point
  that CRISP clients rely on today. A follow-up round should
  migrate, but only after the downstream CRISP integration stops
  sourcing `launch_all.sh` verbatim.
- Multi-robot demo (two UR arms with distinct `tf_prefix`) is still
  a pending shortcoming from R7; needs a launch argument that
  duplicates the stack under different namespaces.

Notes:
The "wait-for-ready based on controller activation" deliverable
landed in R7 as the 15-second controller-retry loop in
`smoke_test.sh`; this round additionally verifies that the final
state after that wait is clean.

---

## Round 7 — tf_prefix parameter threading                   2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (effort 22/22, position 21/21)
Targets: #6

Changes:
- `scripts/gravity_compensation.py`: new `tf_prefix` ROS parameter. All
  runtime joint-name lookups (Pinocchio `existJointName`, `/joint_states`
  mapping, `/joint_trajectory_controller/joint_trajectory` reorder)
  consult `self._joint_names = [f"{tf_prefix}{n}" for n in JOINT_NAMES]`
  instead of the bare constant. The module-level `JOINT_NAMES` is still
  used as the canonical key into `config/ur_types/*.yaml`.
- Launches (`ur_sim_mujoco.launch.py`, `ur_sim_effort.launch.py`):
  forward the launch-time `tf_prefix` configuration into
  `gravity_compensation` AND `sim_broadcasters` node parameters, not
  just into the URDF xacro.
- Smoke test (`scripts/smoke_test.sh`):
  - New `tf_prefix.default_joint_names_unchanged` assertion greps for
    each of the six canonical joint names in `/joint_states` and fails
    if any are missing. Guards the hard rule from
    `.github/copilot-instructions.md` that the default tf_prefix must
    never change `/joint_states` joint order or names.
  - Replaced the one-shot `ros2 control list_controllers` probe with
    a ~15s retry loop that waits until the full expected set is
    loaded. Eliminates the R5/R6 follow-up flake where
    `forward_velocity_controller` (or `scaled_joint_trajectory_controller`,
    etc.) was still mid-spawn at the moment of the check. Much more
    deterministic on cold starts.

Follow-ups (new issues surfaced):
- A multi-robot launch demo (two UR arms with distinct `tf_prefix`es)
  is not wired yet. `launch_all.sh` only spawns one robot. Deferring to
  Round 8 since that round rewrites the launcher in Python anyway; the
  demo should be a launch argument there rather than a one-off script.
- sim_broadcasters publishes `/tcp_pose` / `/wrench` on a fixed
  namespace. With a non-empty `tf_prefix` these topics are still
  global; that's fine for single-robot use but will need
  per-robot namespaces when Round 8 lands the multi-robot demo.

Notes:
The initial implementation of the smoke check used `ros2 topic echo
--once /joint_states` piped through `awk` + `grep` to build an
alphabetical joint-name list, but the one-shot echo sometimes
truncated output mid-list (only the first 3 names were captured).
Switched to per-name `grep -qw` lookups against the full echo
snapshot, which is both simpler and race-free.

---

## Round 6 — MJCF quality (collision, refactor)             2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (effort 21/21, position 20/20)
Targets: #5

Changes:
- New `scripts/mjcf_postprocess.py`: extracts the inline Python post-
  processing block from `generate_mujoco_model.sh` into a standalone,
  documented script. Each transform is a named function with a
  docstring that cites the root cause and how to remove the hack:
    - `_fix_shoulder_quat` — cancels the Rz(π) that leaks onto
      `shoulder_link` from MuJoCo's fixed-joint fusion.
    - `_strip_actuatorfrcrange` — removes compiler-derived joint
      `actuatorfrcrange` that would shadow motor `ctrlrange`.
    - `_inject_mesh_collision_flags` — **NEW** duplicates every
      visual `<geom type="mesh" .../>` into a collision companion
      with `group="3" contype="1" conaffinity="0"`. Via MuJoCo's
      contact filter, the robot now collides with the ground plane
      (and any external objects with the default `conaffinity="1"`)
      while self-collision between robot links is still disabled
      (adjacent UR meshes overlap by design).
    - `_inject_joint_friction` — gear friction/damping per joint.
- `generate_mujoco_model.sh`: replaced the ~50-line inline Python
  heredoc with a single call to `mjcf_postprocess.py`.
- `CMakeLists.txt`: install `mjcf_postprocess.py` under
  `lib/${PROJECT_NAME}` so the script is reachable both from the
  source tree and the installed layout.
- Smoke test: new `mjcf.collision_geoms_present` assertion verifies
  the generated `ur_robot.xml` contains at least one `group="3"`
  geom. This guards against a future refactor regressing to the
  previous all-collisions-disabled state.

Follow-ups (new issues surfaced):
- Self-collision between non-adjacent links is still disabled; a
  proper fix needs either per-link convex decomposition or a
  hand-authored MJCF seeded from `mujoco_menagerie`.
- The shoulder-quat hack and the `actuatorfrcrange` strip remain;
  they are now labeled and individually removable but still present.
  Upstream ur_description fixes (or a hand-authored MJCF per
  `ur_type`) are required to drop them.
- Position-mode smoke still occasionally flakes on spawner race
  ordering (see R5 follow-up); deferred to R8.

Notes:
With collision companions enabled, the MuJoCo simulator now reports
~7 additional `<geom group="3">` entries per URDF-derived MJCF (one
per visual mesh — base + 6 links). The robot no longer passes
through the world plane.

---

## Round 5 — URDF generation overhaul                        2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (effort 20/20, position 19/19)
Targets: #4, #7

Changes:
- New `src/ur_sim_config/urdf/ur_sim.urdf.xacro`: single xacro parameterized
  on `ur_type`, `simulator` (`mujoco`|`ignition`), `control_mode`
  (`position`|`effort`), `controllers_file`, `mjcf_model`, `headless`,
  `tf_prefix`, and the upstream `safety_limits` / `safety_pos_margin` /
  `safety_k_position` knobs. Includes `ur_macro.xacro` with
  `generate_ros2_control_tag:=false` so upstream geometry is reused but
  ur_sim_config owns the `<ros2_control>` block.
- `launch/ur_sim_mujoco.launch.py`: replaced the inline
  `_build_ros2_control_block` regex/string builder with a
  `subprocess.check_output(["xacro", ...])` call against the new xacro.
  Declared new args (`safety_limits`, `safety_pos_margin`,
  `safety_k_position`, `tf_prefix`) and forwarded them. The old builder
  is kept as a `NotImplementedError` stub so any external importer fails
  loudly instead of silently producing a broken URDF.
- `launch/ur_sim_effort.launch.py`: replaced the
  `generate_effort_urdf.sh` `Command([...])` with the same xacro
  invocation (`simulator:=ignition`). `safety_limits`,
  `safety_pos_margin`, `safety_k_position`, `tf_prefix` are now really
  threaded through to the URDF (previously declared but unused).
- Deleted `src/ur_sim_config/urdf/generate_effort_urdf.sh` — the sed
  pipeline that injected `effort` command interfaces into the
  ignition-flavored upstream xacro is no longer needed.
- Smoke test: new `urdf.ur_sim_xacro_installed` assertion verifies the
  unified xacro is installed AND the deprecated `generate_effort_urdf.sh`
  is NOT reinstalled.

Follow-ups (new issues surfaced):
- Position-mode smoke occasionally flakes on
  `controllers.loaded.forward_velocity_controller` (spawner race with
  `joint_trajectory_controller`). Rerun passes. Round 8 (lifecycle-aware
  launcher) is the right place to make this deterministic via
  wait-for-activation logic rather than relying on spawner ordering.
- The xacro's `control_mode` property is currently unused by the block
  selection (both modes produce the same ros2_control XML; interface
  availability is decided by the controller activation in the launch).
  Kept as a knob for future conditional interfaces (e.g. dropping
  `effort` command on simulator=ignition once we replace the old sed
  patch with a proper chained controller).

Notes:
The xacro emits a single `<ros2_control>` block per URDF; grep-counting
`<ros2_control` on the generated XML returns 2 because it matches both
the open and close tags.

---

## Round 4 — Per-ur_type parameterization                    2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (effort 19/19, position 18/18)
Targets: #9

Changes:
- Added `src/ur_sim_config/config/ur_types/` with `_defaults.yaml`
  (home_positions + pid_hold) and one file per UR variant
  (ur3, ur3e, ur5, ur5e, ur7e, ur8long, ur10, ur10e, ur12e, ur15,
  ur16e, ur18, ur20, ur30) carrying per-joint `effort_limits` [Nm].
- New shared loader `scripts/ur_type_loader.py` (deep-merges defaults
  with per-ur_type overrides; installed as +x and resolvable from
  `lib/ur_sim_config`).
- `scripts/gravity_compensation.py`: added `ur_type` parameter (default
  `ur5e`); replaced hardcoded `effort_limits`, `kp/ki/kd`,
  `HOME_POSITIONS` constants with values pulled from the YAML loader
  (UR5e-ish fallback preserved for safety). Logs now include
  `ur_type=<type>, effort_limits=[...]` at startup.
- `scripts/generate_mujoco_model.sh`: effort-mode `<motor>` actuators
  now read `ctrlrange` per joint from `config/ur_types/<ur_type>.yaml`
  (sourced through a temporary env file). Position-mode actuators are
  unchanged (not torque-bound).
- Launches (`ur_sim_mujoco.launch.py`, `ur_sim_effort.launch.py`):
  forward the resolved `ur_type` to the gravity_compensation node via
  its parameter list.
- `CMakeLists.txt`: installs `ur_type_loader.py` alongside the other
  scripts; `config/ur_types/` is picked up by the existing recursive
  `install(DIRECTORY config/)`.
- Smoke test (`scripts/smoke_test.sh`):
  - Bumped broadcaster topic-availability timeout from 3s to 6s
    (eliminates a startup race flake under heavy colcon rebuild).
  - Added effort-mode-only assertion
    `ur_type.gravity_compensation_loaded_config` that confirms the
    gravity_compensation node logged a `ur_type=` startup line.

Follow-ups (new issues surfaced):
- Per-ur_type `joint_limits.yaml` / URDF reach/mass still comes from
  `Universal_Robots_ROS2_Description` defaults; reach/payload
  parameterization is really Round 5's job (URDF overhaul).
- MJCF `<position>` actuator gains are still one-size-fits-all in
  position mode; if per-ur_type stiffness tuning ever matters,
  extend `_defaults.yaml` with a `position_actuators:` block.

Notes:
PID gains in `_defaults.yaml` mirror the previously-hardcoded UR5e
values so the baseline behavior does not change for UR5/UR5e users.
Larger variants (UR15/UR20/UR30) inherit the same gains for now;
gravity_compensation already drives the dominant term via Pinocchio,
so residual PID correction scales with actual tracking error.

---

## Round 3 — Stub broadcasters for driver parity             2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (18/18 position and effort)
Targets: #1

Changes:
- Added `src/ur_sim_config/scripts/sim_broadcasters.py`: Python node that
  publishes the topics the real-UR `ur_controllers` broadcasters would emit,
  but without requiring matching ros2_control state interfaces (interim
  workaround; full hardware-side state interfaces deferred):
    * `/speed_scaling_state_broadcaster/speed_scaling` (Float64, 1.0)
    * `/force_torque_sensor_broadcaster/wrench` (WrenchStamped, zeros)
    * `/tcp_pose_broadcaster/pose` (PoseStamped, computed from TF base→tool0)
    * `/io_and_status_controller/program_running` (Bool, true)
    * `/io_and_status_controller/io_states` (ur_msgs/IOStates) — if pkg present
    * `/io_and_status_controller/robot_mode` (ur_dashboard_msgs/RobotMode) — if pkg present
    * `/io_and_status_controller/safety_mode` (ur_dashboard_msgs/SafetyMode) — if pkg present
- Both launch files start the node; MuJoCo and Gazebo effort both benefit.
- Smoke test: 4 new `topics.*` assertions (one message must appear within 3 s
  on each of speed_scaling, wrench, tcp_pose, program_running).
- Smoke test: refined `stability.no_component_crash` — spawners are one-shot
  and their non-zero exit is not a crash; only persistent-component deaths
  and segfaults now fail the check (previously a spawner race produced a
  false-positive FAIL).
- CMakeLists.txt: install sim_broadcasters.py to `lib/ur_sim_config` for
  `ros2 pkg executables` / launch `Node(executable=...)` discovery.

Follow-ups:
- These are stub publishers, not true ros2_control broadcasters. A future
  round should expose the corresponding state interfaces in the hardware
  plugin and register the real `ur_controllers/*` broadcasters (#1).
- `force_mode_controller`, `freedrive_mode_controller`,
  `friction_model_controller`, `tool_contact_controller`,
  `ur_configuration_controller`, `passthrough_trajectory_controller` remain
  missing — they are services/actions that require dedicated implementations
  and are deferred.
- CMake `--symlink-install` does not chmod +x the source script; forgetting
  the source-side +x will silently break the install symlink. Noted for
  future script additions.

Notes:
Position mode & effort mode both green. Smoke now: 18 checks pass.

---

## Round 2 — MuJoCo position command interface              2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (14/14 in both position and effort modes)
Targets: #2, #3 (partial)

Changes:
- `generate_mujoco_model.sh` now emits `<position>` actuators when
  `CONTROL_MODE=position` (with per-joint kp/kv) and `<motor>` actuators
  when `CONTROL_MODE=effort`. MJCF is cached per-(ur_type, control_mode).
- `ur_sim_mujoco.launch.py` now:
  - resolves MJCF path to `mujoco/<ur_type>/<control_mode>/ur_scene.xml`
    so switching modes does not reuse the wrong actuators;
  - activates `joint_trajectory_controller` in position mode (and the
    `forward_effort_controller` becomes inactive);
  - skips `gravity_compensation.py` in position mode (MuJoCo handles
    gravity natively through the position actuators).
- Smoke test: new `controllers.active.mode.*` assertion — in position mode
  JTC must be `active`, in effort mode `forward_effort_controller` must be
  `active`. This would have caught the pre-R2 silent failure where
  "position mode" actually ran through the gravity-comp effort path.
- README: clarified Position Mode description (MuJoCo backend behaviour).

Follow-ups:
- The gravity_compensation node is still an out-of-band actuator in effort
  mode (#3). It will be promoted to a chained controller or in-hardware
  shim in a later round.
- `forward_position_controller` / `forward_velocity_controller` still loaded
  as inactive placeholders; they currently can't claim the single-mode
  MuJoCo hardware. Proper multi-interface claim is deferred.

Notes:
Position mode now does what it says on the tin: `ros2 action send_goal
/joint_trajectory_controller/follow_joint_trajectory ...` reaches the arm
through JTC instead of the out-of-band gravity-comp path.

---

## Round 1 — Honest controller set                          2026-04-22  [complete]
Commit: uncommitted
Smoke:  PASS (13/13)
Targets: #1, #2 (partial), #13

Changes:
- Rewrote `src/ur_sim_config/config/ur_effort_controllers.yaml` to mirror
  upstream `ur_controllers.yaml` schema (joint/scaled JTC with constraints;
  forward position/velocity/effort with explicit `interface_name`).
- Added `scaled_joint_trajectory_controller` (ur_controllers pkg) as a loaded-
  but-inactive controller in both MuJoCo and Gazebo launches. MoveIt2 on real
  UR targets this controller by default; sim now exposes it for parity.
- Strengthened `scripts/smoke_test.sh`: asserts every controller in the
  canonical set is loaded, and that `joint_state_broadcaster` is *active*.
- Updated README "Expected controllers by mode" to reflect reality (active
  vs. inactive columns; correct controller counts).
- Fixed `scripts/smoke_test.sh` baseline bugs discovered during R0 run:
  dropped `set -u` (conflicts with ROS setup.bash) and strip ANSI color
  escapes from `ros2 control list_controllers` output.

Follow-ups:
- `joint_trajectory_controller` / `scaled_joint_trajectory_controller` still
  cannot *activate* in MuJoCo (hardware only claims effort). Fixed in R2.
- Broadcasters (`speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster`,
  `tcp_pose_broadcaster`, `io_and_status_controller`) remain deferred to R3.

Notes:
Baseline post-R0 smoke was 7/7. After R1 assertions: 13/13 PASS.

---

## Round 0 — Baseline & guardrails                          2026-04-22  [complete]
Commit: uncommitted
Smoke:  N/A (baseline script added, see next invocation)
Targets: scaffolding only

Changes:
- Added `docs/SHORTCOMINGS.md` with 13 catalogued gaps vs. upstream UR driver.
- Added `docs/IMPROVEMENT_PLAN.md` with 10 scoped rounds.
- Added `docs/ITERATION_LOG.md` (this file).
- Added `scripts/smoke_test.sh` for automated pass/fail verification.
- Added `.github/copilot-instructions.md` with iteration rules for the agent.

Follow-ups:
- None; Round 1 is queued.

Notes:
Baseline stack *does* come up (MuJoCo, ur15, position mode) and publishes
`/joint_states` at ~500 Hz. The `launch_all.sh` wait-loop is finicky when
stale processes linger — tracked as shortcoming #10.
