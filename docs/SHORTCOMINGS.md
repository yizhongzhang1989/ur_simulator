# UR Simulator — Deep Shortcomings Analysis

_Last updated: 2026-04-22. Owner: maintainers. Status: baseline analysis, pre-iteration._

This document is the single source of truth for known gaps between this simulator
and a real Universal Robots arm driven by the upstream ROS 2 driver
([`Universal_Robots_ROS2_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver),
[`Universal_Robots_ROS2_GZ_Simulation`](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation)).
Each item has a severity, a concrete observation from this repo, a reference
behavior expected on real hardware, and a suggested fix strategy.

Severity scale:

- **S1 — Blocker:** user-facing feature is broken or silently wrong.
- **S2 — Major:** significant behavior deviation from real UR; external
  controllers (MoveIt, CRISP, ur_robot_driver clients) may break.
- **S3 — Minor:** quality/robustness/UX issue, but system functions.

---

## Baseline evidence (what runs today)

Last verified run (MuJoCo, position mode, `ur_type: ur15`): the stack does come
up. Observed controllers after activation:

| Controller                    | Type                                              | State    |
|-------------------------------|---------------------------------------------------|----------|
| `joint_state_broadcaster`     | `joint_state_broadcaster/JointStateBroadcaster`   | active   |
| `forward_effort_controller`   | `effort_controllers/JointGroupEffortController`   | active   |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | inactive |
| `forward_position_controller` | `position_controllers/JointGroupPositionController` | inactive |
| `forward_velocity_controller` | `velocity_controllers/JointGroupVelocityController` | inactive |

`gravity_compensation.py` is what actually drives the arm via
`/forward_effort_controller/commands`; `joint_trajectory_controller` is loaded
but never activated in MuJoCo mode, so `ros2 control list_controllers` does
not match what the README claims ("position mode uses JTC").

---

## 1. Missing controllers vs. real UR driver (S2)

Upstream [`ur_robot_driver/config/ur_controllers.yaml`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/config/ur_controllers.yaml)
registers a much richer set. Missing here:

| Controller                           | Type                                                | Impact                                       |
|--------------------------------------|-----------------------------------------------------|----------------------------------------------|
| `scaled_joint_trajectory_controller` | `ur_controllers/ScaledJointTrajectoryController`    | MoveIt2 default on real UR; clients that target it fail. |
| `io_and_status_controller`           | `ur_controllers/GPIOController`                     | No digital/analog I/O, no robot_mode, safety_mode. |
| `speed_scaling_state_broadcaster`    | `ur_controllers/SpeedScalingStateBroadcaster`       | No speed-scaling signal; scaled JTC cannot work. |
| `force_torque_sensor_broadcaster`    | `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster` | No `/force_torque_sensor_broadcaster/wrench`. |
| `tcp_pose_broadcaster`               | `pose_broadcaster/PoseBroadcaster`                  | No `/tcp_pose` topic (dashboard computes FK in-browser as workaround). |
| `passthrough_trajectory_controller`  | `ur_controllers/PassthroughTrajectoryController`    | No trajectory passthrough / time-scaled follow. |
| `force_mode_controller`              | `ur_controllers/ForceModeController`                | No `force_mode` service (URScript parity). |
| `freedrive_mode_controller`          | `ur_controllers/FreedriveModeController`            | No teach/freedrive entry. |
| `friction_model_controller`          | `ur_controllers/FrictionModelController`            | No friction compensation toggle. |
| `tool_contact_controller`            | `ur_controllers/ToolContactController`              | No tool-contact detection. |
| `ur_configuration_controller`        | `ur_controllers/URConfigurationController`          | No kinematics hash / payload service. |

Fix strategy: adopt upstream `ur_controllers.yaml` as the authoritative schema;
spawn broadcasters even if the underlying state interface is synthetic (stub out
`speed_scaling/speed_scaling_factor = 1.0`, zero-wrench for FT, identity payload).

## 2. Controller-vs-hardware interface mismatch (S1 in effort mode path)

`ur_sim_config/config/ur_effort_controllers.yaml` declares
`joint_trajectory_controller.command_interfaces: [position]`, but the MuJoCo
hardware plugin is activated with **effort-only** claim:

```
[MujocoSystemInterface]: Joint shoulder_pan_joint: effort control enabled (position, velocity disabled)
```

So JTC can never activate in MuJoCo mode — confirmed by spawner launching it
with `--inactive`. This contradicts README claims and is why "position mode"
silently runs through `gravity_compensation.py` instead of JTC. Either:

- (A) give MuJoCo a real position command interface (add `<position>` actuators
  in MJCF, expose both in the URDF hardware block, let `ros2_control` choose), or
- (B) drop the "position mode" marketing and document that MuJoCo is always
  effort-based, and route JTC commands through the effort shim as a first-class
  controller chain (chainable controller or custom JTC wrapper).

## 3. `gravity_compensation.py` is an out-of-band actuator (S2)

The node subscribes to `/joint_trajectory_controller/joint_trajectory` and
publishes to `/forward_effort_controller/commands`, bypassing ros2_control's
state machine entirely. Problems:

- No preemption, no action feedback, no goal tolerance, no trajectory monitor
  — real UR's `follow_joint_trajectory` action semantics are lost.
- Interpolation is a hand-rolled cubic ease; velocities and accelerations
  from the trajectory message are ignored.
- PID gains are hard-coded constants that do not match any specific UR model
  (same gains used for UR3 and UR20).
- If the user activates `forward_effort_controller` externally (CRISP),
  both this node and the external controller publish to the same topic —
  the README's "torques are added" claim is only true by coincidence of
  last-writer-wins publishing.
- Effort clamp uses UR5e-ish limits (150/28 Nm) regardless of `ur_type`.

Fix strategy: replace with either (a) a proper `ros2_control` chained
controller that combines gravity feedforward + JTC position error, or
(b) a custom hardware-interface layer in `mujoco_ros2_control` that exposes
both position and effort command interfaces and computes gravity inside the
simulator loop.

## 4. URDF patching via `sed` is fragile (S2)

[`urdf/generate_effort_urdf.sh`](../src/ur_sim_config/urdf/generate_effort_urdf.sh)
injects `<command_interface name="effort"/>` via line-addressed sed into xacro
output. It depends on exact formatting of upstream `ur.urdf.xacro` and breaks
silently if the upstream layout changes. It also overwrites all `effort="…"`
limits to `500` (breaking safety limits) and hard-codes friction/damping in
`sed` ranges.

Fix strategy: drive `ros2_control` block construction from a xacro macro we own
(as `ur_simulation_gz` does), parameterized on `control_mode`, `simulator`,
and `controllers_file`. The MuJoCo launch already does this inline with Python
regex ([`ur_sim_mujoco.launch.py`](../src/ur_sim_config/launch/ur_sim_mujoco.launch.py)
`_build_ros2_control_block`) — unify on one path.

## 5. MJCF generation hack pile (S2)

[`scripts/generate_mujoco_model.sh`](../src/ur_sim_config/scripts/generate_mujoco_model.sh)
does all of these by string replacement:

- Flattens `package://` mesh paths by stripping subdirectories.
- Disables **all** mesh collisions (`contype=0 conaffinity=0`) because "UR
  meshes overlap by design". This also means the robot cannot self-collide
  or touch anything in the world — contact forces are impossible.
- Patches out a 180° Z quat on `shoulder_link` that MuJoCo's URDF importer
  produces from the `base_link → base_link_inertia` fixed joint.
- Patches out `actuatorfrcrange` because it conflicts with motor `ctrlrange`.
- Injects damping/friction as string substitutions.

The MJCF is therefore visual-only for contacts, which makes it unusable for
anything touching manipulation research (grasping, pushing, contact-rich tasks).

Fix strategy: hand-author (or scripted-generate from the already-working
`mujoco_menagerie` UR models) an MJCF per ur_type with correct collision
geometry (capsules/convex decomposition), then only template in actuators.

## 6. No `tf_prefix` support (S2)

Upstream driver parameterizes every joint name and broadcaster with
`$(var tf_prefix)`. Nothing in this repo supports that. Two UR arms in the
same ROS graph will name-collide on everything.

## 7. No `safety_limits` / safety controller integration (S3→S2 if using effort)

`ur_sim_effort.launch.py` accepts `safety_limits`/`safety_pos_margin`/
`safety_k_position` args but never passes them through to the xacro.
URDF is generated without safety joint limits, so there is no soft-limit
velocity clamp — large effort commands can violate joint limits with only a
hard stop at URDF limits.

## 8. Real-time scheduling disabled (S3)

Log: `Could not enable FIFO RT scheduling policy: … (Operation not permitted)`.
Controller manager falls back to SCHED_OTHER, giving jittery 500 Hz cycles.
Needs a doc-note + optional `setcap`/`limits.conf` snippet.

## 9. Hardcoded robot defaults don't follow `ur_type` (S2)

- `gravity_compensation.py`: PID gains, effort limits, and home position are
  constants; do not adapt to `ur_type` (a UR20 needs ~1500 Nm base limits).
- `_build_ros2_control_block` hardcodes initial positions `[0, -1.57, 0, -1.57, 0, 0]`.
- MJCF actuator `ctrlrange` is hardcoded `±150 / ±28`.

Fix strategy: single YAML per `ur_type` holding effort limits, PID seeds,
home pose, control-loop rate; load in every component.

## 10. Reliability of `launch_all.sh` (S3)

- Relies on grepping `ros2 topic info ... | grep "Publisher count: 1"`; if a
  stale `controller_manager` from a previous run lingers, the count can be 2
  and the wait-loop silently falls through without the "Simulation ready"
  line ever printing (observed in today's `timeout 15s ./launch_all.sh` run).
- Only `pkill -f` cleanup; no wait-for-pid; races on fast re-launch.
- Runs everything as children of a single shell; one component crash
  (e.g. rosbridge port in use) doesn't teardown the others.

## 11. No automated verification or CI (S2)

No tests, no smoke test, no CI workflow. Every regression has to be caught by
a human remembering to re-run. Multi-round improvement work needs a
repeatable "does the stack come up and do the basic thing" check.

## 12. Dashboard correctness (S3)

- TCP pose is computed in the browser by urdf-loader FK (no `tcp_pose_broadcaster`).
- Joint jog publishes a `JointTrajectory` that in effort mode is consumed by
  `gravity_compensation.py`, not by `joint_trajectory_controller`, so any
  "stop / cancel" UI will not function.
- No controller switcher, no mode indicator, no I/O panel.

## 13. Docs vs reality drift (S3)

README's "Expected controllers by mode" table is out of date vs. the actual
spawned set (the table lists `forward_effort_controller + 3 inactive`, the
real list above has an additional `joint_trajectory_controller` inactive and
shows no `joint_state_broadcaster` count mismatch). `crisp_plan.md` item 7
("Test: build CRISP externally and connect to sim") is still unchecked.

## 14. Only one motion controller "looks alive" after launch (S3)

_Reported 2026-04-22 after running `./launch_all.sh` against the R9 build._

Observation: with the default launch the user only sees
`joint_trajectory_controller` acting on the robot, even though the
spawners load `forward_position_controller`, `forward_velocity_controller`,
`forward_effort_controller`, and (in effort mode) the gravity-compensation
driver. The only way to confirm the stack is working is to open the web
dashboard and watch `/joint_states` move when the jog sliders are used.

Root cause: this is expected ros2_control behaviour — a `position` or
`effort` command interface can only be claimed by one controller at a
time, so only the **default active** controller is "working" at any
moment. The others are loaded/inactive until the user activates them
(R9 added a switcher for this). The surprise is purely a UX / docs
gap, not a regression.

Gap to close:

- README should say explicitly that only the default controller is
  active, and that the others are activated by the dashboard's
  "→ JTC / → fwd_pos / → fwd_eff" buttons (R9) or via
  `ros2 control switch_controllers ...`.
- There is no one-command validation flow: today the user has to
  start the launcher, open a browser, and eyeball the dashboard. We
  already have `scripts/smoke_test.sh`, which launches the stack in
  the background, polls for the expected topics/controllers, and
  returns `RESULT: PASS/FAIL`. Documenting that as the canonical
  "is the sim actually alive?" check closes this item.

Severity S3 — no code change required beyond documentation; covered
by existing R9 (switcher) and R10 (CI) deliverables.

## 15. Stiff MuJoCo position actuators can explode on large steps (S2)

_Reported 2026-04-22 after R9 dashboard test pass._

Symptoms: In `control_mode=position` (MuJoCo backend), commanding a
large (>~0.5 rad) step change on `/forward_position_controller/commands`
while the arm is already far from that target makes the PID in the
`<position>` actuator drive the joint past its target, oscillate,
and eventually diverge into NaN / giant integers (observed
`wrist_3_joint ≈ -452 rad`, then the MuJoCo step loop stalls and
sim time freezes). After this, `joint_states` keeps publishing the
last good sample but no commands have any effect; only a full
relaunch recovers.

Root cause:

- `generate_mujoco_model.sh` emits `<position kp="2000" kv="100"/>`
  for the large joints and `kp=500 kv=25` for the wrist joints; this
  is stiffer than real UR hardware and has no explicit joint-level
  acceleration limit. Aggressive `forward_position_controller` step
| 15  | S2       | physics             | Stiff MuJoCo `<position>` actuators can diverge on large step commands and stall the sim.                 |
  commands therefore produce huge accelerations that the 2 ms
  MuJoCo step cannot integrate stably.
- `joint_trajectory_controller` / `scaled_joint_trajectory_controller`
  interpolate over `time_from_start` so the effective step per
  control cycle is tiny; they do not trigger this issue, which is
  why the dashboard's JTC path stays stable.

Workarounds already shipped (R9 dashboard fixes):

- Pre-seeding the target controller's command topic with the
  current joint positions before `switch_controller` eliminates the
  most common snap (activation with a zero command).
- The Send-Trajectory button now routes to the active controller,
  so users who prefer large step targets naturally stay on JTC.

Proper fix (landed 2026-04-22, R11):

- `generate_mujoco_model.sh` now emits lower, damping-ratio-balanced
  gains (`kp=1500 kv=200` for shoulders, `kp=1000 kv=120` for elbow,
  `kp=300 kv=40` for wrists) and adds `forcerange` per joint sized
  from the per-`ur_type` effort limit so a single bad step can no
  longer launch the wrist past its torque envelope.
- `ur_scene.xml` now sets `<option integrator="implicitfast"/>`,
  which is numerically stable with stiff `<position>` actuators at
  `timestep=0.001` (the previous implicit `RK4` default diverged).
- Repro script (`/tmp/repro_fwdpos_bug.py`): switch to
  `forward_position_controller`, send `shoulder_pan += 0.3 rad`
  step, then switch back to JTC / scaled-JTC — all switches now
  return `ok=True` and the arm tracks to within 3 mrad.

Remaining follow-ups (not blocking):

- Detect sim stall (no change in `/joint_states` header.stamp for
  >1 s while `/clock` also stalled) and surface that in the
  dashboard so the user knows to relaunch.
- Consider switching MuJoCo to integrated-velocity actuators or
  adding a rate-limit in front of `forward_position_controller`
  for even more robustness.

Severity S2 — was: "when it triggers the entire simulation is dead".
Now downgraded after R11 fix (sim stays stable under typical jog
commands). Track alongside shortcoming #5 (MJCF generation hack
pile).

---

## Summary table (priority-ordered)

| #   | Severity | Area                | One-liner                                              |
|-----|----------|---------------------|--------------------------------------------------------|
| 2   | S1       | control             | JTC declared but can never activate in MuJoCo.         |
| 1   | S2       | controllers         | 11 upstream UR controllers missing.                    |
| 3   | S2       | control             | `gravity_compensation.py` bypasses ros2_control.       |
| 4   | S2       | build               | URDF generated by fragile sed pipeline.                |
| 5   | S2       | physics             | MJCF has no collisions; hacky mesh/quat/force patches. |
| 6   | S2       | multi-robot         | No `tf_prefix` anywhere.                               |
| 9   | S2       | correctness         | Params don't adapt to `ur_type`.                       |
| 11  | S2       | CI/verification     | No smoke test, no regression signal.                   |
| 7   | S2→S3    | safety              | Safety limits plumbed in but unused.                   |
| 8   | S3       | realtime            | No RT scheduling guidance / setup.                     |
| 10  | S3       | launcher            | `launch_all.sh` is brittle.                            |
| 12  | S3       | dashboard           | No controller switcher, stale behavior in effort mode. |
| 13  | S3       | docs                | README / `crisp_plan.md` out of date.                  |
| 14  | S3       | docs / UX           | Only the default motion controller appears "alive"; activate others via dashboard or `switch_controllers`. |
