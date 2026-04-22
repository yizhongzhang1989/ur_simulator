# UR Simulator — Multi-Round Improvement Plan

This plan turns [`SHORTCOMINGS.md`](SHORTCOMINGS.md) into discrete, reviewable
rounds. Each round is:

1. **Scoped** to one theme so diffs stay reviewable.
2. **Gated** by the smoke-test in [`scripts/smoke_test.sh`](../scripts/smoke_test.sh) —
   no round is marked "done" until smoke passes.
3. **Logged** in [`ITERATION_LOG.md`](ITERATION_LOG.md) with a per-round entry.

The intent is that the agent executes rounds one at a time, stops, reports,
and waits for review before starting the next.

---

## Guiding principles

- **Prefer upstream conventions.** Align with `Universal_Robots_ROS2_Driver`
  naming, YAML keys, and topic/service surface so real-robot clients
  (MoveIt2, `ur_robot_driver` tests, CRISP) work without sim-specific forks.
- **One source of truth.** Consolidate `ur_type`-dependent constants (effort
  limits, PID seeds, home pose, joint names) into one YAML, read by every
  component.
- **Never regress the smoke test.** Each round ends with a green run.
- **Documentation follows code.** Update README / crisp_plan on the same
  commit, never later.
- **No new feature before removing a hack.** Each round either deletes a
  shortcoming or hard-blocks on one; no cosmetic work on top of bad scaffolding.

---

## Rounds

### Round 0 — Baseline & guardrails (this commit)

Deliverables (no runtime behavior change):

- [x] `docs/SHORTCOMINGS.md` — baseline gap analysis.
- [x] `docs/IMPROVEMENT_PLAN.md` — this file.
- [x] `docs/ITERATION_LOG.md` — empty running log.
- [x] `scripts/smoke_test.sh` — scripted pass/fail verifier (joint_states
      publishing, expected controllers loaded, no crashes for N seconds).
- [x] `.github/copilot-instructions.md` — rules the agent must follow while
      iterating (link plan, update log, never skip smoke, etc.).

Exit criterion: smoke test runs on the *current* code and reports the
**baseline** pass/fail per check — even if some checks fail, the script must
exit cleanly and emit a structured summary.

---

### Round 1 — Honest controller set, v1

Target shortcomings: **#1, #2, #13.**

Deliverables:

- Replace `ur_effort_controllers.yaml` with one aligned to upstream
  `ur_controllers.yaml` schema (keep only controllers whose interfaces
  actually exist today):
  - `joint_state_broadcaster`
  - `joint_trajectory_controller`
  - `scaled_joint_trajectory_controller` (with empty `speed_scaling_interface_name`, matching `ur_simulation_gz`)
  - `forward_position_controller`, `forward_velocity_controller`, `forward_effort_controller`
- Decide per-sim/per-mode **which controller is activated vs. stopped**, and
  make the launch spawner list match the README table exactly.
- Fix README "Expected controllers by mode" table to match reality.
- Smoke test gains controller-set assertions.

Out of scope: `io_and_status_controller`, broadcasters needing new state
interfaces — that's Round 3.

---

### Round 2 — MuJoCo position command interface

Target shortcomings: **#2, #3.**

Deliverables:

- Expose `position` command interface in the MuJoCo hardware, either by:
  (a) adding MJCF `position`/`intvelocity` actuators alongside `motor`, or
  (b) implementing a chained controller that converts JTC position output to
  effort via gravity comp + PID inside ros2_control (not a loose node).
- `joint_trajectory_controller` can be activated in MuJoCo position mode and
  actually drives the arm. `gravity_compensation.py` retained only as the
  effort-mode feedforward.
- Web-dashboard jog works via JTC in position mode and via gravity-comp path
  in effort mode.
- Smoke test: trajectory goal sent to JTC → joint positions converge within
  tolerance within N seconds.

---

### Round 3 — Stub broadcasters for driver parity

Target shortcomings: **#1.**

Deliverables:

- Add a `sim_state_broadcaster` hardware component (or extra state
  interfaces in the existing hardware) exposing:
  - `speed_scaling/speed_scaling_factor` (constant 1.0 unless user pubs)
  - `tcp_fts_sensor/force.*`, `torque.*` (zeros in sim by default)
  - `gpio/*` flags (stubbed)
  - `tcp_pose/*` (wired from FK in the hardware)
- Activate `speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster`,
  `tcp_pose_broadcaster`, `io_and_status_controller` against those interfaces.
- Real-UR ROS 2 clients (MoveIt2, URSim tests) can connect without errors.
- Smoke test: each new topic/service present & publishing.

---

### Round 4 — Per-`ur_type` parameterization

Target shortcomings: **#9.**

Deliverables:

- New `config/ur_types/<ur_type>.yaml` each holding:
  effort limits (base + wrist), initial/home pose, PID seeds, update rate,
  TCP frame name, payload defaults.
- `gravity_compensation.py` + launch + MJCF generator read from this single
  file.
- UR20/UR30 no longer limited to UR5e effort numbers.

---

### Round 5 — URDF generation overhaul

Target shortcomings: **#4, #7.**

Deliverables:

- Replace `generate_effort_urdf.sh` sed pipeline with an xacro macro in
  `ur_sim_config/urdf/ur_sim.urdf.xacro` that takes `ur_type`, `simulator`,
  `control_mode`, `controllers_file`, `safety_limits` and emits the right
  `<ros2_control>` block.
- Unify MuJoCo launch (drop inline Python regex block builder).
- `safety_limits` args are actually passed to upstream xacro.

---

### Round 6 — MJCF quality

Target shortcomings: **#5.**

Deliverables:

- Hand-authored MJCF per `ur_type` (seed from `mujoco_menagerie/universal_robots_ur*`).
- Proper collision geometry — robot can self-collide & interact with world.
- Drop shoulder quat / actuatorfrcrange / mesh-flattening hacks.
- Gravity-comp PIDs retuned against real meshes.

---

### Round 7 — `tf_prefix` + multi-robot

Target shortcoming: **#6.**

Deliverables:

- `tf_prefix` arg threaded through launch, controllers YAML, gravity comp,
  dashboard URDF generation.
- Two-UR demo launch (`launch_all.sh --robots 2`) validates namespacing.

---

### Round 8 — Launcher & reliability

Target shortcomings: **#10, #8.**

Deliverables:

- Replace `launch_all.sh` with a Python launch that owns lifecycle, so
  component crashes tear the stack down properly.
- Wait-for-ready based on controller activation (not topic grep).
- Doc note + `scripts/setup_rt.sh` for FIFO RT priority (optional).
- Graceful shutdown on SIGINT.

---

### Round 9 — Dashboard upgrades

Target shortcoming: **#12.**

Deliverables:

- Controller switcher (list/activate/deactivate via `controller_manager` services).
- Mode indicator, active controller chip.
- TCP pose read from `/tcp_pose_broadcaster/pose` instead of browser FK.
- I/O panel wired to `io_and_status_controller` (Round 3).

---

### Round 10 — CI & regression gate

Target shortcoming: **#11.**

Deliverables:

- GitHub Actions workflow running `colcon build` + `scripts/smoke_test.sh` in
  a ROS 2 Humble container with `mujoco_ros2_control` deps.
- Branch protection / required check documented.

---

## How rounds are tracked

- Active round is the one with a line in [`ITERATION_LOG.md`](ITERATION_LOG.md)
  whose `status` is `in-progress`.
- Completing a round requires:
  1. `scripts/smoke_test.sh` prints `RESULT: PASS`.
  2. New smoke-test assertions added for whatever was promised.
  3. An entry appended to `ITERATION_LOG.md` with the commit hash and a
     one-paragraph summary.
  4. README / `crisp_plan.md` updated if user-facing behavior changed.

- Rounds may be resequenced, but a round that depends on an earlier round's
  deliverable cannot start until that deliverable is logged complete.
