#!/usr/bin/env bash
# UR Simulator smoke test.
#
# Brings up the stack, verifies the minimum guarantees, and tears it down.
# Exit code 0 iff RESULT: PASS is printed.
#
# Usage:
#   scripts/smoke_test.sh [--simulator mujoco|gazebo] [--control_mode position|effort]
#                         [--ur-type ur5e] [--boot-timeout 45] [--hold 5]
#
# Structured output at the end:
#   RESULT: PASS|FAIL
#   CHECKS: <json-ish summary>
#
# The script is intentionally self-contained — it does NOT source any colcon
# workspace automatically beyond `install/setup.bash` in the repo root, so CI
# can re-use it verbatim.

set -o pipefail
# Deliberately NOT using `set -u` — ROS's setup.bash references unset vars.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

SIMULATOR="mujoco"
CONTROL_MODE="position"
UR_TYPE=""
BOOT_TIMEOUT=45
HOLD_SECONDS=5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --simulator)     SIMULATOR="$2";    shift 2;;
    --control_mode)  CONTROL_MODE="$2"; shift 2;;
    --ur-type)       UR_TYPE="$2";      shift 2;;
    --boot-timeout)  BOOT_TIMEOUT="$2"; shift 2;;
    --hold)          HOLD_SECONDS="$2"; shift 2;;
    -h|--help)
      sed -n '2,30p' "$0"; exit 0;;
    *)
      echo "smoke_test: unknown arg '$1'" >&2; exit 2;;
  esac
done

LOG_DIR="$(mktemp -d -t ur_smoke.XXXXXX)"
LAUNCH_LOG="$LOG_DIR/launch.log"
echo "[smoke] log dir: $LOG_DIR"

# ---------- helpers ----------
pass=()
fail=()

record() {
  local name="$1"; local ok="$2"; local detail="${3:-}"
  if [[ "$ok" == "1" ]]; then
    pass+=("$name")
    echo "  PASS  $name $detail"
  else
    fail+=("$name:$detail")
    echo "  FAIL  $name $detail"
  fi
}

cleanup() {
  echo "[smoke] cleaning up..."
  pkill -f "launch_all.sh"          2>/dev/null || true
  pkill -f "ros2 launch ur_sim"     2>/dev/null || true
  pkill -f "mujoco_ros2_control"    2>/dev/null || true
  pkill -f "ros2_control_node"      2>/dev/null || true
  pkill -f "gravity_compensation"   2>/dev/null || true
  pkill -f "rosbridge_websocket"    2>/dev/null || true
  pkill -f "ur_web_dashboard/server.py" 2>/dev/null || true
  pkill -f "ign gazebo"             2>/dev/null || true
  sleep 2
}
trap cleanup EXIT INT TERM

# ---------- pre-checks ----------
echo "[smoke] pre-checks"
command -v ros2  >/dev/null 2>&1 && record "env.ros2_present"  1 \
                                 || record "env.ros2_present"  0 "ros2 not on PATH"

[[ -f "$WS_DIR/install/setup.bash" ]] && record "env.ws_built" 1 \
                                      || record "env.ws_built" 0 "install/setup.bash missing — run colcon build"

# R5: the unified URDF xacro must be installed. generate_effort_urdf.sh was
# removed in R5 — flag its reappearance too so nobody resurrects the
# sed pipeline accidentally.
XACRO_PATH="$WS_DIR/install/ur_sim_config/share/ur_sim_config/urdf/ur_sim.urdf.xacro"
LEGACY_SH="$WS_DIR/install/ur_sim_config/share/ur_sim_config/urdf/generate_effort_urdf.sh"
if [[ -f "$XACRO_PATH" && ! -f "$LEGACY_SH" ]]; then
  record "urdf.ur_sim_xacro_installed" 1
else
  if [[ ! -f "$XACRO_PATH" ]]; then
    record "urdf.ur_sim_xacro_installed" 0 "missing $XACRO_PATH"
  else
    record "urdf.ur_sim_xacro_installed" 0 "legacy generate_effort_urdf.sh is still installed"
  fi
fi

# Source after check so subsequent ros2 calls have our overlays.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

# ---------- boot ----------
cleanup   # flush anything leftover from prior runs
echo "[smoke] starting stack: simulator=$SIMULATOR control_mode=$CONTROL_MODE"

LAUNCH_ARGS=(--simulator "$SIMULATOR" --control_mode "$CONTROL_MODE")
# UR_TYPE override goes through config.yaml; keep it simple and trust config.

( cd "$WS_DIR" && ./launch_all.sh "${LAUNCH_ARGS[@]}" ) >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "[smoke] launch_all.sh pid=$LAUNCH_PID"

# Wait until /joint_states has >=1 publisher and at least one message.
booted=0
for ((i=0; i<BOOT_TIMEOUT; i++)); do
  if ros2 topic list 2>/dev/null | grep -q '^/joint_states$'; then
    if timeout 2 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
      booted=1
      break
    fi
  fi
  sleep 1
done

record "boot.joint_states_publishing" "$booted" \
       "$( ((booted)) && echo "" || echo "timed out after ${BOOT_TIMEOUT}s; see $LAUNCH_LOG" )"

if [[ "$booted" != "1" ]]; then
  tail -n 40 "$LAUNCH_LOG" | sed 's/^/[launch] /'
  echo "RESULT: FAIL"
  echo "CHECKS: pass=${#pass[@]} fail=${#fail[@]}"
  exit 1
fi

# ---------- runtime checks ----------
echo "[smoke] runtime checks"

# joint_states must keep flowing for HOLD seconds
hz_ok=0
if timeout "$((HOLD_SECONDS+3))" ros2 topic hz /joint_states >"$LOG_DIR/hz.log" 2>&1 & then :; fi
HZ_PID=$!
sleep "$HOLD_SECONDS"
kill -INT "$HZ_PID" 2>/dev/null || true
wait "$HZ_PID" 2>/dev/null || true
if grep -Eq 'average rate:\s*[0-9]+\.' "$LOG_DIR/hz.log"; then
  hz_rate=$(grep -E 'average rate:' "$LOG_DIR/hz.log" | tail -1 | awk '{print $3}')
  hz_ok=1
else
  hz_rate="n/a"
fi
record "runtime.joint_states_hz" "$hz_ok" "rate=$hz_rate"

# controllers present — baseline minimum. Wait until the expected set is
# loaded OR up to ~15s elapse (spawners are async; race-sensitive).
_expected_ctrls=(joint_state_broadcaster joint_trajectory_controller \
                 scaled_joint_trajectory_controller forward_position_controller \
                 forward_velocity_controller forward_effort_controller)
for ((_i=0; _i<15; _i++)); do
  controllers_raw="$(timeout 5 ros2 control list_controllers 2>/dev/null || true)"
  controllers_text="$(printf '%s' "$controllers_raw" | sed -e 's/\x1B\[[0-9;]*[A-Za-z]//g')"
  _all_loaded=1
  for c in "${_expected_ctrls[@]}"; do
    echo "$controllers_text" | grep -q "^$c[[:space:]]" || { _all_loaded=0; break; }
  done
  (( _all_loaded )) && break
  sleep 1
done
echo "$controllers_text" > "$LOG_DIR/controllers.txt"

have_ctrl() { echo "$controllers_text" | grep -q "^$1[[:space:]]"; }
for c in joint_state_broadcaster; do
  have_ctrl "$c" && record "controllers.$c"  1 || record "controllers.$c" 0 "not loaded"
done

# R1: full expected controller set (loaded; active state checked separately).
# Every controller below must be loaded for upstream-driver parity.
for c in joint_trajectory_controller \
         scaled_joint_trajectory_controller \
         forward_position_controller \
         forward_velocity_controller \
         forward_effort_controller; do
  have_ctrl "$c" && record "controllers.loaded.$c" 1 \
                 || record "controllers.loaded.$c" 0 "missing from list_controllers"
done

# R1: joint_state_broadcaster MUST be active at runtime.
is_active() { echo "$controllers_text" | grep -E "^$1[[:space:]].*[[:space:]]active" > /dev/null; }
if is_active joint_state_broadcaster; then
  record "controllers.active.joint_state_broadcaster" 1
else
  record "controllers.active.joint_state_broadcaster" 0 "loaded but not active"
fi

# R2: mode-dependent active controller.
#   position mode -> joint_trajectory_controller active.
#   effort mode   -> forward_effort_controller active.
if [[ "$CONTROL_MODE" == "position" ]]; then
  if is_active joint_trajectory_controller; then
    record "controllers.active.mode.joint_trajectory_controller" 1
  else
    record "controllers.active.mode.joint_trajectory_controller" 0 \
           "position mode but JTC not active"
  fi
else
  if is_active forward_effort_controller; then
    record "controllers.active.mode.forward_effort_controller" 1
  else
    record "controllers.active.mode.forward_effort_controller" 0 \
           "effort mode but forward_effort_controller not active"
  fi
fi

# At least one motion controller loaded (active or inactive)
any_motion=0
for c in joint_trajectory_controller scaled_joint_trajectory_controller \
         forward_position_controller forward_velocity_controller forward_effort_controller; do
  if have_ctrl "$c"; then any_motion=1; fi
done
record "controllers.any_motion_controller_loaded" "$any_motion" \
       "$( ((any_motion)) && echo "" || echo "no JTC / forward_* controller in list_controllers" )"

# No persistent-component crash during hold.
# Spawners are expected one-shot processes; their exit is NOT a crash — if a
# spawner failed, the corresponding controller would not be loaded/active and
# the earlier controllers.* checks already cover that.
crash_lines="$(grep -iE 'terminated abnormally|segmentation fault' "$LAUNCH_LOG" || true)"
died_lines="$(grep -E 'process has died' "$LAUNCH_LOG" | grep -vE 'spawner-|spawner_' || true)"
if [[ -n "$crash_lines" || -n "$died_lines" ]]; then
  record "stability.no_component_crash" 0 "see $LAUNCH_LOG"
else
  record "stability.no_component_crash" 1
fi

# R3: upstream-driver topic-parity broadcasters must publish.
check_topic_pub() {
  local topic="$1"
  if timeout 6 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
    record "topics.$topic" 1
  else
    record "topics.$topic" 0 "no message within 6s"
  fi
}
check_topic_pub /speed_scaling_state_broadcaster/speed_scaling
check_topic_pub /force_torque_sensor_broadcaster/wrench
check_topic_pub /tcp_pose_broadcaster/pose
check_topic_pub /io_and_status_controller/program_running

# R6: generated MJCF for the active ur_type/mode must have collision
# geometry (visual meshes go in group=2 and are contype=0; collision
# meshes are in group=3 and participate in contact). This protects against
# a future cleanup accidentally re-disabling all collisions.
_ur_type="$UR_TYPE"
if [[ -z "$_ur_type" && -f "$WS_DIR/config/config.yaml" ]]; then
  _ur_type="$(awk '/^ur_type:/ {print $2}' "$WS_DIR/config/config.yaml" | tr -d '\r\"'"'")"
fi
MJCF_FILE="$WS_DIR/install/ur_sim_config/share/ur_sim_config/mujoco/${_ur_type}/$CONTROL_MODE/ur_robot.xml"
if [[ "$SIMULATOR" == "mujoco" && -f "$MJCF_FILE" ]]; then
  if grep -q 'group="3"' "$MJCF_FILE"; then
    record "mjcf.collision_geoms_present" 1
  else
    record "mjcf.collision_geoms_present" 0 \
           "no group=\"3\" geoms in $MJCF_FILE — all meshes marked visual-only"
  fi
fi
# The log line includes "ur_type=<type>". Any ur_type-aware startup counts.
if [[ "$CONTROL_MODE" == "effort" ]]; then
  if grep -qE 'gravity_compensation.*ur_type=' "$LAUNCH_LOG"; then
    record "ur_type.gravity_compensation_loaded_config" 1
  else
    record "ur_type.gravity_compensation_loaded_config" 0 \
           "no 'ur_type=...' line from gravity_compensation"
  fi
fi

# R7: tf_prefix="" default must still produce the canonical (unprefixed)
# upstream joint-name set in /joint_states. This is the hard rule from
# copilot-instructions.md that joint_states joint order / names never
# change for the default tf_prefix.
JS_SNAPSHOT="$(timeout 4 ros2 topic echo --once /joint_states 2>/dev/null || true)"
_missing=""
for _j in shoulder_pan_joint shoulder_lift_joint elbow_joint \
          wrist_1_joint wrist_2_joint wrist_3_joint ; do
  if ! grep -qw "$_j" <<<"$JS_SNAPSHOT"; then
    _missing+="$_j "
  fi
done
if [[ -z "$_missing" ]]; then
  record "tf_prefix.default_joint_names_unchanged" 1
else
  record "tf_prefix.default_joint_names_unchanged" 0 \
         "missing from /joint_states: $_missing"
fi

# R8: reliability — no loaded controller is in an error state. If any
# controller listed by `ros2 control list_controllers` has state other
# than `active` or `inactive`, the stack is not cleanly usable.
_bad_ctrl_states="$(echo "$controllers_text" \
  | awk 'NF>=3 && $3!="active" && $3!="inactive" { print $1"("$3")" }' \
  || true)"
if [[ -z "$_bad_ctrl_states" ]]; then
  record "reliability.no_controllers_in_error" 1
else
  record "reliability.no_controllers_in_error" 0 \
         "bad states: $_bad_ctrl_states"
fi

# R8: the setup_rt.sh helper must exist and be executable.
if [[ -x "$WS_DIR/scripts/setup_rt.sh" ]]; then
  record "reliability.setup_rt_script_present" 1
else
  record "reliability.setup_rt_script_present" 0 \
         "scripts/setup_rt.sh missing or not executable"
fi

# R9: the dashboard's controller switcher relies on
# /controller_manager/switch_controller; the TCP pose panel relies on
# /tcp_pose_broadcaster/pose (already covered by topics.*). Verify the
# service is reachable AND the dashboard source references it.
if ros2 service list 2>/dev/null | grep -q '^/controller_manager/switch_controller$'; then
  record "dashboard.switch_controller_service_present" 1
else
  record "dashboard.switch_controller_service_present" 0 \
         "/controller_manager/switch_controller not advertised"
fi
if grep -q "switch_controller" "$WS_DIR/src/ur_web_dashboard/index.html" \
   && grep -q "tcp_pose_broadcaster/pose" "$WS_DIR/src/ur_web_dashboard/index.html"; then
  record "dashboard.wired_to_upstream_topics_services" 1
else
  record "dashboard.wired_to_upstream_topics_services" 0 \
         "dashboard index.html does not reference switch_controller or tcp_pose_broadcaster"
fi

# R10: CI workflow must be present in the repo. We don't run the
# workflow here (it runs the smoke test itself), but its absence
# silently removes the regression gate.
if [[ -f "$WS_DIR/.github/workflows/ci.yml" ]] \
   && grep -q "smoke_test.sh" "$WS_DIR/.github/workflows/ci.yml"; then
  record "ci.workflow_present" 1
else
  record "ci.workflow_present" 0 \
         ".github/workflows/ci.yml missing or does not invoke smoke_test.sh"
fi

# ---------- result ----------
total=$(( ${#pass[@]} + ${#fail[@]} ))
echo ""
echo "--- smoke summary ---"
echo "passed : ${#pass[@]} / $total"
echo "failed : ${#fail[@]}"
for f in "${fail[@]}"; do echo "  - $f"; done

if [[ ${#fail[@]} -eq 0 ]]; then
  echo "RESULT: PASS"
  echo "CHECKS: pass=${#pass[@]} fail=0 log=$LOG_DIR"
  exit 0
else
  echo "RESULT: FAIL"
  echo "CHECKS: pass=${#pass[@]} fail=${#fail[@]} log=$LOG_DIR"
  exit 1
fi
