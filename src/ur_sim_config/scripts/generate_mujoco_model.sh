#!/bin/bash
# Generate MJCF model for UR robot from URDF using MuJoCo compile.
#
# Usage: generate_mujoco_model.sh <ur_type> <control_mode> <output_dir>
#   ur_type: ur5e, ur3, ur10e, etc.
#   control_mode: position or effort
#   output_dir: where to store the MJCF and meshes
#
# Produces: <output_dir>/ur_scene.xml (the top-level MJCF scene)

set -e

UR_TYPE="${1:?Usage: $0 <ur_type> <control_mode> <output_dir>}"
CONTROL_MODE="${2:-effort}"
OUTPUT_DIR="${3:?Usage: $0 <ur_type> <control_mode> <output_dir>}"

source /opt/ros/humble/setup.bash

COMPILE="/opt/ros/humble/opt/mujoco_vendor/bin/compile"
if [ ! -x "$COMPILE" ]; then
    echo "ERROR: MuJoCo compile tool not found at $COMPILE"
    exit 1
fi

UR_DESC_DIR="$(ros2 pkg prefix ur_description)/share/ur_description"
MESH_SRC="$UR_DESC_DIR/meshes/$UR_TYPE"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Generate clean URDF (no sim tags, no ros2_control)
URDF_TMP="$OUTPUT_DIR/ur_clean.urdf"
xacro "$UR_DESC_DIR/urdf/ur.urdf.xacro" \
    ur_type:="$UR_TYPE" \
    name:=ur \
    > "$URDF_TMP" 2>/dev/null

# Rewrite mesh paths from package:// to raw filenames (MuJoCo needs flat files)
sed -i "s|package://ur_description/meshes/${UR_TYPE}/[^/]*/||g" "$URDF_TMP"

# Copy collision meshes (STL) next to the URDF
cp "$MESH_SRC"/collision/*.stl "$OUTPUT_DIR/" 2>/dev/null || true

# Convert URDF to MJCF
MJCF_RAW="$OUTPUT_DIR/ur_robot.xml"
cd "$OUTPUT_DIR"
"$COMPILE" ur_clean.urdf ur_robot.xml 2>&1 | head -5

if [ ! -f "$MJCF_RAW" ]; then
    echo "ERROR: MJCF conversion failed"
    exit 1
fi

# Resolve per-ur_type effort limits from config/ur_types/<ur_type>.yaml
# (Falls back to UR5e-ish values if the loader can't find the file.)
eff_limits_file="$OUTPUT_DIR/effort_limits.env"
PKG_SHARE="$(ros2 pkg prefix ur_sim_config)/share/ur_sim_config"
python3 - "$PKG_SHARE" "$UR_TYPE" > "$eff_limits_file" << 'PYEOF'
import os, sys, yaml
share, ur_type = sys.argv[1], sys.argv[2]
cfg_dir = os.path.join(share, "config", "ur_types")
joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
defaults = {"shoulder_pan_joint": 150, "shoulder_lift_joint": 150,
            "elbow_joint": 150, "wrist_1_joint": 28,
            "wrist_2_joint": 28, "wrist_3_joint": 28}
eff = dict(defaults)
path = os.path.join(cfg_dir, f"{ur_type}.yaml")
try:
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    override = doc.get("effort_limits", {}) or {}
    eff.update({k: float(v) for k, v in override.items()})
except Exception as exc:
    sys.stderr.write(f"[mjcf] warn: ur_type config {path} not loaded: {exc}\n")
for j in joints:
    print(f"EFF_{j.upper()}={eff[j]}")
PYEOF
# shellcheck disable=SC1090
source "$eff_limits_file"

# Actuator generation is control_mode-aware:
#   effort  -> motor actuators (ctrl = joint torque). Paired with the
#              gravity_compensation.py node + forward_effort_controller.
#   position -> built-in PD position actuators (MuJoCo handles gravity
#              implicitly; joint_trajectory_controller drives ctrl directly).
ACTUATOR_FILE="$OUTPUT_DIR/actuators.xml"
if [ "$CONTROL_MODE" = "position" ]; then
    # Shortcoming #15 fix: previous kp=2000/500 with kv=100/25 was
    # numerically unstable under MuJoCo's default explicit RK4 when a
    # step-shaped command arrived via forward_position_controller.
    # Mitigation:
    #   * raise damping ratio (kv closer to 2*sqrt(kp*I)) so over-shoot
    #     decays instead of ringing,
    #   * add forcerange to clamp torques at the joint's effort limit so
    #     a single bad step can no longer launch the wrist to 400 rad,
    #   * rely on the 'implicitfast' integrator (set in ur_scene.xml)
    #     for stable stiff actuator integration.
    cat > "$ACTUATOR_FILE" << EOF
  <actuator>
    <position name="shoulder_pan_joint"  joint="shoulder_pan_joint"  kp="1500" kv="200" ctrlrange="-6.2832 6.2832" forcerange="-${EFF_SHOULDER_PAN_JOINT} ${EFF_SHOULDER_PAN_JOINT}"/>
    <position name="shoulder_lift_joint" joint="shoulder_lift_joint" kp="1500" kv="200" ctrlrange="-6.2832 6.2832" forcerange="-${EFF_SHOULDER_LIFT_JOINT} ${EFF_SHOULDER_LIFT_JOINT}"/>
    <position name="elbow_joint"         joint="elbow_joint"         kp="1000" kv="120" ctrlrange="-3.1416 3.1416" forcerange="-${EFF_ELBOW_JOINT} ${EFF_ELBOW_JOINT}"/>
    <position name="wrist_1_joint"       joint="wrist_1_joint"       kp="300"  kv="40"  ctrlrange="-6.2832 6.2832" forcerange="-${EFF_WRIST_1_JOINT} ${EFF_WRIST_1_JOINT}"/>
    <position name="wrist_2_joint"       joint="wrist_2_joint"       kp="300"  kv="40"  ctrlrange="-6.2832 6.2832" forcerange="-${EFF_WRIST_2_JOINT} ${EFF_WRIST_2_JOINT}"/>
    <position name="wrist_3_joint"       joint="wrist_3_joint"       kp="300"  kv="40"  ctrlrange="-6.2832 6.2832" forcerange="-${EFF_WRIST_3_JOINT} ${EFF_WRIST_3_JOINT}"/>
  </actuator>
EOF
else
    cat > "$ACTUATOR_FILE" << EOF
  <actuator>
    <motor name="shoulder_pan_joint"  joint="shoulder_pan_joint"  ctrlrange="-${EFF_SHOULDER_PAN_JOINT} ${EFF_SHOULDER_PAN_JOINT}"/>
    <motor name="shoulder_lift_joint" joint="shoulder_lift_joint" ctrlrange="-${EFF_SHOULDER_LIFT_JOINT} ${EFF_SHOULDER_LIFT_JOINT}"/>
    <motor name="elbow_joint"         joint="elbow_joint"         ctrlrange="-${EFF_ELBOW_JOINT} ${EFF_ELBOW_JOINT}"/>
    <motor name="wrist_1_joint"       joint="wrist_1_joint"       ctrlrange="-${EFF_WRIST_1_JOINT} ${EFF_WRIST_1_JOINT}"/>
    <motor name="wrist_2_joint"       joint="wrist_2_joint"       ctrlrange="-${EFF_WRIST_2_JOINT} ${EFF_WRIST_2_JOINT}"/>
    <motor name="wrist_3_joint"       joint="wrist_3_joint"       ctrlrange="-${EFF_WRIST_3_JOINT} ${EFF_WRIST_3_JOINT}"/>
  </actuator>
EOF
fi

# Post-process MJCF (R6): apply the known hacks as labeled, removable
# transforms in scripts/mjcf_postprocess.py rather than inline regex here.
MJCF_POSTPROCESS="$(dirname "$0")/mjcf_postprocess.py"
if [ ! -x "$MJCF_POSTPROCESS" ]; then
    # Installed layout: same directory as this script under lib/ur_sim_config.
    MJCF_POSTPROCESS="$(ros2 pkg prefix ur_sim_config)/lib/ur_sim_config/mjcf_postprocess.py"
fi
python3 "$MJCF_POSTPROCESS" "$MJCF_RAW" "$ACTUATOR_FILE" "$CONTROL_MODE"

# Create the scene file that includes the robot
cat > "$OUTPUT_DIR/ur_scene.xml" << 'SCENE_EOF'
<mujoco model="ur_scene">
  <!-- integrator="implicitfast" is stable with stiff <position> actuators
       at timestep=0.001; the default (RK4) diverges (shortcoming #15). -->
  <option gravity="0 0 -9.81" timestep="0.001" integrator="implicitfast"/>

  <include file="ur_robot.xml"/>

  <statistic center="0 0 0.5" extent="1.5"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"/>
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="ground" type="plane" size="5 5 0.01" material="groundplane"/>
  </worldbody>

  <keyframe>
    <key name="home" qpos="0 -1.57 0 -1.57 0 0"/>
  </keyframe>
</mujoco>
SCENE_EOF

echo "MuJoCo model generated in $OUTPUT_DIR/"
echo "  Robot: $MJCF_RAW"
echo "  Scene: $OUTPUT_DIR/ur_scene.xml"
echo "  Control mode: $CONTROL_MODE"
