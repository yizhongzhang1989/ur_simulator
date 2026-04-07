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

# MuJoCo always uses motor actuators (effort control) regardless of control_mode.
# Position control is handled by the ros2_control stack (gravity_compensation +
# joint_trajectory_controller), same as the Gazebo effort mode.
ACTUATOR_FILE="$OUTPUT_DIR/actuators.xml"
cat > "$ACTUATOR_FILE" << 'EOF'
  <actuator>
    <motor name="shoulder_pan_joint" joint="shoulder_pan_joint" ctrlrange="-150 150"/>
    <motor name="shoulder_lift_joint" joint="shoulder_lift_joint" ctrlrange="-150 150"/>
    <motor name="elbow_joint" joint="elbow_joint" ctrlrange="-150 150"/>
    <motor name="wrist_1_joint" joint="wrist_1_joint" ctrlrange="-28 28"/>
    <motor name="wrist_2_joint" joint="wrist_2_joint" ctrlrange="-28 28"/>
    <motor name="wrist_3_joint" joint="wrist_3_joint" ctrlrange="-28 28"/>
  </actuator>
EOF

# Add actuators and joint friction/damping to the robot MJCF using python
python3 - "$MJCF_RAW" "$ACTUATOR_FILE" "$CONTROL_MODE" << 'PYEOF'
import sys
import re

mjcf_path, actuator_path, control_mode = sys.argv[1], sys.argv[2], sys.argv[3]

with open(mjcf_path) as f:
    xml = f.read()

with open(actuator_path) as f:
    actuators = f.read()

# Insert actuators before closing </mujoco>
xml = xml.replace('</mujoco>', actuators + '\n</mujoco>')

# Fix shoulder_link body orientation: MuJoCo's URDF compiler fuses the
# base_link->base_link_inertia fixed joint (rpy="0 0 pi") into shoulder_link,
# giving it quat="0 0 0 1" (180 deg around Z). This makes shoulder_pan_joint
# visually appear to not rotate. Remove the spurious rotation.
# Also fix any geom/inertial quats that were set to "0 0 0 1" for same reason.
xml = xml.replace(
    '<body name="shoulder_link" pos="0 0 0.1625" quat="0 0 0 1">',
    '<body name="shoulder_link" pos="0 0 0.1625">'
)
# Fix shoulder geom quat (also inherited the Rz(pi))
xml = re.sub(
    r'(<geom\s+)quat="0 0 0 1"(\s+type="mesh"\s+mesh="shoulder"/>)',
    r'\1\2',
    xml
)
# Fix base geom quat
xml = re.sub(
    r'(<geom\s+pos="0 0 0"\s+)quat="-1 0 0 0"(\s+type="mesh"\s+mesh="base"/>)',
    r'\1\2',
    xml
)

# Remove actuatorfrcrange from joints — these are set by MuJoCo's URDF compiler
# from URDF effort limits. However they conflict with motor actuator ctrlrange.
xml = re.sub(r'\s*actuatorfrcrange="[^"]*"', '', xml)

# Disable collisions on all mesh geoms — UR meshes overlap at joints by design
# (they are visual-only in the real URDF). Without this, adjacent link meshes
# collide and prevent joints from rotating.
xml = re.sub(
    r'type="mesh"',
    'type="mesh" contype="0" conaffinity="0"',
    xml
)

# Add friction and damping to joints (simulating reduction gear friction)
base_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint']
wrist_joints = ['wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
for joint in base_joints:
    xml = xml.replace(
        f'joint name="{joint}"',
        f'joint name="{joint}" frictionloss="5" damping="10"'
    )
for joint in wrist_joints:
    xml = xml.replace(
        f'joint name="{joint}"',
        f'joint name="{joint}" frictionloss="1" damping="3"'
    )

with open(mjcf_path, 'w') as f:
    f.write(xml)

print(f"  Patched: actuators ({control_mode}), friction, damping, shoulder_link quat fix")
PYEOF

# Create the scene file that includes the robot
cat > "$OUTPUT_DIR/ur_scene.xml" << 'SCENE_EOF'
<mujoco model="ur_scene">
  <option gravity="0 0 -9.81" timestep="0.001"/>

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
