#!/bin/bash
# Generate static URDF files for all UR robot types with web-friendly mesh paths.
# Usage: ./generate_urdf.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
XACRO_FILE="$SCRIPT_DIR/../ur_description/urdf/ur.urdf.xacro"
OUTPUT_DIR="$SCRIPT_DIR/urdf"

source /opt/ros/humble/setup.bash

mkdir -p "$OUTPUT_DIR"

UR_TYPES="ur3 ur5 ur10 ur3e ur5e ur7e ur10e ur12e ur16e ur20 ur30"

LINKS="base_link_inertia shoulder_link upper_arm_link forearm_link wrist_1_link wrist_2_link wrist_3_link"

for ur_type in $UR_TYPES; do
    echo "Generating URDF for $ur_type..."
    # Generate URDF, rewrite mesh paths, and enable self-collision on each link
    SELF_COLLIDE_TAGS=""
    for link in $LINKS; do
        SELF_COLLIDE_TAGS="$SELF_COLLIDE_TAGS<gazebo reference=\"$link\"><self_collide>true</self_collide></gazebo>"
    done
    xacro "$XACRO_FILE" ur_type:="$ur_type" name:=ur 2>/dev/null \
        | sed 's|package://ur_description/|../ur_description/|g' \
        | sed "s|</robot>|${SELF_COLLIDE_TAGS}</robot>|" \
        > "$OUTPUT_DIR/${ur_type}.urdf"
done

echo "Done. Generated URDFs in $OUTPUT_DIR/"
ls -1 "$OUTPUT_DIR/"
