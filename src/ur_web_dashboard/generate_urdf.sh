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

for ur_type in $UR_TYPES; do
    echo "Generating URDF for $ur_type..."
    xacro "$XACRO_FILE" ur_type:="$ur_type" name:=ur 2>/dev/null \
        | sed 's|package://ur_description/|../ur_description/|g' \
        > "$OUTPUT_DIR/${ur_type}.urdf"
done

echo "Done. Generated URDFs in $OUTPUT_DIR/"
ls -1 "$OUTPUT_DIR/"
