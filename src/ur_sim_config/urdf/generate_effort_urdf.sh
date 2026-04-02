#!/bin/bash
# Generate a URDF for Gazebo Ignition with effort command interfaces.
# The upstream ur_description xacro excludes effort commands for sim_ignition.
# This script patches the output to re-add them.
#
# Usage: generate_effort_urdf.sh <ur_type> <controllers_yaml> [output_file]
#
# If output_file is omitted, prints to stdout (for use with Command() in launch).

set -e

UR_TYPE="${1:?Usage: $0 <ur_type> <controllers_yaml> [output_file]}"
CONTROLLERS="${2:?Usage: $0 <ur_type> <controllers_yaml> [output_file]}"
OUTPUT="${3:-/dev/stdout}"

source /opt/ros/humble/setup.bash

xacro "$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro" \
    ur_type:="$UR_TYPE" \
    name:=ur \
    sim_ignition:=true \
    simulation_controllers:="$CONTROLLERS" \
  | sed '/<joint name=.*_joint">/,/<\/joint>/ {
    /<command_interface name="velocity"\/>/a\        <command_interface name="effort"/>
  }' \
  | sed 's|<plugin>ign_ros2_control/IgnitionSystem</plugin>|<plugin>ign_ros2_control/IgnitionSystem</plugin>\n      <param name="position_proportional_gain">5000</param>|' \
  | sed 's|effort="\([0-9.]*\)"|effort="500"|g' \
  > "$OUTPUT"
