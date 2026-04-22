#!/usr/bin/env bash
# Grant the current user FIFO real-time scheduling privileges, which
# ros2_control's controller_manager benefits from (lower jitter on the
# 500 Hz control loop).
#
# This is OPTIONAL. The simulator runs without it; turning it on just
# tightens the joint_states rate distribution.
#
# Usage:
#   sudo bash scripts/setup_rt.sh
#
# What it does:
#   * Appends `<user> - rtprio 95` and `<user> - memlock unlimited` to
#     /etc/security/limits.d/99-ur-sim.conf (idempotent).
#   * Suggests adding the user to `realtime` group if the distro uses it.
#
# After running, log out + back in for the limits to apply.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "error: re-run with sudo." >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-${USER:-root}}"
CONF=/etc/security/limits.d/99-ur-sim.conf

{
  echo "# Added by ur_simulator/scripts/setup_rt.sh"
  echo "${TARGET_USER}    -    rtprio     95"
  echo "${TARGET_USER}    -    memlock    unlimited"
} > "$CONF"

chmod 0644 "$CONF"

echo "wrote $CONF for user=$TARGET_USER"
echo "log out + back in for the limits to apply, then verify with:"
echo "    ulimit -r        # should print 95"
echo "    ulimit -l        # should print unlimited"
