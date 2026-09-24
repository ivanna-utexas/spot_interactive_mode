#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/spot_companion_mode"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ROOT}/bags/live_debug/navstack_teleop_${STAMP}"

TOPICS=(
  /joy
  /cmd_vel
  /cmd_vel_intermediate
  /cmd_vel_nav
  /status/feedback
  /status/power_states
  /status/leases
  /status/mobility_params
  /status/behavior_faults
  /status/estop
  /odometry/twist
)

echo "=== Topic graph ==="
for topic in /cmd_vel /cmd_vel_intermediate /cmd_vel_nav; do
  echo
  echo "--- ros2 topic info -v ${topic}"
  ros2 topic info -v "${topic}" || true
done

echo
echo "=== Spot state snapshots ==="
for topic in /status/leases /status/power_states /status/feedback /status/mobility_params /status/behavior_faults /status/estop; do
  echo
  echo "--- ros2 topic echo --once ${topic}"
  ros2 topic echo --once "${topic}" || true
done

echo
echo "=== Recording focused debug bag ==="
echo "Output: ${OUT_DIR}"
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Reproduce this exact sequence in the navstack session:"
echo "  1. Press Options once"
echo "  2. Press Square once and wait for full stand"
echo "  3. Press Triangle once"
echo "  4. Hold L1 and push the left stick forward for 3 seconds"
echo "  5. Release and repeat once"
echo
echo "Watch the spot_joy pane for any warnings, then Ctrl-C here after 20-30 seconds."
echo

mkdir -p "$(dirname "${OUT_DIR}")"
exec ros2 bag record -o "${OUT_DIR}" "${TOPICS[@]}"
