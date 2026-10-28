#!/usr/bin/env bash
# interactive_mode_stack.sh: helpers for tmux/interactive_mode/.tmuxinator.yaml.
#
#   scripts/interactive_mode_stack.sh preflight   checks run by tmuxinator before the session starts
#   scripts/interactive_mode_stack.sh stop        Ctrl-C every pane (interactive mode first), close the session
#
# `tmuxinator stop` can't find a project started with `tmuxinator local`
# (tmuxinator 3.0), so stopping goes through here. Run `stop` where you ran
# tmuxinator (host or container), since that's where the tmux server is.
set -euo pipefail

SESSION=interactive_mode
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

warn() { printf '\033[0;33m[WARN]\033[0m %s\n' "$1"; }
fail() { printf '\033[0;31m[FAIL]\033[0m %s\n' "$1"; exit 1; }

preflight() {
  [[ -f "$WS/install/setup.bash" ]] || fail "No install/ yet. Build first (see README, One-time setup)."
  [[ -f "$WS/params/spot_driver.yaml" ]] || fail "params/spot_driver.yaml missing (holds the Spot login; not in git)."
  # Same SONG -> clip mapping as the tmuxinator project.
  local clip
  case "${SONG:-}" in
    queen) clip=DancingQueenDemoClip.wav ;;
    dior) clip=DiorDemoClip.wav ;;
    *) clip=SultansOfSwingDemoClip.wav ;;
  esac
  [[ -f "$WS/media/$clip" ]] ||
    warn "media/$clip missing: Spot will dance without music."

  local joy_dev="${JOY_DEV:-/dev/input/ps4}"
  [[ -e "$joy_dev" ]] ||
    warn "$joy_dev not found: plug the PS4 controller in by USB (Bluetooth gives no input). The teleop pane retries.
       Plugged in already? Install the udev rule on the host: see config/99-ps4-controller.rules"

  # The installed package must match the source, or the controller bindings
  # (or any other recent change) won't be there.
  local installed src rel
  installed="$WS/install/spot_interactive_mode/lib/python$(python3 -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')/site-packages"
  for src in "$WS"/src/spot_interactive_mode/spot_interactive_mode/*.py; do
    rel="spot_interactive_mode/$(basename "$src")"
    if ! cmp -s "$src" "$installed/$rel"; then
      fail "spot_interactive_mode install is out of date ($rel). Run once, in the container:
       cd ~/spot_interactive_mode && colcon build --packages-select spot_interactive_mode"
    fi
  done

  # A second dog mode, teleop or driver would fight this stack for Spot.
  local running
  running=$(pgrep -a -f '(interactive_mode_node|gaze_controller_node|spot_joy/teleop_node|joy_linux_node|joy/joy_node|spot_driver/spot_ros2|velodyne_driver_node)' || true)
  if [[ -n "$running" && "${FORCE:-0}" != 1 ]]; then
    echo "$running" | sed 's/^/       /'
    fail "Parts of the stack are already running (above). Stop them first (e.g. another tmux session), or start with FORCE=1."
  fi
}

stop() {
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "No '$SESSION' session on this tmux server"; exit 0
  fi
  # SIGINT, not SIGHUP: interactive_mode only stops motion and turns dog mode
  # off on Ctrl-C. Interactive mode goes first, while dog mode and the driver
  # are still up to receive that. Panes name themselves (see the yaml).
  local mode_pane pane
  mode_pane=$(tmux list-panes -t "$SESSION:stack" -F '#{pane_id} #{pane_title}' | awk '$2 == "mode" {print $1}')
  if [[ -n "$mode_pane" ]]; then
    tmux send-keys -t "$mode_pane" C-c
    sleep 2
  fi
  for pane in $(tmux list-panes -t "$SESSION:stack" -F '#{pane_id}'); do
    tmux send-keys -t "$pane" C-c
  done
  echo "Waiting for nodes to shut down..."
  sleep 5
  tmux kill-session -t "$SESSION"
  echo "Session '$SESSION' closed"
}

case "${1:-}" in
  preflight) preflight ;;
  stop)      stop ;;
  *) sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
