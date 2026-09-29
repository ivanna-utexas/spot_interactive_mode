# Latched snapshot of the interactive-mode state machine.
# Published on /interactive_mode/state on every change and at 1 Hz.

std_msgs/Header header
string mode            # mode currently in control: "idle" | "stay" | "follow" | "dance"
string target_mode     # mode being switched to; equals `mode` when settled
string phase           # "active" | "exiting" | "entering"
string reason          # why the last transition happened
string last_rejection  # why the last command was ignored (empty if none yet)
bool deadman_held      # operator's autonomy deadman (R1 by default)
bool standing          # from /status/feedback
