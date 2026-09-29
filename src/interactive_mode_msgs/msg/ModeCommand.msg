# A request to switch interactive mode, from any command source.
#
# The speech layer (Milestone 2/3) publishes these on /interactive_mode/command
# once it has mapped an utterance to a mode. The state machine, not the
# recognizer, decides whether to act: commands below the configured confidence
# threshold, inside the switch cooldown, or from a source that is not allowed
# to make that change are dropped and reported in ModeState.last_rejection.

std_msgs/Header header
string mode          # "stay" | "follow" | "dance"
float32 confidence   # 0..1; operator sources should send 1.0
string source        # "voice", "keyboard", "operator", ...
string utterance     # raw transcript, for logging only (may be empty)
