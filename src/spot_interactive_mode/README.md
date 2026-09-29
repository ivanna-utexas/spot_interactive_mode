# spot_interactive_mode

One switchable mode for Spot: **STAY**, **FOLLOW**, **DANCE**, switched by an
operator (Milestone 1) or by voice (Milestone 3).

## Who controls Spot

Following, holding still and dancing need control regimes that can't run
together, so exactly one controller commands Spot at any time:

| Mode   | Controller | Publishes | Stand re-assert |
|--------|------------|-----------|-----------------|
| IDLE   | none (operator has Spot) | nothing | no |
| STAY   | `spot_dog_mode` gaze controller, `approach_enabled=false`, `base_turn_enabled=stay_allow_turn` | `/body_pose`, `/stand` | dog mode's own, while still |
| FOLLOW | same controller, `approach_enabled=true`, `base_turn_enabled=true` | `/body_pose`, `/stand`, `cmd_vel_intermediate` | dog mode's own, while still |
| DANCE  | `DanceRunner` inside `interactive_mode_node` | `/body_pose` + `/stand` (pose segments) **or** `cmd_vel_intermediate` (locomotion segments) | pose segments only |

`interactive_mode_node` doesn't reimplement following. Dog mode already
computes bearing and range from CenterPoint tracks and runs
`BaseMotionController`. This node decides *which* controller may run and
confirms each handoff.

### Switching

Every switch has two phases, and both have timeouts:

1. **exiting**: every controller is told to stop. The switch waits for dog mode
   to report `/dog_mode/enabled=false` and for the dance runner to finish its
   ramp-out. A switch into DANCE also waits for the base to be still
   (`dance_settle_time`).
2. **entering**: only the new controller is turned on (dog-mode parameter
   profile first, then enable), and the switch waits for it to confirm.

STAY↔FOLLOW also goes through a full dog-mode disable. Clearing
`approach_enabled` on a live gaze controller doesn't stop a walk already in
progress, because its walk commitment latches. A disable resets it and sends a
zero twist.

Fallbacks:

| Event | Result |
|---|---|
| Voice command below `min_command_confidence`, inside `switch_cooldown`, or trying to turn interactive mode on/off | ignored, reported in `last_rejection` |
| Deadman (R1) released during a dance | dance ramps out, then `post_dance_mode` |
| Dance finishes | `post_dance_mode` (default STAY) |
| Base won't settle before a dance | dance skipped, back to STAY |
| Dog mode doesn't enable in time | IDLE |
| Dog mode disabled by someone else (e.g. Circle on the companion teleop) | IDLE (operator takes over) |
| Robot not standing for `not_standing_grace` | IDLE |

When a dance ends (finished or aborted), a pose segment ramps to neutral, then
a neutral pose and one stand are sent, and the gait is reset to
`rest_locomotion_mode` (1). Without the reset, FOLLOW would inherit HOP from
`foot_shuffle`.

## Operator deadman

All autonomous base motion goes to `cmd_vel_intermediate`, which `spot_joy`
teleop relays to `/cmd_vel` only while **R1** is held. The launch file also moves
dog mode's own deadman from L1 to R1, so **holding R1 authorizes all
autonomous motion, and releasing it stops the base in every mode**. This
works with both the upstream and the companion-mode `teleop_node`: with R1
held and L1 released, neither one publishes `body_pose` or `cmd_vel` itself.

This node doesn't claim, power on or stand Spot. Use the controller:
Options, then Square.

## Running

```bash
# with the Spot driver, Velodyne, teleop and CenterPoint already running
ros2 launch spot_interactive_mode interactive_mode.launch.py \
    audio_file:=$HOME/spot_interactive_mode/media/DancingQueenDemoClip.wav \
    audio_lead_sec:=<calibrated value>

# switch modes (operator)
ros2 run spot_interactive_mode mode_keyboard          # s / f / d / i
ros2 service call /interactive_mode/set_mode interactive_mode_msgs/srv/SetMode "{mode: follow}"

# what a speech recognizer should publish (M2/M3)
ros2 topic pub --once /interactive_mode/command interactive_mode_msgs/msg/ModeCommand \
    "{mode: dance, confidence: 0.9, source: voice, utterance: 'spot, dance!'}"

ros2 topic echo /interactive_mode/state
```

Sync testing: `calibration_mode:=true` turns "dance" into a single 3 s bounce,
the same as `dance_sequence.py`.

## Code

| File | ROS? | What |
|---|---|---|
| `mode_machine.py` | no | modes, two-phase switching, command filtering, fallbacks |
| `dance_timeline.py` | no | segment evaluation from `dance_sequence.py` (blend, cross-fade, gait lookahead) |
| `dance_runner.py` | no | start / abort / ramp-out / cleanup, non-blocking audio lead |
| `dance_moves.py` | no | copied unchanged from `spot_joy` |
| `interactive_mode_node.py` | yes | reconciles dog mode and the runner to the machine's directive |
| `mode_keyboard.py` | yes | keypress triggers |

`python3 -m pytest test` runs without ROS.

### Change from the summer dance code

Between two back-to-back pose segments, `dance_sequence.py` faded the outgoing
move toward neutral over its last 0.2 s, and then the incoming segment's
cross-fade started from the outgoing move's *full* end pose: a one-tick snap
at every pose→pose boundary (11.0 s→17.3 s→23.7 s in the Dancing Queen
timeline). `dance_timeline.py` removes the fade-out when a pose segment
follows, so the cross-fade is continuous. Check it on the robot before
relying on it.
