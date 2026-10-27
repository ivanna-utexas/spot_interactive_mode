# Spot Interactive Mode

A single switchable "interactive mode" for Boston Dynamics' Spot, built for live
public demos (CoRL 2026 demo track, Texas Robotics Robot Parade). In one mode
Spot can:

- **stay**: hold its position and look at the nearest person,
- **follow**: turn toward a person and walk after them, stopping about 1 m away,
- **dance**: play the 44 s Dancing Queen routine with music.

Modes are switched from the PS4 controller or a terminal. Voice switching (Milestones 2–3)
uses the same interface.

UT Austin AMRL. Following and gaze come from the companion "dog mode"
(`spot_dog_mode`, `people_detector`). Dancing comes from the summer
Dancing Queen work. This repo adds the state machine that switches between
them safely.

**Platform:** the robot computer is a Jetson AGX Thor (JetPack 7 / L4T R38,
Ubuntu 24.04, CUDA 13). The container runs ROS 2 Jazzy on
`nvcr.io/nvidia/cuda:13.0.2-cudnn-devel-ubuntu24.04` with TensorRT 10.13.
The earlier Jetson Orin (JetPack 6 / ROS 2 Humble) setup is on `main`.

---

## Status (Oct 2, 2026)

| Piece | State |
| --- | --- |
| State machine, dance runner, unit tests (34) | Done, passing |
| PS4 mode buttons + tmuxinator session (`tmux/interactive_mode/`) | Passing in simulation (fake `/joy`); not yet tried on the real pad |
| End-to-end run against a simulated robot and dog mode | Passing |
| Build of everything the demo needs | Passing (use the build command below, **not** plain `colcon build`) |
| On the real robot | **Not yet verified**. Blocked by the two issues below. |

**Open blockers. Fix these before the next robot session:**

1. **LiDAR sends its data to the wrong address.** The Velodyne (`192.168.50.201`)
   streams to `192.168.50.10`, but the robot computer is `192.168.50.100`, and nothing
   answers at `.10`. With no point cloud there are no people detections, so stay
   and follow have nothing to react to. Fix: open `http://192.168.50.201`, set
   the Host (Destination) IP to `192.168.50.100`, then **Set** and **Save
   Configuration**. Check with anyone else who uses this LiDAR first.
2. **Use this repo's teleop launch**, not `spot_joy`'s (see "Every session",
   pane 4). The upstream launch uses a joystick driver that numbers the PS4
   buttons differently, so R1 is never recognized and every controller button
   does the wrong thing.

Also still to do: set the calibrated `audio_lead_sec` (the team's summer value).

---

## One-time setup

```bash
# 1. Clone with submodules (CUDA-CenterPoint and its nested submodules)
git clone --recursive https://github.com/ivanna-utexas/spot_interactive_mode.git ~/spot_interactive_mode
cd ~/spot_interactive_mode

# 2. Clone the external repos into src/ (spot_ros2 driver, spot_nav/spot_joy,
#    spot_velodyne, ironback_description). They are gitignored.
./scripts/checkout.sh

# 3. Copy in two files that are deliberately NOT in git:
mkdir -p params media
cp ~/spot_companion_mode/params/spot_driver.yaml params/   # holds the Spot login
cp ~/dance_ws/DancingQueenDemoClip.wav media/                 # copyrighted, 7.7 MB

# 4. Build the image and start the container (the name is stored in config/name)
./container build
```

Then **inside the container** (`./container shell`, prompt starts with `[docker]:`):

```bash
cd ~/spot_interactive_mode
source /opt/ros/jazzy/setup.bash
./scripts/generate_centerpoint_engine.sh     # once per machine/container

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to spot_driver spot_velodyne people_detector spot_joy \
                   spot_interactive_mode spot_audio_msgs
```

**Why not plain `colcon build`:** the CenterPoint submodule contains unrelated
projects (e.g. `bevfusion`) that fail to build here, and colcon aborts
everything else when they do. `--packages-up-to` builds only what the demo
needs. Companion mode avoided the same problem by building selectively too.
Run only one build at a time; two builds in the same workspace corrupt each
other.

After changing only this repo's Python code, a quick rebuild is enough:
`colcon build --packages-select spot_interactive_mode`

---

## Every session

### Quick start: one tmux session (tmuxinator)

```bash
cd ~/spot_interactive_mode/tmux/interactive_mode && tmuxinator local
```

Host or container: from the host, every pane hops into the container
itself. This opens every pane below (driver, Velodyne, people tracking,
teleop, interactive mode + dog mode, mode keys) in one tmux session,
`interactive_mode`, with each pane's name in its border. It also starts a
VNC desktop on `:9` (connect to the robot computer on port 5909; the first run in a
fresh container asks for a VNC password in the vnc pane) with rviz showing
the LiDAR points and CenterPoint people markers (`rviz2/interactive_mode.rviz`,
copied from companion mode's `auto_dog_mode.rviz`). The same view runs full
screen on the display mounted on Spot (X display `:0`, 1024×600) from
`rviz2/mounted_display.rviz`, alongside the eye animation; press `v` in the
keyboard pane to flip that screen between the eyes and tracking. rviz has no
full-screen flag, so `x_fullscreen` (in `screen_windows.py`) presses its F11.
Dog mode and the
dance are both loaded, so switching between them is just a controller button
(see Controls), with nothing to rebuild or relaunch. Running it again while
the session exists just reattaches.

Options go in front as environment variables: `CALIBRATION=1` (dance = 3 s
sync bounce), `NO_LIDAR=1` (dance-only test), `AUDIO_LEAD_SEC=<s>`,
`JOY_DEV=<path>`, `FORCE=1`. Before starting, it checks
(`scripts/interactive_mode_stack.sh preflight`) and refuses to start if the
`spot_interactive_mode` install is older than the source, or if a driver,
teleop or dog mode is already running.

To stop, run this where you started it. It presses Ctrl-C in interactive mode
first (so dog mode is turned off cleanly), then in every other pane, then
closes the session:

```bash
~/spot_interactive_mode/scripts/interactive_mode_stack.sh stop
```

(`tmuxinator stop` doesn't find projects started with `tmuxinator local`.)

To start the panes by hand instead, follow the next two sections.

### Get into the container first

ROS exists only inside the container. If `source install/setup.bash` prints
`not found: "/opt/ros/jazzy/local_setup.bash"`, you are on the host.

```bash
cd ~/spot_interactive_mode
./container start && ./container shell     # on the host
tmux new -s interactive                     # inside the container: every pane is now inside too
```

### Start these panes, in order

Each pane needs `source ~/spot_interactive_mode/install/setup.bash` first.

| # | Pane | Command | Needed for |
| --- | --- | --- | --- |
| 1 | Spot driver | `SPOT_PACK=1 SPOT_LIDAR_MOUNT=1 SPOT_VELODYNE=1 ros2 launch spot_driver spot_driver.launch.py config_file:=/home/ros/spot_interactive_mode/params/spot_driver.yaml` | everything |
| 2 | Velodyne | `ros2 launch spot_velodyne velodyne.launch.py device_ip:=192.168.50.201` | stay, follow |
| 3 | People tracking | `ros2 launch people_detector pedestrian_tracking.launch.py` | stay, follow |
| 4 | Teleop | `ros2 launch spot_interactive_mode teleop.launch.py` | everything |
| 5 | Interactive mode | `ros2 launch spot_interactive_mode interactive_mode.launch.py audio_file:=$HOME/spot_interactive_mode/media/DancingQueenDemoClip.wav` | everything |
| 6 | Mode keys | `ros2 run spot_interactive_mode mode_keyboard` | switching modes |

Notes:
- **Pane 1:** the driver config must come from this workspace's `params/`.
  Companion mode's folder isn't mounted in this container.
- **Pane 4:** use `spot_interactive_mode teleop.launch.py`, **not**
  `spot_joy teleop.launch.py` (see blocker 2). The PS4 controller must be
  plugged in by **USB**; Bluetooth pairs but never produces input on this kernel.
  The pad appears as `/dev/input/ps4` once `config/99-ps4-controller.rules`
  is installed on the host (one-time, commands in the file).
- **Pane 5** also starts dog mode with the right settings. **Do not** also run
  `spot_dog_mode dog_mode.launch.py`: two copies would fight.
- Dance-only test: panes 2 and 3 can be skipped.

### Check that everything is connected

```bash
ros2 topic hz /velodyne_points             # ~10 Hz        (silent = blocker 1)
ros2 topic echo /people_detections         # stand in front of Spot; you should appear
ros2 topic echo /interactive_mode/state    # hold R1 -> deadman_held: true (false = blocker 2)
```

---

## Controls

### PS4 controller

| Button | Effect |
| --- | --- |
| Options | Claim + power on |
| Square | Stand |
| Cross | Sit. Interactive mode turns itself off (idle) after about 2.5 s. |
| **R1 (hold)** | **Allows all autonomous motion.** Hold it the whole time Spot is in stay, follow or dance. |
| L1 + sticks | Drive by hand; following yields to you |
| **D-pad left** | Mode: **stay** (dog mode, feet planted) |
| **D-pad right** | Mode: **follow** (dog mode, walks after you) |
| **Circle** | Mode: **dance**. Hold R1 first; ignored without it. |
| **Share** | Mode: **idle** (interactive mode off) |
| E-stop | Always within reach of the operator |

What letting go of R1 does depends on the mode:

| Mode | Release R1 | Hold R1 again |
| --- | --- | --- |
| stay / follow | Spot stops moving and relaxes to neutral, but stays in that mode | Resumes on its own |
| dance | The dance ends (music stops, body eases to neutral), then Spot goes to stay | Doesn't restart; press Circle (or `d`) again |

Mode buttons act once per press and count as the operator (no cooldown,
can turn interactive mode on and off). They are set in
`config/interactive_mode.yaml` (`joy_stay`, `joy_follow`, `joy_dance`,
`joy_idle`). A rejected press (for example Circle without R1, or Spot not
standing) is explained in the interactive-mode pane.

### Without the controller

Spot can be stood from a terminal (pane with the driver running):

```bash
ros2 service call /claim    std_srvs/srv/Trigger
ros2 service call /power_on std_srvs/srv/Trigger
ros2 service call /stand    std_srvs/srv/Trigger
ros2 service call /sit      std_srvs/srv/Trigger
```

Spot still won't move autonomously without R1 held. That's intentional:
there's no terminal stand-in for the deadman.

### The modes

| Mode | What Spot does | Requires |
| --- | --- | --- |
| idle | Nothing autonomous; the operator has Spot. Start-up state. | — |
| stay | Looks at the nearest person by moving its body; feet stay planted | standing |
| follow | Turns toward you and walks after you, stopping ~1 m away | standing |
| dance | 44 s Dancing Queen routine with music, then stay | standing, **R1 held when requested** |

### Switching modes from a terminal

All three methods go to the same node as the controller buttons.

```bash
# 1. Keyboard (pane 6): s = stay, f = follow, d = dance, i = idle, q = quit
#    v flips the Spot-mounted screen between the eyes and pedestrian tracking
ros2 run spot_interactive_mode mode_keyboard

# 2. Service call, good for scripts
ros2 service call /interactive_mode/set_mode interactive_mode_msgs/srv/SetMode "{mode: follow}"

# 3. Simulated voice command: what the speech layer will publish
ros2 topic pub --once /interactive_mode/command interactive_mode_msgs/msg/ModeCommand \
  "{mode: dance, confidence: 0.9, source: voice, utterance: 'spot dance'}"
```

Methods 1 and 2 count as the **operator**. Voice commands (method 3) follow
stricter rules:
- voice can't turn interactive mode on or off (the operator starts it first),
- commands with confidence below 0.6 are ignored,
- commands within 2 s of the last switch are ignored.

Ignored and rejected requests say why, both in the keyboard pane and in
`last_rejection` on `/interactive_mode/state`.

### Running the dance

1. Panes 1, 4, 5 and 6 running (2 and 3 optional).
2. Controller: **Options**, then **Square**. Wait until Spot is standing.
3. **Hold R1** and keep holding it.
4. D-pad left or `s` (optional check that everything is connected; Spot should look at you).
5. Circle or `d`. Spot stops following or staying, waits until it has been still
   for 0.75 s, then starts the music and the routine. At the end it goes back to stay.
6. To stop early, release R1 or press `s`.

First time on a new setup: add `calibration_mode:=true` to pane 5. "Dance"
then plays a single 3 s bounce against the music. Adjust `audio_lead_sec`
until the bounce lands on the beat: positive if Spot moves before the beat,
negative if after.

### Picking the song

Three routines ship, all run by the same node with the same controls:

| Song | Start it with | Clip (in `media/`, not in git) |
| --- | --- | --- |
| Dancing Queen (44 s, default) | `interactive_mode.launch.py` / `tmuxinator local` | `DancingQueenDemoClip.wav` |
| Sultans of Swing (46 s, outro solo) | `sultans_of_swing.launch.py` / `SONG=sultans tmuxinator local` | `SultansOfSwingDemoClip.wav` |
| Dior, Pop Smoke (44 s, draft) | `dior.launch.py` / `SONG=dior tmuxinator local` | `DiorDemoClip.wav` |

Dior's timeline (`config/dior.json`) is a draft: it reuses Dancing Queen's
moves and gaits, with placeholder section times. Once you have your own copy
of the song, cut a 44 s clip to `media/DiorDemoClip.wav` and move the section
boundaries onto its downbeats.

Sultans of Swing, bar by bar (~148.5 BPM; each section starts on a downbeat
detected in the clip, so the moves stay on the beat even as the band drifts):

| Bars | Move | What Spot does |
| --- | --- | --- |
| 1–4 | `head_bob` | Nods on every beat, rocks side to side across the bar |
| 5–8 | `swing_step` | Steps left and right in trot, one bar per sway |
| 9–12 | `air_guitar` | Hunches to one side and strums on the 8th notes, head toss on beat 1, swaps sides |
| 13–16 | `sultan_sway` | Wide swinging sway with hips against shoulders, bounce on each beat |
| 17–20 | `hop_shuffle` | Hops forward and back (HOP gait) under the arpeggios |
| 21–25 | `arpeggio_ripple` | Body circles that grow, with a triplet shimmer on top |
| 26–end | `take_a_bow` | Two big nods per bar, then a deep bow held through the fade |

Both stepping sections cover whole cycles, so Spot ends them where it
started. Calibrate `audio_lead_sec` with Sultans of Swing the same way as for
Dancing Queen (the delay comes from the speaker, not the song).

To recreate the clip: download the album version (5:50) as a WAV, then cut
46.256 s starting at 289.390 s:

```bash
ffmpeg -ss 289.390 -t 46.256 -i sultans_of_swing_full.wav -af "afade=t=in:d=0.01,afade=t=out:st=44.256:d=2.0" -ar 44100 -ac 2 -c:a pcm_s16le media/SultansOfSwingDemoClip.wav
```

The start is a downbeat; the timeline's section times depend on it, so a clip
cut anywhere else plays off the beat.

---

## How it works

Following, standing still and dancing use control methods that conflict on
Spot. Pose animation needs `/stand` re-sent at 10 Hz, and that fights any
walking. So **exactly one controller commands Spot at a time**, and
`interactive_mode_node` decides which:

| Mode | Controller | Sends |
| --- | --- | --- |
| idle | none | — |
| stay | partner's gaze controller (dog mode), walking toward people **off** | body pose + its own stand refresh |
| follow | same controller, walking toward people **on** | body pose + stand refresh, walking commands |
| dance | dance runner inside `interactive_mode_node` | body pose + 10 Hz stand (pose moves) **or** walking commands (stepping moves), never both |

**Every switch has two phases, each with a timeout:**
1. **exiting:** everything is told to stop. The switch waits until dog mode
   reports it is disabled and the dance has finished easing out. A switch into
   dance also waits until the base is still.
2. **entering:** the new controller is turned on (dog mode gets its stay or
   follow settings first) and the switch waits for it to confirm.

**If something goes wrong, Spot falls back to a safe mode instead of hanging:**

| Event | Result |
| --- | --- |
| R1 released during a dance | dance eases out, then stay |
| Dance finishes | stay |
| Base won't stop moving before a dance | dance skipped, stay |
| Dog mode doesn't respond | idle |
| Dog mode turned off by something else | idle (operator takes over) |
| Spot not standing for 2.5 s (sat or e-stopped) | idle |
| Ctrl-C on pane 5 | stops motion, turns dog mode off |

**R1 as the single deadman:** all autonomous walking is sent to
`cmd_vel_intermediate`, and teleop passes it to Spot only while R1 is held.
Dog mode's own deadman is also moved to R1 (in
`config/dog_mode_overrides.yaml`). So R1 is the one button that authorizes
movement, and letting go stops the base in every mode.

**After every dance**, the walking style is reset to normal walk. The routine
ends in the hopping gait, and follow must not hop toward people. A neutral
body pose plus one final stand clears the dance pose before dog mode takes over.

**This node doesn't claim, power on or stand Spot.** The operator does that
from the controller, so they always hold it.

---

## Configuration

| What | Where |
| --- | --- |
| Launch options (pane 5): `audio_file`, `audio_lead_sec`, `calibration_mode:=true`, `start_mode:=stay`, `include_tracking:=true` | command line |
| Thresholds, timeouts, audio device (`plughw:3,0`), where to go after a dance | `src/spot_interactive_mode/config/interactive_mode.yaml` |
| Dog-mode settings under interactive mode, including **follow distance** (`approach_distance`, read once at launch) | `src/spot_interactive_mode/config/dog_mode_overrides.yaml` |
| Choreography (read at launch, no rebuild needed) | `src/spot_interactive_mode/config/timeline.json` (Dancing Queen), `sultans_of_swing.json` |

Settings you can change while it's running (they apply on the next switch into that mode):

```bash
ros2 param set /interactive_mode stay_allow_turn true     # stay may turn in place to face people
ros2 param set /interactive_mode follow_allow_turn false
```

---

## Repo layout

```
config/            container network/mount config (mounts, ports, cyclonedds*.xml, name)
docker/            Dockerfile + bash profiles
scripts/           checkout.sh (clones externals), engine generation, setup helpers,
                   interactive_mode_stack.sh (preflight / stop for the tmux session)
tmux/              tmuxinator projects: interactive_mode/ (the whole demo stack)
rviz2/             interactive_mode.rviz (LiDAR + CenterPoint people, odom frame),
                   mounted_display.rviz (same, sized for Spot's 1024x600 screen)
params/            spot_driver.yaml          (local only, gitignored: Spot login)
media/             DancingQueenDemoClip.wav, SultansOfSwingDemoClip.wav  (local only, gitignored)
src/
  spot_interactive_mode/   NEW: state machine, dance runner, node, launch files, tests
  interactive_mode_msgs/   NEW: ModeCommand, ModeState, SetMode
  spot_dog_mode/           partner's gaze/follow controller (copied from companion mode)
  people_detector/, bva_msgs/        CenterPoint people tracking (copied)
  spot_audio/, spot_audio_msgs/      mic capture + speech-to-text (copied; for M2/M3)
  spot_eye_animation*/, tl_expected/ (copied)
  Lidar_AI_Solution/       submodule: CUDA-CenterPoint-People
  spot_ros2/, spot_nav/, spot_velodyne/, ironback_description/   cloned by checkout.sh, gitignored
```

Inside `src/spot_interactive_mode/spot_interactive_mode/`:

| File | Role |
| --- | --- |
| `mode_machine.py` | Modes, two-phase switching, command filtering, fallbacks. Pure Python, no ROS. |
| `dance_timeline.py` | Evaluates `timeline.json` at any time t (blends, cross-fades, gait lookahead) |
| `dance_runner.py` | Starts, aborts and cleans up a dance; non-blocking audio offset |
| `dance_moves.py` | Move library, unchanged from the summer `spot_joy` code |
| `interactive_mode_node.py` | The ROS node: turns dog mode and the dance runner on and off to match the state machine |
| `mode_keyboard.py` | Keypress mode switching |
| `joy_bindings.py` | PS4 mode buttons (which input requests which mode, once per press) |
| `screen_windows.py` | Spot-mounted screen: rviz full screen, `v` flips eyes / tracking (X11 via ctypes) |

Launch files: `interactive_mode.launch.py` (dog mode + interactive node) and
`teleop.launch.py` (PS4 controller driver + `spot_joy` teleop).

### Tests

```bash
cd src/spot_interactive_mode && python3 -m pytest test    # no ROS or robot needed
```

They cover every switch path and fallback, that only one controller runs at
a time, the dance timing and limits, ending a dance early, the post-dance
cleanup, and the audio offset.

---

## Changes from earlier code that the team should know

- **Dance transitions:** between two back-to-back pose moves (about 11 s, 17 s
  and 24 s), the summer `dance_sequence.py` faded toward neutral and then snapped
  back to full pose in a single step. Transitions are now continuous. Watch them
  on the robot.
- **Gait after dancing** is now reset to normal walk (it was left in HOP).
- **`spot_audio_msgs`** never had a `CMakeLists.txt` (in companion mode
  either), so it never actually built. Added.
- **`config/mounts` and `config/ports`** were missing, so the container started
  with no workspace mounted and no host networking. Added.

## Testing on the robot

Run in this order and don't advance until each step passes. Rope off a
15 × 15 ft area before step 2.

1. **Spot sitting:** `set_mode stay` is rejected ("robot not standing").
2. **Stay:** Spot looks at you and its feet stay planted. Release R1 → it relaxes; hold R1 → it resumes.
3. **Follow:** walk away slowly. Spot follows and stops about 1 m away. Release R1 mid-walk → it stops at once.
4. **Dance, calibration** (`calibration_mode:=true`): the bounce lands on the beat; Spot returns to stay.
5. **Full dance:** transitions are smooth. Afterwards, in follow, Spot walks normally and doesn't hop.
6. **Safety:** release R1 mid-dance; send a low-confidence voice command (ignored); send two voice commands less than 2 s apart (second ignored); sit during follow (idle); Ctrl-C pane 5 (Spot stops, dog mode off).

## Next

- M2 (Oct 1–8): command recognition (keyword spotting vs. LLM). Publish
  `ModeCommand` on `/interactive_mode/command` with a real confidence; the
  state machine already handles the rest.
- Check the PS4 mode buttons on the real pad (D-pad left/right direction in
  particular; the interactive-mode pane logs every press).
