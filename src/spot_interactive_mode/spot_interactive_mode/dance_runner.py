"""Plays a DanceTimeline as a startable, abortable mode, with no ROS dependency.

dance_sequence.py blocked on input() and busy-waited out the audio offset,
which is fine for a one-shot demo but not for a mode that has to start on a
voice command and stop on the next one. Here every call is non-blocking: the
node calls step(now) at the control rate and does what the returned
RunnerOutput says.

Ending a dance, whether it finished or was aborted:
  - a pose segment in progress ramps to neutral over `abort_ramp_sec`, with
    the stand re-assert still running so the ramp is actually applied;
  - a locomotion segment in progress is stopped with a zero twist;
  - then a neutral pose and one final stand are sent, so the driver's stored
    body offset is neutral before the next controller takes over;
  - and the gait is set back to `rest_locomotion_mode`. The routine leaves
    Spot in HOP (foot_shuffle), and FOLLOW must not walk up to someone in it.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

from .dance_timeline import NEUTRAL_POSE, POSE, DanceTimeline

Pose4 = Tuple[float, float, float, float]
Twist3 = Tuple[float, float, float]
ZERO_TWIST: Twist3 = (0.0, 0.0, 0.0)


@dataclass
class RunnerOutput:
    pose: Optional[Pose4] = None
    twist: Optional[Twist3] = None
    stand_reassert: bool = False     # keep the 10 Hz stand re-assert running
    final_stand: bool = False        # send exactly one stand now
    gait_request: Optional[int] = None
    launch_audio: bool = False
    stop_audio: bool = False
    finished: bool = False           # routine completed on its own this cycle


class DanceRunner:
    IDLE = 'idle'
    WAITING = 'waiting'       # started; motion clock not yet running (audio lead)
    PLAYING = 'playing'
    STOPPING = 'stopping'     # ramping out after abort() or the end of the routine

    def __init__(self, timeline: DanceTimeline, audio_lead_sec: float = 0.0,
                 abort_ramp_sec: float = 0.5, rest_locomotion_mode: int = 1):
        self.timeline = timeline
        self.audio_lead_sec = audio_lead_sec
        self.abort_ramp_sec = max(0.0, abort_ramp_sec)
        self.rest_locomotion_mode = rest_locomotion_mode
        self.state = self.IDLE
        self._motion_t0 = 0.0
        self._audio_at = 0.0
        self._audio_launched = False
        self._stop_t0 = 0.0
        self._stop_from_pose: Optional[Pose4] = None
        self._stop_zero_twist = False
        self._last_kind: Optional[str] = None
        self._last_pose: Pose4 = NEUTRAL_POSE
        self._completed = False
        self.segment_index: Optional[int] = None

    @property
    def running(self) -> bool:
        """True from start() until the stop ramp and cleanup are done."""
        return self.state != self.IDLE

    def elapsed(self, now: float) -> float:
        return now - self._motion_t0

    def start(self, now: float) -> None:
        """Begin the routine. A positive audio lead starts audio first."""
        lead = self.audio_lead_sec
        self._audio_at = now if lead >= 0 else now - lead
        self._motion_t0 = now + max(0.0, lead)
        self._audio_launched = False
        self._completed = False
        self._last_kind = None
        self._last_pose = NEUTRAL_POSE
        self.segment_index = None
        self.state = self.WAITING

    def abort(self, now: float) -> None:
        if self.state in (self.WAITING, self.PLAYING):
            self._begin_stop(now)

    def step(self, now: float) -> RunnerOutput:
        out = RunnerOutput()
        if self.state == self.IDLE:
            return out

        if (self.state in (self.WAITING, self.PLAYING)
                and not self._audio_launched and now >= self._audio_at):
            self._audio_launched = True
            out.launch_audio = True

        if self.state == self.WAITING:
            if now < self._motion_t0:
                return out
            self.state = self.PLAYING

        if self.state == self.PLAYING:
            sample = self.timeline.sample(now - self._motion_t0)
            if sample.finished:
                self._completed = True
                self._begin_stop(now)
            else:
                self.segment_index = sample.segment_index
                out.gait_request = sample.gait_request
                if sample.pose is not None:
                    out.pose = sample.pose
                    out.stand_reassert = True
                    self._last_pose = sample.pose
                elif sample.twist is not None:
                    out.twist = sample.twist
                else:
                    # Gap or unwired gesture: hold still.
                    out.twist = ZERO_TWIST
                if sample.kind == POSE and self._last_kind not in (None, POSE):
                    # Entering a pose segment from locomotion: make sure the
                    # last velocity command is a stop before stands resume.
                    out.twist = ZERO_TWIST
                self._last_kind = sample.kind
                return out

        # STOPPING
        return self._step_stop(now, out)

    # ----------------------------------------------------------------- stopping

    def _begin_stop(self, now: float) -> None:
        self.state = self.STOPPING
        self._stop_t0 = now
        self._stop_from_pose = self._last_pose if self._last_kind == POSE else None
        self._stop_zero_twist = self._last_kind != POSE

    def _step_stop(self, now: float, out: RunnerOutput) -> RunnerOutput:
        if self._stop_zero_twist:
            out.twist = ZERO_TWIST
            self._stop_zero_twist = False
        if not self._completed and self._audio_launched:
            out.stop_audio = True
            self._audio_launched = False

        elapsed = now - self._stop_t0
        if self._stop_from_pose is not None and elapsed < self.abort_ramp_sec:
            scale = 1.0 - elapsed / self.abort_ramp_sec
            out.pose = tuple(v * scale for v in self._stop_from_pose)
            out.stand_reassert = True
            return out

        out.pose = NEUTRAL_POSE
        out.final_stand = True
        out.gait_request = self.rest_locomotion_mode
        out.finished = self._completed
        self.state = self.IDLE
        return out
