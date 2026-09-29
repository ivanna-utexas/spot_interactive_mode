"""Interactive-mode state machine, with no ROS dependency.

Modes are served by two controllers that must never command Spot together:

  STAY, FOLLOW -> spot_dog_mode's gaze controller (body pose + stand refresh,
                  plus base turn/approach relayed through cmd_vel_intermediate).
                  STAY and FOLLOW are two parameter profiles of it.
  DANCE        -> the dance runner in this package (body pose + its own stand
                  re-assert for pose segments, cmd_vel for locomotion segments).
  IDLE         -> neither; interactive mode is off and the operator has Spot.

Every switch goes through the same two phases:

  EXITING   everything is told to stop, and the machine waits until each
            controller has confirmed it has let go (dog mode reports
            enabled=false and the dance runner is no longer running). A switch
            into DANCE also waits for the base to stop moving, since pose
            animation needs stand re-asserts and those fight any walking
            still in progress.
  ENTERING  only the incoming controller is turned on, and the machine waits
            for it to confirm.

STAY <-> FOLLOW goes through a full disable/enable of dog mode too. Clearing
approach_enabled on a live gaze controller does not stop a walk already
underway (its walk commitment latches), whereas a disable resets its motion
controller and publishes a zero twist.

The machine only decides what *should* be running (a Directive). The ROS node
reconciles the real controllers toward it and reports back what they are
actually doing (an Observation). A controller that misbehaves therefore shows
up as a stalled phase, and the phase timeouts turn that into a safe
fallback instead of a hang.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class Mode(str, Enum):
    IDLE = 'idle'
    STAY = 'stay'
    FOLLOW = 'follow'
    DANCE = 'dance'

    @classmethod
    def parse(cls, text: str) -> Optional['Mode']:
        try:
            return cls(text.strip().lower())
        except ValueError:
            return None


DOG_MODES = (Mode.STAY, Mode.FOLLOW)


class Phase(str, Enum):
    ACTIVE = 'active'
    EXITING = 'exiting'
    ENTERING = 'entering'


@dataclass
class MachineConfig:
    # Commands from non-operator sources (voice) below this are ignored.
    min_confidence: float = 0.6
    # Minimum time between accepted non-operator switches, so one noisy
    # utterance or an echo cannot bounce Spot between modes.
    switch_cooldown: float = 2.0
    # Sources treated as the operator: full confidence, no cooldown, and the
    # only ones allowed to turn interactive mode on or off.
    operator_sources: Tuple[str, ...] = ('operator', 'keyboard', 'service')
    exit_timeout: float = 4.0
    enter_timeout: float = 3.0
    # The base must be still this long before a dance starts.
    dance_settle_time: float = 0.75
    # Where to go once a dance finishes (or is cut short by the deadman).
    post_dance_mode: Mode = Mode.STAY
    # Robot must report standing to leave IDLE; losing standing for this long
    # drops back to IDLE. Kept above dog mode's standing_grace, because
    # feedback can flicker while a stand settles.
    require_standing: bool = True
    not_standing_grace: float = 2.5


@dataclass
class Observation:
    now: float
    deadman_held: bool = False
    standing: Optional[bool] = None        # None until /status/feedback arrives
    moving: bool = False
    dog_enabled: Optional[bool] = None     # None until /dog_mode/enabled arrives
    dance_active: bool = False             # runner running, including its stop ramp
    dance_finished: bool = False           # runner completed the routine on its own


@dataclass
class Directive:
    """Desired controller configuration for this cycle."""

    dog_enabled: bool = False
    dog_profile: Optional[Mode] = None     # STAY/FOLLOW profile to apply before enabling
    dance_run: bool = False


@dataclass
class Decision:
    accepted: bool
    message: str


@dataclass
class _Transition:
    reason: str = ''
    last_rejection: str = ''
    changed: bool = True                   # something worth republishing happened


class ModeMachine:
    def __init__(self, config: Optional[MachineConfig] = None):
        self.config = config or MachineConfig()
        self.mode = Mode.IDLE        # mode currently in control
        self.target = Mode.IDLE      # mode being switched to (== mode when ACTIVE)
        self.phase = Phase.ACTIVE
        self._phase_since = 0.0
        self._last_switch = float('-inf')
        self._still_since: Optional[float] = None
        self._not_standing_since: Optional[float] = None
        self._last_obs: Optional[Observation] = None
        self._t = _Transition()

    # ------------------------------------------------------------------ status

    @property
    def reason(self) -> str:
        return self._t.reason

    @property
    def last_rejection(self) -> str:
        return self._t.last_rejection

    @property
    def settled(self) -> bool:
        return self.phase == Phase.ACTIVE

    def consume_changed(self) -> bool:
        """True once after any state or rejection change (for publishing)."""
        changed, self._t.changed = self._t.changed, False
        return changed

    # ---------------------------------------------------------------- commands

    def request(self, mode_text: str, now: float, source: str = 'operator',
                confidence: float = 1.0) -> Decision:
        """Ask for a mode. Validates it, then starts a transition or rejects it."""
        mode = Mode.parse(mode_text)
        if mode is None:
            return self._reject(f'unknown mode {mode_text!r}')

        operator = source in self.config.operator_sources
        if not operator:
            if confidence < self.config.min_confidence:
                return self._reject(
                    f'{source} command {mode.value!r} below confidence '
                    f'({confidence:.2f} < {self.config.min_confidence:.2f})')
            if self.mode == Mode.IDLE and self.target == Mode.IDLE:
                return self._reject(
                    f'{source} cannot start interactive mode; operator must enable it')
            if mode == Mode.IDLE:
                return self._reject(f'{source} cannot turn interactive mode off')
            if now - self._last_switch < self.config.switch_cooldown:
                return self._reject(
                    f'{source} command {mode.value!r} inside switch cooldown')

        if mode == self.target:
            return self._reject(f'already {self._describe()}')

        if mode != Mode.IDLE and not self._robot_ready():
            return self._reject(f'cannot switch to {mode.value}: robot not standing')
        if mode == Mode.DANCE and not self._deadman_held():
            return self._reject('cannot dance: autonomy deadman not held')

        self._last_switch = now
        self._begin(mode, now, f'{source} requested {mode.value}')
        return Decision(True, f'switching to {mode.value}')

    # -------------------------------------------------------------------- tick

    def step(self, obs: Observation) -> Directive:
        self._last_obs = obs
        self._track_motion(obs)
        self._safety(obs)

        if self.phase == Phase.EXITING:
            self._run_exit(obs)
        if self.phase == Phase.ENTERING:
            self._run_enter(obs)
        if self.phase == Phase.ACTIVE:
            self._run_active(obs)

        return self._directive()

    # ---------------------------------------------------------------- internals

    def _directive(self) -> Directive:
        if self.phase == Phase.EXITING:
            return Directive()
        mode = self.target if self.phase == Phase.ENTERING else self.mode
        if mode in DOG_MODES:
            return Directive(dog_enabled=True, dog_profile=mode)
        if mode == Mode.DANCE:
            return Directive(dance_run=True)
        return Directive()

    def _safety(self, obs: Observation) -> None:
        if self.mode == Mode.IDLE and self.target == Mode.IDLE:
            return
        if self.config.require_standing:
            if obs.standing is False:
                if self._not_standing_since is None:
                    self._not_standing_since = obs.now
                elif obs.now - self._not_standing_since > self.config.not_standing_grace:
                    self._begin(Mode.IDLE, obs.now, 'robot no longer standing')
                    return
            else:
                self._not_standing_since = None
        # The deadman gates dog mode internally (it neutralizes and resumes on
        # its own), but a half-played dance can't be paused and picked back up
        # on the beat, so releasing the deadman ends it.
        if (Mode.DANCE in (self.mode, self.target) and not obs.deadman_held
                and self.target != self.config.post_dance_mode):
            self._begin(self.config.post_dance_mode, obs.now,
                        'deadman released during dance')

    def _run_exit(self, obs: Observation) -> None:
        released = obs.dog_enabled is not True and not obs.dance_active
        still = (self.target != Mode.DANCE or
                 (self._still_since is not None and
                  obs.now - self._still_since >= self.config.dance_settle_time))
        if released and still:
            if self.target == Mode.IDLE:
                self._settle(Mode.IDLE, obs.now)
            else:
                self._set_phase(Phase.ENTERING, obs.now)
            return
        if obs.now - self._phase_since > self.config.exit_timeout:
            if released and self.target == Mode.DANCE:
                self._begin(self.config.post_dance_mode, obs.now,
                            'dance skipped: base did not come to rest')
            elif self.target != Mode.IDLE:
                what = 'dog mode' if obs.dog_enabled else 'dance'
                self._begin(Mode.IDLE, obs.now, f'{what} did not release control')
            # Already heading to IDLE: keep asking everything to stop.

    def _run_enter(self, obs: Observation) -> None:
        if self.target in DOG_MODES and obs.dog_enabled is True:
            self._settle(self.target, obs.now)
            return
        if self.target == Mode.DANCE and obs.dance_active:
            self._settle(Mode.DANCE, obs.now)
            return
        if obs.now - self._phase_since > self.config.enter_timeout:
            if self.target == Mode.DANCE:
                self._begin(self.config.post_dance_mode, obs.now, 'dance failed to start')
            else:
                self._begin(Mode.IDLE, obs.now,
                            f'dog mode did not enable for {self.target.value}')

    def _run_active(self, obs: Observation) -> None:
        if self.mode in DOG_MODES and obs.dog_enabled is False:
            # Someone else turned dog mode off (e.g. the Circle button on the
            # companion teleop). Treat that as the operator taking over rather
            # than fighting them for it.
            self._begin(Mode.IDLE, obs.now, 'dog mode disabled externally')
        elif self.mode == Mode.DANCE and (obs.dance_finished or not obs.dance_active):
            self._begin(self.config.post_dance_mode, obs.now, 'dance finished')

    def _begin(self, mode: Mode, now: float, reason: str) -> None:
        self.target = mode
        self._set_phase(Phase.EXITING, now)
        self._t.reason = reason

    def _settle(self, mode: Mode, now: float) -> None:
        self.mode = mode
        self.target = mode
        self._set_phase(Phase.ACTIVE, now)

    def _set_phase(self, phase: Phase, now: float) -> None:
        # `mode` keeps naming the outgoing mode until the switch settles. A
        # retarget mid-ENTERING lands back in EXITING, which then waits for the
        # half-started controller to let go like any other.
        self.phase = phase
        self._phase_since = now
        self._t.changed = True

    def _track_motion(self, obs: Observation) -> None:
        if obs.moving:
            self._still_since = None
        elif self._still_since is None:
            self._still_since = obs.now

    def _robot_ready(self) -> bool:
        if not self.config.require_standing:
            return True
        return self._last_obs is not None and self._last_obs.standing is True

    def _deadman_held(self) -> bool:
        return self._last_obs is not None and self._last_obs.deadman_held

    def _reject(self, message: str) -> Decision:
        self._t.last_rejection = message
        self._t.changed = True
        return Decision(False, message)

    def _describe(self) -> str:
        if self.phase == Phase.ACTIVE:
            return f'in {self.mode.value}'
        return f'switching to {self.target.value}'
