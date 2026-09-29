"""State machine tests: a fake world stands in for dog mode and the runner."""

from spot_interactive_mode.mode_machine import (
    MachineConfig, Mode, ModeMachine, Observation, Phase)


class World:
    """Controllers that follow the directive after a configurable lag."""

    def __init__(self, machine, dog_lag=0.1, dance_len=None):
        self.m = machine
        self.t = 0.0
        self.standing = True
        self.deadman = True
        self.moving = False
        self.dog = False
        self.dog_responds = True
        self.dance = False
        self.dance_len = dance_len
        self.dance_started = None
        self.dog_lag = dog_lag
        self._dog_want_since = None
        self.log = []

    def run(self, seconds, dt=0.02):
        end = self.t + seconds
        while self.t < end:
            self.tick(dt)

    def tick(self, dt=0.02):
        self.t += dt
        finished = False
        if self.dance and self.dance_len is not None and \
                self.t - self.dance_started >= self.dance_len:
            self.dance, finished = False, True
        d = self.m.step(Observation(
            now=self.t, deadman_held=self.deadman, standing=self.standing,
            moving=self.moving, dog_enabled=self.dog, dance_active=self.dance,
            dance_finished=finished))
        # Only one controller may ever be asked to run.
        assert not (d.dog_enabled and d.dance_run)
        # Only one controller may ever actually be running.
        assert not (self.dog and self.dance)
        if d.dog_enabled != self.dog and self.dog_responds:
            self._dog_want_since = self._dog_want_since or self.t
            if self.t - self._dog_want_since >= self.dog_lag:
                self.dog, self._dog_want_since = d.dog_enabled, None
        else:
            self._dog_want_since = None
        if d.dance_run and not self.dance:
            self.dance, self.dance_started = True, self.t
        elif not d.dance_run and self.dance:
            self.dance = False
        self.log.append((self.t, self.m.mode, self.m.phase, d))
        return d


def make(**cfg):
    m = ModeMachine(MachineConfig(**cfg))
    w = World(m)
    w.tick()
    return m, w


def test_enable_stay_then_follow():
    m, w = make()
    assert m.request('stay', w.t).accepted
    w.run(0.5)
    assert (m.mode, m.phase) == (Mode.STAY, Phase.ACTIVE)
    assert w.dog
    assert m.request('follow', w.t).accepted
    w.run(0.1)
    assert m.phase != Phase.ACTIVE
    w.run(0.5)
    assert (m.mode, m.phase) == (Mode.FOLLOW, Phase.ACTIVE)
    assert m.step(Observation(now=w.t, standing=True, dog_enabled=True,
                              deadman_held=True)).dog_profile == Mode.FOLLOW


def test_stay_to_follow_cycles_dog_mode_off_first():
    m, w = make()
    m.request('stay', w.t)
    w.run(0.5)
    m.request('follow', w.t)
    w.run(0.5)
    profiles = [d.dog_profile for _, _, _, d in w.log if d.dog_enabled]
    seen_off_between = False
    for i in range(1, len(w.log)):
        if w.log[i - 1][3].dog_profile == Mode.STAY and not w.log[i][3].dog_enabled:
            seen_off_between = True
    assert seen_off_between
    assert profiles[-1] == Mode.FOLLOW


def test_dance_waits_for_dog_release_and_stillness():
    m, w = make(dance_settle_time=0.5)
    m.request('follow', w.t)
    w.run(0.5)
    w.moving = True
    m.request('dance', w.t)
    w.run(1.0)
    assert not w.dance, 'must not dance while the base is still moving'
    assert not w.dog
    w.moving = False
    w.run(0.3)
    assert not w.dance, 'must wait out the settle time'
    w.run(0.4)
    assert w.dance and m.mode == Mode.DANCE


def test_dance_skipped_if_base_never_stops():
    m, w = make(exit_timeout=1.0)
    m.request('stay', w.t)
    w.run(0.5)
    w.moving = True
    m.request('dance', w.t)
    w.run(1.5)
    assert m.target == Mode.STAY
    assert 'did not come to rest' in m.reason
    assert not w.dance


def test_dance_finishes_into_post_dance_mode():
    m, w = make(post_dance_mode=Mode.STAY)
    w.dance_len = 2.0
    m.request('follow', w.t)
    w.run(0.5)
    m.request('dance', w.t)
    w.run(1.5)
    assert m.mode == Mode.DANCE
    w.run(2.0)
    assert (m.mode, m.phase) == (Mode.STAY, Phase.ACTIVE)
    assert m.reason == 'dance finished'


def test_deadman_release_ends_dance():
    m, w = make()
    m.request('stay', w.t)
    w.run(0.5)
    m.request('dance', w.t)
    w.run(1.5)
    assert m.mode == Mode.DANCE
    w.deadman = False
    w.run(0.5)
    assert m.mode == Mode.STAY and not w.dance
    assert 'deadman' in m.reason


def test_dance_rejected_without_deadman():
    m, w = make()
    m.request('stay', w.t)
    w.run(0.5)
    w.deadman = False
    w.tick()
    d = m.request('dance', w.t)
    assert not d.accepted and 'deadman' in d.message


def test_voice_filters():
    m, w = make(min_confidence=0.6, switch_cooldown=2.0)
    assert not m.request('stay', w.t, source='voice', confidence=0.9).accepted, \
        'voice cannot enable interactive mode'
    m.request('stay', w.t)
    w.run(3.0)
    low = m.request('follow', w.t, source='voice', confidence=0.3)
    assert not low.accepted and 'confidence' in low.message
    assert m.request('follow', w.t, source='voice', confidence=0.9).accepted
    w.run(0.5)
    fast = m.request('stay', w.t, source='voice', confidence=0.9)
    assert not fast.accepted and 'cooldown' in fast.message
    assert not m.request('idle', w.t + 5, source='voice', confidence=1.0).accepted
    assert m.request('stay', w.t, source='keyboard').accepted, 'operator bypasses cooldown'


def test_duplicate_and_unknown_rejected():
    m, w = make()
    assert not m.request('jump', w.t).accepted
    m.request('stay', w.t)
    w.run(0.5)
    assert not m.request('stay', w.t).accepted


def test_not_standing_drops_to_idle():
    m, w = make(not_standing_grace=1.0)
    m.request('follow', w.t)
    w.run(0.5)
    w.standing = False
    w.run(0.5)
    assert m.mode == Mode.FOLLOW, 'brief flicker is tolerated'
    w.run(1.0)
    assert m.target == Mode.IDLE
    w.run(0.5)
    assert (m.mode, m.phase) == (Mode.IDLE, Phase.ACTIVE) and not w.dog


def test_cannot_enable_while_sitting():
    m, w = make()
    w.standing = False
    w.tick()
    assert not m.request('stay', w.t).accepted


def test_external_disable_hands_back_to_operator():
    m, w = make()
    m.request('stay', w.t)
    w.run(0.5)
    w.dog = False
    w.dog_responds = False
    w.run(0.2)
    assert m.target == Mode.IDLE and 'externally' in m.reason


def test_unresponsive_dog_mode_falls_back_to_idle():
    m, w = make(enter_timeout=1.0)
    w.dog_responds = False
    m.request('stay', w.t)
    w.run(1.5)
    assert (m.mode, m.phase) == (Mode.IDLE, Phase.ACTIVE)
    assert 'did not enable' in m.reason


def test_retarget_mid_transition():
    m, w = make()
    w.dog_lag = 0.3
    m.request('stay', w.t)
    w.run(0.1)
    assert m.phase == Phase.ENTERING
    m.request('follow', w.t)
    w.run(1.5)
    assert (m.mode, m.phase) == (Mode.FOLLOW, Phase.ACTIVE)


def test_idle_turns_everything_off():
    m, w = make()
    m.request('dance', w.t)  # operator may go straight from idle to dance
    w.run(1.5)
    m.request('idle', w.t)
    w.run(0.5)
    assert (m.mode, m.phase) == (Mode.IDLE, Phase.ACTIVE)
    assert not w.dog and not w.dance
