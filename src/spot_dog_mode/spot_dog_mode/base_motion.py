"""Blended base rotation and person approach as one (v, w) stream.

Turning to face a person and walking toward them used to be mutually
exclusive states, with a full stop at every handoff: turn in place, stop,
settle, then walk. This controller merges the two into one continuous
unicycle command. Yaw follows the bearing servo throughout; forward speed
follows the range error scaled by a continuous alignment factor, so motion
toward an off-axis person is an arc that tightens as the bearing closes
rather than a turn-then-walk sequence.
"""

import math
from typing import Optional, Tuple

from .gaze_logic import clamp


def wrap_angle(angle: float) -> float:
    """Fold an angle into (-pi, pi].

    Bearings to a person behind Spot sit near +/-pi, where an unwrapped angle
    flips sign across the boundary and would spin the robot the long way round.
    """
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


class BaseMotionController:
    """Acceleration-limited (v, w) toward a tracked person.

    Rotation keeps the hysteretic commitment of the old in-place turn servo:
    entry and exit thresholds are widely separated because a single one makes
    the robot start and stop repeatedly while a person stands at the boundary.
    Walking keeps the old approach controller's braking-distance bound and
    resume margin. What is new is that both run in the same cycle: forward
    speed feathers in and out with alignment instead of gating on it.
    """

    # Below roughly this forward speed Spot steps in place rather than
    # walking, so a proportional tail creeping under it is fictional motion:
    # the range would close only asymptotically while the robot marches on
    # the spot. Commands under it count as having arrived.
    ARRIVAL_SPEED = 0.05

    # Under this commanded forward speed the base is effectively turning in
    # place, which is the only regime where Spot ignores small yaw rates and
    # the minimum-rate floor is needed. While walking, a small yaw correction
    # is honored, and a floor there makes the heading snap between the
    # floor's two signs — a visible steering oscillation.
    IN_PLACE_SPEED = 0.05

    def __init__(
        self,
        enter_yaw: float,
        exit_yaw: float,
        enter_debounce: float,
        yaw_gain: float,
        max_yaw_rate: float,
        min_yaw_rate: float,
        yaw_accel_limit: float,
        stop_distance: float,
        resume_margin: float,
        speed_gain: float,
        max_speed: float,
        accel_limit: float,
        alignment_yaw: float,
    ) -> None:
        self.enter_yaw = abs(enter_yaw)
        self.exit_yaw = min(abs(exit_yaw), self.enter_yaw)
        self.enter_debounce = max(0.0, enter_debounce)
        self.yaw_gain = abs(yaw_gain)
        self.max_yaw_rate = abs(max_yaw_rate)
        self.min_yaw_rate = min(abs(min_yaw_rate), self.max_yaw_rate)
        self.yaw_accel_limit = max(0.0, yaw_accel_limit)
        self.stop_distance = max(0.0, stop_distance)
        self.resume_margin = max(0.0, resume_margin)
        self.speed_gain = abs(speed_gain)
        self.max_speed = abs(max_speed)
        self.accel_limit = max(0.0, accel_limit)
        self.alignment_yaw = abs(alignment_yaw)
        self.turning = False
        self.walking = False
        self._linear = 0.0
        self._angular = 0.0
        self._over_since: Optional[float] = None

    @property
    def linear(self) -> float:
        """Most recently commanded forward speed."""
        return self._linear

    @property
    def angular(self) -> float:
        """Most recently commanded yaw rate."""
        return self._angular

    @property
    def engaged(self) -> bool:
        """Whether either axis still has a claim on the base."""
        return self.turning or self.walking

    def reset(self) -> None:
        self.turning = False
        self.walking = False
        self._linear = 0.0
        self._angular = 0.0
        self._over_since = None

    def should_turn(self, bearing: float, now: float) -> bool:
        """Update and report the rotation commitment.

        While turning, only the exit threshold applies, so the turn always
        runs far enough to bring the person back inside the pose envelope.
        """
        magnitude = abs(wrap_angle(bearing))
        if self.turning:
            self.turning = magnitude > self.exit_yaw
            if not self.turning:
                self._over_since = None
            return self.turning

        if magnitude < self.enter_yaw:
            self._over_since = None
            return False

        if self._over_since is None:
            self._over_since = now
        # Someone crossing behind Spot briefly is not a reason to turn.
        self.turning = (now - self._over_since) >= self.enter_debounce
        return self.turning

    def commit_turn(self) -> None:
        """Adopt the rotation commitment without threshold or debounce.

        Lets a caller start a deliberate recentring turn from inside the
        entry threshold; the turn then runs to the exit threshold as usual.
        """
        self.turning = True
        self._over_since = None

    def wants_walk(self, range_: float) -> bool:
        """Whether a fresh walk should start toward `range_`.

        Restart only beyond distance + margin, so arriving and drifting a
        few centimetres does not chatter across the threshold. Hysteresis
        lives here; the stopping distance itself is never moved.
        """
        return range_ > self.stop_distance + self.resume_margin

    def alignment_factor(self, bearing: float) -> float:
        """Continuous 1..0 scale on forward speed as the bearing opens.

        A raised cosine over twice `alignment_yaw`: full speed straight
        ahead, half speed at `alignment_yaw`, zero beyond twice it. The old
        binary gate at `alignment_yaw` froze and released the walk around
        one threshold, which read as stutter whenever a person drifted
        across it.
        """
        cutoff = 2.0 * self.alignment_yaw
        magnitude = abs(wrap_angle(bearing))
        if cutoff <= 0.0 or magnitude >= cutoff:
            return 0.0
        return 0.5 * (1.0 + math.cos(math.pi * magnitude / cutoff))

    def update_walk(self, range_: float, allow_walk: bool) -> bool:
        """Advance the walk commitment one cycle and report it.

        `allow_walk` is the caller's authorization for translation (approach
        enabled and a real, fresh detection behind the range). The
        commitment is not cleared by a momentary gap — the target-lost
        timers decide that — so a one-frame occlusion does not force the
        walk back out through the resume margin before it can continue.
        """
        if allow_walk:
            if self.walking:
                self.walking = not self.arrived(range_)
            else:
                self.walking = self.wants_walk(range_)
        return self.walking

    def step(
        self, bearing: float, range_: float, dt: float, allow_walk: bool
    ) -> Tuple[float, float]:
        """Advance both commands one control cycle.

        Rotation may coast on a modelled bearing through a data gap, but
        translation never runs blind: withdrawing `allow_walk` zeroes it at
        once rather than ramping it out.
        """
        bearing = wrap_angle(bearing)
        desired_v = 0.0
        if allow_walk and self.walking:
            error = max(0.0, range_ - self.stop_distance)
            desired_v = clamp(self.speed_gain * error, 0.0, self.max_speed)
            # Never faster than the deceleration limit can stop from within
            # the remaining error: decelerate into the goal, not brake at it.
            desired_v = min(
                desired_v, math.sqrt(2.0 * self.accel_limit * error)
            )
            desired_v *= self.alignment_factor(bearing)

        desired_w = clamp(
            self.yaw_gain * bearing, -self.max_yaw_rate, self.max_yaw_rate
        )
        in_place = (
            self.turning
            and desired_v == 0.0
            and abs(self._linear) <= self.IN_PLACE_SPEED
        )
        if in_place and desired_w != 0.0 and abs(desired_w) < self.min_yaw_rate:
            # Spot ignores very small in-place yaw rates, so a proportional
            # command that decays smoothly to zero would stall short.
            desired_w = math.copysign(self.min_yaw_rate, desired_w)

        if allow_walk:
            self._linear = self._toward(
                self._linear, desired_v, self.accel_limit, dt
            )
        else:
            self._linear = 0.0
        self._angular = self._toward(
            self._angular, desired_w, self.yaw_accel_limit, dt
        )
        return (self._linear, self._angular)

    def decay(self, dt: float) -> Tuple[float, float]:
        """Ramp both commands to zero without a step change.

        The rate floor means a turn is still commanding real motion when the
        target angle is reached, so cutting straight to zero asks Spot to
        plant mid-stride.
        """
        self._linear = self._toward(self._linear, 0.0, self.accel_limit, dt)
        self._angular = self._toward(
            self._angular, 0.0, self.yaw_accel_limit, dt
        )
        return (self._linear, self._angular)

    def arrived(self, range_: float) -> bool:
        """Whether the remaining error is too small to keep walking on.

        Both the request and the command must be under the arrival speed:
        range alone would declare arrival while still moving briskly through
        the threshold, and speed alone would declare it while the alignment
        factor holds the walk paused far from the goal.
        """
        error = max(0.0, range_ - self.stop_distance)
        return (
            self.speed_gain * error <= self.ARRIVAL_SPEED
            and self._linear <= self.ARRIVAL_SPEED
        )

    @staticmethod
    def _toward(
        current: float, desired: float, limit: float, dt: float
    ) -> float:
        if dt <= 0.0 or limit <= 0.0:
            return desired
        max_delta = limit * dt
        return current + clamp(desired - current, -max_delta, max_delta)
