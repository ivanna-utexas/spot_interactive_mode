"""ROS-independent target selection and gaze-control helpers."""

from dataclasses import dataclass, replace
import math
from typing import Optional, Sequence, Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class TargetCandidate:
    """One human track expressed in Spot's body frame."""

    track_id: int
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    head_z: float = 0.0

    @property
    def distance(self) -> float:
        """Planar proximity used for nearest-human selection."""
        return math.hypot(self.x, self.y)

    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def predicted(self, horizon: float) -> 'TargetCandidate':
        horizon = max(0.0, horizon)
        return replace(
            self,
            x=self.x + self.vx * horizon,
            y=self.y + self.vy * horizon,
            z=self.z + self.vz * horizon,
            head_z=self.head_z + self.vz * horizon,
        )


@dataclass(frozen=True)
class SelectionResult:
    candidate: Optional[TargetCandidate]
    changed: bool = False


class StableNearestSelector:
    """Nearest-planar target selector with hysteresis and switch debounce."""

    def __init__(
        self,
        switch_margin: float,
        switch_debounce: float,
        immediate_margin: float,
        match_radius: float,
    ) -> None:
        self.switch_margin = max(0.0, switch_margin)
        self.switch_debounce = max(0.0, switch_debounce)
        self.immediate_margin = max(self.switch_margin, immediate_margin)
        self.match_radius = max(0.0, match_radius)
        self.current_id: Optional[int] = None
        self.current_position: Optional[Tuple[float, float]] = None
        self._pending_id: Optional[int] = None
        self._pending_position: Optional[Tuple[float, float]] = None
        self._pending_since = 0.0

    def clear(self) -> None:
        self.current_id = None
        self.current_position = None
        self._clear_pending()

    def choose(
        self, candidates: Sequence[TargetCandidate], now: float
    ) -> SelectionResult:
        ordered = sorted(
            candidates, key=lambda item: (item.distance, item.track_id)
        )
        if not ordered:
            self._clear_pending()
            return SelectionResult(None)

        nearest = ordered[0]
        if self.current_id is None:
            self._adopt(nearest)
            return SelectionResult(nearest, changed=True)

        current = next(
            (
                item
                for item in ordered
                if item.track_id == self.current_id
            ),
            None,
        )
        if current is None and self.current_position is not None:
            last_x, last_y = self.current_position
            nearby = [
                item
                for item in ordered
                if math.hypot(item.x - last_x, item.y - last_y)
                <= self.match_radius
            ]
            if nearby:
                # Spatial matching handles detector IDs that reset or flicker.
                current = min(
                    nearby,
                    key=lambda item: math.hypot(
                        item.x - last_x, item.y - last_y
                    ),
                )
                self.current_id = current.track_id

        if current is None:
            if self._switch_ready(nearest, now):
                self._adopt(nearest)
                return SelectionResult(nearest, changed=True)
            return SelectionResult(None)

        if nearest.track_id == current.track_id:
            self._clear_pending()
            self._remember(current)
            return SelectionResult(current)

        advantage = current.distance - nearest.distance
        if advantage < self.switch_margin:
            self._clear_pending()
            self._remember(current)
            return SelectionResult(current)

        immediate = advantage >= self.immediate_margin
        if immediate or self._switch_ready(nearest, now):
            self._adopt(nearest)
            return SelectionResult(nearest, changed=True)

        self._remember(current)
        return SelectionResult(current)

    def _switch_ready(self, candidate: TargetCandidate, now: float) -> bool:
        if self.switch_debounce <= 0.0:
            return True

        same_pending = self._pending_id == candidate.track_id
        if not same_pending and self._pending_position is not None:
            px, py = self._pending_position
            same_pending = (
                math.hypot(candidate.x - px, candidate.y - py)
                <= self.match_radius
            )

        if not same_pending:
            self._pending_since = now

        self._pending_id = candidate.track_id
        self._pending_position = candidate.position
        pending_age = now - self._pending_since
        return same_pending and pending_age >= self.switch_debounce

    def _adopt(self, candidate: TargetCandidate) -> None:
        self.current_id = candidate.track_id
        self.current_position = candidate.position
        self._clear_pending()

    def _remember(self, candidate: TargetCandidate) -> None:
        self.current_id = candidate.track_id
        self.current_position = candidate.position

    def _clear_pending(self) -> None:
        self._pending_id = None
        self._pending_position = None
        self._pending_since = 0.0


def gaze_angles(
    x: float,
    y: float,
    head_z: float,
    pitch_sign: float,
    max_yaw: float,
    max_pitch: float,
) -> Tuple[float, float]:
    """Return bounded body yaw/pitch toward a body-frame head position."""
    yaw = math.atan2(y, x)
    horizontal = max(math.hypot(x, y), 0.1)
    pitch = pitch_sign * math.atan2(head_z, horizontal)
    return (
        clamp(yaw, -abs(max_yaw), abs(max_yaw)),
        clamp(pitch, -abs(max_pitch), abs(max_pitch)),
    )


def absolute_aim(relative: float, offset: float, limit: float) -> float:
    """Convert a body-relative angle into a footprint-relative command.

    Body pose is commanded relative to Spot's footprint, but detections are
    measured in the `body` frame, which rotates with that same commanded pose.
    Aiming straight at the measured angle therefore closes a feedback loop on
    the controller's own output: the body turns, the measured angle shrinks,
    the command backs off, and the gaze oscillates. Adding the offset the body
    is already holding recovers the footprint-relative angle, which is the
    quantity the driver actually accepts.
    """
    return clamp(relative + offset, -abs(limit), abs(limit))


def deadbanded_velocity(
    velocity: Sequence[float], min_speed: float, max_speed: float
) -> Tuple[float, float, float]:
    """Drop tracker velocity noise while preserving genuine motion.

    The pedestrian tracker blends a noisy learned velocity into every update,
    so a person standing still still reports a few tenths of a metre per second
    in a random direction. Multiplied by the look-ahead horizon that becomes a
    position that wanders, which the gaze faithfully follows as a sway. The
    magnitude is shrunk rather than gated so motion starts continuously.
    """
    vx, vy, vz = (float(value) for value in velocity)
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    if speed > max(0.0, max_speed):
        return (0.0, 0.0, 0.0)
    floor = max(0.0, min_speed)
    if speed <= floor:
        return (0.0, 0.0, 0.0)
    scale = (speed - floor) / speed
    return (vx * scale, vy * scale, vz * scale)


def elevation_height(elevation: float, horizontal: float) -> float:
    """Head height above the aim plane implied by an elevation angle."""
    return horizontal * math.tan(clamp(elevation, -1.4, 1.4))


def height_elevation(height: float, horizontal: float) -> float:
    """Inverse of :func:`elevation_height`."""
    return math.atan2(height, max(horizontal, 0.1))


def first_order_step(current: float, target: float, dt: float, tau: float) -> float:
    """Advance a first-order lag toward `target` over `dt`."""
    if dt <= 0.0:
        return current
    if tau <= 0.0:
        return target
    return current + (1.0 - math.exp(-dt / tau)) * (target - current)


def smoothed_rate_limited_step(
    current: float,
    desired: float,
    dt: float,
    smoothing_tau: float,
    max_rate: float,
) -> float:
    """Apply a time-correct low-pass step followed by a hard slew limit."""
    if dt <= 0.0:
        return current
    if smoothing_tau <= 0.0:
        filtered_delta = desired - current
    else:
        alpha = 1.0 - math.exp(-dt / smoothing_tau)
        filtered_delta = alpha * (desired - current)
    max_delta = max(0.0, max_rate) * dt
    return current + clamp(filtered_delta, -max_delta, max_delta)


def estimated_head_z(
    transformed_center_z: float,
    transformed_top_z: Optional[float],
    body_height_above_ground: float,
    default_person_height: float,
) -> float:
    """Use a transformed box top or a standing-height fallback."""
    if transformed_top_z is not None and math.isfinite(transformed_top_z):
        return transformed_top_z
    return -body_height_above_ground + default_person_height


def finite_vector(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)
