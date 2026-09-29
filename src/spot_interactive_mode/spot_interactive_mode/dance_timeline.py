"""Timeline evaluation for the dance routine, with no ROS dependency.

This is the segment logic of spot_joy's dance_sequence.py (the summer
Dancing Queen demo) pulled out of its node so the interactive-mode state
machine can run a routine as one of its modes, start it on demand, and abort
it part-way. Behavior matches dance_sequence.py:

  - "pose" segments return a body pose (roll, pitch, yaw, height). They need
    the stand re-assert running, because /body_pose alone does not move Spot:
    the driver only applies it on the next stand command.
  - "cmd_vel" / "cmd_vel_turn" segments return a twist. The stand re-assert
    must be OFF for these; stand is a hold-still command and fights walking.
  - Gaits are requested on entering a cmd_vel segment, and also
    GAIT_LOOKAHEAD_SEC before it starts so the switch has time to land.
  - A TRANSITION_BLEND_SEC envelope ramps every segment in and out; two
    adjacent pose segments cross-fade instead.
"""

import json
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .dance_moves import MOVES

TRANSITION_BLEND_SEC = 0.2
GAIT_LOOKAHEAD_SEC = 1.5

POSE = 'pose'
CMD_VEL = 'cmd_vel'
CMD_VEL_TURN = 'cmd_vel_turn'
RECORDED_GESTURE = 'recorded_gesture'
_VELOCITY_TYPES = (CMD_VEL, CMD_VEL_TURN)
_KNOWN_TYPES = (POSE, CMD_VEL, CMD_VEL_TURN, RECORDED_GESTURE)

NEUTRAL_POSE = (0.0, 0.0, 0.0, 0.0)


@dataclass
class DanceSample:
    """What the routine wants at one instant."""

    finished: bool = False
    segment_index: Optional[int] = None
    kind: Optional[str] = None                  # segment type, None when finished
    pose: Optional[Tuple[float, float, float, float]] = None   # roll, pitch, yaw, height
    twist: Optional[Tuple[float, float, float]] = None         # vx, vy, wz
    gait_request: Optional[int] = None          # locomotion mode to ask for now

    @property
    def needs_stand_reassert(self) -> bool:
        return self.kind == POSE


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """(x, y, z, w), same convention as teleop and dance_sequence.py."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def cmd_vel_oscillation(t: float, axis: str, amplitude: float, period: float):
    """Back-and-forth velocity used by side_step (y) and foot_shuffle (x)."""
    v = amplitude * math.sin(2.0 * math.pi * t / period)
    if axis == 'y':
        return (0.0, v)
    return (v, 0.0)


def angular_turn_profile(t: float, duration: float, net_yaw_rad: float) -> float:
    """Half-sine yaw rate that integrates to exactly net_yaw_rad."""
    if duration <= 0:
        return 0.0
    peak_omega = net_yaw_rad * math.pi / (2.0 * duration)
    return peak_omega * math.sin(math.pi * t / duration)


class DanceTimeline:
    """A validated list of segments that can be sampled at any time t."""

    def __init__(self, segments: List[dict], duration: Optional[float] = None):
        if not segments:
            raise ValueError('timeline has no segments')
        self.segments = sorted(segments, key=lambda s: s['start'])
        self.duration = (
            float(duration) if duration is not None
            else float(self.segments[-1]['end'])
        )
        self._validate()

    @classmethod
    def from_file(cls, path: str) -> 'DanceTimeline':
        with open(path, 'r') as f:
            data = json.load(f)
        return cls(data['segments'], data.get('clip_duration_sec'))

    @classmethod
    def calibration(cls) -> 'DanceTimeline':
        """A single 3 s bounce at t=0 for audio-sync testing."""
        return cls([{'start': 0.0, 'end': 3.0, 'section': 'calibration',
                     'move': 'bounce', 'type': POSE}], 3.0)

    def _validate(self) -> None:
        """Fail loudly at load time rather than mid-routine in front of people."""
        for i, seg in enumerate(self.segments):
            where = f"segment {i} ({seg.get('move', '?')})"
            if seg.get('type') not in _KNOWN_TYPES:
                raise ValueError(f"{where}: unknown type {seg.get('type')!r}")
            if not seg['end'] > seg['start']:
                raise ValueError(f'{where}: end must be after start')
            if seg['type'] == POSE and seg['move'] not in MOVES:
                raise ValueError(
                    f"{where}: unknown pose move; available: {sorted(MOVES)}"
                )
            if seg['type'] == CMD_VEL:
                for key in ('axis', 'amplitude', 'period'):
                    if key not in seg:
                        raise ValueError(f'{where}: cmd_vel segment needs {key!r}')
            if seg['type'] == CMD_VEL_TURN and 'net_yaw_deg' not in seg:
                raise ValueError(f"{where}: cmd_vel_turn segment needs 'net_yaw_deg'")

    @staticmethod
    def _pose_handoff(first: Optional[dict], second: Optional[dict]) -> bool:
        """Whether `second` cross-fades directly out of `first`."""
        return (first is not None and second is not None
                and first['type'] == POSE and second['type'] == POSE
                and math.isclose(first['end'], second['start'], abs_tol=1e-6))

    @property
    def gaits_used(self) -> List[int]:
        return sorted({int(s['locomotion_mode']) for s in self.segments
                       if 'locomotion_mode' in s})

    def find_segment_index(self, t: float) -> Optional[int]:
        """Index of the active segment, len(segments) past the end, None in a gap."""
        for i, seg in enumerate(self.segments):
            if seg['start'] <= t < seg['end']:
                return i
        if t >= self.segments[-1]['end']:
            return len(self.segments)
        return None

    def sample(self, t: float) -> DanceSample:
        index = self.find_segment_index(t)
        if index is None:
            # A gap between segments: hold still rather than guess.
            return DanceSample(kind=None)
        if index >= len(self.segments):
            return DanceSample(finished=True, segment_index=index)

        seg = self.segments[index]
        local_t = t - seg['start']
        time_remaining = seg['end'] - t
        out = DanceSample(segment_index=index, kind=seg['type'])

        # This segment's own gait, then the next one's if it starts soon.
        # The caller de-duplicates, so asking every tick is fine.
        if seg['type'] in _VELOCITY_TYPES and 'locomotion_mode' in seg:
            out.gait_request = int(seg['locomotion_mode'])
        nxt = self.segments[index + 1] if index + 1 < len(self.segments) else None
        if (nxt is not None and time_remaining <= GAIT_LOOKAHEAD_SEC
                and nxt['type'] in _VELOCITY_TYPES and 'locomotion_mode' in nxt):
            out.gait_request = int(nxt['locomotion_mode'])

        blend_in = min(1.0, local_t / TRANSITION_BLEND_SEC)
        blend_out = min(1.0, time_remaining / TRANSITION_BLEND_SEC)
        envelope = min(blend_in, blend_out)

        if seg['type'] == POSE:
            pose = MOVES[seg['move']](local_t)
            prev = self.segments[index - 1] if index > 0 else None
            fade_from_prev = self._pose_handoff(prev, seg)
            # The incoming pose segment owns the cross-fade, so the outgoing
            # one must not also fade itself to neutral. dance_sequence.py did
            # both, which dipped toward neutral over the last 0.2 s and then
            # snapped back to the full outgoing pose on the boundary tick.
            fade_in = 1.0 if fade_from_prev else blend_in
            fade_out = 1.0 if self._pose_handoff(seg, nxt) else blend_out
            if fade_from_prev and blend_in < 1.0:
                prev_pose = MOVES[prev['move']](prev['end'] - prev['start'])
                pose = tuple(p * (1.0 - blend_in) + c * blend_in
                             for p, c in zip(prev_pose, pose))
            scale = min(fade_in, fade_out)
            out.pose = tuple(v * scale for v in pose)

        elif seg['type'] == CMD_VEL:
            vx, vy = cmd_vel_oscillation(
                local_t, seg['axis'], seg['amplitude'], seg['period'])
            out.twist = (vx * envelope, vy * envelope, 0.0)

        elif seg['type'] == CMD_VEL_TURN:
            # No envelope: the half-sine is already zero at both ends, and
            # scaling it would undershoot the intended net turn.
            omega = angular_turn_profile(
                local_t, seg['end'] - seg['start'],
                math.radians(seg['net_yaw_deg']))
            out.twist = (0.0, 0.0, omega)

        # RECORDED_GESTURE: not wired up (same as dance_sequence.py); the
        # caller holds still through it.
        return out
