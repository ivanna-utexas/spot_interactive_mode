"""Tests for the Sultans of Swing timeline and its beat-locked moves."""

import os

import pytest

from spot_interactive_mode.dance_moves import MOVES, SOS_BEAT
from spot_interactive_mode.dance_runner import DanceRunner
from spot_interactive_mode.dance_timeline import NEUTRAL_POSE, POSE, DanceTimeline

TIMELINE = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'sultans_of_swing.json')
DT = 0.02
BAR = 4 * SOS_BEAT


@pytest.fixture
def timeline():
    return DanceTimeline.from_file(TIMELINE)


def test_timeline_loads(timeline):
    assert len(timeline.segments) == 7
    assert timeline.duration == pytest.approx(46.256)
    assert timeline.gaits_used == [2, 8], 'only gaits Dancing Queen already uses'


def test_segments_start_on_downbeats(timeline):
    # Boundaries are the downbeats detected in the clip, so each segment
    # re-locks the moves to the band (which drifts up to ~65 ms off a fixed
    # grid). Anything over a quarter beat off is a misplaced boundary.
    for seg in timeline.segments:
        bars = seg['start'] / BAR
        assert abs(bars - round(bars)) * BAR < SOS_BEAT / 4, seg['move']


def test_poses_within_limits(timeline):
    t = 0.0
    while t < timeline.duration:
        s = timeline.sample(t)
        assert not (s.pose is not None and s.twist is not None)
        if s.pose:
            r, p, y, h = s.pose
            assert max(abs(r), abs(p), abs(y)) <= 0.4 + 1e-9
            assert abs(h) <= 0.3 + 1e-9
        t += DT


def test_locomotion_returns_to_start(timeline):
    # Whole periods only, so a stepping segment ends where it began.
    for seg in timeline.segments:
        if seg['type'] != 'cmd_vel':
            continue
        cycles = (seg['end'] - seg['start']) / seg['period']
        assert cycles == pytest.approx(round(cycles), abs=0.01), seg['move']
        dist, t, dt = 0.0, seg['start'], 0.001
        while t < seg['end']:
            vx, vy, _ = timeline.sample(t).twist
            dist += (vx if seg['axis'] == 'x' else vy) * dt
            t += dt
        assert abs(dist) < 0.01, seg['move']


def test_pose_boundaries_are_continuous(timeline):
    segs = timeline.segments
    for prev, seg in zip(segs, segs[1:]):
        if prev['type'] == POSE and seg['type'] == POSE:
            b = seg['start']
            before = timeline.sample(b - 1e-4).pose
            after = timeline.sample(b + 1e-4).pose
            assert max(abs(x - y) for x, y in zip(before, after)) < 0.01, seg['move']


def test_head_bob_nods_on_the_beat():
    for beat in range(8):
        on = MOVES['head_bob'](beat * SOS_BEAT)
        off = MOVES['head_bob']((beat + 0.5) * SOS_BEAT)
        assert on[1] == pytest.approx(0.10)      # nose fully down on the beat
        assert off[1] == pytest.approx(0.0)


def test_bow_holds_at_the_end():
    a, b = MOVES['take_a_bow'](5.0), MOVES['take_a_bow'](5.8)
    assert a == pytest.approx(b)
    assert a[1] > 0.25 and a[3] < -0.15          # nose down, body low


def test_runner_full_routine_cleans_up(timeline):
    r = DanceRunner(timeline, rest_locomotion_mode=1)
    r.start(0.0)
    t, outs = 0.0, []
    while t < timeline.duration + 1.0:
        outs.append(r.step(t))
        t += DT
    last = [o for o in outs if o.finished]
    assert len(last) == 1
    assert last[0].pose == NEUTRAL_POSE and last[0].final_stand
    assert last[0].gait_request == 1
    assert not r.running
