"""Tests for the draft Dior (Pop Smoke) timeline."""

import os

import pytest

from spot_interactive_mode.dance_runner import DanceRunner
from spot_interactive_mode.dance_timeline import NEUTRAL_POSE, DanceTimeline

TIMELINE = os.path.join(os.path.dirname(__file__), '..', 'config', 'dior.json')
DT = 0.02


@pytest.fixture
def timeline():
    return DanceTimeline.from_file(TIMELINE)


def test_timeline_loads(timeline):
    assert len(timeline.segments) == 7
    assert timeline.duration == pytest.approx(44.0)
    assert timeline.gaits_used == [2, 8], 'only gaits Dancing Queen already uses'


def test_no_gaps(timeline):
    segs = timeline.segments
    assert segs[0]['start'] == 0.0
    assert segs[-1]['end'] == pytest.approx(timeline.duration)
    for prev, seg in zip(segs, segs[1:]):
        assert seg['start'] == pytest.approx(prev['end']), seg['move']


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
