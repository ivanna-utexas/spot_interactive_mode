"""Timeline and runner tests against the shipped Dancing Queen timeline."""

import os

import pytest

from spot_interactive_mode.dance_moves import MOVES
from spot_interactive_mode.dance_runner import DanceRunner
from spot_interactive_mode.dance_timeline import (
    NEUTRAL_POSE, TRANSITION_BLEND_SEC, DanceTimeline)

TIMELINE = os.path.join(os.path.dirname(__file__), '..', 'config', 'timeline.json')
DT = 0.02


@pytest.fixture
def timeline():
    return DanceTimeline.from_file(TIMELINE)


def test_shipped_timeline_loads(timeline):
    assert len(timeline.segments) == 6
    assert timeline.duration == pytest.approx(44.011)
    assert timeline.gaits_used == [2, 8]


def test_rejects_unknown_move():
    with pytest.raises(ValueError, match='unknown pose move'):
        DanceTimeline([{'start': 0, 'end': 1, 'move': 'moonwalk', 'type': 'pose'}])


def test_pose_and_twist_never_together(timeline):
    t = 0.0
    while t < timeline.duration:
        s = timeline.sample(t)
        assert not (s.pose is not None and s.twist is not None)
        assert s.needs_stand_reassert == (s.pose is not None)
        t += DT


def test_poses_within_limits(timeline):
    t = 0.0
    while t < timeline.duration:
        s = timeline.sample(t)
        if s.pose:
            r, p, y, h = s.pose
            assert max(abs(r), abs(p), abs(y)) <= 0.4 + 1e-9
            assert abs(h) <= 0.3 + 1e-9
        t += DT


def test_pose_to_pose_boundary_is_continuous(timeline):
    # circle_sway -> turn_around_sway at 17.317 s: no snap across the boundary.
    b = 17.317
    before = timeline.sample(b - 1e-4).pose
    after = timeline.sample(b + 1e-4).pose
    assert max(abs(x - y) for x, y in zip(before, after)) < 0.01
    # and the outgoing segment is not faded toward neutral in its tail
    tail = timeline.sample(b - TRANSITION_BLEND_SEC / 2).pose
    full = MOVES['circle_sway'](b - TRANSITION_BLEND_SEC / 2 - 11.020)
    assert tail == pytest.approx(full)


def test_gait_requested_ahead_of_locomotion(timeline):
    # foot_shuffle (HOP=8) starts at 29.670; ask for it 1.5 s early.
    assert timeline.sample(29.670 - 1.4).gait_request == 8
    assert timeline.sample(29.670 - 1.6).gait_request is None
    assert timeline.sample(0.0).gait_request == 2


def run(runner, t0, seconds):
    outs, t = [], t0
    while t < t0 + seconds:
        outs.append((t, runner.step(t)))
        t += DT
    return outs, t


def test_runner_full_routine_cleans_up(timeline):
    r = DanceRunner(timeline, audio_lead_sec=0.0, rest_locomotion_mode=1)
    r.start(0.0)
    outs, _ = run(r, 0.0, timeline.duration + 1.0)
    assert outs[0][1].launch_audio
    assert sum(o.launch_audio for _, o in outs) == 1
    assert not any(o.stop_audio for _, o in outs), 'natural end lets audio finish'
    last = [o for _, o in outs if o.finished]
    assert len(last) == 1
    assert last[0].pose == NEUTRAL_POSE and last[0].final_stand
    assert last[0].gait_request == 1, 'gait must go back to walk after HOP'
    assert not r.running


def test_runner_abort_mid_pose_ramps_out(timeline):
    r = DanceRunner(timeline, abort_ramp_sec=0.5)
    r.start(0.0)
    _, t = run(r, 0.0, 14.0)          # inside circle_sway (pose)
    r.abort(t)
    outs, _ = run(r, t, 1.0)
    ramp = [o for _, o in outs if o.pose is not None and not o.final_stand]
    assert ramp and all(o.stand_reassert for o in ramp)
    mags = [max(abs(v) for v in o.pose) for o in ramp]
    assert mags == sorted(mags, reverse=True), 'monotonic ramp to neutral'
    assert outs[0][1].stop_audio
    end = [o for _, o in outs if o.final_stand]
    assert len(end) == 1 and not end[0].finished and end[0].gait_request == 1
    assert not r.running


def test_runner_abort_mid_locomotion_stops_base(timeline):
    r = DanceRunner(timeline)
    r.start(0.0)
    _, t = run(r, 0.0, 5.0)           # inside side_step (cmd_vel)
    r.abort(t)
    out = r.step(t)
    assert out.twist == (0.0, 0.0, 0.0)
    assert not out.stand_reassert
    assert out.final_stand and not r.running


def test_audio_lead_positive_and_negative(timeline):
    r = DanceRunner(timeline, audio_lead_sec=0.3)
    r.start(0.0)
    first = r.step(0.0)
    assert first.launch_audio and first.pose is None and first.twist is None
    assert r.step(0.31).twist is not None

    r = DanceRunner(timeline, audio_lead_sec=-0.3)
    r.start(0.0)
    first = r.step(0.0)
    assert not first.launch_audio and first.twist is not None
    assert r.step(0.31).launch_audio
