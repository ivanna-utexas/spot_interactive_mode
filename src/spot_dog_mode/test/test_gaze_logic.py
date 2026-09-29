import math

import pytest

from spot_dog_mode.gaze_logic import (
    StableNearestSelector,
    TargetCandidate,
    absolute_aim,
    deadbanded_velocity,
    elevation_height,
    estimated_head_z,
    first_order_step,
    gaze_angles,
    height_elevation,
    smoothed_rate_limited_step,
)


def candidate(track_id, distance, lateral=0.0, z=0.0):
    return TargetCandidate(track_id, distance, lateral, z, head_z=1.2)


def test_selection_uses_planar_nearest_distance():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    low = candidate(1, 2.0, z=0.0)
    high = candidate(2, 1.5, z=10.0)

    result = selector.choose([low, high], now=0.0)

    assert result.candidate.track_id == 2


def test_hysteresis_keeps_current_target_near_a_noisy_tie():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    initial = selector.choose([candidate(1, 2.0)], now=0.0)
    assert initial.candidate.track_id == 1

    noisy_distances = (1.85, 1.72, 1.88, 1.75)
    for index, contender_distance in enumerate(noisy_distances, start=1):
        result = selector.choose(
            [candidate(1, 2.0), candidate(2, contender_distance)],
            now=index * 0.1,
        )
        assert result.candidate.track_id == 1
        assert not result.changed


def test_sustained_meaningful_advantage_switches_after_debounce():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    selector.choose([candidate(1, 2.0)], now=0.0)

    first = selector.choose(
        [candidate(1, 2.0), candidate(2, 1.5)], now=0.10
    )
    second = selector.choose(
        [candidate(1, 2.0), candidate(2, 1.45)], now=0.31
    )

    assert first.candidate.track_id == 1
    assert second.candidate.track_id == 2
    assert second.changed


def test_large_advantage_switches_immediately():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    selector.choose([candidate(1, 2.0)], now=0.0)

    result = selector.choose(
        [candidate(1, 2.0), candidate(2, 1.0)], now=0.05
    )

    assert result.candidate.track_id == 2
    assert result.changed


def test_crossing_targets_do_not_churn_until_one_is_clearly_nearer():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    selector.choose([candidate(1, 1.8), candidate(2, 2.2)], now=0.0)

    crossing_samples = (
        (0.10, 1.9, 2.0),
        (0.20, 2.0, 1.9),
        (0.30, 2.1, 1.8),
    )
    for sample_time, first_distance, second_distance in crossing_samples:
        result = selector.choose(
            [
                candidate(1, first_distance),
                candidate(2, second_distance),
            ],
            now=sample_time,
        )
        assert result.candidate.track_id == 1

    selector.choose(
        [candidate(1, 2.3), candidate(2, 1.7)], now=0.4
    )
    switched = selector.choose(
        [candidate(1, 2.4), candidate(2, 1.7)], now=0.61
    )
    assert switched.candidate.track_id == 2


def test_rapidly_approaching_prediction_prompts_an_immediate_switch():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    selector.choose([candidate(1, 2.0)], now=0.0)
    approaching = TargetCandidate(
        track_id=2,
        x=2.2,
        y=0.0,
        z=0.0,
        vx=-3.0,
        head_z=1.2,
    ).predicted(0.35)

    result = selector.choose(
        [candidate(1, 2.0), approaching], now=0.05
    )

    assert result.candidate.track_id == 2
    assert result.changed


def test_spatial_reassociation_tolerates_an_unstable_id():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.8)
    selector.choose([candidate(10, 2.0)], now=0.0)

    result = selector.choose([candidate(99, 2.1, lateral=0.1)], now=0.1)

    assert result.candidate.track_id == 99
    assert not result.changed


def test_missing_current_target_requires_debounce_before_switch():
    selector = StableNearestSelector(0.3, 0.2, 0.75, 0.2)
    selector.choose([candidate(1, 2.0)], now=0.0)

    assert selector.choose([candidate(2, 3.0)], now=0.1).candidate is None
    result = selector.choose([candidate(2, 2.9)], now=0.31)

    assert result.candidate.track_id == 2
    assert result.changed


def test_yaw_pitch_geometry_and_saturation():
    yaw, pitch = gaze_angles(
        x=1.0,
        y=1.0,
        head_z=1.0,
        pitch_sign=-1.0,
        max_yaw=0.4,
        max_pitch=1.0,
    )
    assert yaw == pytest.approx(0.4)
    assert pitch == pytest.approx(-math.atan2(1.0, math.sqrt(2.0)))

    behind_yaw, saturated_pitch = gaze_angles(
        -1.0, 0.1, 1.0, -1.0, 0.4, 0.4
    )
    assert behind_yaw == pytest.approx(0.4)
    assert saturated_pitch == pytest.approx(-0.4)


def test_smoothing_is_time_correct_and_rate_limited():
    slow = smoothed_rate_limited_step(0.0, 1.0, 0.1, 0.2, 0.8)
    assert slow == pytest.approx(0.08)

    filtered = smoothed_rate_limited_step(0.0, 0.1, 0.1, 0.2, 10.0)
    assert filtered == pytest.approx(0.1 * (1.0 - math.exp(-0.5)))


def test_head_height_fallback_handles_missing_or_invalid_boxes():
    assert estimated_head_z(0.2, 1.6, 0.52, 1.7) == pytest.approx(1.6)
    assert estimated_head_z(0.2, None, 0.52, 1.7) == pytest.approx(1.18)


def test_velocity_prediction_is_bounded_by_requested_horizon():
    observed = TargetCandidate(
        track_id=1,
        x=2.0,
        y=0.0,
        z=0.0,
        vx=-1.0,
        head_z=1.2,
    )

    assert observed.predicted(0.15).x == pytest.approx(1.85)
    assert observed.predicted(-1.0).x == pytest.approx(2.0)


def test_absolute_aim_adds_the_offset_the_body_already_holds():
    # Someone 0.10 rad off a body already turned 0.25 rad sits at 0.35 rad
    # from the footprint, which is what the driver is commanded with.
    assert absolute_aim(0.10, 0.25, 0.4) == pytest.approx(0.35)
    # The sum is what gets bounded, not the parts.
    assert absolute_aim(0.30, 0.25, 0.4) == pytest.approx(0.4)
    assert absolute_aim(-0.30, -0.25, 0.4) == pytest.approx(-0.4)


def test_velocity_deadband_drops_noise_and_keeps_walking_speed():
    assert deadbanded_velocity((0.2, -0.1, 0.0), 0.35, 3.0) == (0.0, 0.0, 0.0)
    # Implausible speeds are still rejected outright.
    assert deadbanded_velocity((5.0, 0.0, 0.0), 0.35, 3.0) == (0.0, 0.0, 0.0)

    walking = deadbanded_velocity((1.4, 0.0, 0.0), 0.35, 3.0)
    assert walking[0] == pytest.approx(1.4 - 0.35)

    # Shrinking rather than gating keeps motion continuous at the threshold.
    just_over = deadbanded_velocity((0.36, 0.0, 0.0), 0.35, 3.0)
    assert 0.0 < just_over[0] < 0.02


def test_head_height_and_elevation_round_trip():
    height = elevation_height(0.35, 3.0)
    assert height_elevation(height, 3.0) == pytest.approx(0.35)
    # Height is range-independent, so the same person closer reads a
    # larger angle without the filtered height having to change.
    assert height_elevation(height, 1.5) > 0.35


def test_first_order_step_is_time_correct():
    assert first_order_step(0.0, 1.0, 0.0, 0.5) == 0.0
    assert first_order_step(0.0, 1.0, 0.1, 0.0) == 1.0
    one_tau = first_order_step(0.0, 1.0, 0.5, 0.5)
    assert one_tau == pytest.approx(1.0 - math.exp(-1.0))
    # Two half steps must match one whole step.
    halves = first_order_step(first_order_step(0.0, 1.0, 0.25, 0.5), 1.0, 0.25, 0.5)
    assert halves == pytest.approx(one_tau)
