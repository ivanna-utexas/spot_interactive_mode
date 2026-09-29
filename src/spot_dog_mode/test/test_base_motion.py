import math

import pytest

from spot_dog_mode.base_motion import BaseMotionController, wrap_angle


def make_controller(**overrides):
    settings = dict(
        enter_yaw=0.35,
        exit_yaw=0.12,
        enter_debounce=0.3,
        yaw_gain=1.2,
        max_yaw_rate=0.8,
        min_yaw_rate=0.15,
        yaw_accel_limit=1.5,
        stop_distance=2.0,
        resume_margin=0.25,
        speed_gain=0.5,
        max_speed=0.4,
        accel_limit=0.5,
        alignment_yaw=0.3,
    )
    settings.update(overrides)
    return BaseMotionController(**settings)


def settle(
    controller, bearing=0.0, range_=0.0, steps=200, dt=0.05, allow_walk=True
):
    for _ in range(steps):
        controller.update_walk(range_, allow_walk)
        linear, angular = controller.step(bearing, range_, dt, allow_walk)
    return linear, angular


def integrate(controller, start, dt=0.05, steps=1000):
    """Walk the closed loop against a stationary person straight ahead."""
    distance = start
    speeds = []
    for _ in range(steps):
        controller.update_walk(distance, True)
        linear, _ = controller.step(0.0, distance, dt, True)
        distance -= linear * dt
        speeds.append(linear)
    return distance, speeds


@pytest.mark.parametrize(
    'angle, expected',
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),
        (1.05 * math.pi, -0.95 * math.pi),
        (3.0 * math.pi, math.pi),
    ],
)
def test_bearings_fold_into_a_half_turn(angle, expected):
    # A person just past directly behind must read as a small turn to the near
    # side, not a whole revolution the other way.
    assert wrap_angle(angle) == pytest.approx(expected)


# Rotation commitment.


def test_a_target_inside_the_pose_envelope_never_starts_a_turn():
    controller = make_controller()

    assert not any(
        controller.should_turn(0.3, now) for now in (0.0, 1.0, 2.0)
    )


def test_someone_crossing_behind_briefly_is_ignored():
    controller = make_controller()

    assert not controller.should_turn(2.0, 0.0)
    assert not controller.should_turn(2.0, 0.2)
    # Back inside the envelope before the debounce elapsed.
    assert not controller.should_turn(0.1, 0.25)
    assert not controller.should_turn(2.0, 0.45)


def test_a_sustained_bearing_starts_a_turn():
    controller = make_controller()

    controller.should_turn(2.0, 0.0)
    assert controller.should_turn(2.0, 0.35)


def test_a_turn_runs_past_the_entry_threshold_before_stopping():
    controller = make_controller()
    controller.should_turn(2.0, 0.0)
    assert controller.should_turn(2.0, 0.35)

    # Falling below the entry threshold is not enough: stopping there would
    # leave the person at the edge of the pose envelope and start again at once.
    assert controller.should_turn(0.30, 0.40)
    assert controller.should_turn(0.15, 0.45)
    assert not controller.should_turn(0.10, 0.50)


def test_a_target_sitting_on_the_threshold_does_not_chatter():
    controller = make_controller()
    decisions = []
    for step in range(40):
        # Jitter across the entry threshold, as a standing person's detections do.
        bearing = 0.35 + 0.01 * math.sin(step)
        decisions.append(controller.should_turn(bearing, step * 0.05))

    # Either it never commits or it commits once and holds to the exit
    # threshold. What it must not do is start and stop over and over.
    assert sum(
        later != earlier for earlier, later in zip(decisions, decisions[1:])
    ) <= 1


def test_a_committed_turn_skips_threshold_and_debounce():
    controller = make_controller()

    # A deliberate recentring turn starts from inside the entry threshold...
    controller.commit_turn()
    assert controller.turning
    assert controller.should_turn(0.2, 0.0)
    # ...and still runs to the exit threshold as usual.
    assert not controller.should_turn(0.1, 0.05)


def test_the_turn_rate_follows_the_side_the_person_is_on():
    left = make_controller()
    right = make_controller()

    assert left.step(1.0, 0.0, 1.0, False)[1] > 0.0
    assert right.step(-1.0, 0.0, 1.0, False)[1] < 0.0


def test_the_turn_rate_is_capped_and_ramped():
    controller = make_controller()

    first = controller.step(math.pi, 0.0, 0.05, False)[1]
    assert first == pytest.approx(1.5 * 0.05)

    for _ in range(100):
        rate = controller.step(math.pi, 0.0, 0.05, False)[1]
    assert rate == pytest.approx(0.8)


def test_a_small_residual_error_still_commands_a_usable_rate_in_place():
    controller = make_controller(yaw_accel_limit=0.0)
    controller.should_turn(2.0, 0.0)
    assert controller.should_turn(2.0, 0.5)

    # Spot ignores very small in-place yaw rates, so a purely proportional
    # command would stall short of the target instead of finishing the turn.
    assert controller.step(0.02, 0.0, 0.05, False)[1] == pytest.approx(0.15)


def test_steering_while_walking_has_no_minimum_rate_floor():
    controller = make_controller(yaw_accel_limit=0.0)
    settle(controller, bearing=0.0, range_=4.0)
    assert controller.linear > 0.2

    # While walking, a small yaw correction is honored; a floor there makes
    # the heading snap between the floor's two signs.
    _, angular = controller.step(0.02, 4.0, 0.05, True)
    assert angular == pytest.approx(0.02 * 1.2)


# Walking and blending.


def test_a_person_beyond_the_stopping_distance_is_walked_toward():
    linear, _ = settle(make_controller(), range_=4.0)

    assert linear == pytest.approx(0.4)


@pytest.mark.parametrize('range_', [2.0, 1.5, 0.5])
def test_a_person_at_or_inside_the_stopping_distance_is_not(range_):
    linear, _ = settle(make_controller(), range_=range_)

    assert linear == 0.0


def test_an_off_axis_person_is_approached_as_an_arc():
    # Turning and walking blend in one command: forward speed feathers with
    # alignment while steering closes the bearing, instead of the old
    # turn-then-walk sequence with a stop between.
    linear, angular = settle(make_controller(), bearing=0.25, range_=4.0)

    assert 0.0 < linear < 0.4
    assert angular == pytest.approx(0.25 * 1.2)


def test_forward_speed_scales_continuously_with_alignment():
    controller = make_controller()
    factors = [
        controller.alignment_factor(bearing)
        for bearing in (0.0, 0.15, 0.3, 0.45, 0.6)
    ]

    assert factors[0] == pytest.approx(1.0)
    assert factors[2] == pytest.approx(0.5)  # half speed at alignment_yaw
    assert factors[-1] == 0.0  # zero beyond twice it
    assert all(
        later < earlier for earlier, later in zip(factors, factors[1:])
    )


def test_a_badly_misaligned_person_is_steered_toward_not_walked_toward():
    linear, angular = settle(make_controller(), bearing=0.7, range_=4.0)

    assert linear == 0.0
    # Proportional steering, saturated at the yaw-rate cap.
    assert angular == pytest.approx(min(0.7 * 1.2, 0.8))


def test_losing_alignment_mid_walk_ramps_the_speed_off():
    controller = make_controller()
    settle(controller, range_=4.0)

    speeds = []
    for _ in range(30):
        controller.update_walk(4.0, True)
        speeds.append(controller.step(1.0, 4.0, 0.05, True)[0])

    # Ramped down under the deceleration limit, not cut in one step.
    assert speeds[0] == pytest.approx(0.4 - 0.5 * 0.05)
    assert speeds[-1] == 0.0


def test_withdrawing_walk_authorization_zeroes_translation_at_once():
    controller = make_controller()
    settle(controller, range_=4.0)
    assert controller.linear > 0.2

    linear, _ = controller.step(0.0, 4.0, 0.05, False)

    # Translation never runs blind: no ramp, straight to zero.
    assert linear == 0.0
    # The commitment survives the gap, so the walk does not have to re-enter
    # through the resume margin when data returns.
    assert controller.walking


def test_speed_ramps_down_smoothly_into_the_goal():
    distance, speeds = integrate(make_controller(), 4.0)

    peak = speeds.index(max(speeds))
    tail = speeds[peak:]
    # Decelerating into the goal, never speeding back up or stepping.
    assert all(
        later <= earlier + 1e-9 for earlier, later in zip(tail, tail[1:])
    )
    assert 1.9 <= distance <= 2.15


def test_acceleration_and_deceleration_are_limited():
    controller = make_controller()
    dt = 0.05
    distance = 4.0
    previous = 0.0
    for _ in range(1000):
        controller.update_walk(distance, True)
        linear, _ = controller.step(0.0, distance, dt, True)
        assert abs(linear - previous) <= 0.5 * dt + 1e-9
        previous = linear
        distance -= linear * dt


def test_braking_distance_caps_the_speed_near_the_goal():
    # An aggressive gain alone would ask for 0.5 m/s at 0.1 m of error, a
    # speed the acceleration limit cannot stop from within that distance.
    linear, _ = settle(
        make_controller(speed_gain=5.0, resume_margin=0.0), range_=2.1
    )

    assert linear <= math.sqrt(2.0 * 0.5 * 0.1) + 1e-6


def test_the_stop_is_not_substantially_crossed():
    distance, _ = integrate(make_controller(), 4.0)

    assert distance >= 1.9


def test_the_walk_restarts_only_beyond_the_resume_margin():
    controller = make_controller()
    distance, _ = integrate(controller, 4.0)
    assert not controller.walking

    # Drifting a few centimetres past the stopping distance must not restart.
    controller.update_walk(distance + 0.1, True)
    assert not controller.walking

    controller.update_walk(2.0 + 0.25 + 0.05, True)
    assert controller.walking


def test_decay_ramps_both_commands_to_zero():
    controller = make_controller()
    settle(controller, bearing=0.2, range_=4.0)

    walking_speed = controller.linear
    assert walking_speed > 0.2
    linear, angular = controller.decay(0.05)
    assert linear == pytest.approx(walking_speed - 0.5 * 0.05)
    for _ in range(100):
        linear, angular = controller.decay(0.05)
    assert linear == 0.0
    assert angular == 0.0


def test_reset_clears_commitments_and_commands():
    controller = make_controller()
    controller.should_turn(2.0, 0.0)
    controller.should_turn(2.0, 0.5)
    settle(controller, bearing=0.2, range_=4.0)

    controller.reset()

    assert not controller.turning
    assert not controller.walking
    assert controller.linear == 0.0
    assert controller.angular == 0.0


def test_arrival_requires_both_a_closed_range_and_a_low_speed():
    controller = make_controller()
    assert not controller.arrived(4.0)

    # Cruising through the threshold is not arrival.
    settle(controller, range_=4.0)
    assert not controller.arrived(2.05)

    # The alignment pause far from the goal is not arrival either.
    paused = make_controller()
    settle(paused, bearing=1.0, range_=4.0)
    assert paused.linear == 0.0
    assert not paused.arrived(4.0)

    distance, _ = integrate(make_controller(), 4.0)
    settled = make_controller()
    assert settled.arrived(distance)
