import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from people_detector.msg import People, PeopleArray
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.task import Future
from sensor_msgs.msg import Joy
from spot_eye_animation_msgs.msg import Gaze
from spot_msgs.msg import Feedback
from std_srvs.srv import SetBool, Trigger

from spot_dog_mode import gaze_controller_node
from spot_dog_mode.base_motion import wrap_angle
from spot_dog_mode.gaze_controller_node import (
    GazeControllerNode,
    GazeState,
    _yaw_pitch_to_quaternion,
)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class StandClient:
    """Stand-service stub recording requests and completing them at once."""

    def __init__(self):
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        self.calls += 1
        response = Trigger.Response()
        response.success = True
        future = Future()
        future.set_result(response)
        return future

    def remove_pending_request(self, _future):
        pass


def make_node(*extra_overrides):
    controller = GazeControllerNode(
        parameter_overrides=[
            Parameter('start_enabled', value=True),
            Parameter('command_rate', value=20.0),
            Parameter('greet_ramp', value=0.01),
            Parameter('greet_hold', value=0.0),
            Parameter('prediction_horizon', value=0.0),
            Parameter('max_prediction_horizon', value=0.0),
            Parameter('stand_refresh_rate', value=1000.0),
            *extra_overrides,
        ]
    )
    controller.body_pose_pub = Recorder()
    controller._eye_gaze_pub = Recorder()
    controller._gaze_msg_type = Gaze
    controller._stand_client = StandClient()
    if controller._turn_pub is not None:
        controller._turn_pub = Recorder()

    feedback = Feedback()
    feedback.standing = True
    feedback.moving = False
    controller._on_feedback(feedback)
    # Gaze runs only while the deadman (button 4) is held.
    controller._on_joy(Joy(buttons=[0, 0, 0, 0, 1]))
    return controller


@pytest.fixture
def node():
    controller = make_node()
    yield controller
    controller.destroy_node()


class VirtualClock:
    """Stand-in for time.monotonic driven by the simulated control step.

    Turning the base plays out over seconds of debounce, settle and watchdog
    timeouts, but a few hundred simulated cycles take no measurable real time,
    so against the real clock none of those timeouts would ever expire.
    """

    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def advance(self, dt):
        self.now += dt


@pytest.fixture
def clock(monkeypatch):
    virtual = VirtualClock()
    monkeypatch.setattr(gaze_controller_node, 'time', virtual)
    return virtual


@pytest.fixture
def turning_node(clock):
    controller = make_node()
    yield controller
    controller.destroy_node()


@pytest.fixture
def node_without_base_turn(clock):
    controller = make_node(Parameter('base_turn_enabled', value=False))
    yield controller
    controller.destroy_node()


def people_message(node, people, frame='sensor', age=0.0):
    msg = PeopleArray()
    msg.header.frame_id = frame
    # The detector stamps its output with the lidar sweep it came from, so `age`
    # is how far back the pipeline's latency puts the measurement.
    stamp = node.get_clock().now() - Duration(seconds=age)
    msg.header.stamp = stamp.to_msg()
    msg.people = people
    return msg


def person(track_id, x, y, z=0.4, height=1.6, vx=0.0):
    msg = People()
    msg.id = track_id
    msg.is_human = True
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.size.z = height
    msg.velocity.x = vx
    return msg


def install_identity_tf(node, stamp, source='sensor'):
    """Publish sensor->body for a body carrying the commanded gaze offset.

    Detections are given in footprint-relative coordinates, and Spot's `body`
    frame rotates with the pose this controller commands, so the transform into
    it must undo that pose. Modelling the body as fixed hides the feedback loop
    the controller has to close.
    """
    transform = TransformStamped()
    transform.header.frame_id = 'body'
    transform.child_frame_id = source
    transform.header.stamp = stamp
    body = Pose()
    _yaw_pitch_to_quaternion(body, node._applied_pitch, node._applied_yaw)
    # body <- footprint is the inverse of the commanded footprint <- body pose.
    transform.transform.rotation.w = body.orientation.w
    transform.transform.rotation.x = -body.orientation.x
    transform.transform.rotation.y = -body.orientation.y
    transform.transform.rotation.z = -body.orientation.z
    node.tf_buffer.set_transform(transform, 'test')


def tick(node, dt=0.05):
    node._last_control_time = time.monotonic() - dt
    node._tick()


def acquire_target(node, target):
    msg = people_message(node, [target])
    install_identity_tf(node, msg.header.stamp)
    node._on_people(msg)
    tick(node)
    tick(node)


def test_synthetic_tf_people_feedback_and_eye_commands_are_synchronized(node):
    acquire_target(node, person(7, 2.0, 1.0))

    assert node._target_id == 7
    assert node._state == GazeState.TRACK
    assert node.body_pose_pub.messages
    assert node._eye_gaze_pub.messages
    eye = node._eye_gaze_pub.messages[-1]
    assert eye.detected
    assert eye.num_faces == 1
    assert eye.x == pytest.approx(-node._cmd_yaw / 0.4)
    assert eye.y == pytest.approx(node._cmd_pitch / 0.4)


def test_lateral_and_vertical_motion_update_the_same_target(node):
    acquire_target(node, person(4, 3.0, 0.0, z=0.1, height=0.8))
    initial_yaw, initial_pitch = node._gaze_angles()

    moved = people_message(
        node, [person(4, 2.8, 1.0, z=0.4, height=1.6)]
    )
    install_identity_tf(node, moved.header.stamp)
    node._on_people(moved)
    tick(node)
    updated_yaw, updated_pitch = node._gaze_angles()

    assert node._target_id == 4
    assert updated_yaw > initial_yaw
    assert updated_pitch < initial_pitch
    eye = node._eye_gaze_pub.messages[-1]
    assert eye.x == pytest.approx(-node._cmd_yaw / 0.4)
    assert eye.y == pytest.approx(node._cmd_pitch / 0.4)


def walk_to(node, target, ticks=1):
    msg = people_message(node, [target])
    install_identity_tf(node, msg.header.stamp)
    node._on_people(msg)
    for _ in range(ticks):
        tick(node)


def test_a_walking_person_keeps_the_gaze_updating_every_cycle(node):
    acquire_target(node, person(9, 3.0, -1.5))
    walk_to(node, person(9, 3.0, -1.5), ticks=60)
    settled_yaw = node._cmd_yaw
    desired = []

    # Walk the person across Spot's front over several detection cycles.
    for y in (-1.0, -0.5, 0.0, 0.5, 1.0, 1.5):
        walk_to(node, person(9, 3.0, y))
        desired.append(node._gaze_angles()[0])

    stands_before = node._stand_client.calls
    walk_to(node, person(9, 3.0, 1.5), ticks=60)

    assert node._target_id == 9
    # Each fresh detection must re-aim, not replay one greeting glance.
    assert all(
        later > earlier for earlier, later in zip(desired, desired[1:])
    ), desired
    assert settled_yaw < 0.0 < node._cmd_yaw
    assert node._cmd_yaw == pytest.approx(desired[-1], abs=1e-3)
    # Tracking only reaches the robot if stands keep re-applying the pose.
    assert node._stand_client.calls > stands_before


def run_closed_loop(node, target, seconds=8.0, dt=0.05):
    """Track a person while the body frame moves with the commanded pose."""
    samples = []
    for step in range(int(seconds / dt)):
        if step % 2 == 0:  # detections arrive at half the control rate
            msg = people_message(node, [target])
            install_identity_tf(node, msg.header.stamp)
            node._on_people(msg)
        tick(node, dt)
        samples.append((node._cmd_yaw, node._cmd_pitch))
    return samples


def test_a_still_person_settles_the_gaze_instead_of_oscillating(node):
    acquire_target(node, person(5, 3.0, 1.0))
    samples = run_closed_loop(node, person(5, 3.0, 1.0))

    settled_yaw = [yaw for yaw, _ in samples[-40:]]
    settled_pitch = [pitch for _, pitch in samples[-40:]]

    # Aim at the person's true bearing, not the half-angle a controller chasing
    # its own body offset converges to.
    assert settled_yaw[-1] == pytest.approx(math.atan2(1.0, 3.0), abs=0.02)
    # And hold it: any residual swing is the bob and sway seen on the robot.
    assert max(settled_yaw) - min(settled_yaw) < 0.005
    assert max(settled_pitch) - min(settled_pitch) < 0.005


def test_without_compensation_the_gaze_undershoots_the_person(node):
    node.set_parameters(
        [Parameter('body_feedback_compensation', value=False)]
    )
    acquire_target(node, person(5, 3.0, 1.0))
    samples = run_closed_loop(node, person(5, 3.0, 1.0))

    # Aiming straight at the body-relative angle settles near half of it,
    # because turning the body shrinks the very angle being aimed at.
    assert samples[-1][0] == pytest.approx(math.atan2(1.0, 3.0) / 2.0, abs=0.03)


def test_current_target_is_kept_out_to_the_release_radius(node):
    acquire_target(node, person(3, 3.5, 0.0))

    drifted = people_message(node, [person(3, 5.0, 0.0)])
    install_identity_tf(node, drifted.header.stamp)
    node._on_people(drifted)
    tick(node)

    assert node._target_id == 3
    assert node._target_visible

    # A newcomer beyond the entry radius is still ignored.
    node._clear_target()
    far = people_message(node, [person(8, 5.0, 0.0)])
    install_identity_tf(node, far.header.stamp)
    node._on_people(far)
    tick(node)

    assert not node._target_visible


def test_an_in_progress_stand_does_not_interrupt_the_gaze(node):
    acquire_target(node, person(6, 2.0, 1.0))

    # The driver clears `standing` while a stand command is still settling,
    # which the pose refresh itself provokes on every cycle.
    settling = Feedback()
    settling.standing = False
    settling.sitting = False
    settling.moving = False
    node._on_feedback(settling)
    tick(node)

    assert node._state == GazeState.TRACK
    assert node._eye_gaze_pub.messages[-1].detected

    node._standing_confirmed -= 5.0
    tick(node)

    assert node._state == GazeState.IDLE


def test_stand_refresh_is_suppressed_while_spot_is_moving(node):
    acquire_target(node, person(1, 2.0, 1.0))
    before = node._stand_client.calls

    feedback = Feedback()
    feedback.standing = True
    feedback.moving = True
    node._on_feedback(feedback)
    tick(node)

    assert node._stand_client.calls == before


def test_disable_mid_track_returns_body_and_eye_to_neutral(node):
    acquire_target(node, person(1, 2.0, 1.0))
    request = SetBool.Request()
    request.data = False
    node._on_enable(request, SetBool.Response())

    tick(node)

    pose = node.body_pose_pub.messages[-1]
    eye = node._eye_gaze_pub.messages[-1]
    assert pose.orientation.x == pytest.approx(0.0)
    assert pose.orientation.y == pytest.approx(0.0)
    assert pose.orientation.z == pytest.approx(0.0)
    assert pose.orientation.w == pytest.approx(1.0)
    assert (eye.x, eye.y, eye.detected, eye.num_faces) == (0.0, 0.0, False, 0)
    assert node._state == GazeState.IDLE


@pytest.mark.parametrize('gate', ['deadman', 'sitting', 'moving'])
def test_each_safety_gate_neutralizes_an_active_command(node, gate):
    acquire_target(node, person(1, 2.0, 1.0))

    if gate == 'deadman':
        node._on_joy(Joy(buttons=[0, 0, 0, 0, 0]))
    elif gate == 'sitting':
        feedback = Feedback()
        feedback.standing = False
        feedback.sitting = True
        feedback.moving = False
        node._on_feedback(feedback)
    else:
        feedback = Feedback()
        feedback.standing = True
        feedback.moving = True
        node._on_feedback(feedback)

    tick(node)

    assert node._state == GazeState.IDLE
    assert node._cmd_yaw == 0.0
    assert node._cmd_pitch == 0.0
    assert not node._eye_gaze_pub.messages[-1].detected


@pytest.mark.parametrize('fault', ['receive_gap', 'stale_stamp', 'missing_tf'])
def test_a_brief_data_fault_holds_the_aim_instead_of_recentering(node, fault):
    acquire_target(node, person(1, 2.0, 1.0))
    held_yaw = node._cmd_yaw
    assert held_yaw != 0.0

    def inject():
        if fault == 'receive_gap':
            node._people_rx_time = time.monotonic() - 1.0
        elif fault == 'stale_stamp':
            stale = people_message(node, [person(1, 2.0, 1.0)])
            stale.header.stamp.sec -= 2
            node._on_people(stale)
        else:
            untransformable = people_message(
                node, [person(1, 2.0, 1.0)], frame='missing'
            )
            node._on_people(untransformable)

    inject()
    tick(node)

    # One bad cycle must not fling the body to centre and back.
    assert node._cmd_yaw == pytest.approx(held_yaw)
    assert node._state == GazeState.TRACK
    assert not node._eye_gaze_pub.messages[-1].detected

    # A fault that outlives the grace eases out rather than snapping.
    inject()
    node._fault_since -= 5.0
    tick(node)

    assert node._state == GazeState.RELAX


def relax_after_losing(node):
    empty = people_message(node, [])
    install_identity_tf(node, empty.header.stamp)
    node._on_people(empty)
    tick(node)
    node._target_last_seen -= 5.0
    tick(node)
    assert node._state == GazeState.RELAX


def test_a_returning_person_resumes_tracking_without_a_second_greeting(node):
    acquire_target(node, person(2, 2.0, 1.0))
    assert not node._target_familiar
    relax_after_losing(node)

    # The detector hands out a fresh id after an occlusion; the same person in
    # the same place must not be greeted all over again.
    walk_to(node, person(77, 2.0, 1.05))

    assert node._target_familiar
    assert node._state == GazeState.TRACK
    assert not node._greet_pending
    assert node._greet_elapsed == 0.0


def test_a_new_arrival_is_still_greeted(node):
    acquire_target(node, person(2, 2.0, 1.0))
    relax_after_losing(node)

    walk_to(node, person(77, 2.0, -2.0))

    assert not node._target_familiar
    # The greeting overlay was queued and has started playing over tracking.
    assert node._greet_pending or node._greet_elapsed > 0.0


def test_an_accepted_stand_keeps_the_upright_gate_open(node):
    acquire_target(node, person(1, 2.0, 1.0))

    # The driver reports standing=false for as long as the refresh stand is
    # still settling, which is most of the time while gaze is tracking.
    settling = Feedback()
    settling.standing = False
    settling.sitting = False
    settling.moving = False
    node._on_feedback(settling)

    # Without stands renewing it, the flag-based confirmation expires and the
    # gaze drops to neutral about once per grace window.
    node._standing_confirmed = time.monotonic() - 0.9
    node._stand_request_time = 0.0  # let this cycle's refresh through
    tick(node)

    assert node._standing_confirmed == pytest.approx(
        time.monotonic(), abs=0.1
    )
    assert node._upright(time.monotonic())
    assert node._state == GazeState.TRACK


def step(node, clock, dt=0.05):
    clock.advance(dt)
    node._tick()


def hold_at(node, clock, bearing, radius=3.0, ticks=12, dt=0.05):
    """Keep a person at one bearing while the base stays put."""
    for _ in range(ticks):
        msg = people_message(
            node,
            [person(1, radius * math.cos(bearing), radius * math.sin(bearing))],
        )
        install_identity_tf(node, msg.header.stamp)
        node._on_people(msg)
        step(node, clock, dt)


def start_turning(node, clock, bearing=2.5):
    """Acquire a person well outside the pose envelope and commit to a turn."""
    hold_at(node, clock, bearing)
    assert node._state == GazeState.ENGAGE
    return bearing


def orbit_loop(node, clock, bearing, radius=3.0, steps=240, dt=0.05):
    """Track a stationary person while the base turns under its own commands.

    Rotating the reported detection by the yaw the base has accumulated is what
    closes the loop: without it the bearing never shrinks and the turn is only
    an open-loop command that nothing ever satisfies.
    """
    base_yaw = 0.0
    for _ in range(steps):
        relative = wrap_angle(bearing - base_yaw)
        msg = people_message(
            node,
            [person(1, radius * math.cos(relative), radius * math.sin(relative))],
        )
        install_identity_tf(node, msg.header.stamp)
        node._on_people(msg)
        step(node, clock, dt)
        if node._state == GazeState.ENGAGE and node._turn_pub.messages:
            base_yaw += node._turn_pub.messages[-1].angular.z * dt
    return base_yaw


def stale_orbit_loop(
    node, clock, bearing, latency=0.2, radius=3.0, steps=240, dt=0.05
):
    """Orbit loop where detections describe a base heading Spot has left.

    The real pipeline is a lidar sweep plus CenterPoint inference, so a bearing
    is a couple of tenths of a second old by the time the servo sees it. Replay
    the base heading from `latency` ago to reproduce that: it is the condition
    under which an uncompensated servo keeps commanding a turn it has already
    completed, and sails past the person.
    """
    delay_ticks = max(int(round(latency / dt)), 1)
    base_yaw = 0.0
    history = [0.0] * delay_ticks
    residuals = []
    for _ in range(steps):
        relative = wrap_angle(bearing - history.pop(0))
        history.append(base_yaw)
        msg = people_message(
            node,
            [
                person(
                    1, radius * math.cos(relative), radius * math.sin(relative)
                )
            ],
            age=latency,
        )
        install_identity_tf(node, msg.header.stamp)
        node._on_people(msg)
        step(node, clock, dt)
        if node._state == GazeState.ENGAGE and node._turn_pub.messages:
            base_yaw += node._turn_pub.messages[-1].angular.z * dt
        residuals.append(wrap_angle(bearing - base_yaw))
    return residuals


def test_a_turn_does_not_overshoot_a_person_it_sees_late(turning_node, clock):
    bearing = start_turning(turning_node, clock)

    residuals = stale_orbit_loop(turning_node, clock, bearing)

    # Turning past the person and having to come back is the failure this
    # guards: the bearing must close on zero from one side, not cross it.
    assert min(residuals) > -0.1
    assert abs(residuals[-1]) < 0.4
    assert turning_node._state == GazeState.TRACK


def test_a_late_detection_is_referred_back_to_the_heading_it_was_captured_at(
    turning_node, clock
):
    start_turning(turning_node, clock)
    turning_node._observation_capture_time = clock.now - 0.2
    turning_node._base_yaw = 0.5
    turning_node._base_yaw_history.append((clock.now - 0.2, 0.2))

    # The detection was captured 0.3 rad of rotation ago, so the angle it
    # reports is that much further round than where the person is now.
    assert turning_node._target_bearing() == pytest.approx(
        wrap_angle(
            math.atan2(
                turning_node._target_body_xyz[1],
                turning_node._target_body_xyz[0],
            )
            + turning_node._aim_offset[0]
            - 0.3
        )
    )


def test_a_person_behind_spot_turns_the_base(turning_node, clock):
    # The body pose tops out at 0.4 rad, so nothing short of rotating the base
    # can bring someone at 2.5 rad into view.
    start_turning(turning_node, clock)

    assert turning_node._turn_pub.messages[-1].angular.z > 0.0


def test_a_person_inside_the_pose_envelope_never_turns_the_base(
    turning_node, clock
):
    hold_at(turning_node, clock, bearing=0.2, ticks=40)

    assert turning_node._state == GazeState.TRACK
    assert not turning_node._turn_pub.messages


def test_the_body_pose_stands_down_while_the_base_turns(turning_node, clock):
    bearing = start_turning(turning_node, clock)
    poses = len(turning_node.body_pose_pub.messages)
    stands = turning_node._stand_client.calls

    hold_at(turning_node, clock, bearing, ticks=10)

    # A stand issued alongside a velocity command cancels the turn, and the pose
    # it would apply is overridden by the motion in any case.
    assert turning_node._state == GazeState.ENGAGE
    assert len(turning_node.body_pose_pub.messages) == poses
    assert turning_node._stand_client.calls == stands


def test_the_eyes_keep_chasing_the_person_during_a_turn(turning_node, clock):
    start_turning(turning_node, clock)

    eye = turning_node._eye_gaze_pub.messages[-1]
    assert eye.detected
    # The body command is neutral mid-turn, so eyes driven from it would stare
    # straight ahead instead of toward the person Spot is turning to face.
    assert eye.x == pytest.approx(-1.0)


def test_turning_brings_the_person_into_the_pose_envelope(turning_node, clock):
    bearing = start_turning(turning_node, clock)
    base_yaw = orbit_loop(turning_node, clock, bearing)

    residual = wrap_angle(bearing - base_yaw)
    assert turning_node._state == GazeState.TRACK
    assert abs(residual) < 0.4
    # The base is released once the pose can do the aiming on its own again.
    assert turning_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)
    assert turning_node._cmd_yaw == pytest.approx(residual, abs=0.1)


def test_the_body_pose_is_held_rather_than_reset_across_a_turn(
    turning_node, clock
):
    bearing = start_turning(turning_node, clock)
    held = turning_node._cmd_yaw
    assert held != 0.0

    hold_at(turning_node, clock, bearing, ticks=10)

    # A pose reaches Spot only on a stand, and stands are suppressed for the
    # whole turn. Neutralizing the command here would leave the body physically
    # holding its old offset while the model of it decayed to zero -- and then
    # land that reset in a single step on the first stand after the turn.
    assert turning_node._cmd_yaw == pytest.approx(held)
    assert turning_node._applied_yaw == pytest.approx(held, abs=0.05)


def test_the_turn_ramps_down_instead_of_cutting_the_command(
    turning_node, clock
):
    bearing = start_turning(turning_node, clock)
    orbit_loop(turning_node, clock, bearing)

    rates = [msg.angular.z for msg in turning_node._turn_pub.messages]
    moving = [rate for rate in rates if abs(rate) > 1e-9]

    # The rate floor means the turn is still commanding real motion when it
    # reaches its target angle, so cutting straight to zero asks Spot to plant
    # mid-stride. The command has to walk down to zero first.
    assert abs(moving[-1]) < 0.1
    assert rates[-1] == pytest.approx(0.0)


def test_a_greeting_pauses_during_a_turn_and_resumes_facing_the_person(
    clock,
):
    controller = make_node()
    # Realistic greeting length so it cannot complete before the turn starts.
    controller.set_parameters(
        [
            Parameter('greet_ramp', value=0.35),
            Parameter('greet_hold', value=0.65),
        ]
    )
    bearing = start_turning(controller, clock)
    assert controller._greet_pending
    frozen = controller._greet_elapsed

    hold_at(controller, clock, bearing, ticks=10)

    # The overlay's envelope does not advance while the base owns the body:
    # the look-up is delivered facing the person, not spent mid-rotation.
    assert controller._state == GazeState.ENGAGE
    assert controller._greet_elapsed == frozen

    orbit_loop(controller, clock, bearing)
    assert controller._state == GazeState.TRACK
    assert controller._greet_elapsed > frozen
    controller.destroy_node()


def test_the_greeting_overlay_boosts_pitch_then_expires(node):
    acquire_target(node, person(7, 2.0, 0.0))

    node._start_greet()
    node.set_parameters(
        [
            Parameter('greet_ramp', value=0.2),
            Parameter('greet_hold', value=0.4),
        ]
    )
    boosted = node._apply_greet_overlay(0.0, 0.05)
    assert boosted < 0.0  # pitch_sign is negative: boost looks up

    node._greet_elapsed = 10.0
    assert node._apply_greet_overlay(0.0, 0.05) == 0.0
    assert not node._greet_pending


def test_settle_holds_while_the_driver_still_reports_motion(
    turning_node, clock
):
    start_turning(turning_node, clock)
    moving = Feedback()
    moving.standing = True
    moving.moving = True
    turning_node._on_feedback(moving)
    turning_node._finish_engage(clock.now)
    assert turning_node._state == GazeState.SETTLE

    # The driver is still finishing the motion, so the pose stays back.
    hold_at(turning_node, clock, bearing=0.1, ticks=4)
    assert turning_node._state == GazeState.SETTLE

    still = Feedback()
    still.standing = True
    still.moving = False
    turning_node._on_feedback(still)
    hold_at(turning_node, clock, bearing=0.1, ticks=14)
    assert turning_node._state == GazeState.TRACK


def test_settle_releases_promptly_once_the_base_is_at_rest(
    turning_node, clock
):
    start_turning(turning_node, clock)
    turning_node._stop_base_motion()
    turning_node._base_rate = 0.0
    turning_node._base_speed = 0.0
    turning_node._finish_engage(clock.now)
    assert turning_node._state == GazeState.SETTLE

    hold_at(turning_node, clock, bearing=0.1, ticks=1)

    # No fixed dead time: the fallback timer has most of its window left,
    # but the base is demonstrably at rest, so the very next cycle resumes.
    assert turning_node._state == GazeState.TRACK


def test_a_person_who_walks_all_the_way_around_is_followed(
    turning_node, clock
):
    # Detections are 360-degree, so the far side is a turn away, not a loss.
    base_yaw = 0.0
    for bearing in (1.2, 2.4, -2.4, -1.2, 0.0):
        base_yaw += orbit_loop(
            turning_node, clock, wrap_angle(bearing - base_yaw), steps=160
        )
        assert turning_node._state == GazeState.TRACK
        assert abs(wrap_angle(bearing - base_yaw)) < 0.4


def test_releasing_the_deadman_mid_turn_stops_the_base(turning_node, clock):
    start_turning(turning_node, clock)

    turning_node._on_joy(Joy(buttons=[0, 0, 0, 0, 0]))
    step(turning_node, clock)

    assert turning_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)
    assert turning_node._state == GazeState.IDLE


def test_spots_own_turn_is_not_read_as_an_external_motion_fault(
    turning_node, clock
):
    bearing = start_turning(turning_node, clock)

    # The driver reports moving for the whole of the turn; treating that as a
    # fault would make every turn abort on its first cycle.
    feedback = Feedback()
    feedback.standing = True
    feedback.moving = True
    turning_node._on_feedback(feedback)
    hold_at(turning_node, clock, bearing, ticks=5)

    assert turning_node._state == GazeState.ENGAGE


def test_motion_this_controller_did_not_command_still_closes_the_gate(
    turning_node, clock
):
    start_turning(turning_node, clock)
    feedback = Feedback()
    feedback.standing = True
    feedback.moving = True
    turning_node._on_feedback(feedback)

    # No velocity command of ours is recent enough to account for the motion.
    turning_node._base_cmd_time -= 5.0
    step(turning_node, clock)

    assert turning_node._state == GazeState.IDLE
    assert turning_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def test_a_turn_that_stops_closing_on_the_target_is_abandoned(
    turning_node, clock
):
    start_turning(turning_node, clock)

    # The base never actually rotated, so the bearing has not improved.
    turning_node._turn_progress_time -= 10.0
    step(turning_node, clock)

    assert turning_node._state == GazeState.RELAX
    assert turning_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def drive_sticks(node, forward=0.0, strafe=0.0, turn=0.0):
    """Hold the deadman and deflect the axes spot_joy drives the base with."""
    node._on_joy(
        Joy(axes=[strafe, forward, turn], buttons=[0, 0, 0, 0, 1])
    )


def test_the_operator_driving_takes_the_base_back_from_a_turn(
    turning_node, clock
):
    start_turning(turning_node, clock)

    drive_sticks(turning_node, forward=0.8)
    step(turning_node, clock)

    # Both publish to cmd_vel, so a turn that kept streaming would interleave
    # with the sticks and the robot would follow whichever arrived last.
    assert turning_node._state == GazeState.IDLE
    assert turning_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def test_the_operator_keeps_the_base_while_a_person_stays_off_to_the_side(
    turning_node, clock
):
    bearing = start_turning(turning_node, clock)

    # Driving with someone off-axis is exactly when the turn wants to run, so
    # this is the case where the two would fight for the whole drive.
    for _ in range(40):
        drive_sticks(turning_node, forward=0.8)
        hold_at(turning_node, clock, bearing, ticks=1)

    assert turning_node._state == GazeState.IDLE
    assert turning_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def test_no_stand_is_issued_while_the_operator_drives(turning_node, clock):
    hold_at(turning_node, clock, bearing=0.2, ticks=10)
    stands = turning_node._stand_client.calls

    drive_sticks(turning_node, forward=0.8)
    hold_at(turning_node, clock, bearing=0.2, ticks=10)

    # A stand rebuilds the mobility params and preempts the velocity command
    # the operator's stick just issued, which reads as Spot refusing to walk.
    assert turning_node._stand_client.calls == stands


def test_stick_noise_below_the_deadzone_does_not_interrupt_tracking(
    turning_node, clock
):
    drive_sticks(turning_node, forward=0.05, turn=-0.04)
    hold_at(turning_node, clock, bearing=0.2, ticks=10)

    assert turning_node._state == GazeState.TRACK


def test_tracking_resumes_once_the_sticks_are_released(turning_node, clock):
    bearing = start_turning(turning_node, clock)
    drive_sticks(turning_node, forward=0.8)
    step(turning_node, clock)
    assert turning_node._state == GazeState.IDLE

    # Centring the sticks stops refreshing the claim; the grace then expires.
    drive_sticks(turning_node)
    clock.advance(1.0)
    hold_at(turning_node, clock, bearing, ticks=12)

    assert turning_node._state == GazeState.ENGAGE


def test_a_sustained_offset_recenters_the_base_under_the_gaze(
    turning_node, clock
):
    # Inside the envelope-edge entry threshold but past the recentring one.
    hold_at(turning_node, clock, bearing=0.3, ticks=20)
    assert turning_node._state == GazeState.TRACK
    assert not turning_node._turn_pub.messages

    hold_at(turning_node, clock, bearing=0.3, ticks=20)

    # After the long debounce the body commits a turn and squares up.
    assert turning_node._state == GazeState.ENGAGE
    assert turning_node._turn_pub.messages[-1].angular.z > 0.0


def test_recentring_leaves_the_body_nearly_facing_the_person(
    turning_node, clock
):
    hold_at(turning_node, clock, bearing=0.3, ticks=40)
    assert turning_node._state == GazeState.ENGAGE

    base_yaw = orbit_loop(turning_node, clock, 0.3)

    residual = wrap_angle(0.3 - base_yaw)
    assert turning_node._state == GazeState.TRACK
    assert abs(residual) < 0.2


def test_a_person_merely_passing_through_never_triggers_recentring(
    turning_node, clock
):
    for bearing in (0.3, 0.1, 0.3, 0.1):
        hold_at(turning_node, clock, bearing=bearing, ticks=10)

    assert turning_node._state == GazeState.TRACK
    assert not turning_node._turn_pub.messages


def test_base_turning_can_be_disabled(node_without_base_turn, clock):
    node = node_without_base_turn
    hold_at(node, clock, bearing=2.5, ticks=40)

    assert node._turn_pub is None
    assert node._state == GazeState.TRACK
    # Aim stays pinned at the pose limit, as it did before base turning existed.
    assert node._cmd_yaw == pytest.approx(0.4, abs=1e-3)


# Person approach.


@pytest.fixture
def approach_node(clock):
    # The stopping distance is pinned so these tests assert the controller's
    # behavior rather than whatever the deployed default is tuned to.
    controller = make_node(
        Parameter('approach_enabled', value=True),
        Parameter('approach_distance', value=2.0),
    )
    yield controller
    controller.destroy_node()


def feed_person(node, clock, target, dt=0.05, age=0.0):
    msg = people_message(node, [target], age=age)
    install_identity_tf(node, msg.header.stamp)
    node._on_people(msg)
    step(node, clock, dt)


def start_approach(node, clock, distance=3.5, ticks=60):
    """Greet a person straight ahead and commit to walking toward them."""
    for _ in range(ticks):
        feed_person(node, clock, person(1, distance, 0.0))
        if node._state == GazeState.ENGAGE:
            return
    raise AssertionError('approach never started')


def approach_loop(node, clock, start_distance, latency=0.0, steps=600, dt=0.05):
    """Close the loop: the robot advances by its own published commands.

    With latency, each detection reports the range as it stood `latency` ago —
    the real pipeline's sweep-plus-inference delay — which is the condition
    where acting on the raw measurement walks the stale error a second time
    and overshoots the stopping distance.
    """
    delay_ticks = max(int(round(latency / dt)), 1) if latency else 0
    history = [0.0] * delay_ticks
    travelled = 0.0
    states = []
    for _ in range(steps):
        if delay_ticks:
            seen = history.pop(0)
            history.append(travelled)
        else:
            seen = travelled
        feed_person(
            node, clock, person(1, start_distance - seen, 0.0), dt, age=latency
        )
        if node._turn_pub.messages:
            travelled += node._turn_pub.messages[-1].linear.x * dt
        states.append(node._state)
    return start_distance - travelled, states


def test_approach_disabled_preserves_the_gaze_only_behavior(
    turning_node, clock
):
    final, states = approach_loop(turning_node, clock, 3.5, steps=200)

    assert GazeState.ENGAGE not in states
    assert turning_node._state == GazeState.TRACK
    # Not a single base command: the person straight ahead needs no turn, and
    # walking is opt-in.
    assert not turning_node._turn_pub.messages
    assert final == pytest.approx(3.5)


def test_spot_walks_up_to_a_person_and_stops_at_the_configured_distance(
    approach_node, clock
):
    final, states = approach_loop(approach_node, clock, 3.5)

    assert GazeState.ENGAGE in states
    # The person is beyond the threshold, so real forward speed is commanded.
    assert max(
        msg.linear.x for msg in approach_node._turn_pub.messages
    ) > 0.2
    # Default stopping distance is 2.0 m with a small arrival tolerance.
    assert 1.8 <= final <= 2.2
    assert approach_node._state == GazeState.TRACK


def test_arrival_settles_before_tracking_resumes(approach_node, clock):
    final, states = approach_loop(approach_node, clock, 3.5)

    last_walk = max(
        index
        for index, state in enumerate(states)
        if state == GazeState.ENGAGE
    )
    assert states[last_walk + 1] == GazeState.SETTLE
    assert GazeState.TRACK in states[last_walk + 1:]
    # Every exit from APPROACH leaves a zero Twist behind.
    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)
    assert approach_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def test_arriving_does_not_chatter_across_the_stopping_threshold(
    approach_node, clock
):
    final, _ = approach_loop(approach_node, clock, 3.5)
    baseline = len(approach_node._turn_pub.messages)

    # Detection jitter around the arrival range must not restart the walk:
    # re-entry needs the resume margin beyond the stopping distance.
    for index in range(60):
        wobble = 0.08 * math.sin(index)
        feed_person(approach_node, clock, person(1, final + wobble, 0.0))
        assert approach_node._state != GazeState.ENGAGE

    assert all(
        msg.linear.x == 0.0
        for msg in approach_node._turn_pub.messages[baseline:]
    )

    # Once the person retreats beyond distance + margin, approach resumes.
    for _ in range(20):
        feed_person(approach_node, clock, person(1, final + 0.6, 0.0))
    assert approach_node._state == GazeState.ENGAGE


def test_delayed_detections_do_not_cause_material_overshoot(
    approach_node, clock
):
    # Each range is 0.4 s old when acted on; the base-motion history must
    # subtract the ground covered since capture, or Spot stops well inside
    # the requested distance.
    final, states = approach_loop(approach_node, clock, 3.5, latency=0.4)

    assert GazeState.ENGAGE in states
    assert 1.8 <= final <= 2.3


def test_a_person_behind_spot_is_oriented_to_before_any_walking(
    approach_node, clock
):
    hold_at(approach_node, clock, bearing=2.5)

    assert approach_node._state == GazeState.ENGAGE
    assert all(
        msg.linear.x == 0.0 for msg in approach_node._turn_pub.messages
    )


def test_poses_and_stands_stay_suppressed_while_walking(approach_node, clock):
    start_approach(approach_node, clock)
    poses = len(approach_node.body_pose_pub.messages)
    stands = approach_node._stand_client.calls

    for _ in range(20):
        feed_person(approach_node, clock, person(1, 3.5, 0.0))

    assert approach_node._state == GazeState.ENGAGE
    assert len(approach_node.body_pose_pub.messages) == poses
    assert approach_node._stand_client.calls == stands


def test_the_eyes_keep_following_the_person_while_walking(
    approach_node, clock
):
    start_approach(approach_node, clock)

    # Move the person a little off-axis, still inside the alignment envelope.
    for _ in range(5):
        feed_person(
            approach_node,
            clock,
            person(1, 3.5 * math.cos(0.2), 3.5 * math.sin(0.2)),
        )

    assert approach_node._state == GazeState.ENGAGE
    eye = approach_node._eye_gaze_pub.messages[-1]
    assert eye.detected
    # The body command is held mid-walk, so the eyes follow the raw bearing.
    assert eye.x == pytest.approx(-0.2 / 0.4, abs=0.25)


def test_a_person_drifting_off_axis_mid_walk_keeps_one_engagement(
    approach_node, clock
):
    start_approach(approach_node, clock)
    for _ in range(10):
        feed_person(approach_node, clock, person(1, 3.5, 0.0))
    assert approach_node._turn_pub.messages[-1].linear.x > 0.2

    # The person cuts sideways to a bearing well outside the old alignment
    # gate and the turn-entry threshold.
    states = set()
    for _ in range(30):
        feed_person(
            approach_node,
            clock,
            person(1, 3.5 * math.cos(0.9), 3.5 * math.sin(0.9)),
        )
        states.add(approach_node._state)

    # One continuous engagement: rotation takes the lead and forward speed
    # feathers out, with no stop-and-settle handoff in between.
    assert states == {GazeState.ENGAGE}
    assert approach_node._turn_pub.messages[-1].angular.z > 0.0
    linears = [
        msg.linear.x for msg in approach_node._turn_pub.messages[-30:]
    ]
    # The walk ramps off under the acceleration limit rather than cutting.
    assert all(
        later <= earlier + 1e-9 for earlier, later in zip(linears, linears[1:])
    )


def test_one_missing_detection_immediately_stops_forward_motion(
    approach_node, clock
):
    start_approach(approach_node, clock)
    feed_person(approach_node, clock, person(1, 3.5, 0.0))
    assert approach_node._turn_pub.messages[-1].linear.x > 0.0

    empty = people_message(approach_node, [])
    install_identity_tf(approach_node, empty.header.stamp)
    approach_node._on_people(empty)
    step(approach_node, clock)

    # One occluded frame zeroes translation at once; the state itself may
    # ride out the grace interval, but never walking blind.
    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)
    assert approach_node._state == GazeState.ENGAGE


def test_a_stale_detection_array_immediately_stops_forward_motion(
    approach_node, clock
):
    start_approach(approach_node, clock)
    feed_person(approach_node, clock, person(1, 3.5, 0.0))
    assert approach_node._turn_pub.messages[-1].linear.x > 0.0

    approach_node._people_rx_time -= 1.0
    step(approach_node, clock)

    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)


def test_losing_the_person_while_walking_relaxes(approach_node, clock):
    start_approach(approach_node, clock)

    empty = people_message(approach_node, [])
    install_identity_tf(approach_node, empty.header.stamp)
    approach_node._on_people(empty)
    step(approach_node, clock)
    approach_node._target_last_seen -= 5.0
    step(approach_node, clock)

    assert approach_node._state == GazeState.RELAX
    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)
    assert approach_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def test_releasing_the_deadman_mid_walk_stops_the_base(approach_node, clock):
    start_approach(approach_node, clock)

    approach_node._on_joy(Joy(buttons=[0, 0, 0, 0, 0]))
    step(approach_node, clock)

    assert approach_node._state == GazeState.IDLE
    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)
    assert approach_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)


def test_the_operator_sticks_take_the_base_back_mid_walk(
    approach_node, clock
):
    start_approach(approach_node, clock)

    drive_sticks(approach_node, forward=0.8)
    step(approach_node, clock)

    assert approach_node._state == GazeState.IDLE
    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)


def test_spots_own_walking_is_not_read_as_external_motion(
    approach_node, clock
):
    start_approach(approach_node, clock)

    # The driver reports moving for the whole walk; treating that as a fault
    # would make every approach abort on its first cycle.
    feedback = Feedback()
    feedback.standing = True
    feedback.moving = True
    approach_node._on_feedback(feedback)
    for _ in range(5):
        feed_person(approach_node, clock, person(1, 3.5, 0.0))

    assert approach_node._state == GazeState.ENGAGE


def test_a_target_switch_mid_walk_stops_before_reengaging(
    approach_node, clock
):
    start_approach(approach_node, clock)

    # A much nearer newcomer forces an immediate selector switch.
    msg = people_message(
        approach_node, [person(1, 3.5, 0.0), person(2, 1.2, 0.3)]
    )
    install_identity_tf(approach_node, msg.header.stamp)
    approach_node._on_people(msg)
    step(approach_node, clock)

    # The walk is dropped at once; with the base barely moving, settle may
    # hand straight back to tracking rather than waiting a fixed period.
    assert approach_node._state in (GazeState.SETTLE, GazeState.TRACK)
    assert approach_node._turn_pub.messages[-1].linear.x == pytest.approx(0.0)
    assert approach_node._turn_pub.messages[-1].angular.z == pytest.approx(0.0)

    # Once settled, the unfamiliar newcomer is greeted, not walked at.
    states = [approach_node._state]
    for _ in range(20):
        feed_person(approach_node, clock, person(2, 1.2, 0.3))
        states.append(approach_node._state)
    assert approach_node._greet_elapsed > 0.0
    assert GazeState.ENGAGE not in states


def test_brief_occlusion_holds_then_target_loss_starts_relax(node):
    acquire_target(node, person(1, 2.0, 1.0, vx=-0.5))
    empty = people_message(node, [])
    install_identity_tf(node, empty.header.stamp)
    node._on_people(empty)

    tick(node)
    assert node._state == GazeState.TRACK
    assert not node._eye_gaze_pub.messages[-1].detected

    node._target_last_seen -= 2.0
    tick(node)
    assert node._state == GazeState.RELAX
