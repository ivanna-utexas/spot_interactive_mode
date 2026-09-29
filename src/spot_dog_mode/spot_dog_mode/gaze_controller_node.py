#!/usr/bin/env python3
"""Real-time nearest-human gaze controller for Auto Dog Mode.

Fresh detections drive target selection while a higher-rate timer predicts the
selected track, updates the TRACK/ENGAGE/RELAX state machine, and publishes one
synchronized body/animated-eye command state. Greeting a new arrival is a
timed overlay on tracking — a look-up boost blended into the pose — rather
than a state of its own, so it never blocks or replays around base motion.

Detections come from a 360-degree lidar, so a person beside or behind Spot is
already tracked; the body-pose envelope simply cannot aim that far. ENGAGE
owns the base whenever it must move: it blends rotation toward the person's
bearing with (opt-in) forward motion toward a configured planar distance in
one continuous velocity stream, so turning and walking interleave instead of
alternating through full stops. SETTLE waits out the resulting motion before
the pose controller takes over again.
"""

import math
import time
from collections import deque
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Twist
from people_detector.msg import PeopleArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Joy
from spot_msgs.msg import Feedback
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from .base_motion import BaseMotionController, wrap_angle
from .gaze_logic import (
    StableNearestSelector,
    TargetCandidate,
    absolute_aim,
    clamp,
    deadbanded_velocity,
    elevation_height,
    estimated_head_z,
    finite_vector,
    first_order_step,
    gaze_angles,
    height_elevation,
    smoothed_rate_limited_step,
)


class GazeState(Enum):
    IDLE = 0
    TRACK = 2
    RELAX = 3
    ENGAGE = 4
    SETTLE = 5


def _yaw_pitch_to_quaternion(pose: Pose, pitch: float, yaw: float) -> None:
    """Fill pose orientation using the spot_joy body-pose convention."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    pose.orientation.w = cp * cy
    pose.orientation.x = -sp * sy
    pose.orientation.y = sp * cy
    pose.orientation.z = cp * sy


def _transform_point(
    tf, x: float, y: float, z: float
) -> Tuple[float, float, float]:
    """Apply a geometry_msgs TransformStamped to a point."""
    rx, ry, rz = _rotate_vector(tf, x, y, z)
    t = tf.transform.translation
    return (rx + t.x, ry + t.y, rz + t.z)


def _rotate_vector(
    tf, x: float, y: float, z: float
) -> Tuple[float, float, float]:
    """Rotate a vector by a geometry_msgs TransformStamped quaternion."""
    q = tf.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ux, uy, uz = (
        qy * z - qz * y,
        qz * x - qx * z,
        qx * y - qy * x,
    )
    ux, uy, uz = ux + qw * x, uy + qw * y, uz + qw * z
    return (
        x + 2.0 * (qy * uz - qz * uy),
        y + 2.0 * (qz * ux - qx * uz),
        z + 2.0 * (qx * uy - qy * ux),
    )


class GazeControllerNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            'dog_mode_gaze_controller', parameter_overrides=parameter_overrides
        )

        # Topics and frames.
        self.declare_parameter('people_topic', '/people_detections')
        self.declare_parameter('body_pose_topic', 'body_pose')
        self.declare_parameter('feedback_topic', '/status/feedback')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('body_frame', 'body')

        # Selection and timing.
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('attention_radius', 4.0)
        self.declare_parameter('attention_release_radius', 6.0)
        self.declare_parameter('target_switch_margin', 0.30)
        self.declare_parameter('target_switch_debounce', 0.20)
        self.declare_parameter('target_switch_immediate_margin', 0.75)
        self.declare_parameter('target_match_radius', 0.80)
        self.declare_parameter('target_lost_timeout', 1.0)
        self.declare_parameter('detection_timeout', 0.75)
        self.declare_parameter('data_fault_grace', 0.4)
        self.declare_parameter('greet_repeat_timeout', 8.0)

        # Timestamped TF and bounded velocity prediction.
        self.declare_parameter('tf_lookup_timeout', 0.03)
        self.declare_parameter('allow_latest_tf_fallback', True)
        self.declare_parameter('prediction_horizon', 0.15)
        self.declare_parameter('max_prediction_horizon', 0.35)
        self.declare_parameter('max_prediction_speed', 3.0)
        self.declare_parameter('min_prediction_speed', 0.35)

        # Motion shaping and Spot-safe limits.
        self.declare_parameter('greet_ramp', 0.35)
        self.declare_parameter('greet_hold', 0.65)
        self.declare_parameter('greet_pitch_boost', 0.10)
        self.declare_parameter('smoothing_tau', 0.30)
        self.declare_parameter('pitch_smoothing_tau', 0.35)
        self.declare_parameter('head_height_tau', 1.0)
        self.declare_parameter('max_rate_rad_s', 0.80)
        # Detections are measured in the body frame, which moves with the pose
        # this controller commands. Compensation converts that relative angle
        # back into the footprint-relative command the driver expects.
        self.declare_parameter('body_feedback_compensation', True)
        self.declare_parameter('body_response_tau', 0.30)
        self.declare_parameter('relax_duration', 1.0)
        self.declare_parameter('max_yaw', 0.4)
        self.declare_parameter('max_pitch', 0.4)
        # spot_ros2 uses positive pitch for nose-down body motion.
        self.declare_parameter('pitch_sign', -1.0)
        self.declare_parameter('default_person_height', 1.7)
        self.declare_parameter('min_person_height', 0.5)
        self.declare_parameter('max_person_height', 2.5)
        self.declare_parameter('body_height_above_ground', 0.52)

        # Body-pose delivery. The driver's body_pose subscriber only latches
        # the pose into its mobility params; Spot adopts it on the next stand
        # command, so a streamed pose alone leaves the body frozen.
        self.declare_parameter('stand_refresh', True)
        self.declare_parameter('stand_service', 'stand')
        self.declare_parameter('stand_refresh_rate', 10.0)
        self.declare_parameter('stand_request_timeout', 1.0)
        # Must exceed stand_request_timeout plus one refresh period so a single
        # slow stand response cannot expire the confirmation mid-gaze.
        self.declare_parameter('standing_grace', 2.0)

        # Full-circle tracking. The body pose can only aim within max_yaw, so a
        # person beyond it is reached by rotating the base instead. Velocity and
        # body pose cannot be commanded at once — the driver applies a pose on
        # the next stand, which would cancel the turn — so the two alternate.
        self.declare_parameter('base_turn_enabled', True)
        self.declare_parameter('turn_cmd_topic', 'cmd_vel_intermediate')
        self.declare_parameter('turn_enter_yaw', 0.40)
        # Below recenter_yaw, so a recentring turn has room to run, and deep
        # enough that the body ends nearly square to the person; the decay
        # tail past release stays small because the rate is already near the
        # floor when the exit threshold is crossed.
        self.declare_parameter('turn_exit_yaw', 0.15)
        self.declare_parameter('turn_enter_debounce', 0.30)
        # Slow-path turn entry: a person held at a yaw offset the pose can
        # reach, but only by watching sideways indefinitely. After this long
        # debounce the base commits a turn and squares up under the gaze the
        # way a dog's body follows its head. Zero or negative disables it.
        self.declare_parameter('recenter_yaw', 0.25)
        self.declare_parameter('recenter_debounce', 1.5)
        self.declare_parameter('turn_rate_gain', 1.2)
        self.declare_parameter('turn_max_rate', 0.80)
        # Spot walks a slow in-place yaw as discrete steps rather than a smooth
        # rotation, so the floor is set high enough that the whole turn stays
        # above it: the proportional tail is where the stepping shows up.
        self.declare_parameter('turn_min_rate', 0.35)
        self.declare_parameter('turn_accel_limit', 1.5)
        # Detections are several tenths of a second old by the time a bearing is
        # computed from them, and during a turn the base has kept rotating for
        # all of it. Modelling how fast Spot reaches a commanded yaw rate lets
        # that rotation be subtracted back out; without it the servo closes on a
        # bearing the robot has already turned past, and overshoots by roughly
        # the turn rate times the latency.
        self.declare_parameter('turn_response_tau', 0.25)
        self.declare_parameter('turn_settle_time', 0.5)
        self.declare_parameter('turn_stall_timeout', 4.0)
        self.declare_parameter('turn_progress_epsilon', 0.05)
        self.declare_parameter('turn_moving_grace', 1.0)
        # Stick axes that drive the base in spot_joy: left stick x/y and right
        # stick x. Deflecting any of them hands the base back to the operator.
        self.declare_parameter('drive_axes', [0, 1, 2])
        # Matches spot_joy's stick deadzone: a larger value here would leave a
        # band where the sticks drive but dog mode does not yield.
        self.declare_parameter('drive_axis_deadzone', 0.08)
        self.declare_parameter('operator_drive_grace', 0.75)

        # Person approach (opt-in). Walks toward the tracked person and stops
        # at approach_distance, the planar distance from the body-frame origin
        # to the person's tracked position. Off by default because it commands
        # autonomous translation rather than posture alone.
        self.declare_parameter('approach_enabled', False)
        self.declare_parameter('approach_distance', 1.0)
        # Restart the walk only beyond distance + margin, so arriving and
        # drifting a few centimetres does not chatter across the threshold.
        # Hysteresis lives here, at the state machine; the stopping distance
        # itself is never moved.
        self.declare_parameter('approach_resume_margin', 0.25)
        self.declare_parameter('approach_max_speed', 0.40)
        self.declare_parameter('approach_speed_gain', 0.50)
        self.declare_parameter('approach_accel_limit', 0.50)
        # No forward motion while the person's bearing is outside this;
        # steering re-centres them first.
        self.declare_parameter('approach_alignment_yaw', 0.30)

        # Integration.
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('publish_eye_gaze', False)
        self.declare_parameter('eye_gaze_topic', 'eye_animation/gaze')

        self._enabled = bool(self.get_parameter('start_enabled').value)
        self._state = GazeState.IDLE
        self._state_since = time.monotonic()
        self._people_msg: Optional[PeopleArray] = None
        self._people_rx_time = 0.0
        self._people_generation = 0
        self._processed_generation = 0
        self._standing = False
        self._sitting = False
        self._standing_confirmed = 0.0
        self._moving = False
        self._deadman_held = False
        self._operator_drive_time = 0.0

        self._selector = StableNearestSelector(
            switch_margin=float(
                self.get_parameter('target_switch_margin').value
            ),
            switch_debounce=float(
                self.get_parameter('target_switch_debounce').value
            ),
            immediate_margin=float(
                self.get_parameter('target_switch_immediate_margin').value
            ),
            match_radius=float(
                self.get_parameter('target_match_radius').value
            ),
        )
        self._target_id: Optional[int] = None
        self._target_observation: Optional[TargetCandidate] = None
        self._target_observation_time = 0.0
        self._target_prediction_base = 0.0
        self._target_body_xyz: Optional[Tuple[float, float, float]] = None
        self._target_head_z = float(
            self.get_parameter('default_person_height').value
        )
        self._target_last_seen = 0.0
        self._target_visible = False
        self._target_familiar = False
        self._fault_since = 0.0

        # Identity of the person most recently engaged, kept across occlusion
        # and relax so a return resumes tracking instead of re-greeting.
        self._familiar_id: Optional[int] = None
        self._familiar_xy: Optional[Tuple[float, float]] = None
        self._familiar_time = 0.0

        self._cmd_yaw = 0.0
        self._cmd_pitch = 0.0
        # Modelled body attitude actually reached, kept as a short history so a
        # detection is compensated with the offset that was applied when it was
        # captured rather than the newer one the body has not settled into yet.
        self._applied_yaw = 0.0
        self._applied_pitch = 0.0
        self._applied_history = deque(maxlen=128)
        self._aim_offset: Tuple[float, float] = (0.0, 0.0)
        self._head_height: Optional[float] = None
        self._commanding = False
        self._eye_active = False
        self._relax_start: Tuple[float, float] = (0.0, 0.0)
        self._last_control_time = time.monotonic()
        self._tf_warned = False
        self._tf_fallback_warned = False

        self._motion = BaseMotionController(
            enter_yaw=float(self.get_parameter('turn_enter_yaw').value),
            exit_yaw=float(self.get_parameter('turn_exit_yaw').value),
            enter_debounce=float(
                self.get_parameter('turn_enter_debounce').value
            ),
            yaw_gain=float(self.get_parameter('turn_rate_gain').value),
            max_yaw_rate=float(self.get_parameter('turn_max_rate').value),
            min_yaw_rate=float(self.get_parameter('turn_min_rate').value),
            yaw_accel_limit=float(
                self.get_parameter('turn_accel_limit').value
            ),
            stop_distance=float(
                self.get_parameter('approach_distance').value
            ),
            resume_margin=float(
                self.get_parameter('approach_resume_margin').value
            ),
            speed_gain=float(self.get_parameter('approach_speed_gain').value),
            max_speed=float(self.get_parameter('approach_max_speed').value),
            accel_limit=float(
                self.get_parameter('approach_accel_limit').value
            ),
            alignment_yaw=float(
                self.get_parameter('approach_alignment_yaw').value
            ),
        )
        # Greeting overlay. Pending until the tracking pose has played the
        # whole look-up envelope; elapsed advances only while TRACK owns the
        # pose, so a greeting begun as the base turns or walks is still
        # delivered facing the person.
        self._greet_pending = False
        self._greet_elapsed = 0.0
        self._recenter_since = 0.0
        self._base_motion_active = False
        self._turn_progress_time = 0.0
        self._turn_best_bearing = math.pi
        self._settle_until = 0.0
        # Spot reports `moving` while this controller is turning or walking it.
        # Motion this controller did not command must still close the gate.
        self._base_cmd_time = 0.0
        # Most recent base velocities commanded, feeding the ego-motion model.
        self._cmd_base_linear = 0.0
        self._cmd_base_angular = 0.0
        # Modelled base heading and planar position, integrated from the
        # commanded velocities and kept as histories so a detection can be
        # referred back to the pose Spot actually held when the sweep was
        # captured. Only differences are used, so the absolute values are free
        # to drift.
        self._base_rate = 0.0
        self._base_yaw = 0.0
        self._base_yaw_history = deque(maxlen=128)
        self._base_speed = 0.0
        self._base_pos: Tuple[float, float] = (0.0, 0.0)
        self._base_pos_history = deque(maxlen=128)
        self._observation_capture_time = 0.0

        self._stand_future = None
        self._stand_request_time = 0.0
        self._stand_warned = False
        refresh_rate = max(
            float(self.get_parameter('stand_refresh_rate').value), 1.0e-3
        )
        self._stand_refresh_period = 1.0 / refresh_rate

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PeopleArray,
            self.get_parameter('people_topic').value,
            self._on_people,
            10,
        )
        self.create_subscription(
            Feedback,
            self.get_parameter('feedback_topic').value,
            self._on_feedback,
            10,
        )
        self.create_subscription(
            Joy, self.get_parameter('joy_topic').value, self._on_joy, 10
        )
        self.body_pose_pub = self.create_publisher(
            Pose, self.get_parameter('body_pose_topic').value, 1
        )

        self._turn_pub = None
        if bool(self.get_parameter('base_turn_enabled').value) or bool(
            self.get_parameter('approach_enabled').value
        ):
            self._turn_pub = self.create_publisher(
                Twist, str(self.get_parameter('turn_cmd_topic').value), 1
            )

        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.enabled_pub = self.create_publisher(
            Bool, '/dog_mode/enabled', latched
        )
        self._publish_enabled()
        self.create_service(SetBool, '/dog_mode/enable', self._on_enable)

        self._stand_client = None
        if bool(self.get_parameter('stand_refresh').value):
            self._stand_client = self.create_client(
                Trigger, str(self.get_parameter('stand_service').value)
            )

        self._eye_gaze_pub = None
        if bool(self.get_parameter('publish_eye_gaze').value):
            try:
                from spot_eye_animation_msgs.msg import Gaze

                self._gaze_msg_type = Gaze
                self._eye_gaze_pub = self.create_publisher(
                    Gaze, self.get_parameter('eye_gaze_topic').value, 10
                )
            except ImportError:
                self.get_logger().warn(
                    'publish_eye_gaze=true but spot_eye_animation_msgs '
                    'is unavailable'
                )

        rate = max(float(self.get_parameter('command_rate').value), 1.0)
        self._nominal_dt = 1.0 / rate
        self.create_timer(self._nominal_dt, self._tick)

        self.get_logger().info(
            f'Dog mode gaze controller started at {rate:.1f} Hz '
            f'(enabled={self._enabled}); toggle via /dog_mode/enable'
        )

    # Callbacks.

    def _on_people(self, msg: PeopleArray) -> None:
        self._people_msg = msg
        self._people_rx_time = time.monotonic()
        self._people_generation += 1

    def _on_feedback(self, msg: Feedback) -> None:
        self._standing = msg.standing
        self._sitting = msg.sitting
        self._moving = msg.moving
        if msg.standing:
            self._standing_confirmed = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        index = int(self.get_parameter('deadman_button').value)
        valid_index = 0 <= index < len(msg.buttons)
        self._deadman_held = bool(msg.buttons[index]) if valid_index else False
        if self._sticks_deflected(msg):
            self._operator_drive_time = time.monotonic()

    def _sticks_deflected(self, msg: Joy) -> bool:
        """Whether the operator is asking to drive the base themselves.

        Both this controller and the sticks command `cmd_vel`, so the two have
        to be arbitrated somewhere. Watching the sticks directly rather than
        `feedback.moving` yields on the first deflection, before the robot has
        started moving and before either command can fight the other.
        """
        deadzone = abs(float(self.get_parameter('drive_axis_deadzone').value))
        for axis in self.get_parameter('drive_axes').value:
            index = int(axis)
            if 0 <= index < len(msg.axes) and abs(msg.axes[index]) > deadzone:
                return True
        return False

    def _operator_driving(self, now: float) -> bool:
        """Whether the operator has the base, including a short release grace."""
        if self._operator_drive_time == 0.0:
            return False
        grace = max(
            float(self.get_parameter('operator_drive_grace').value), 0.0
        )
        return (now - self._operator_drive_time) <= grace

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self._enabled = request.data
        if self._enabled:
            # A fresh session should greet, however recently gaze last ran.
            self._forget_familiar()
        self._publish_enabled()
        enabled_text = 'ENABLED' if self._enabled else 'DISABLED'
        self.get_logger().info(f'Dog mode {enabled_text}')
        response.success = True
        response.message = f'dog mode enabled={self._enabled}'
        return response

    def _publish_enabled(self) -> None:
        self.enabled_pub.publish(Bool(data=self._enabled))

    # Timed control.

    def _tick(self) -> None:
        now = time.monotonic()
        dt = clamp(now - self._last_control_time, 0.0, 2.0 * self._nominal_dt)
        self._last_control_time = now
        self._track_applied_pose(now, dt)
        self._track_base_motion(now, dt)

        # Operator and posture faults must neutralize at once. A missing TF or a
        # late detection array is usually one dropped frame, so it gets a grace
        # window instead: snapping to centre and back on every hiccup is what
        # makes the gaze swerve.
        safety_block = self._safety_block_reason(now)
        if safety_block is not None:
            self._clear_fault()
            self._drop_to_idle(f'safety gate closed: {safety_block}')
            return

        healthy = self._detections_fresh(now) and self._process_fresh_people(
            now
        )
        if healthy:
            self._clear_fault()
            self._refresh_predicted_target(now)
        elif not self._fault_persisted(now):
            self._hold_command(now, dt)
            return
        else:
            self._target_visible = False
            if self._state in (
                GazeState.TRACK,
                GazeState.ENGAGE,
                GazeState.SETTLE,
            ):
                # Ease out of a sustained fault rather than dropping the body.
                # From an engaged base this also zeroes the base command on
                # the way out.
                self._begin_relax('detection or transform fault persisted')
            elif self._state != GazeState.RELAX:
                self._drop_to_idle('data fault persisted while inactive')
                return

        if self._state == GazeState.IDLE:
            if self._target_visible and self._target_body_xyz is not None:
                self._enter_attention()
            else:
                self._publish_eye_gaze(detected=False)
            return

        if self._state == GazeState.SETTLE:
            self._run_settle(now)
            return

        if (
            not self._target_visible
            and self._state in (GazeState.TRACK, GazeState.ENGAGE)
            and (now - self._target_last_seen)
            > float(self.get_parameter('target_lost_timeout').value)
        ):
            # Including ENGAGE stops the base rather than letting it keep
            # moving toward a predicted position no detection supports.
            self._begin_relax('target lost timeout expired')

        if (
            self._state == GazeState.RELAX
            and self._target_visible
            and self._target_body_xyz is not None
        ):
            self._enter_attention()

        # A target outside the pose envelope is reached by turning the base,
        # and one beyond the approach distance by walking toward it; while
        # either runs it owns the outputs and the pose controller stands down.
        if self._update_base_motion(now, dt):
            return

        state_time = now - self._state_since
        if self._state == GazeState.TRACK:
            self._run_track(dt)
        elif self._state == GazeState.RELAX:
            self._run_relax(state_time, dt)

    def _track_applied_pose(self, now: float, dt: float) -> None:
        """Follow the commanded pose with a first-order model of the body.

        Spot eases into a commanded body offset, so the offset in effect when a
        detection was captured is not the newest command. Compensating with the
        newer value over-corrects, and over-correction at the detection rate is
        what turns a lag into a visible sway.
        """
        tau = max(float(self.get_parameter('body_response_tau').value), 0.0)
        self._applied_yaw = first_order_step(
            self._applied_yaw, self._cmd_yaw, dt, tau
        )
        self._applied_pitch = first_order_step(
            self._applied_pitch, self._cmd_pitch, dt, tau
        )
        self._applied_history.append(
            (now, self._applied_yaw, self._applied_pitch)
        )

    def _track_base_motion(self, now: float, dt: float) -> None:
        """Integrate how far the base has actually moved under this controller.

        Spot eases into a commanded velocity rather than adopting it instantly,
        so the commanded values integrated directly would over-report the
        motion. The lag model keeps the integrals close enough that the latency
        corrections they feed do not themselves become a source of error.

        Translation gets the same treatment as rotation: an approach acts on a
        range measured before the body covered the last few tenths of a second
        of ground, and without subtracting that ground the robot walks the
        stale error a second time and overshoots the stopping distance.
        """
        tau = max(float(self.get_parameter('turn_response_tau').value), 0.0)
        self._base_rate = first_order_step(
            self._base_rate, self._cmd_base_angular, dt, tau
        )
        self._base_yaw += self._base_rate * dt
        self._base_yaw_history.append((now, self._base_yaw))
        self._base_speed = first_order_step(
            self._base_speed, self._cmd_base_linear, dt, tau
        )
        self._base_pos = (
            self._base_pos[0]
            + self._base_speed * math.cos(self._base_yaw) * dt,
            self._base_pos[1]
            + self._base_speed * math.sin(self._base_yaw) * dt,
        )
        self._base_pos_history.append((now, self._base_pos))

    def _base_yaw_at(self, when: float) -> float:
        """Modelled base heading at `when`, defaulting to the newest sample."""
        heading = self._base_yaw
        for sample_time, yaw in reversed(self._base_yaw_history):
            if sample_time <= when:
                return yaw
            heading = yaw
        return heading

    def _base_pos_at(self, when: float) -> Tuple[float, float]:
        """Modelled base position at `when`, defaulting to the newest sample."""
        position = self._base_pos
        for sample_time, sample in reversed(self._base_pos_history):
            if sample_time <= when:
                return sample
            position = sample
        return position

    def _applied_pose_at(self, when: float) -> Tuple[float, float]:
        """Modelled body attitude at `when`, defaulting to the newest sample."""
        offset = (self._applied_yaw, self._applied_pitch)
        for sample_time, yaw, pitch in reversed(self._applied_history):
            if sample_time <= when:
                return (yaw, pitch)
            offset = (yaw, pitch)
        return offset

    def _upright(self, now: float) -> bool:
        """Report whether Spot is up, tolerating an in-progress stand.

        The driver clears `standing` for as long as a stand command is still
        settling, and the pose refresh issues those constantly, so the raw flag
        blinks off mid-gaze. `sitting` is tracked from the sit command instead
        and is never masked, so a real sit still closes the gate immediately.
        """
        if self._sitting:
            return False
        if self._standing:
            return True
        if self._standing_confirmed <= 0.0:
            return False
        grace = max(float(self.get_parameter('standing_grace').value), 0.0)
        return (now - self._standing_confirmed) <= grace

    def _safety_allowed(self, now: float) -> bool:
        """Operator and posture conditions that must neutralize immediately."""
        return self._safety_block_reason(now) is None

    def _safety_block_reason(self, now: float) -> Optional[str]:
        """Describe the safety condition preventing gaze, if any."""
        # The held deadman is the operator's authorization to move the body;
        # releasing it must neutralize gaze immediately.
        if not self._enabled:
            return 'controller disabled'
        if not self._deadman_held:
            return 'deadman released'
        # The operator driving outranks gaze: dog mode hands the base back
        # rather than steering against them.
        if self._operator_driving(now):
            return 'operator took base control'
        if not self._upright(now):
            return 'robot not upright'
        if self._external_motion(now):
            return 'external motion detected'
        return None

    def _external_motion(self, now: float) -> bool:
        """Whether Spot is moving under something other than this controller.

        Turning the base to face someone behind Spot, or walking toward them,
        necessarily raises the driver's `moving` flag, so treating that flag as
        an unconditional fault would make a turn or an approach abort itself on
        its first cycle. Motion is attributed to this controller only while its
        own velocity commands are recent; once they stop, an unexplained
        `moving` closes the gate as before.
        """
        if not self._moving:
            return False
        grace = max(float(self.get_parameter('turn_moving_grace').value), 0.0)
        return (now - self._base_cmd_time) > grace

    def _detections_fresh(self, now: float) -> bool:
        """Whether the newest detection array is recent enough to act on."""
        timeout = float(self.get_parameter('detection_timeout').value)
        if (now - self._people_rx_time) > timeout:
            return False
        if self._people_msg is not None:
            stamp = Time.from_msg(self._people_msg.header.stamp)
            if stamp.nanoseconds != 0 and self._message_age(
                self._people_msg
            ) > timeout:
                return False
        return True

    def _fault_persisted(self, now: float) -> bool:
        """Start or extend the data-fault window, reporting once it expires."""
        if self._fault_since == 0.0:
            self._fault_since = now
        grace = max(float(self.get_parameter('data_fault_grace').value), 0.0)
        return (now - self._fault_since) > grace

    def _clear_fault(self) -> None:
        self._fault_since = 0.0

    def _hold_command(self, now: float, dt: float) -> None:
        """Re-send the current aim so a momentary data gap changes nothing."""
        if self._state == GazeState.ENGAGE:
            # Velocity commands expire in the driver, so a dropped detection
            # frame would otherwise stall the base part-way through a turn.
            # Rotation keeps closing on the modelled bearing — the base
            # rotation commanded since the last detection is subtracted from
            # it — but translation must never run blind: the stale data
            # withdraws the walk authorization inside `_run_engage`, which
            # zeroes forward speed at once.
            self._update_base_motion(now, dt)
            return
        if self._state == GazeState.SETTLE:
            self._publish_eye_gaze(detected=False)
            return
        if self._commanding:
            self._publish_outputs(detected=False)
        else:
            self._publish_eye_gaze(detected=False)

    def _drop_to_idle(self, reason: str) -> None:
        """Neutralize outputs once, clear selection, then stay idle."""
        self._stop_base_motion()
        if self._state != GazeState.IDLE:
            # Log before clearing the target and command so the transition line
            # preserves the context that caused the shutdown.
            self._transition(GazeState.IDLE, reason)
        if self._commanding:
            self._publish_pose(0.0, 0.0)
        if self._eye_active:
            self._cmd_yaw = 0.0
            self._cmd_pitch = 0.0
            self._publish_eye_gaze(detected=False)
        self._commanding = False
        self._eye_active = False
        self._cmd_yaw = 0.0
        self._cmd_pitch = 0.0
        self._clear_target()

    # Detection processing and target selection.

    def _process_fresh_people(self, now: float) -> bool:
        """Process one new detection message and update the gaze target.

        Returns False only when detections cannot be transformed safely into the
        body frame. Having no eligible target is still successful processing.
        """
        # The control loop runs faster than detections; process each message once.
        if self._processed_generation == self._people_generation:
            return True
        self._processed_generation = self._people_generation
        msg = self._people_msg

        # A target is visible only if this message produces a selected candidate.
        self._target_visible = False
        if msg is None:
            return True

        # Normalize all positions and velocities into Spot's body frame.
        transform = None
        source_frame = msg.header.frame_id
        body_frame = str(self.get_parameter('body_frame').value)
        if source_frame != body_frame:
            transform = self._lookup_people_transform(msg, body_frame)
            if transform is None:
                return False

        # Keep only valid human detections and convert them to gaze candidates.
        candidates = []
        for person in msg.people:
            if not person.is_human:
                continue
            candidate = self._candidate_from_person(person, transform)
            if candidate is not None:
                candidates.append(candidate)

        # Lead moving targets and compensate for time spent delivering the message.
        message_age = self._message_age(msg)

        # These detections were measured in the body frame as it stood when the
        # sweep was captured, so that is the offset they must be referred back to.
        self._observation_capture_time = now - message_age
        self._aim_offset = self._applied_pose_at(self._observation_capture_time)

        base_horizon = min(
            max(
                0.0,
                float(self.get_parameter('prediction_horizon').value)
                + message_age,
            ),
            max(
                0.0,
                float(
                    self.get_parameter('max_prediction_horizon').value
                ),
            ),
        )
        predicted = [
            candidate.predicted(base_horizon) for candidate in candidates
        ]

        # Ignore people outside the area where Spot should engage its gaze. The
        # person already being followed is kept out to a wider release radius so
        # drifting past the entry radius does not drop an active track.
        radius = float(self.get_parameter('attention_radius').value)
        release = max(
            radius,
            float(self.get_parameter('attention_release_radius').value),
        )
        predicted = [
            candidate
            for candidate in predicted
            if candidate.distance
            <= (release if self._is_current_target(candidate) else radius)
        ]

        # Select the nearest target without switching on brief or noisy changes.
        result = self._selector.choose(predicted, now)
        self._target_id = self._selector.current_id
        if result.candidate is None:
            # No target is valid yet, but the message itself was processed safely.
            return True

        # Cache the selected observation for continued prediction between messages.
        self._target_observation = result.candidate
        self._target_observation_time = now
        self._target_prediction_base = base_horizon
        self._target_last_seen = now
        self._target_visible = True
        self._set_target_position(result.candidate)

        # Decide familiarity against the previous memory, then take this
        # observation as the new one.
        self._target_familiar = self._is_familiar(result.candidate, now)
        self._remember_familiar(result.candidate, now)

        if result.changed:
            # A different person is a different stature, not filter noise.
            self._head_height = None
            self.get_logger().info(
                f'Gaze target selected: id={result.candidate.track_id}, '
                f'distance={result.candidate.distance:.2f} m'
            )
            if self._state == GazeState.ENGAGE:
                # Never keep moving toward where the previous person stood.
                self._interrupt_engage()
            elif self._state == GazeState.TRACK:
                self._enter_attention()
        return True

    def _is_familiar(self, candidate: TargetCandidate, now: float) -> bool:
        """Whether this is the person Spot was just engaging.

        A detector id reset or a short occlusion should not read as a new
        arrival; re-greeting on every one of those is what makes the gaze bob
        away and back.
        """
        if self._familiar_id is None:
            return False
        timeout = float(self.get_parameter('greet_repeat_timeout').value)
        if (now - self._familiar_time) > timeout:
            return False
        if candidate.track_id == self._familiar_id:
            return True
        if self._familiar_xy is None:
            return False
        return (
            math.hypot(
                candidate.x - self._familiar_xy[0],
                candidate.y - self._familiar_xy[1],
            )
            <= float(self.get_parameter('target_match_radius').value)
        )

    def _remember_familiar(
        self, candidate: TargetCandidate, now: float
    ) -> None:
        self._familiar_id = candidate.track_id
        self._familiar_xy = candidate.position
        self._familiar_time = now

    def _forget_familiar(self) -> None:
        self._familiar_id = None
        self._familiar_xy = None
        self._familiar_time = 0.0
        self._target_familiar = False

    def _enter_attention(self) -> None:
        """Track the person, queueing a greeting overlay for a new arrival."""
        if self._target_familiar:
            self._skip_greet()
        else:
            self._start_greet()
        if self._state != GazeState.TRACK:
            self._transition(
                GazeState.TRACK,
                'familiar target reacquired'
                if self._target_familiar
                else 'new target acquired',
            )

    def _start_greet(self) -> None:
        self._greet_pending = True
        self._greet_elapsed = 0.0

    def _skip_greet(self) -> None:
        self._greet_pending = False
        self._greet_elapsed = 0.0

    def _lookup_people_transform(self, msg: PeopleArray, body_frame: str):
        timeout = Duration(
            seconds=max(
                0.0,
                float(self.get_parameter('tf_lookup_timeout').value),
            )
        )
        stamp = Time.from_msg(msg.header.stamp)
        stamp_is_zero = stamp.nanoseconds == 0
        requested_time = Time() if stamp_is_zero else stamp
        try:
            transform = self.tf_buffer.lookup_transform(
                body_frame,
                msg.header.frame_id,
                requested_time,
                timeout=timeout,
            )
            self._tf_warned = False
            if not stamp_is_zero:
                self._tf_fallback_warned = False
            return transform
        except Exception as stamped_error:
            if (
                not stamp_is_zero
                and bool(self.get_parameter('allow_latest_tf_fallback').value)
            ):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        body_frame,
                        msg.header.frame_id,
                        Time(),
                        timeout=timeout,
                    )
                    self._tf_warned = False
                    if not self._tf_fallback_warned:
                        self.get_logger().warn(
                            'Timestamped people TF unavailable; '
                            'using latest TF fallback'
                        )
                        self._tf_fallback_warned = True
                    return transform
                except Exception:
                    pass

            if not self._tf_warned:
                self.get_logger().warn(
                    f'TF {msg.header.frame_id}->{body_frame} unavailable: '
                    f'{stamped_error}'
                )
                self._tf_warned = True
            return None

    def _candidate_from_person(
        self, person, transform
    ) -> Optional[TargetCandidate]:
        position = (
            float(person.position.x),
            float(person.position.y),
            float(person.position.z),
        )
        if not finite_vector(position):
            return None

        if transform is None:
            x, y, z = position
        else:
            x, y, z = _transform_point(transform, *position)

        velocity = (
            float(person.velocity.x),
            float(person.velocity.y),
            float(person.velocity.z),
        )
        if not finite_vector(velocity):
            velocity = (0.0, 0.0, 0.0)
        elif transform is not None:
            velocity = _rotate_vector(transform, *velocity)

        velocity = deadbanded_velocity(
            velocity,
            min_speed=float(self.get_parameter('min_prediction_speed').value),
            max_speed=float(self.get_parameter('max_prediction_speed').value),
        )

        height = float(person.size.z)
        min_height = float(self.get_parameter('min_person_height').value)
        max_height = float(self.get_parameter('max_person_height').value)
        top_z = None
        if math.isfinite(height) and min_height <= height <= max_height:
            if transform is None:
                top_z = position[2] + height / 2.0
            else:
                _, _, top_z = _transform_point(
                    transform,
                    position[0],
                    position[1],
                    position[2] + height / 2.0,
                )

        head_z = estimated_head_z(
            transformed_center_z=z,
            transformed_top_z=top_z,
            body_height_above_ground=float(
                self.get_parameter('body_height_above_ground').value
            ),
            default_person_height=float(
                self.get_parameter('default_person_height').value
            ),
        )
        values = (x, y, z, head_z, *velocity)
        if not finite_vector(values):
            return None
        return TargetCandidate(
            track_id=int(person.id),
            x=x,
            y=y,
            z=z,
            vx=velocity[0],
            vy=velocity[1],
            vz=velocity[2],
            head_z=head_z,
        )

    def _message_age(self, msg: PeopleArray) -> float:
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            return 0.0
        age_ns = self.get_clock().now().nanoseconds - stamp.nanoseconds
        return max(0.0, age_ns * 1.0e-9)

    def _refresh_predicted_target(self, now: float) -> None:
        if self._target_observation is None:
            self._target_body_xyz = None
            return
        max_horizon = max(
            0.0, float(self.get_parameter('max_prediction_horizon').value)
        )
        remaining = max(0.0, max_horizon - self._target_prediction_base)
        elapsed = clamp(now - self._target_observation_time, 0.0, remaining)
        self._set_target_position(self._target_observation.predicted(elapsed))

    def _is_current_target(self, candidate: TargetCandidate) -> bool:
        """Match the followed person by stable id or by last known position."""
        if self._selector.current_id is None:
            return False
        if candidate.track_id == self._selector.current_id:
            return True
        last = self._selector.current_position
        if last is None:
            return False
        return (
            math.hypot(candidate.x - last[0], candidate.y - last[1])
            <= float(self.get_parameter('target_match_radius').value)
        )

    def _set_target_position(self, candidate: TargetCandidate) -> None:
        self._target_body_xyz = (candidate.x, candidate.y, candidate.z)
        self._target_head_z = candidate.head_z

    def _clear_target(self) -> None:
        self._selector.clear()
        self._target_id = None
        self._target_observation = None
        self._target_body_xyz = None
        self._target_visible = False
        self._aim_offset = (0.0, 0.0)
        self._head_height = None

    # State behavior.

    def _gaze_angles(self, dt: float = 0.0) -> Tuple[float, float]:
        x, y, _ = self._target_body_xyz
        max_yaw = float(self.get_parameter('max_yaw').value)
        max_pitch = float(self.get_parameter('max_pitch').value)
        pitch_sign = float(self.get_parameter('pitch_sign').value)
        compensate = bool(
            self.get_parameter('body_feedback_compensation').value
        )
        # Bound the sum rather than the relative angle when compensating, so an
        # offset the body already holds is not clipped away before it is used.
        relative_yaw, relative_pitch = gaze_angles(
            x=x,
            y=y,
            head_z=self._target_head_z,
            pitch_sign=pitch_sign,
            max_yaw=math.pi if compensate else max_yaw,
            max_pitch=math.pi if compensate else max_pitch,
        )
        offset_yaw, offset_pitch = (
            self._aim_offset if compensate else (0.0, 0.0)
        )
        horizontal = max(math.hypot(x, y), 0.1)
        aim_pitch = self._smoothed_head_pitch(
            relative_pitch + offset_pitch, horizontal, pitch_sign, dt
        )
        return (
            absolute_aim(relative_yaw, offset_yaw, max_yaw),
            clamp(aim_pitch, -abs(max_pitch), abs(max_pitch)),
        )

    def _smoothed_head_pitch(
        self, aim_pitch: float, horizontal: float, pitch_sign: float, dt: float
    ) -> float:
        """Smooth the target's head height rather than the pitch angle itself.

        The detector's box height is raw, per-frame output (the tracker filters
        only planar position and velocity), so head height is the noisiest input
        the gaze has, and it drives pitch directly. A person's actual head height
        barely changes, so it takes heavy filtering without costing anything,
        whereas the pitch angle must stay free to follow someone walking closer.
        """
        tau = float(self.get_parameter('head_height_tau').value)
        if tau <= 0.0:
            return aim_pitch
        # Elevation is measured from the aim plane, so it is already free of the
        # body attitude that the compensated aim removed.
        measured = elevation_height(aim_pitch * pitch_sign, horizontal)
        if self._head_height is None:
            self._head_height = measured
        else:
            self._head_height = first_order_step(
                self._head_height, measured, dt, tau
            )
        return pitch_sign * height_elevation(self._head_height, horizontal)

    def _run_track(self, dt: float) -> None:
        if self._target_body_xyz is not None:
            yaw, pitch = self._gaze_angles(dt)
            pitch = self._apply_greet_overlay(pitch, dt)
            self.get_logger().info(
                f'Gaze target: id={self._target_id}, '
                f'yaw={yaw:.2f} rad, pitch={pitch:.2f} rad',
                throttle_duration_sec=1.0,
            )
            self._move_command_toward(yaw, pitch, dt)
        self._publish_outputs(detected=self._target_visible)

    def _apply_greet_overlay(self, pitch: float, dt: float) -> float:
        """Blend the greeting look-up into the tracking pitch.

        The greeting is an overlay on tracking, not a state: the envelope
        advances only while the tracking pose is live, so a new arrival who
        first needs the base turned or walked still gets the full greeting
        delivered facing them once the pose controller has the body back.
        Ramping the boost out rather than dropping it is what keeps the end
        of every greeting from reading as a bob.
        """
        if not self._greet_pending:
            return pitch
        ramp = max(float(self.get_parameter('greet_ramp').value), 1.0e-3)
        hold = max(0.0, float(self.get_parameter('greet_hold').value))
        self._greet_elapsed += dt
        elapsed = self._greet_elapsed
        total = ramp + hold + ramp
        if elapsed >= total:
            self._greet_pending = False
            return pitch
        boost_scale = clamp(
            min(elapsed, total - elapsed) / ramp, 0.0, 1.0
        )
        return clamp(
            pitch
            + float(self.get_parameter('pitch_sign').value)
            * float(self.get_parameter('greet_pitch_boost').value)
            * boost_scale,
            -abs(float(self.get_parameter('max_pitch').value)),
            abs(float(self.get_parameter('max_pitch').value)),
        )

    # Base rotation and approach for targets outside the body-pose envelope
    # or beyond the approach distance.

    def _compensated_target_planar(self) -> Optional[Tuple[float, float]]:
        """Target position in the current footprint, ego-motion corrected.

        The detection behind this position describes where the person stood
        relative to a base pose Spot has since turned and walked away from.
        Referring it through the modelled heading and position held at capture
        time is what closes the loop on where the person is now rather than on
        rotation and ground the base has already covered.
        """
        if self._target_body_xyz is None:
            return None
        x, y, _ = self._target_body_xyz
        if bool(self.get_parameter('body_feedback_compensation').value):
            offset = self._aim_offset[0]
            x, y = (
                math.cos(offset) * x - math.sin(offset) * y,
                math.sin(offset) * x + math.cos(offset) * y,
            )
        capture = self._observation_capture_time
        capture_yaw = self._base_yaw_at(capture)
        capture_pos = self._base_pos_at(capture)
        # Where the person stands in the model's world frame...
        world_x = (
            capture_pos[0]
            + math.cos(capture_yaw) * x
            - math.sin(capture_yaw) * y
        )
        world_y = (
            capture_pos[1]
            + math.sin(capture_yaw) * x
            + math.cos(capture_yaw) * y
        )
        # ...seen from the pose the base has reached since.
        dx = world_x - self._base_pos[0]
        dy = world_y - self._base_pos[1]
        return (
            math.cos(self._base_yaw) * dx + math.sin(self._base_yaw) * dy,
            -math.sin(self._base_yaw) * dx + math.cos(self._base_yaw) * dy,
        )

    def _target_bearing(self) -> Optional[float]:
        """Footprint-relative bearing to the target, unbounded in (-pi, pi].

        The gaze command is clamped to `max_yaw`, which is exactly the limit
        this has to see past, so the bearing is taken before that clamp.
        """
        planar = self._compensated_target_planar()
        if planar is None:
            return None
        return wrap_angle(math.atan2(planar[1], planar[0]))

    def _target_range(self) -> Optional[float]:
        """Planar body-origin-to-person distance, ego-motion corrected."""
        planar = self._compensated_target_planar()
        if planar is None:
            return None
        return math.hypot(planar[0], planar[1])

    def _update_base_motion(self, now: float, dt: float) -> bool:
        """Run the blended base motion, reporting whether it owns this cycle.

        ENGAGE owns base velocity exclusively; everything else leaves the
        base alone.
        """
        if self._turn_pub is None:
            return False
        bearing = self._target_bearing()

        if self._state == GazeState.ENGAGE:
            return self._run_engage(now, dt, bearing)

        if self._state != GazeState.TRACK:
            return False
        if bearing is None or not self._target_visible:
            self._motion.reset()
            return False
        if self._turning_enabled() and self._motion.should_turn(bearing, now):
            self._begin_engage(
                now, bearing, 'target outside body-pose yaw envelope'
            )
            return self._run_engage(now, dt, bearing)
        if self._turning_enabled() and self._recenter_wanted(now, bearing):
            self._motion.commit_turn()
            self._begin_engage(
                now, bearing, 'recentring base under sustained gaze offset'
            )
            return self._run_engage(now, dt, bearing)
        if self._approach_wanted(now):
            self._begin_engage(
                now, bearing, 'target beyond approach distance'
            )
            return self._run_engage(now, dt, bearing)
        return False

    def _turning_enabled(self) -> bool:
        return bool(self.get_parameter('base_turn_enabled').value)

    def _recenter_wanted(self, now: float, bearing: float) -> bool:
        """Whether a sustained gaze offset should square the base up.

        The pose envelope can hold a person at a constant yaw offset
        indefinitely, which reads as Spot watching them sideways. This is
        the slow path into a turn: a much lower threshold than the envelope
        edge, guarded by a long debounce so someone merely passing through
        never triggers it. The committed turn then runs to the ordinary
        exit threshold, leaving the body nearly facing the person.
        """
        threshold = float(self.get_parameter('recenter_yaw').value)
        if threshold <= 0.0:
            return False
        if abs(bearing) <= threshold:
            self._recenter_since = 0.0
            return False
        if self._recenter_since == 0.0:
            self._recenter_since = now
            return False
        debounce = max(
            float(self.get_parameter('recenter_debounce').value), 0.0
        )
        return (now - self._recenter_since) >= debounce

    def _approach_wanted(self, now: float) -> bool:
        """Whether TRACK should engage the base to start walking.

        Entry requires the resume margin beyond the stopping distance; the
        bearing no longer gates entry because the alignment factor feathers
        forward speed continuously — steering and walking start together as
        an arc.
        """
        if not bool(self.get_parameter('approach_enabled').value):
            return False
        if not self._approach_data_fresh(now):
            return False
        rng = self._target_range()
        if rng is None:
            return False
        return self._motion.wants_walk(rng)

    def _approach_data_fresh(self, now: float) -> bool:
        """Whether a real detection, not just prediction, backs the target.

        Eye gaze and orientation may coast on the short prediction window, but
        a predicted or stale position is never sole authorization for forward
        translation.
        """
        if not self._target_visible or self._target_body_xyz is None:
            return False
        timeout = float(self.get_parameter('detection_timeout').value)
        return (now - self._target_last_seen) <= timeout

    def _begin_engage(self, now: float, bearing: float, reason: str) -> None:
        """Take the base, leaving the body pose exactly as it is.

        The pose deliberately is not neutralized here. A pose only reaches the
        robot on the next stand, and stands are suppressed while the base is
        engaged, so a neutral pose published now would not be applied: the
        body would physically hold its pre-engage offset while the model of it
        decayed to zero, throwing off the bearing being closed on. Worse, the
        queued neutral pose would then land in one step on the first stand
        afterwards. Holding the pose keeps the model truthful and lets the
        pose controller ease to the residual angle once the base has stopped.
        """
        self._recenter_since = 0.0
        self._turn_progress_time = now
        self._turn_best_bearing = abs(bearing)
        self._transition(GazeState.ENGAGE, reason)

    def _walk_allowed(self, now: float) -> bool:
        """Authorization for forward motion this cycle.

        Walking demands a real, fresh detection: a predicted or stale
        position is never sole authorization for translation. The greeting
        no longer blocks it — the overlay plays out over the tracking pose
        once the base hands back, so approach and greeting can interleave.
        """
        if not bool(self.get_parameter('approach_enabled').value):
            return False
        # A stale detection array is as blind as a missing one: rotation may
        # coast on the model through the fault grace, translation may not.
        return self._detections_fresh(now) and self._approach_data_fresh(now)

    def _run_engage(
        self, now: float, dt: float, bearing: Optional[float]
    ) -> bool:
        planar = self._compensated_target_planar()
        if bearing is None or planar is None:
            return self._release_engage(now, dt, bearing)

        rng = math.hypot(planar[0], planar[1])
        allow_walk = self._walk_allowed(now)
        turning = self._turning_enabled() and self._motion.should_turn(
            bearing, now
        )
        walking = self._motion.update_walk(rng, allow_walk)
        if not turning and not walking:
            return self._release_engage(now, dt, bearing)

        in_place = (
            turning and abs(self._motion.linear) <= self._motion.IN_PLACE_SPEED
        )
        if in_place:
            if self._turn_stalled(now, bearing):
                self.get_logger().warn(
                    'Base turn made no progress; releasing to relax'
                )
                self._stop_base_motion()
                self._begin_relax('base turn stalled')
                return True
        else:
            # The progress watchdog only means anything for an in-place turn:
            # while walking, the bearing legitimately holds steady, so keep
            # the markers re-armed for the next pure-turn stretch.
            self._turn_progress_time = now
            self._turn_best_bearing = abs(bearing)

        linear, angular = self._motion.step(bearing, rng, dt, allow_walk)
        self._publish_base_motion(linear, angular)
        # Everything needed to tell an engagement that is converging from one
        # that is oscillating, stalling or chasing a jumping detection.
        self.get_logger().info(
            f'engage: bearing={math.degrees(bearing):+.0f}deg '
            f'distance={rng:.2f}m goal={self._motion.stop_distance:.2f}m '
            f'linear={linear:.2f} angular={angular:+.2f} '
            f'id={self._target_id} pose_yaw={self._aim_offset[0]:+.2f} '
            f'visible={self._target_visible} moving={self._moving}',
            throttle_duration_sec=0.25,
        )
        # Drive the eyes from the true bearing: the body command holds its
        # pre-engage offset, so they would otherwise lag behind the motion.
        self._publish_eye_gaze(detected=self._target_visible, aim_yaw=bearing)
        return True

    def _interrupt_engage(self) -> None:
        """Stop the base before engaging a different person."""
        now = time.monotonic()
        self._stop_base_motion()
        if self._target_familiar:
            self._skip_greet()
        else:
            self._start_greet()
        self._settle_until = now + max(
            float(self.get_parameter('turn_settle_time').value), 0.0
        )
        self._transition(GazeState.SETTLE, 'target changed while engaged')

    def _turn_stalled(self, now: float, bearing: float) -> bool:
        """Abort a turn that stops closing on the target.

        A person circling Spot can legitimately keep it turning indefinitely, so
        elapsed time alone is not the fault condition — lack of progress is.
        """
        epsilon = max(
            float(self.get_parameter('turn_progress_epsilon').value), 0.0
        )
        if abs(bearing) < (self._turn_best_bearing - epsilon):
            self._turn_best_bearing = abs(bearing)
            self._turn_progress_time = now
            return False
        timeout = max(
            float(self.get_parameter('turn_stall_timeout').value), 0.0
        )
        return (now - self._turn_progress_time) > timeout

    def _release_engage(
        self, now: float, dt: float, bearing: Optional[float]
    ) -> bool:
        """Ramp the base to a stop instead of dropping the command.

        The rate floor means a turn is still commanding real motion when the
        target angle is reached, so cutting straight to zero asks Spot to plant
        mid-stride. Ramping down over the same acceleration limits the motion
        started with ends it on a settled stance instead.
        """
        linear, angular = self._motion.decay(dt)
        if abs(linear) > 1.0e-3 or abs(angular) > 1.0e-3:
            self._publish_base_motion(linear, angular)
            self._publish_eye_gaze(
                detected=self._target_visible, aim_yaw=bearing
            )
            return True
        self._finish_engage(now)
        return True

    def _finish_engage(self, now: float) -> None:
        """Stop the base and wait out its motion before posing again."""
        self._stop_base_motion()
        self._settle_until = now + max(
            float(self.get_parameter('turn_settle_time').value), 0.0
        )
        self._transition(
            GazeState.SETTLE,
            'base motion stopped; waiting for it to settle',
        )

    def _run_settle(self, now: float) -> None:
        """Hand the pose back as soon as the base is demonstrably at rest.

        A body pose published while Spot is still finishing its motion is
        overridden by it, so the gaze would appear to skip a beat. But a
        fixed wait charges that worst case on every handback, and the dead
        time between stopping and looking is exactly the hitch being
        engineered out. The driver's `moving` flag and the modelled base
        velocity decide instead; `turn_settle_time` remains only as the
        upper bound for a driver that never clears the flag.
        """
        self._publish_eye_gaze(
            detected=self._target_visible, aim_yaw=self._target_bearing()
        )
        at_rest = (
            not self._moving
            and abs(self._base_rate) < 0.05
            and abs(self._base_speed) < 0.05
        )
        if not at_rest and now < self._settle_until:
            return
        if self._target_visible and self._target_body_xyz is not None:
            self._transition(
                GazeState.TRACK, 'post-motion settle period complete'
            )
        else:
            self._begin_relax('target unavailable after base motion')

    def _publish_base_motion(self, linear: float, angular: float) -> None:
        """Publish one base velocity command and record it as our own motion."""
        if self._turn_pub is None:
            return
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self._turn_pub.publish(twist)
        self._base_cmd_time = time.monotonic()
        self._cmd_base_linear = float(linear)
        self._cmd_base_angular = float(angular)
        self._base_motion_active = (
            abs(linear) > 1.0e-6 or abs(angular) > 1.0e-6
        )

    def _stop_base_motion(self) -> None:
        """Zero the base command once, from any exit path."""
        self._motion.reset()
        if self._base_motion_active:
            self._publish_base_motion(0.0, 0.0)
        self._base_motion_active = False

    def _begin_relax(self, reason: str) -> None:
        self._stop_base_motion()
        self._relax_start = (self._cmd_yaw, self._cmd_pitch)
        self._transition(GazeState.RELAX, reason)
        self._clear_target()

    def _run_relax(self, state_time: float, dt: float) -> None:
        duration = max(
            float(self.get_parameter('relax_duration').value), 1.0e-3
        )
        fraction = 1.0 - clamp(state_time / duration, 0.0, 1.0)
        desired_yaw = self._relax_start[0] * fraction
        desired_pitch = self._relax_start[1] * fraction
        max_rate = float(self.get_parameter('max_rate_rad_s').value)
        self._cmd_yaw += clamp(
            desired_yaw - self._cmd_yaw, -max_rate * dt, max_rate * dt
        )
        self._cmd_pitch += clamp(
            desired_pitch - self._cmd_pitch, -max_rate * dt, max_rate * dt
        )
        self._publish_outputs(detected=False)

        neutral = abs(self._cmd_yaw) < 1.0e-6 and abs(self._cmd_pitch) < 1.0e-6
        if state_time >= duration and neutral:
            self._commanding = False
            self._eye_active = False
            self._transition(GazeState.IDLE, 'neutral pose reached')

    def _move_command_toward(
        self, desired_yaw: float, desired_pitch: float, dt: float
    ) -> None:
        max_rate = float(self.get_parameter('max_rate_rad_s').value)
        self._cmd_yaw = smoothed_rate_limited_step(
            self._cmd_yaw,
            desired_yaw,
            dt,
            float(self.get_parameter('smoothing_tau').value),
            max_rate,
        )
        # Pitch gets its own, slower constant: head height comes from the raw
        # per-frame detection box rather than the tracker's filtered state, so
        # it is by far the noisier of the two axes.
        self._cmd_pitch = smoothed_rate_limited_step(
            self._cmd_pitch,
            desired_pitch,
            dt,
            float(self.get_parameter('pitch_smoothing_tau').value),
            max_rate,
        )

    def _transition(self, state: GazeState, reason: str) -> None:
        now = time.monotonic()
        state_age = max(now - self._state_since, 0.0)
        bearing = self._target_bearing()
        bearing_text = (
            'none' if bearing is None else f'{math.degrees(bearing):+.1f}deg'
        )
        distance = self._target_range()
        distance_text = 'none' if distance is None else f'{distance:.2f}m'
        self.get_logger().info(
            f'Gaze state: {self._state.name} -> {state.name}; '
            f'reason={reason}; age={state_age:.2f}s; '
            f'target={self._target_id}; visible={self._target_visible}; '
            f'bearing={bearing_text}; distance={distance_text}; '
            f'cmd_yaw={self._cmd_yaw:+.2f}; cmd_pitch={self._cmd_pitch:+.2f}'
        )
        self._state = state
        self._state_since = now

    # Synchronized outputs.

    def _publish_outputs(self, detected: bool) -> None:
        self._publish_pose(self._cmd_pitch, self._cmd_yaw)
        self._publish_eye_gaze(detected=detected)

    def _publish_pose(
        self, pitch: float, yaw: float, refresh: bool = True
    ) -> None:
        pose = Pose()
        _yaw_pitch_to_quaternion(pose, pitch, yaw)
        self.body_pose_pub.publish(pose)
        self._commanding = True
        if refresh:
            self._request_stand_refresh()

    def _request_stand_refresh(self) -> None:
        """Re-issue a stand so the streamed body pose actually reaches Spot.

        `body_pose` only stores the pose in the driver's mobility params; the
        robot adopts it when the next stand command is built from those params.
        Without this the gaze holds whatever pose the last stand latched, which
        looks like a single glance instead of continuous tracking. Refreshing
        is skipped unless Spot is standing still so an in-progress walk or sit
        is never interrupted.
        """
        if self._stand_client is None:
            return

        now = time.monotonic()
        # A stand rebuilds the mobility params and preempts whatever velocity
        # command is in flight, so it must never land while the operator is
        # driving — including the neutralizing pose gaze publishes on its way out.
        if (
            not self._upright(now)
            or self._moving
            or self._operator_driving(now)
        ):
            return
        # Nor while this controller itself owns the base: a stand issued
        # alongside an active turn or approach command would cancel it.
        if self._state == GazeState.ENGAGE or self._base_motion_active:
            return

        timeout = max(
            float(self.get_parameter('stand_request_timeout').value), 0.0
        )
        if self._stand_future is not None:
            if not self._stand_future.done():
                # Drop a hung request rather than stalling the refresh forever.
                if (now - self._stand_request_time) <= timeout:
                    return
                self._stand_client.remove_pending_request(self._stand_future)
            self._stand_future = None

        if (now - self._stand_request_time) < self._stand_refresh_period:
            return
        if not self._stand_client.service_is_ready():
            if not self._stand_warned:
                self.get_logger().warn(
                    f'Stand service '
                    f'{self.get_parameter("stand_service").value} '
                    'unavailable; body pose will not reach the robot'
                )
                self._stand_warned = True
            return

        self._stand_warned = False
        self._stand_request_time = now
        self._stand_future = self._stand_client.call_async(Trigger.Request())
        self._stand_future.add_done_callback(self._on_stand_response)

    def _on_stand_response(self, future) -> None:
        """Treat an accepted stand as proof Spot is up.

        The refresh keeps issuing new stands, so the driver's `standing` flag
        can read false indefinitely while the latest one is still settling.
        Waiting for that flag alone drops the gaze to neutral roughly once per
        grace window and picks it straight back up, which reads as a swerve.
        A rejected stand stops renewing the confirmation, so a robot that is
        no longer able to stand still closes the gate.
        """
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f'Stand refresh failed: {error}')
            return
        if response is not None and response.success:
            self._standing_confirmed = time.monotonic()

    def _publish_eye_gaze(
        self, detected: bool, aim_yaw: Optional[float] = None
    ) -> None:
        if self._eye_gaze_pub is None:
            return
        msg = self._gaze_msg_type()
        msg.header.stamp = self.get_clock().now().to_msg()
        max_yaw = max(
            abs(float(self.get_parameter('max_yaw').value)), 1.0e-6
        )
        max_pitch = max(
            abs(float(self.get_parameter('max_pitch').value)), 1.0e-6
        )
        # While the base is turning the body command is neutral, so the eyes
        # follow the raw bearing instead and pin to the side they are chasing.
        yaw = self._cmd_yaw if aim_yaw is None else aim_yaw
        # Screen convention: +x is robot-left and +y is down.
        msg.x = clamp(-yaw / max_yaw, -1.0, 1.0)
        msg.y = clamp(self._cmd_pitch / max_pitch, -1.0, 1.0)
        msg.detected = detected
        msg.num_faces = 1 if detected else 0
        self._eye_gaze_pub.publish(msg)
        self._eye_active = (
            detected or abs(msg.x) > 1.0e-6 or abs(msg.y) > 1.0e-6
        )


def main(args=None):
    rclpy.init(args=args)
    node = GazeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave a velocity command outstanding on the way out.
        node._stop_base_motion()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
