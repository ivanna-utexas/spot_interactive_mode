#!/usr/bin/env python3
"""Interactive mode: one switchable STAY / FOLLOW / DANCE behavior for Spot.

This node is the single authority over *which* controller may command Spot.
The state machine (mode_machine.py) decides what should be running; this node
reconciles the real controllers toward that and reports what they are
actually doing:

  - spot_dog_mode's gaze controller, for STAY and FOLLOW. Driven through its
    /dog_mode/enable service and its approach/turn parameters; its latched
    /dog_mode/enabled topic is the confirmation.
  - the dance runner (dance_runner.py), for DANCE. Runs inside this node:
    /body_pose + a 10 Hz stand re-assert for pose segments, and base velocity
    for locomotion segments, never both at once.

Control paths and the operator:
  - All base velocity goes to cmd_vel_intermediate, the same topic dog mode
    uses, which spot_joy's teleop relays to /cmd_vel only while the auto
    deadman (R1) is held. Releasing R1 therefore stops the base no matter
    which mode is running. The launch file also moves dog mode's own deadman
    to R1 so a single button authorizes all autonomous motion.
  - This node does not claim, power on or stand Spot. The operator does that
    from the controller (Options, then Square), and keeps the controller and
    e-stop throughout.

Every service call is asynchronous: a blocking call inside a timer callback
deadlocks the single-threaded executor until it times out.

Interfaces:
  srv  /interactive_mode/set_mode   interactive_mode_msgs/SetMode (operator)
  sub  /interactive_mode/command    interactive_mode_msgs/ModeCommand (voice etc.)
  pub  /interactive_mode/state      interactive_mode_msgs/ModeState (latched)
"""

import os
import signal
import subprocess
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from spot_msgs.msg import Feedback
from spot_msgs.srv import SetLocomotion
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from interactive_mode_msgs.msg import ModeCommand, ModeState
from interactive_mode_msgs.srv import SetMode

from .dance_runner import DanceRunner
from .dance_timeline import DanceTimeline, rpy_to_quaternion
from .mode_machine import MachineConfig, Mode, ModeMachine, Observation

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class InteractiveModeNode(Node):
    def __init__(self):
        super().__init__('interactive_mode')

        p = self.declare_parameter
        p('control_rate', 50.0)
        p('stand_reassert_rate', 10.0)
        # Operator input
        p('joy_topic', '/joy')
        p('deadman_button', 5)            # R1: teleop's auto deadman
        p('joy_timeout', 0.5)             # joy_linux autorepeats at 50 Hz
        # Command filtering / transitions
        p('min_command_confidence', 0.6)
        p('switch_cooldown', 2.0)
        p('exit_timeout', 4.0)
        p('enter_timeout', 3.0)
        p('dance_settle_time', 0.75)
        p('post_dance_mode', 'stay')
        p('require_standing', True)
        p('not_standing_grace', 2.5)
        p('start_mode', 'idle')           # 'stay' to come up already enabled
        # Dog mode
        p('dog_mode_node', '/dog_mode_gaze_controller')
        p('dog_mode_retry_period', 0.5)
        p('stay_allow_turn', False)       # STAY may turn in place to face people
        p('follow_allow_turn', True)
        # Dance
        p('timeline', '')
        p('calibration_mode', False)
        p('audio_file', '')
        p('audio_device', 'plughw:3,0')   # ALSA 'default' is not configured here
        p('audio_lead_sec', 0.0)
        p('abort_ramp_sec', 0.5)
        p('rest_locomotion_mode', 1)      # teleop walk mode's gait
        p('base_cmd_topic', 'cmd_vel_intermediate')

        g = lambda name: self.get_parameter(name).value  # noqa: E731

        post_dance = Mode.parse(g('post_dance_mode'))
        if post_dance not in (Mode.STAY, Mode.FOLLOW, Mode.IDLE):
            raise ValueError("post_dance_mode must be 'stay', 'follow' or 'idle'")
        self.machine = ModeMachine(MachineConfig(
            min_confidence=float(g('min_command_confidence')),
            switch_cooldown=float(g('switch_cooldown')),
            exit_timeout=float(g('exit_timeout')),
            enter_timeout=float(g('enter_timeout')),
            dance_settle_time=float(g('dance_settle_time')),
            post_dance_mode=post_dance,
            require_standing=bool(g('require_standing')),
            not_standing_grace=float(g('not_standing_grace')),
        ))

        if g('calibration_mode'):
            timeline = DanceTimeline.calibration()
            self.get_logger().warn('CALIBRATION MODE: "dance" plays one 3 s bounce')
        else:
            if not g('timeline'):
                raise ValueError('timeline parameter is required')
            timeline = DanceTimeline.from_file(g('timeline'))
        self.get_logger().info(
            f'Dance timeline: {len(timeline.segments)} segments, '
            f'{timeline.duration:.2f} s, gaits {timeline.gaits_used}')
        self.runner = DanceRunner(
            timeline,
            audio_lead_sec=float(g('audio_lead_sec')),
            abort_ramp_sec=float(g('abort_ramp_sec')),
            rest_locomotion_mode=int(g('rest_locomotion_mode')),
        )
        self.audio_file = g('audio_file')
        self.audio_device = g('audio_device')
        if self.audio_file and not os.path.isfile(self.audio_file):
            self.get_logger().error(f'audio_file not found: {self.audio_file}; dancing silently')
            self.audio_file = ''
        self._audio_proc: Optional[subprocess.Popen] = None

        # Observed robot / operator / controller state
        self._joy_time = 0.0
        self._deadman = False
        self._standing: Optional[bool] = None
        self._moving = False
        self._dog_enabled: Optional[bool] = None
        self._dance_finished = False

        # Reconciliation bookkeeping (one request in flight per service)
        self._dog_enable_future = None
        self._dog_enable_sent_at = 0.0
        self._dog_enable_sent_value: Optional[bool] = None
        self._dog_params_future = None
        self._dog_params_sent_at = 0.0
        self._dog_profile_applied: Optional[Mode] = None
        self._stand_future = None
        self._gait_requested: Optional[int] = None
        self._stand_reassert = False

        # I/O
        self.create_subscription(Joy, g('joy_topic'), self._on_joy, 10)
        self.create_subscription(Feedback, '/status/feedback', self._on_feedback, 10)
        self.create_subscription(Bool, '/dog_mode/enabled', self._on_dog_enabled, LATCHED)
        self.create_subscription(ModeCommand, '/interactive_mode/command', self._on_command, 10)
        self.create_service(SetMode, '/interactive_mode/set_mode', self._on_set_mode)

        self.pose_pub = self.create_publisher(Pose, 'body_pose', 1)
        self.twist_pub = self.create_publisher(Twist, g('base_cmd_topic'), 1)
        self.state_pub = self.create_publisher(ModeState, '/interactive_mode/state', LATCHED)

        dog = g('dog_mode_node').rstrip('/')
        self.cli_dog_enable = self.create_client(SetBool, '/dog_mode/enable')
        self.cli_dog_params = self.create_client(SetParameters, f'{dog}/set_parameters')
        self.cli_stand = self.create_client(Trigger, 'stand')
        self.cli_locomotion = self.create_client(SetLocomotion, 'locomotion_mode')

        self._dt = 1.0 / max(float(g('control_rate')), 1.0)
        self.create_timer(self._dt, self._tick)
        self.create_timer(1.0 / max(float(g('stand_reassert_rate')), 0.1), self._reassert_stand)
        self.create_timer(1.0, self._publish_state)

        start = g('start_mode')
        if start and start != 'idle':
            # Applied once feedback reports standing; see _tick.
            self._pending_start: Optional[str] = start
        else:
            self._pending_start = None

        self.get_logger().info(
            'Interactive mode ready (idle). Hold R1 for autonomy. '
            'ros2 service call /interactive_mode/set_mode '
            'interactive_mode_msgs/srv/SetMode "{mode: stay}"')

    # ------------------------------------------------------------- callbacks

    def _on_joy(self, msg: Joy) -> None:
        idx = int(self.get_parameter('deadman_button').value)
        self._deadman = 0 <= idx < len(msg.buttons) and bool(msg.buttons[idx])
        self._joy_time = time.monotonic()

    def _on_feedback(self, msg: Feedback) -> None:
        self._standing = msg.standing
        self._moving = msg.moving

    def _on_dog_enabled(self, msg: Bool) -> None:
        if msg.data != self._dog_enabled:
            self.get_logger().info(f'dog mode reports enabled={msg.data}')
        self._dog_enabled = msg.data

    def _on_command(self, msg: ModeCommand) -> None:
        decision = self.machine.request(
            msg.mode, time.monotonic(), source=msg.source or 'unknown',
            confidence=float(msg.confidence))
        heard = f' ("{msg.utterance}")' if msg.utterance else ''
        self._log_decision(
            decision, f'command {msg.mode!r} from {msg.source}{heard}')
        self._publish_state()

    def _on_set_mode(self, request: SetMode.Request, response: SetMode.Response):
        decision = self.machine.request(request.mode, time.monotonic(), source='service')
        self._log_decision(decision, f'set_mode {request.mode!r}')
        response.success = decision.accepted
        response.message = decision.message
        response.current_mode = self.machine.mode.value
        self._publish_state()
        return response

    def _log_decision(self, decision, what: str) -> None:
        # Separate call sites: rclpy raises if one call site logs at two
        # different severities.
        if decision.accepted:
            self.get_logger().info(f'{what}: {decision.message}')
        else:
            self.get_logger().warn(f'{what}: {decision.message}')

    # ------------------------------------------------------------------ tick

    def _deadman_held(self, now: float) -> bool:
        timeout = float(self.get_parameter('joy_timeout').value)
        return self._deadman and (now - self._joy_time) <= timeout

    def _tick(self) -> None:
        now = time.monotonic()

        if self._pending_start and self._standing:
            self.machine.request(self._pending_start, now, source='operator')
            self._pending_start = None

        obs = Observation(
            now=now,
            deadman_held=self._deadman_held(now),
            standing=self._standing,
            moving=self._moving,
            dog_enabled=self._dog_enabled,
            dance_active=self.runner.running,
            dance_finished=self._dance_finished,
        )
        self._dance_finished = False
        prev = (self.machine.mode, self.machine.target, self.machine.phase)
        directive = self.machine.step(obs)
        if (self.machine.mode, self.machine.target, self.machine.phase) != prev:
            self.get_logger().info(
                f'[{self.machine.phase.value}] mode={self.machine.mode.value} '
                f'target={self.machine.target.value}: {self.machine.reason}')

        self._reconcile_dance(directive.dance_run, now)
        self._reconcile_dog(directive.dog_enabled, directive.dog_profile, now)

        if self.machine.consume_changed():
            self._publish_state()

    # ------------------------------------------------------------ dance side

    def _reconcile_dance(self, want: bool, now: float) -> None:
        if want and not self.runner.running:
            self.get_logger().info('Dance: GO')
            self.runner.start(now)
        elif not want and self.runner.state in (DanceRunner.WAITING, DanceRunner.PLAYING):
            self.get_logger().info('Dance: stopping early')
            self.runner.abort(now)

        prev_index = self.runner.segment_index
        out = self.runner.step(now)
        if self.runner.segment_index != prev_index and self.runner.segment_index is not None:
            seg = self.runner.timeline.segments[self.runner.segment_index]
            self.get_logger().info(
                f"Dance [{seg['start']:.2f}s] {seg['move']} ({seg['type']})")

        if out.launch_audio:
            self._launch_audio()
        if out.stop_audio:
            self._stop_audio()
        if out.gait_request is not None:
            self._request_gait(out.gait_request)
        if out.twist is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = out.twist
            self.twist_pub.publish(t)
        if out.pose is not None:
            self._publish_pose(out.pose)
        self._stand_reassert = out.stand_reassert
        if out.final_stand:
            self._call_stand()
        if out.finished:
            self.get_logger().info('Dance: routine complete')
            self._dance_finished = True

    def _publish_pose(self, rpyh) -> None:
        roll, pitch, yaw, height = rpyh
        pose = Pose()
        pose.position.z = float(height)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = rpy_to_quaternion(roll, pitch, yaw)
        self.pose_pub.publish(pose)

    def _reassert_stand(self) -> None:
        # /body_pose is only applied when a stand is issued, so pose animation
        # needs stands streamed. Only during pose segments: a stand during
        # cmd_vel motion fights the walk.
        if self._stand_reassert:
            self._call_stand()

    def _call_stand(self) -> None:
        if self._stand_future is not None and not self._stand_future.done():
            return
        if not self.cli_stand.service_is_ready():
            self.get_logger().warn('stand service not ready', throttle_duration_sec=2.0)
            return
        self._stand_future = self.cli_stand.call_async(Trigger.Request())
        self._stand_future.add_done_callback(self._on_stand_done)

    def _on_stand_done(self, future) -> None:
        try:
            if not future.result().success:
                self.get_logger().warn('stand returned failure', throttle_duration_sec=2.0)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'stand errored: {e}', throttle_duration_sec=2.0)

    def _request_gait(self, mode: int) -> None:
        if mode == self._gait_requested:
            return
        if not self.cli_locomotion.service_is_ready():
            self.get_logger().warn(f'locomotion_mode not ready; gait {mode} skipped',
                                   throttle_duration_sec=2.0)
            return
        self._gait_requested = mode
        req = SetLocomotion.Request()
        req.locomotion_mode = mode
        self.cli_locomotion.call_async(req).add_done_callback(
            lambda f, m=mode: self._on_gait_done(f, m))

    def _on_gait_done(self, future, mode: int) -> None:
        try:
            result = future.result()
            ok = result is not None and result.success
        except Exception as e:  # noqa: BLE001
            ok, result = False, e
        if ok:
            self.get_logger().info(f'gait {mode} confirmed')
        else:
            self.get_logger().warn(f'gait {mode} failed: {result}')
            if self._gait_requested == mode:
                self._gait_requested = None   # allow a retry

    def _launch_audio(self) -> None:
        if not self.audio_file:
            return
        cmd = ['aplay']
        if self.audio_device:
            cmd += ['-D', self.audio_device]
        cmd.append(self.audio_file)
        try:
            self._audio_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"audio: {' '.join(cmd)}")
        except FileNotFoundError:
            self.get_logger().error("'aplay' not found (apt install alsa-utils)")

    def _stop_audio(self) -> None:
        proc, self._audio_proc = self._audio_proc, None
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGTERM)

    # -------------------------------------------------------------- dog side

    def _reconcile_dog(self, want_enabled: bool, profile: Optional[Mode], now: float) -> None:
        retry = float(self.get_parameter('dog_mode_retry_period').value)

        if not want_enabled:
            self._dog_profile_applied = None   # re-apply on the next enable
            if self._dog_enabled is True:
                self._send_dog_enable(False, now, retry)
            return

        # Profile first, then enable, so dog mode never runs a stale profile.
        if self._dog_profile_applied != profile:
            if self._dog_enabled is True:
                # Never change the profile under a live controller; the machine
                # only gets here after dog mode has confirmed it is disabled.
                self._send_dog_enable(False, now, retry)
                return
            self._send_dog_profile(profile, now, retry)
            return
        if self._dog_enabled is not True:
            self._send_dog_enable(True, now, retry)

    def _send_dog_enable(self, value: bool, now: float, retry: float) -> None:
        if self._dog_enable_future is not None and not self._dog_enable_future.done():
            return
        # Rate-limit repeats only; a disable followed by an enable goes at once.
        if value == self._dog_enable_sent_value and now - self._dog_enable_sent_at < retry:
            return
        if not self.cli_dog_enable.service_is_ready():
            self.get_logger().warn('/dog_mode/enable not available; is dog mode running?',
                                   throttle_duration_sec=5.0)
            return
        self._dog_enable_sent_at = now
        self._dog_enable_sent_value = value
        self._dog_enable_future = self.cli_dog_enable.call_async(SetBool.Request(data=value))

    def _send_dog_profile(self, profile: Mode, now: float, retry: float) -> None:
        if self._dog_params_future is not None and not self._dog_params_future.done():
            return
        if now - self._dog_params_sent_at < retry:
            return
        if not self.cli_dog_params.service_is_ready():
            self.get_logger().warn(f'{self.cli_dog_params.srv_name} not available',
                                   throttle_duration_sec=5.0)
            return
        follow = profile == Mode.FOLLOW
        turn = bool(self.get_parameter(
            'follow_allow_turn' if follow else 'stay_allow_turn').value)
        params = [
            Parameter('approach_enabled', value=follow),
            Parameter('base_turn_enabled', value=turn),
        ]
        req = SetParameters.Request(parameters=[x.to_parameter_msg() for x in params])
        self._dog_params_sent_at = now
        self._dog_params_future = self.cli_dog_params.call_async(req)
        self._dog_params_future.add_done_callback(
            lambda f, prof=profile: self._on_dog_profile_done(f, prof))

    def _on_dog_profile_done(self, future, profile: Mode) -> None:
        try:
            results = future.result().results
            ok = all(r.successful for r in results)
            why = '; '.join(r.reason for r in results if not r.successful)
        except Exception as e:  # noqa: BLE001
            ok, why = False, str(e)
        if ok:
            self._dog_profile_applied = profile
            self.get_logger().info(f'dog mode profile set: {profile.value}')
        else:
            self.get_logger().warn(f'dog mode profile {profile.value} rejected: {why}')

    # ---------------------------------------------------------------- state

    def _publish_state(self) -> None:
        m = self.machine
        msg = ModeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = m.mode.value
        msg.target_mode = m.target.value
        msg.phase = m.phase.value
        msg.reason = m.reason
        msg.last_rejection = m.last_rejection
        msg.deadman_held = self._deadman_held(time.monotonic())
        msg.standing = bool(self._standing)
        self.state_pub.publish(msg)

    def shutdown(self) -> None:
        """Leave Spot neutral and dog mode off. Guarded against a dead context."""
        self._stop_audio()
        if not rclpy.ok():
            return
        try:
            self.twist_pub.publish(Twist())
            if self.runner.running:
                self._publish_pose((0.0, 0.0, 0.0, 0.0))
                self._call_stand()
            if self._dog_enabled and self.cli_dog_enable.service_is_ready():
                future = self.cli_dog_enable.call_async(SetBool.Request(data=False))
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'shutdown cleanup failed: {e}')


def main(args=None):
    # Handle SIGINT ourselves: rclpy's default handler shuts the context down
    # before `finally` runs, so the cleanup in shutdown() could never publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = InteractiveModeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
