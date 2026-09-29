#!/usr/bin/env python3
"""Node B: Gaze topic → pygame eye renderer.

Subscribes to the gaze topic and renders animated cartoon eyes on
a fullscreen pygame display. ROS2 spin runs in a daemon thread;
the main thread owns the pygame event loop.
"""

import os
import signal
import threading
import time

import pygame
import rclpy
from rclpy.node import Node
from spot_eye_animation_msgs.msg import Gaze

from spot_eye_animation.renderer import EyeRenderer


class EyeDisplayNode(Node):
    def __init__(self):
        super().__init__('eye_display')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('display', ':0')
        self.declare_parameter('display_index', 0)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('target_fps', 30)
        self.declare_parameter('smoothing_tau', 0.25)
        self.declare_parameter('idle_timeout', 3.0)
        self.declare_parameter('sleepy_timeout', 30.0)
        self.declare_parameter('gaze_stale_timeout', 1.0)
        self.declare_parameter('eye_color', '#F0F0F5')
        self.declare_parameter('gaze_topic', 'eye_animation/gaze')

        # ── Gaze subscription ────────────────────────────────────────────
        self._latest_gaze = None
        self._last_gaze_time = 0.0

        gaze_topic = self.get_parameter('gaze_topic').value
        self.create_subscription(Gaze, gaze_topic, self._gaze_cb, 10)
        self.get_logger().info(f'Subscribing to: {gaze_topic}')

    def _gaze_cb(self, msg: Gaze):
        """Store latest gaze. Atomic reference swap."""
        self._latest_gaze = msg
        self._last_gaze_time = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = EyeDisplayNode()

    # ── Read parameters ──────────────────────────────────────────────────
    display_env = node.get_parameter('display').value
    display_index = node.get_parameter('display_index').value
    width = node.get_parameter('width').value
    height = node.get_parameter('height').value
    target_fps = node.get_parameter('target_fps').value
    smoothing_tau = node.get_parameter('smoothing_tau').value
    idle_timeout = node.get_parameter('idle_timeout').value
    sleepy_timeout = node.get_parameter('sleepy_timeout').value
    gaze_stale_timeout = node.get_parameter('gaze_stale_timeout').value
    eye_color = node.get_parameter('eye_color').value

    # ── Set DISPLAY before pygame init ───────────────────────────────────
    os.environ['DISPLAY'] = display_env
    os.environ['SDL_VIDEO_WINDOW_POS'] = '0,0'

    pygame.init()

    # ── Version + multi-display gate ─────────────────────────────────────
    sdl_ver = pygame.get_sdl_version()
    node.get_logger().info(
        f'pygame {pygame.version.ver}, SDL {".".join(str(v) for v in sdl_ver)}'
    )
    sdl_major = sdl_ver[0]
    use_display_index = sdl_major >= 2 and hasattr(pygame.display, 'get_num_displays')

    if use_display_index:
        num_displays = pygame.display.get_num_displays()
        node.get_logger().info(
            f'Detected {num_displays} display(s), requesting index {display_index}'
        )
    else:
        node.get_logger().warn(
            'SDL < 2 or get_num_displays unavailable — using primary display only'
        )

    # ── Auto-detect resolution ───────────────────────────────────────────
    if width <= 0 or height <= 0:
        info = pygame.display.Info()
        width = info.current_w
        height = info.current_h
        node.get_logger().info(f'Auto-detected resolution: {width}x{height}')

    # ── Create fullscreen display ────────────────────────────────────────
    flags = pygame.FULLSCREEN | pygame.NOFRAME
    try:
        screen = pygame.display.set_mode((width, height), flags, display=display_index)
    except TypeError:
        node.get_logger().warn(
            'display kwarg not supported, falling back to primary display'
        )
        screen = pygame.display.set_mode((width, height), flags)

    pygame.mouse.set_visible(False)
    pygame.display.set_caption('Spot Eyes')

    actual_w, actual_h = screen.get_size()
    node.get_logger().info(f'Actual display surface: {actual_w}x{actual_h}')
    if (actual_w, actual_h) != (width, height):
        node.get_logger().warn(
            f'Requested {width}x{height} but got {actual_w}x{actual_h} '
            '— check display index or set DP-1 as primary'
        )

    # ── Create renderer ──────────────────────────────────────────────────
    renderer = EyeRenderer(
        width=actual_w,
        height=actual_h,
        smoothing_tau=smoothing_tau,
        idle_timeout=idle_timeout,
        sleepy_timeout=sleepy_timeout,
        eye_color=eye_color,
    )

    # ── ROS2 spin in daemon thread ───────────────────────────────────────
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Signal handler for graceful shutdown ──────────────────────────────
    running = True

    def _signal_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # ── Main render loop ─────────────────────────────────────────────────
    clock = pygame.time.Clock()
    node.get_logger().info('Eye animation started')

    try:
        while running:
            dt = clock.tick(target_fps) / 1000.0

            # Event pump (required for long-running pygame apps)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            # Read latest gaze + check staleness
            gaze = node._latest_gaze
            stale = (time.monotonic() - node._last_gaze_time) > gaze_stale_timeout \
                if node._last_gaze_time > 0 else True

            renderer.update(dt, gaze, force_idle=stale)
            renderer.draw(screen)
            pygame.display.flip()
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.try_shutdown()
        node.get_logger().info('Eye animation stopped')


if __name__ == '__main__':
    main()
