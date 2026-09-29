#!/usr/bin/env python3
"""Keypress mode switching for lab tests (Milestone 1 triggers).

  s = stay   f = follow   d = dance   i = idle (interactive mode off)   q = quit

Sends operator-level SetMode requests, so there is no confidence threshold or
cooldown. Run it in its own terminal: it needs a real TTY.
"""

import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

from interactive_mode_msgs.msg import ModeState
from interactive_mode_msgs.srv import SetMode

KEYS = {'s': 'stay', 'f': 'follow', 'd': 'dance', 'i': 'idle'}


class ModeKeyboard(Node):
    def __init__(self):
        super().__init__('interactive_mode_keyboard')
        self.cli = self.create_client(SetMode, '/interactive_mode/set_mode')
        self.create_subscription(ModeState, '/interactive_mode/state', self._on_state, 10)
        self._last = None

    def send(self, mode: str) -> None:
        if not self.cli.service_is_ready():
            print('\r/interactive_mode/set_mode not available\r')
            return
        self.cli.call_async(SetMode.Request(mode=mode)).add_done_callback(
            lambda f: print(f'\r{mode}: {f.result().message}\r'))

    def _on_state(self, msg: ModeState) -> None:
        line = f'{msg.phase}: {msg.mode} -> {msg.target_mode} ({msg.reason})'
        if line != self._last:
            self._last = line
            print(f'\r[state] {line}\r')


def main(args=None):
    if not sys.stdin.isatty():
        print('mode_keyboard needs an interactive terminal')
        return
    rclpy.init(args=args)
    node = ModeKeyboard()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    print('s=stay  f=follow  d=dance  i=idle  q=quit')
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1).lower()
            if key == 'q':
                break
            if key in KEYS:
                node.send(KEYS[key])
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
