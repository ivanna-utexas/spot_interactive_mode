"""PS4 mode switching: which controller input requests which mode.

No ROS, so it runs under plain pytest. A binding is written as
'button<N>' (pressed while buttons[N] is 1) or 'axis<N>+' / 'axis<N>-'
(pressed while axes[N] is past +/-0.5, e.g. the D-pad). Indices follow the
joy_linux PS4 layout that teleop.launch.py starts:

  buttons: 0 Square  1 Cross  2 Circle  3 Triangle  4 L1  5 R1  6 L2  7 R2
           8 Share   9 Options  10 PS  11 L3  12 R3
  axes:    6 D-pad left(+) / right(-)   7 D-pad up(+) / down(-)

A mode is requested once per press (rising edge), never while held, so
joy_linux's 50 Hz autorepeat can't fire it again.
"""

import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

AXIS_THRESHOLD = 0.5
_SPEC = re.compile(r'^(?:button(\d+)|axis(\d+)([+-]))$')
_BUTTON_NAMES = ('Square', 'Cross', 'Circle', 'Triangle', 'L1', 'R1', 'L2', 'R2',
                 'Share', 'Options', 'PS', 'L3', 'R3')
_AXIS_NAMES = {(6, 1): 'D-pad left', (6, -1): 'D-pad right',
               (7, 1): 'D-pad up', (7, -1): 'D-pad down'}


@dataclass(frozen=True)
class JoyInput:
    index: int
    axis_sign: int = 0    # 0 for a button, +1 / -1 for one direction of an axis

    @classmethod
    def parse(cls, spec: str) -> 'JoyInput':
        m = _SPEC.match(spec.strip().lower())
        if m is None:
            raise ValueError(
                f"bad controller binding {spec!r}: use 'button<N>', 'axis<N>+' or 'axis<N>-'")
        if m.group(1) is not None:
            return cls(int(m.group(1)))
        return cls(int(m.group(2)), 1 if m.group(3) == '+' else -1)

    def label(self) -> str:
        if self.axis_sign == 0:
            if self.index < len(_BUTTON_NAMES):
                return _BUTTON_NAMES[self.index]
            return f'button {self.index}'
        sign = '+' if self.axis_sign > 0 else '-'
        return _AXIS_NAMES.get((self.index, self.axis_sign), f'axis {self.index}{sign}')

    def active(self, buttons: Sequence[int], axes: Sequence[float]) -> bool:
        if self.axis_sign == 0:
            return self.index < len(buttons) and bool(buttons[self.index])
        return self.index < len(axes) and self.axis_sign * axes[self.index] > AXIS_THRESHOLD


class JoyModeBindings:
    """Turns successive /joy messages into mode requests."""

    def __init__(self, bindings: Dict[str, str]):
        """bindings: mode name -> spec. An empty spec leaves that mode unbound."""
        self.inputs = [(mode, JoyInput.parse(spec)) for mode, spec in bindings.items() if spec]
        seen = {}
        for mode, inp in self.inputs:
            if inp in seen:
                raise ValueError(f'{mode!r} and {seen[inp]!r} are bound to the same input')
            seen[inp] = mode
        self._prev: Optional[List[bool]] = None

    def describe(self) -> str:
        return ', '.join(f'{inp.label()}={mode}' for mode, inp in self.inputs)

    def update(self, buttons: Sequence[int], axes: Sequence[float]) -> List[str]:
        """Modes whose input went from released to pressed in this message."""
        now = [inp.active(buttons, axes) for _, inp in self.inputs]
        prev, self._prev = self._prev, now
        if prev is None:
            # First message: an input already held at startup is not a press.
            return []
        return [mode for (mode, _), was, held in zip(self.inputs, prev, now) if held and not was]
