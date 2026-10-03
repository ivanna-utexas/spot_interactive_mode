"""PS4 mode bindings: parsing, edge detection, D-pad directions."""

import pytest

from spot_interactive_mode.joy_bindings import JoyInput, JoyModeBindings

DEFAULTS = {'stay': 'axis6+', 'follow': 'axis6-', 'dance': 'button2', 'idle': 'button8'}


def joy(pressed=(), dpad_lr=0.0):
    buttons = [0] * 13
    for i in pressed:
        buttons[i] = 1
    axes = [0.0] * 8
    axes[6] = dpad_lr
    return buttons, axes


def test_parse():
    assert JoyInput.parse('button2') == JoyInput(2)
    assert JoyInput.parse(' Axis6+ ') == JoyInput(6, 1)
    assert JoyInput.parse('axis7-') == JoyInput(7, -1)
    for bad in ('2', 'button', 'axis6', 'button-1', 'circle'):
        with pytest.raises(ValueError):
            JoyInput.parse(bad)


def test_labels():
    assert JoyModeBindings(DEFAULTS).describe() == \
        'D-pad left=stay, D-pad right=follow, Circle=dance, Share=idle'


def test_empty_spec_unbinds():
    b = JoyModeBindings({'stay': '', 'dance': 'button2'})
    assert [m for m, _ in b.inputs] == ['dance']


def test_duplicate_input_rejected():
    with pytest.raises(ValueError):
        JoyModeBindings({'stay': 'button2', 'dance': 'button2'})


def test_press_fires_once_while_held():
    b = JoyModeBindings(DEFAULTS)
    assert b.update(*joy()) == []
    assert b.update(*joy(pressed=[5, 2])) == ['dance']
    # joy_linux autorepeat: same held state must not fire again
    for _ in range(5):
        assert b.update(*joy(pressed=[5, 2])) == []
    assert b.update(*joy(pressed=[5])) == []
    assert b.update(*joy(pressed=[5, 2])) == ['dance']


def test_held_at_startup_is_not_a_press():
    b = JoyModeBindings(DEFAULTS)
    assert b.update(*joy(pressed=[8])) == []
    assert b.update(*joy(pressed=[8])) == []


def test_dpad_directions():
    b = JoyModeBindings(DEFAULTS)
    b.update(*joy())
    assert b.update(*joy(dpad_lr=1.0)) == ['stay']
    assert b.update(*joy(dpad_lr=1.0)) == []
    assert b.update(*joy(dpad_lr=-1.0)) == ['follow']
    assert b.update(*joy(dpad_lr=0.3)) == []      # under threshold


def test_short_message_reads_released():
    b = JoyModeBindings(DEFAULTS)
    b.update(*joy())
    assert b.update([0, 0], []) == []
    assert b.update(*joy(pressed=[8])) == ['idle']
