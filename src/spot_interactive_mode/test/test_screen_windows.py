"""Which view the keyboard's `v` switches the mounted screen to."""

from spot_interactive_mode.screen_windows import next_view


def test_flips_between_views():
    windows = {'eyes': 11, 'tracking': 22}
    assert next_view(22, windows) == 'eyes'
    assert next_view(11, windows) == 'tracking'


def test_defaults_to_tracking():
    windows = {'eyes': 11, 'tracking': 22}
    assert next_view(None, windows) == 'tracking'
    assert next_view(99, windows) == 'tracking', 'some other window (a dialog) is on top'
    assert next_view(None, {'eyes': 11, 'tracking': None}) == 'tracking'
