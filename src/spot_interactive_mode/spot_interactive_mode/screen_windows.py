"""Windows on the Spot-mounted screen (X display :0): full screen and switching.

No ROS. Talks to X through ctypes (libX11 + libXtst) because the image has
neither xdotool nor python-xlib.

  ros2 run spot_interactive_mode x_fullscreen <title substring> [timeout_s]

rviz2 can't start full screen from its config or command line, but its F11
(View > Fullscreen) also hides the menu bar and toolbar, which is what the
small screen wants. `x_fullscreen` waits for the window, activates it,
presses F11 through XTest, and checks the window manager reports it full
screen (F11 toggles, so it only presses again if the press didn't take).

mode_keyboard's `v` uses `toggle_view` to flip the screen between the eyes
and the pedestrian-tracking rviz: both stay running full screen, and
switching only raises the other window, so it is instant.
"""

import ctypes
import sys
import time
from ctypes import POINTER, byref, c_char_p, c_int, c_long, c_ubyte, c_uint, c_ulong, c_void_p
from typing import List, Optional

EYES_TITLE = 'Spot Eyes'                  # eye_display_node's pygame caption
TRACKING_TITLE = 'mounted_display.rviz'   # rviz puts the config name in its title
VIEWS = {'eyes': EYES_TITLE, 'tracking': TRACKING_TITLE}

Window = Atom = c_ulong
_CLIENT_MESSAGE = 33
_SUBSTRUCTURE_MASK = (1 << 19) | (1 << 20)   # SubstructureNotify | SubstructureRedirect
_XA_ATOM = 4
_XA_WINDOW = 33


class _XClientMessageEvent(ctypes.Structure):
    _fields_ = [('type', c_int), ('serial', c_ulong), ('send_event', c_int),
                ('display', c_void_p), ('window', Window), ('message_type', Atom),
                ('format', c_int), ('data', c_long * 5)]


class _XEvent(ctypes.Union):
    _fields_ = [('xclient', _XClientMessageEvent), ('pad', c_long * 24)]


def _load():
    x11 = ctypes.CDLL('libX11.so.6')
    xtst = ctypes.CDLL('libXtst.so.6')
    x11.XOpenDisplay.restype = c_void_p
    x11.XOpenDisplay.argtypes = [c_char_p]
    x11.XCloseDisplay.argtypes = [c_void_p]
    x11.XDefaultRootWindow.restype = Window
    x11.XDefaultRootWindow.argtypes = [c_void_p]
    x11.XInternAtom.restype = Atom
    x11.XInternAtom.argtypes = [c_void_p, c_char_p, c_int]
    x11.XQueryTree.argtypes = [c_void_p, Window, POINTER(Window), POINTER(Window),
                               POINTER(POINTER(Window)), POINTER(c_uint)]
    x11.XGetWindowProperty.argtypes = [c_void_p, Window, Atom, c_long, c_long, c_int, Atom,
                                       POINTER(Atom), POINTER(c_int), POINTER(c_ulong),
                                       POINTER(c_ulong), POINTER(POINTER(c_ubyte))]
    x11.XFree.argtypes = [c_void_p]
    x11.XSendEvent.argtypes = [c_void_p, Window, c_int, c_long, c_void_p]
    x11.XStringToKeysym.restype = c_ulong
    x11.XStringToKeysym.argtypes = [c_char_p]
    x11.XKeysymToKeycode.restype = c_ubyte
    x11.XKeysymToKeycode.argtypes = [c_void_p, c_ulong]
    x11.XFlush.argtypes = [c_void_p]
    xtst.XTestFakeKeyEvent.argtypes = [c_void_p, c_uint, c_int, c_ulong]
    return x11, xtst


class Screen:
    """One connection to an X display."""

    def __init__(self, display: Optional[str] = None):
        self.x11, self.xtst = _load()
        self.d = self.x11.XOpenDisplay(display.encode() if display else None)
        if not self.d:
            raise RuntimeError(f'cannot open X display {display or "$DISPLAY"}')
        self.root = self.x11.XDefaultRootWindow(self.d)
        atom = lambda name: self.x11.XInternAtom(self.d, name, 0)  # noqa: E731
        self._name, self._utf8 = atom(b'_NET_WM_NAME'), atom(b'UTF8_STRING')
        self._state, self._fullscreen = atom(b'_NET_WM_STATE'), atom(b'_NET_WM_STATE_FULLSCREEN')
        self._active = atom(b'_NET_ACTIVE_WINDOW')

    def close(self) -> None:
        if self.d:
            self.x11.XCloseDisplay(self.d)
            self.d = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---------------------------------------------------------------- query

    def _property(self, w: int, atom: int, kind: int) -> bytes:
        actual, fmt = Atom(), c_int()
        n, after, data = c_ulong(), c_ulong(), POINTER(c_ubyte)()
        if self.x11.XGetWindowProperty(self.d, w, atom, 0, 1024, 0, kind, byref(actual),
                                       byref(fmt), byref(n), byref(after), byref(data)) != 0 \
                or not data:
            return b''
        size = n.value * {8: 1, 16: 2, 32: ctypes.sizeof(c_long)}.get(fmt.value, 0)
        raw = ctypes.string_at(data, size)
        self.x11.XFree(data)
        return raw

    def _ids(self, raw: bytes) -> List[int]:
        step = ctypes.sizeof(c_long)
        return [int.from_bytes(raw[i:i + step], sys.byteorder) for i in range(0, len(raw), step)]

    def _windows(self, w: int):
        yield w
        root, parent = Window(), Window()
        children, n = POINTER(Window)(), c_uint()
        if not self.x11.XQueryTree(self.d, w, byref(root), byref(parent),
                                   byref(children), byref(n)):
            return
        kids = [children[i] for i in range(n.value)]
        if children:
            self.x11.XFree(children)
        for k in kids:
            yield from self._windows(k)

    def find(self, title: str) -> Optional[int]:
        """First window whose title contains `title`."""
        needle = title.encode()
        for w in self._windows(self.root):
            if needle in self._property(w, self._name, self._utf8):
                return w
        return None

    def active(self) -> Optional[int]:
        ids = self._ids(self._property(self.root, self._active, _XA_WINDOW))
        return ids[0] if ids and ids[0] else None

    def is_fullscreen(self, w: int) -> bool:
        return self._fullscreen in self._ids(self._property(w, self._state, _XA_ATOM))

    # --------------------------------------------------------------- action

    def activate(self, w: int) -> None:
        """Raise and focus `w` (un-minimizing it) through the window manager.

        Source 2 ("pager") is honored by mutter's focus-stealing prevention.
        """
        ev = _XEvent()
        ev.xclient.type = _CLIENT_MESSAGE
        ev.xclient.window = w
        ev.xclient.message_type = self._active
        ev.xclient.format = 32
        ev.xclient.data[0] = 2
        self.x11.XSendEvent(self.d, self.root, 0, _SUBSTRUCTURE_MASK, byref(ev))
        self.x11.XFlush(self.d)

    def press(self, keysym: str) -> None:
        code = self.x11.XKeysymToKeycode(self.d, self.x11.XStringToKeysym(keysym.encode()))
        self.xtst.XTestFakeKeyEvent(self.d, code, 1, 0)
        self.xtst.XTestFakeKeyEvent(self.d, code, 0, 0)
        self.x11.XFlush(self.d)


def next_view(active: Optional[int], windows: dict) -> str:
    """Which view `v` switches to: tracking unless tracking is what's showing."""
    if active is not None and active == windows.get('tracking'):
        return 'eyes'
    return 'tracking'


def toggle_view(display: str = ':0') -> str:
    """Flip the screen between the eyes and tracking; returns a status line."""
    with Screen(display) as s:
        windows = {view: s.find(title) for view, title in VIEWS.items()}
        want = next_view(s.active(), windows)
        win = windows[want]
        if win is None:
            pane = 'screen' if want == 'tracking' else 'eyes'
            return f'{want} is not running on {display} (check the {pane} pane)'
        s.activate(win)
        return f'screen {display}: {want}'


def make_fullscreen(title: str, timeout: float = 90.0, display: Optional[str] = None) -> bool:
    with Screen(display) as s:
        deadline = time.monotonic() + timeout
        win = None
        while win is None and time.monotonic() < deadline:
            win = s.find(title)
            if win is None:
                time.sleep(1.0)
        if win is None:
            print(f'x_fullscreen: no window titled *{title}* within {timeout:.0f} s')
            return False
        time.sleep(2.0)   # let it finish loading and map
        for _ in range(3):
            if s.is_fullscreen(win):
                break
            s.activate(win)
            time.sleep(0.5)
            s.press('F11')
            time.sleep(1.5)
        ok = s.is_fullscreen(win)
        print(f'x_fullscreen: *{title}* ' + ('is full screen' if ok else 'did not go full screen'))
        return ok


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit('usage: x_fullscreen <title substring> [timeout_s]   (uses $DISPLAY)')
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    sys.exit(0 if make_fullscreen(sys.argv[1], timeout) else 1)


if __name__ == '__main__':
    main()
