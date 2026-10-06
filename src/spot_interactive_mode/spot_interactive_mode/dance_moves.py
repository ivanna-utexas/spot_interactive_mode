"""Dance moves library for Spot.

Each move is a function that takes the elapsed time `t` (seconds) and returns
a tuple (roll, pitch, yaw, height):
  - roll   : tilt left/right   (radians, body)
  - pitch  : lean front/back   (radians, body)
  - yaw    : turn left/right   (radians, body)
  - height : body up/down      (meters, position.z)

dance_node looks moves up by name in the MOVES dict at the bottom.

Safety limits (don't exceed these in a move):
  roll/pitch/yaw : +-0.4 rad
  height         : +-0.3 m
Keep amplitudes a bit under the max for margin.
"""

import math

# --- shared limits (keep moves within these) ---
MAX_TILT = 0.4     # rad, for roll/pitch/yaw
MAX_HEIGHT = 0.3   # m

# Global tempo: scales the speed of every move at once. 1.0 = periods as
# written below, 2.0 = twice as fast (half the period), 0.5 = half speed.
# NOTE: faster moves = sharper body accelerations = harder for Spot to stay
# balanced. Raise this gradually and watch for wobble.
TEMPO = 2.0


def _osc(t, period, amplitude, phase=0.0):
    """A simple sine oscillation helper.
    period: seconds per full cycle. amplitude: peak value. phase: radians offset.
    The effective period is shortened by the global TEMPO multiplier."""
    return amplitude * math.sin(2.0 * math.pi * t * TEMPO / period + phase)


# ------------------------------------------------------------------ #
# Waveform shaping helpers (for the punchier / more dramatic moves)
# ------------------------------------------------------------------ #

def _sharpen(x, sharpness):
    """Reshape a value in [-1, 1] so it snaps toward the extremes and holds
    there, instead of smoothly gliding through zero like a plain sine.
    sharpness = 1.0 leaves the shape unchanged; sharpness < 1 makes it
    snappier (spends more time near +-1, less time transitioning)."""
    if x == 0.0:
        return 0.0
    return math.copysign(abs(x) ** sharpness, x)


def _osc_sharp(t, period, amplitude, phase=0.0, sharpness=1.0):
    """Like _osc, but shaped by _sharpen -- a 'hit and hold' feel instead of
    a smooth continuous glide. Use sharpness ~0.3-0.6 for a punchy snap."""
    raw = math.sin(2.0 * math.pi * t * TEMPO / period + phase)
    return amplitude * _sharpen(raw, sharpness)


def _pulse(t, period, phase=0.0, sharpness=0.3):
    """A one-sided spike: 0 for half the cycle, a sharp spike 0->1->0 for
    the other half. Good for discrete taps/hits rather than continuous
    oscillation. Returns a value in [0, 1]."""
    raw = math.sin(2.0 * math.pi * t * TEMPO / period + phase)
    return max(0.0, raw) ** sharpness


def _settle(t, tau, target):
    """Exponential approach toward `target`, starting at 0 when t=0.
    Reaches ~95% of target after ~3*tau seconds and stays there. Used for
    moves that sink into a pose and hold it rather than oscillating
    forever. `t` should be time since the move started."""
    return target * (1.0 - math.exp(-t / tau))


# ------------------------------------------------------------------ #
# Single-axis moves
# ------------------------------------------------------------------ #

# Each "single-axis" move keeps one dominant axis but couples in a small
# second axis a quarter cycle (phase pi/2) out of phase, on the SAME period.
# That secondary axis is moving fastest exactly when the primary one is
# turning around, so the body never fully stops -- it traces a continuous,
# flowing path (the trick that makes Boston Dynamics' dances look fluid)
# instead of stalling at each extreme. The secondary amplitude is small so
# the move still reads as "sway", "nod", etc.
QUAD = math.pi / 2.0   # quarter-cycle phase offset


def sway(t):
    """Tilt side to side (roll), with a subtle bob so it flows."""
    roll = _osc(t, period=3.0, amplitude=0.2)
    height = _osc(t, period=3.0, amplitude=0.05, phase=QUAD)
    return (roll, 0.0, 0.0, height)


def nod(t):
    """Lean front and back (pitch) -- like nodding, with a flowing bob."""
    pitch = _osc(t, period=2.0, amplitude=0.2)
    height = _osc(t, period=2.0, amplitude=0.05, phase=QUAD)
    return (0.0, pitch, 0.0, height)


def twist(t):
    """Turn body left and right (yaw) -- pure side-to-side, no middle pose.

    Deliberately single-axis: a sine moves fastest through center and slows
    at the extremes, so this reads as a clean left -> right -> left swing. No
    quadrature coupling here, because a second axis would put a distinct
    leaned pose at the center (the "middle move" we don't want)."""
    yaw = _osc(t, period=3.0, amplitude=0.2)
    return (0.0, 0.0, yaw, 0.0)


def bounce(t):
    """Bob up and down (height), with a subtle lean so it flows."""
    height = _osc(t, period=1.5, amplitude=0.1)
    pitch = _osc(t, period=1.5, amplitude=0.05, phase=QUAD)
    return (0.0, pitch, 0.0, height)


# ------------------------------------------------------------------ #
# Combo moves
# ------------------------------------------------------------------ #

def sway_bounce(t):
    """Sway side to side while bobbing up and down."""
    roll = _osc(t, period=3.0, amplitude=0.2)
    height = _osc(t, period=1.5, amplitude=0.08)
    return (roll, 0.0, 0.0, height)


def circle(t):
    """Roll and pitch 90 degrees out of phase -> body circles around."""
    roll = _osc(t, period=3.0, amplitude=0.18)
    pitch = _osc(t, period=3.0, amplitude=0.18, phase=math.pi / 2.0)
    return (roll, pitch, 0.0, 0.0)


def groove(t):
    """A bigger combo: sway + a slower twist + a gentle bob.

    roll and height share a period, so offset height by a quarter cycle
    (phase pi/2). That way height is at full speed when roll is turning
    around, instead of both axes stopping together -- keeps the body flowing
    rather than pausing at each extreme."""
    roll = _osc(t, period=2.0, amplitude=0.18)
    yaw = _osc(t, period=4.0, amplitude=0.12)
    height = _osc(t, period=2.0, amplitude=0.06, phase=math.pi / 2.0)
    return (roll, 0.0, yaw, height)


# ------------------------------------------------------------------ #
# Dancing Queen routine moves -- bigger amplitude, sharper transitions.
# These push closer to MAX_TILT/MAX_HEIGHT than the moves above on
# purpose (that's what makes them read as dramatic on camera). Test each
# one individually at a lower TEMPO before running the full sequence.
# ------------------------------------------------------------------ #

def side_step(t):
    """Big, punchy weight-shift side to side -- approximates stepping in
    place using body pose only (no real lateral translation -- see note
    below). Sharp snap to each side with a hold, plus a quick downward
    dip each time the body passes center (weight 'landing')."""
    roll = _osc_sharp(t, period=1.2, amplitude=0.34, sharpness=0.45)
    height = -abs(_osc_sharp(t, period=1.2, amplitude=0.12,
                              phase=QUAD, sharpness=0.45))
    return (roll, 0.0, 0.0, height)


def circle_sway(t):
    """A bigger, faster circle than `circle` -- roll and pitch sweep a wide
    circular path, with a quick bob layered on top for extra flourish."""
    roll = _osc(t, period=2.0, amplitude=0.32)
    pitch = _osc(t, period=2.0, amplitude=0.32, phase=QUAD)
    height = _osc(t, period=1.0, amplitude=0.08)
    return (roll, pitch, 0.0, height)


def turn_around_sway(t):
    """Wide yaw sweep (reads like spinning/turning around) combined with a
    roll sway underneath -- big enough to read as a "turn," not a wiggle."""
    yaw = _osc_sharp(t, period=3.5, amplitude=0.35, sharpness=0.7)
    roll = _osc(t, period=1.75, amplitude=0.15)
    return (roll, 0.0, yaw, 0.0)


def foot_tap(t):
    """Quick, sharp downward taps with a flat (level) pose in between --
    reads as discrete taps to the beat rather than a continuous bob.
    Approximates footwork via body pose only -- see locomotion note below."""
    pulse = _pulse(t, period=0.6, sharpness=0.3)
    height = -0.14 * pulse
    pitch = 0.06 * pulse  # slight forward nod on each tap, adds punch
    return (0.0, pitch, 0.0, height)


def foot_shuffle(t):
    """Fast alternating shuffle: quick left-right weight shifts layered
    with double-time height taps. Faster/busier than foot_tap -- meant for
    the "dancing queen" hero moment. Approximates footwork via body pose
    only -- see locomotion note below."""
    roll = _osc_sharp(t, period=0.5, amplitude=0.22, sharpness=0.5)
    pulse = _pulse(t, period=0.25, phase=QUAD, sharpness=0.3)
    height = -0.12 * pulse
    return (roll, 0.0, 0.0, height)


def twist_down(t):
    """Twist side to side while sinking into a lower stance and holding it
    -- a strong closing move. Settles low within ~1.5s (independent of
    total move duration) then keeps twisting on top of the lowered pose."""
    yaw = _osc_sharp(t, period=1.0, amplitude=0.3, sharpness=0.6)
    height = -_settle(t, tau=0.5, target=0.22)
    return (0.0, 0.0, yaw, height)


def twerk(t):
    """Quick, animated left-right sway with a coupled yaw wobble and a
    STATIC (non-bobbing) shallow crouch. Pairs with turn_180 as a two-part
    'turn around and twerk' combo. NOTE: sign of `pitch` for 'backward
    lean' is a first guess -- flip BACKWARD_LEAN's sign if it visually
    leans forward instead."""
    LEAN_BIAS = 0.15  # base backward pitch bias
    roll = _osc_sharp(t, period=0.25, amplitude=0.34, sharpness=0.35)  # bigger,
                           # faster, sharper sway than before -- the main motion
    yaw = _osc_sharp(t, period=0.25, amplitude=0.12, phase=QUAD, sharpness=0.4)
                           # coupled twist, quarter-cycle offset from roll --
                           # adds character/animation without adding bob
    height = -0.05         # STATIC crouch bias, no oscillation term at all --
                           # this is what actually kills the residual bobbing
    pitch = LEAN_BIAS
    return (roll, pitch, yaw, height)


# ------------------------------------------------------------------ #
# Sultans of Swing routine moves (Dire Straits, outro solo, ~148.5 BPM).
# Unlike the moves above these are locked to the song's beat and NOT scaled
# by TEMPO: every segment in sultans_of_swing.json starts on a detected
# downbeat, so a move whose period is a whole number of beats lands its
# hits on the beat. Retune SOS_BEAT if the clip is ever re-cut.
# Pitch sign: positive pitch = nose down (right-hand rule about +y).
# ------------------------------------------------------------------ #

SOS_BEAT = 60.0 / 148.5   # s per beat, measured from the clip (28 bars / 45.256 s)


def _beat_cos(t, beats, phase=0.0):
    """cos wave with a period of `beats` beats: +1 exactly on beat 1 (t=0)."""
    return math.cos(2.0 * math.pi * t / (beats * SOS_BEAT) + phase)


def _on_beat(t, beats=1.0, sharpness=0.6):
    """A one-sided hit that peaks exactly on every `beats`-th beat and is 0
    for the off half of the cycle. sharpness < 1 widens the hit (punchier),
    > 1 narrows it to a quick flick."""
    return max(0.0, _beat_cos(t, beats)) ** sharpness


def _smoothstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def head_bob(t):
    """Laid-back cool: nose dips on every beat, the body rocks left on beat 1
    and right on beat 3, and the gaze drifts slowly across the crowd."""
    hit = _on_beat(t, 1.0, sharpness=0.6)
    pitch = 0.10 * hit
    height = -0.04 * hit
    roll = 0.12 * _beat_cos(t, 4.0)
    yaw = 0.08 * math.sin(2.0 * math.pi * t / (8.0 * SOS_BEAT))
    return (roll, pitch, yaw, height)


def air_guitar(t):
    """Hunched over an imaginary Strat: leans hard into one side, nose down,
    'strums' a quick dip on every 8th note and tosses its head up on beat 1
    of each bar. Swaps sides after two bars, like a guitarist turning to the
    drummer and back."""
    side = _sharpen(math.sin(2.0 * math.pi * t / (16.0 * SOS_BEAT)), 0.3)
    roll = 0.18 * side
    yaw = 0.10 * side
    strum = _on_beat(t, 0.5, sharpness=0.4)
    toss = _on_beat(t, 4.0, sharpness=3.0)
    pitch = _settle(t, tau=0.3, target=0.08) - 0.14 * toss
    height = -_settle(t, tau=0.3, target=0.06) - 0.04 * strum
    return (roll, pitch, yaw, height)


def sultan_sway(t):
    """The swing in 'Sultans of Swing': a wide roll across the bar with the
    yaw countering it (hips against shoulders), a quarter-cycle pitch so the
    body draws a loop instead of stopping at each side, and a bounce on
    every beat."""
    roll = 0.22 * _beat_cos(t, 4.0)
    yaw = -0.14 * _beat_cos(t, 4.0)
    pitch = 0.08 * _beat_cos(t, 4.0, phase=-QUAD)
    height = -0.05 * _on_beat(t, 1.0, sharpness=0.8)
    return (roll, pitch, yaw, height)


def arpeggio_ripple(t):
    """The triplet arpeggio flurry: a body circle every two beats, growing
    over four bars, with a shimmer of three small height pulses per beat on
    top (one per note of the triplet)."""
    grow = 0.55 + 0.45 * _smoothstep(t / (16.0 * SOS_BEAT))
    roll = 0.22 * grow * _beat_cos(t, 2.0)
    pitch = 0.22 * grow * _beat_cos(t, 2.0, phase=-QUAD)
    height = -0.03 - 0.03 * _on_beat(t, 1.0 / 3.0, sharpness=0.5)
    return (roll, pitch, 0.0, height)


def take_a_bow(t):
    """Finale: two big 'da-DUM' nods per bar for two bars, then sinks into a
    deep bow and holds it while the song fades out. The routine's end ramp
    brings Spot back up from the bow."""
    nods = 0.20 * _on_beat(t, 2.0, sharpness=1.0)
    nod_roll = 0.10 * _beat_cos(t, 4.0)
    bow = _smoothstep((t - 8.0 * SOS_BEAT) / (2.0 * SOS_BEAT))
    pitch = (1.0 - bow) * nods + bow * 0.30
    roll = (1.0 - bow) * nod_roll
    height = -0.03 * (1.0 - bow) - 0.18 * bow
    return (roll, pitch, 0.0, height)

# ------------------------------------------------------------------ #
# Registry
# ------------------------------------------------------------------ #

MOVES = {
    'sway': sway,
    'nod': nod,
    'twist': twist,
    'bounce': bounce,
    'sway_bounce': sway_bounce,
    'circle': circle,
    'groove': groove,
    'side_step': side_step,
    'circle_sway': circle_sway,
    'turn_around_sway': turn_around_sway,
    'foot_tap': foot_tap,
    'foot_shuffle': foot_shuffle,
    'twist_down': twist_down,
    'twerk': twerk,
    # Sultans of Swing
    'head_bob': head_bob,
    'air_guitar': air_guitar,
    'sultan_sway': sultan_sway,
    'arpeggio_ripple': arpeggio_ripple,
    'take_a_bow': take_a_bow,
}