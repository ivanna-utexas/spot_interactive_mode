"""Eye renderer — large expressive circles on dark background.

Draws two large circular eyes that track gaze by shifting position.
Expressions are created through curved eyelid overlays: happy squint
when tracking, droopy lids when sleepy, widening on alert.
"""

import math
import random
import time

import pygame


# ── State machine ────────────────────────────────────────────────────────────
TRACKING = 'tracking'
IDLE = 'idle'
SLEEPY = 'sleepy'
ALERT = 'alert'


class EyeRenderer:
    """Astro-inspired eye renderer with gaze tracking and expressions."""

    BG_COLOR = (8, 10, 18)

    def __init__(
        self,
        width: int,
        height: int,
        smoothing_tau: float = 0.25,
        idle_timeout: float = 3.0,
        sleepy_timeout: float = 30.0,
        eye_color: str = '#F0F0F5',
    ):
        self.w = width
        self.h = height
        self.tau = smoothing_tau
        self.idle_timeout = idle_timeout
        self.sleepy_timeout = sleepy_timeout

        # Parse eye color
        ec = eye_color.lstrip('#')
        self.eye_color = (int(ec[0:2], 16), int(ec[2:4], 16), int(ec[4:6], 16))

        # ── Eye geometry ────────────────────────────────────────────────────
        self.eye_radius = int(min(width, height) * 0.23)
        gap = int(width * 0.035)
        cy = int(height * 0.48)
        self.left_rest = (width // 2 - gap - self.eye_radius, cy)
        self.right_rest = (width // 2 + gap + self.eye_radius, cy)

        # How far eyes shift when tracking
        self.max_offset_x = int(width * 0.12)
        self.max_offset_y = int(height * 0.08)

        # ── Animation state ─────────────────────────────────────────────────
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

        # Blink
        self.blink_t = 0.0  # 0=open, 1=closed
        self.blink_phase = 'none'  # 'none', 'closing', 'opening'
        self.blink_start = 0.0
        self.next_blink = time.monotonic() + random.uniform(3.0, 7.0)

        # State machine
        self.state = IDLE
        self.last_detected_time = 0.0
        self.alert_start = 0.0

        # Idle drift
        self.idle_target_x = 0.0
        self.idle_target_y = 0.0
        self.next_idle_move = 0.0

        # Expression
        self.eyelid_droop = 0.0   # 0=normal, positive=drooping
        self.happy_squint = 0.0   # lower lid rise for friendly look
        self.eye_scale = 1.0      # brief enlargement on alert

        # Pre-build glow surface
        self._build_glow()

    def _build_glow(self):
        """Pre-render soft glow halo behind each eye."""
        glow_pad = 25
        size = (self.eye_radius + glow_pad) * 2
        self.glow_surf = pygame.Surface((size, size), pygame.SRCALPHA)
        center = (size // 2, size // 2)

        for i in range(glow_pad, 0, -2):
            frac = i / glow_pad
            alpha = int(18 * (1.0 - frac))
            color = (*self.eye_color, alpha)
            pygame.draw.circle(self.glow_surf, color, center, self.eye_radius + i)

        self.glow_offset = size // 2

    # ── Update ───────────────────────────────────────────────────────────────

    def update(self, dt: float, gaze, force_idle: bool = False):
        """Update animation state. Call once per frame."""
        now = time.monotonic()

        detected = False
        if not force_idle and gaze is not None and gaze.detected:
            detected = True
            self.target_x = gaze.x
            self.target_y = gaze.y
            self.last_detected_time = now

        time_since = (
            now - self.last_detected_time
            if self.last_detected_time > 0 else float('inf')
        )

        # ── State transitions ───────────────────────────────────────────────
        if detected:
            if self.state in (IDLE, SLEEPY):
                self.state = ALERT
                self.alert_start = now
            elif self.state == ALERT and (now - self.alert_start) > 0.5:
                self.state = TRACKING
        else:
            if time_since > self.sleepy_timeout:
                self.state = SLEEPY
            elif time_since > self.idle_timeout:
                self.state = IDLE
            elif self.state == ALERT:
                self.state = IDLE

        # ── Smoothing ───────────────────────────────────────────────────────
        tau = self.tau * 3.0 if self.state == SLEEPY else self.tau
        alpha = 1.0 - math.exp(-dt / tau) if dt > 0 and tau > 0 else 0.0

        # ── Pupil target ────────────────────────────────────────────────────
        if self.state in (TRACKING, ALERT):
            pass  # target set from gaze
        elif self.state == IDLE:
            self._update_idle_drift(now)
            self.target_x = self.idle_target_x
            self.target_y = self.idle_target_y
        elif self.state == SLEEPY:
            self._update_idle_drift(now, slow=True)
            self.target_x = self.idle_target_x
            self.target_y = max(self.idle_target_y, 0.2)

        self.current_x += alpha * (self.target_x - self.current_x)
        self.current_y += alpha * (self.target_y - self.current_y)
        self.current_x = max(-1.0, min(1.0, self.current_x))
        self.current_y = max(-1.0, min(1.0, self.current_y))

        # ── Eyelid droop ────────────────────────────────────────────────────
        if self.state == SLEEPY:
            droop_target = 0.45
        elif self.state == ALERT:
            droop_target = -0.08
        else:
            droop_target = 0.0
        self.eyelid_droop += alpha * (droop_target - self.eyelid_droop)

        # ── Happy squint (friendly tracking expression) ─────────────────────
        happy_target = 0.18 if self.state == TRACKING else 0.0
        self.happy_squint += alpha * (happy_target - self.happy_squint)

        # ── Eye scale (alert surprise pulse) ────────────────────────────────
        scale_target = 1.08 if self.state == ALERT else 1.0
        self.eye_scale += alpha * (scale_target - self.eye_scale)

        # ── Blink ───────────────────────────────────────────────────────────
        self._update_blink(now)

    def _update_idle_drift(self, now: float, slow: bool = False):
        """Pick random gaze targets for idle animation."""
        if now >= self.next_idle_move:
            r = 0.3 if slow else 0.5
            self.idle_target_x = random.uniform(-r, r)
            self.idle_target_y = random.uniform(-0.15, 0.15)
            interval = random.uniform(4, 8) if slow else random.uniform(2, 4)
            self.next_idle_move = now + interval

    def _update_blink(self, now: float):
        """Drive blink animation."""
        if self.blink_phase == 'none':
            if now >= self.next_blink:
                self.blink_phase = 'closing'
                self.blink_start = now
        elif self.blink_phase == 'closing':
            t = min(1.0, (now - self.blink_start) / 0.12)
            self.blink_t = t * t  # ease-in
            if t >= 1.0:
                self.blink_phase = 'opening'
                self.blink_start = now
        elif self.blink_phase == 'opening':
            t = min(1.0, (now - self.blink_start) / 0.08)
            self.blink_t = 1.0 - t * t  # ease-out
            if t >= 1.0:
                self.blink_t = 0.0
                self.blink_phase = 'none'
                lo = 2.0 if self.state == SLEEPY else 3.0
                hi = 5.0 if self.state == SLEEPY else 7.0
                self.next_blink = now + random.uniform(lo, hi)

    # ── Draw ─────────────────────────────────────────────────────────────────

    def draw(self, screen: pygame.Surface):
        """Render the current frame."""
        screen.fill(self.BG_COLOR)

        ox = int(self.current_x * self.max_offset_x)
        oy = int(self.current_y * self.max_offset_y)

        # Subtle breathing micro-movement
        breath = math.sin(time.monotonic() * 0.8) * 2
        oy += int(breath)

        left = (self.left_rest[0] + ox, self.left_rest[1] + oy)
        right = (self.right_rest[0] + ox, self.right_rest[1] + oy)

        self._draw_eye(screen, left)
        self._draw_eye(screen, right)

    def _draw_eye(self, screen: pygame.Surface, center: tuple):
        """Draw a single eye: glow + circle + eyelid overlays."""
        cx, cy = center
        r = int(self.eye_radius * self.eye_scale)

        # Glow halo
        screen.blit(
            self.glow_surf,
            (cx - self.glow_offset, cy - self.glow_offset),
        )

        # Main eye circle
        pygame.draw.circle(screen, self.eye_color, (cx, cy), r)

        # Eyelid overlays
        self._draw_eyelids(screen, cx, cy, r)

    def _draw_eyelids(self, screen: pygame.Surface, cx: int, cy: int, r: int):
        """Draw upper and lower eyelids as background-colored curved polygons."""
        pad = r + 15       # horizontal extent beyond eye edge
        overshoot = 45     # vertical extent beyond eye to cover glow
        n = 24             # curve resolution

        # ── Upper eyelid ────────────────────────────────────────────────────
        upper = max(0.0, min(1.0, self.blink_t + self.eyelid_droop))

        if upper > 0.005:
            # lid_y: top of eye when open → bottom of eye when fully closed
            lid_y = cy - r + upper * 2 * r

            pts = [
                (cx - pad, cy - r - overshoot),   # top-left
                (cx + pad, cy - r - overshoot),   # top-right
            ]
            # Curve from right to left
            for i in range(n + 1):
                t = i / n
                x = cx + pad - t * 2 * pad
                # Sine bulge: lid curves down in center for natural shape
                bulge = math.sin(t * math.pi) * r * 0.12
                pts.append((int(x), int(lid_y + bulge)))

            pygame.draw.polygon(screen, self.BG_COLOR, pts)

        # ── Lower eyelid ────────────────────────────────────────────────────
        lower = max(0.0, min(1.0, self.blink_t * 0.4 + self.happy_squint))

        if lower > 0.005:
            lid_y = cy + r - lower * 2 * r

            pts = [
                (cx - pad, cy + r + overshoot),   # bottom-left
                (cx + pad, cy + r + overshoot),   # bottom-right
            ]
            # Curve from right to left
            for i in range(n + 1):
                t = i / n
                x = cx + pad - t * 2 * pad
                bulge = -math.sin(t * math.pi) * r * 0.12
                pts.append((int(x), int(lid_y + bulge)))

            pygame.draw.polygon(screen, self.BG_COLOR, pts)
