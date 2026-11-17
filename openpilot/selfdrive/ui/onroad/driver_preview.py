"""Pure Mochi screensaver character placed at the onroad DM slot (C3 and C4)."""
import math
import time

import pyray as rl

from openpilot.selfdrive.ui.dm_preview import preview_rect
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget


BLACK_BG = rl.Color(0, 0, 0, 255)
BORDER = rl.Color(255, 255, 255, 28)
WHITE = rl.Color(255, 255, 255, 255)
PUPIL = rl.Color(24, 24, 28, 255)
TONGUE = rl.Color(255, 130, 168, 255)
CHEEK = rl.Color(255, 145, 175, 130)
SPARKLE = rl.Color(255, 214, 64, 255)


class DriverPreview(Widget):
  def __init__(self, compact: bool = False):
    super().__init__()
    self.compact = compact
    self.set_enabled(False)
    self._closed = False
    self._start_time = time.monotonic()

  def close(self):
    self._closed = True

  def draw_onroad(self, parent: rl.Rectangle, alert_visible: bool) -> bool:
    visible = not self._closed and ui_state.started and not alert_visible
    if not visible:
      return False
    self.render(rl.Rectangle(*preview_rect(parent.x, parent.y, parent.width, parent.height, self.compact)))
    return True

  def _render(self, rect):
    now = time.monotonic() - self._start_time
    scale = min(rect.width / 120.0, rect.height / 120.0)

    # Pure black screensaver card
    rl.draw_rectangle_rounded(rect, 0.28, 16, BLACK_BG)
    rl.draw_rectangle_rounded_lines_ex(rect, 0.28, 16, max(1.0, 1.5 * scale), BORDER)

    cx = rect.x + rect.width / 2
    cy = rect.y + rect.height / 2

    # Cycle expressions every 5 seconds: 0=happy+sparkles, 1=normal+blink, 2=wink, 3=cozy
    phase = int(now // 5.0) % 4
    self._draw_mochi(cx, cy, scale, phase, now)

  @staticmethod
  def _draw_arch_eye(ex: float, ey: float, width: float, height: float, thick: float, color: rl.Color):
    steps = 10
    prev = None
    for i in range(steps + 1):
      t = i / steps
      px = ex - width / 2 + width * t
      py = ey - math.sin(t * math.pi) * height
      pt = rl.Vector2(px, py)
      if prev is not None:
        rl.draw_line_ex(prev, pt, thick, color)
      prev = pt
    rl.draw_circle_v(rl.Vector2(ex - width / 2, ey), thick * 0.5, color)
    rl.draw_circle_v(rl.Vector2(ex + width / 2, ey), thick * 0.5, color)

  @staticmethod
  def _draw_smile_mouth(cx: float, my: float, mw: float, mh: float, thick: float, color: rl.Color):
    steps = 10
    prev = None
    for i in range(steps + 1):
      t = i / steps
      px = cx - mw / 2 + mw * t
      py = my + math.sin(t * math.pi) * mh
      pt = rl.Vector2(px, py)
      if prev is not None:
        rl.draw_line_ex(prev, pt, thick, color)
      prev = pt

  @staticmethod
  def _draw_sparkle(sx: float, sy: float, size: float, alpha: int):
    c = rl.Color(SPARKLE.r, SPARKLE.g, SPARKLE.b, alpha)
    rl.draw_triangle(rl.Vector2(sx, sy - size), rl.Vector2(sx - size * 0.28, sy), rl.Vector2(sx + size * 0.28, sy), c)
    rl.draw_triangle(rl.Vector2(sx, sy + size), rl.Vector2(sx + size * 0.28, sy), rl.Vector2(sx - size * 0.28, sy), c)
    rl.draw_triangle(rl.Vector2(sx - size, sy), rl.Vector2(sx, sy + size * 0.28), rl.Vector2(sx, sy - size * 0.28), c)
    rl.draw_triangle(rl.Vector2(sx + size, sy), rl.Vector2(sx, sy - size * 0.28), rl.Vector2(sx, sy + size * 0.28), c)

  @classmethod
  def _draw_mochi(cls, cx: float, cy: float, scale: float, phase: int, now: float):
    eye_dx = 24 * scale
    eye_y = cy - 6 * scale
    thick = max(2.0, 5.2 * scale)
    bounce = math.sin(now * 2.8) * 1.8 * scale

    if phase == 0:
      # Happy ^ ^ + pink tongue + yellow sparkles
      for sign in (-1, 1):
        cls._draw_arch_eye(cx + sign * eye_dx, eye_y + bounce, 28 * scale, 14 * scale, thick, WHITE)
      my = cy + 12 * scale + bounce * 0.6
      cls._draw_smile_mouth(cx, my, 18 * scale, 11 * scale, max(1.8, 3.6 * scale), WHITE)
      rl.draw_circle_v(rl.Vector2(cx, my + 8 * scale), 4.8 * scale, TONGUE)
      a1 = int(165 + 90 * math.sin(now * 3.5))
      a2 = int(165 + 90 * math.cos(now * 2.9))
      cls._draw_sparkle(cx - 42 * scale, eye_y - 16 * scale - bounce, 6.5 * scale, a1)
      cls._draw_sparkle(cx + 42 * scale, eye_y - 12 * scale + bounce, 8.0 * scale, a2)

    elif phase == 1:
      # Cute round eyes with natural blink + subtle look-around
      blink_cycle = now % 3.2
      blink = 0.12 if 3.05 < blink_cycle < 3.18 else 1.0
      look_x = math.sin(now * 1.4) * 3.0 * scale
      for sign in (-1, 1):
        ex = cx + sign * eye_dx
        if blink < 0.3:
          rl.draw_line_ex(rl.Vector2(ex - 11 * scale, eye_y), rl.Vector2(ex + 11 * scale, eye_y), thick, WHITE)
        else:
          rl.draw_circle_v(rl.Vector2(ex, eye_y), 11 * scale, WHITE)
          rl.draw_circle_v(rl.Vector2(ex + look_x * 0.4, eye_y), 5.2 * scale, PUPIL)
          rl.draw_circle_v(rl.Vector2(ex + look_x * 0.4 + 2.2 * scale, eye_y - 2.5 * scale), 2.0 * scale, WHITE)
      cls._draw_smile_mouth(cx, cy + 14 * scale, 14 * scale, 6 * scale, max(1.8, 3.2 * scale), WHITE)

    elif phase == 2:
      # Playful wink (^ o) + rosy cheeks
      cls._draw_arch_eye(cx - eye_dx, eye_y + bounce * 0.5, 26 * scale, 13 * scale, thick, WHITE)
      rl.draw_circle_v(rl.Vector2(cx + eye_dx, eye_y), 10.5 * scale, WHITE)
      rl.draw_circle_v(rl.Vector2(cx + eye_dx, eye_y), 4.8 * scale, PUPIL)
      rl.draw_circle_v(rl.Vector2(cx + eye_dx + 2 * scale, eye_y - 2.5 * scale), 2.0 * scale, WHITE)
      rl.draw_circle_v(rl.Vector2(cx - eye_dx - 6 * scale, eye_y + 13 * scale), 5.5 * scale, CHEEK)
      rl.draw_circle_v(rl.Vector2(cx + eye_dx + 6 * scale, eye_y + 13 * scale), 5.5 * scale, CHEEK)
      cls._draw_smile_mouth(cx, cy + 13 * scale, 16 * scale, 9 * scale, max(1.8, 3.4 * scale), WHITE)

    else:
      # Cozy relaxed eyes (- -) + gentle smile
      for sign in (-1, 1):
        ex = cx + sign * eye_dx
        cls._draw_arch_eye(ex, eye_y + 3 * scale + bounce * 0.5, 24 * scale, 4 * scale, thick, WHITE)
      cls._draw_smile_mouth(cx, cy + 15 * scale, 13 * scale, 5 * scale, max(1.8, 3.0 * scale), WHITE)
