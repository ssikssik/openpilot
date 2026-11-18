"""Mochi full-screen C4 screensaver with 13 driving-state expressions."""
import math
import time

import pyray as rl

from openpilot.cereal import car, log
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import MouseEvent, gui_app
from openpilot.system.ui.widgets import Widget

LONG_PRESS_S = 0.65
PRESS_SLOP = 28.0


BLACK = rl.Color(0, 0, 0, 255)
WHITE = rl.Color(255, 255, 255, 255)
INK = rl.Color(20, 22, 26, 255)
YELLOW = rl.Color(255, 210, 63, 255)
AMBER = rl.Color(255, 176, 32, 255)
SWEAT_BLUE = rl.Color(127, 212, 255, 255)
ZZZ_BLUE = rl.Color(159, 216, 255, 255)
TONGUE_PINK = rl.Color(255, 159, 176, 255)
ENGAGE_GREEN = rl.Color(43, 217, 138, 255)
RED_LIGHT = rl.Color(255, 82, 82, 255)
TL_BODY = rl.Color(34, 36, 42, 255)
TL_DIM_Y = rl.Color(255, 201, 63, 72)
TL_DIM_G = rl.Color(43, 217, 138, 72)


class MochiScreensaver(Widget):
  def __init__(self):
    super().__init__()
    self.set_enabled(False)
    self._enabled_param = False
    self._frame = 0
    self._prev_engaged = False
    self._prev_standstill = False
    self._engage_until = 0.0
    self._disengage_until = 0.0
    self._launch_until = 0.0
    self._active = False
    self._press: MouseEvent | None = None
    self._press_start = 0.0
    self._start_time = time.monotonic()

  @property
  def gestures_enabled(self) -> bool:
    return self._enabled_param and ui_state.started

  def handle_mouse_event(self, event: MouseEvent) -> None:
    if not self.gestures_enabled:
      return
    if event.left_pressed:
      if self._active:
        self._active = False
      else:
        self._press = event
        self._press_start = time.monotonic()
    elif event.left_released or (self._press is not None and
        math.hypot(event.pos.x - self._press.pos.x, event.pos.y - self._press.pos.y) > PRESS_SLOP):
      self._press = None

  def update_gesture(self, alert_visible: bool) -> None:
    self._frame += 1
    if self._frame == 1 or self._frame % 30 == 0:
      self._enabled_param = ui_state.params.get_bool("MochiScreensaver")
    if not self.gestures_enabled:
      self._active = False
      self._press = None
    elif self._press is not None:
      if alert_visible or not gui_app.last_mouse_event.left_down:
        self._press = None
      elif time.monotonic() - self._press_start >= LONG_PRESS_S:
        self._active = True
        self._press = None

  def is_active(self, alert_visible: bool) -> bool:
    return self.gestures_enabled and self._active and not alert_visible

  def render_press_progress(self, rect: rl.Rectangle) -> None:
    if self._press is None:
      return
    progress = min(1.0, (time.monotonic() - self._press_start) / LONG_PRESS_S)
    radius = 70 * min(rect.width / 2160.0, rect.height / 1080.0)
    inner = radius * 0.87
    pos = rl.Vector2(self._press.pos.x, self._press.pos.y)
    rl.draw_ring(pos, inner, radius, 0.0, 360.0, 48, rl.Color(255, 255, 255, 60))
    if progress > 0.0:
      rl.draw_ring(pos, inner, radius, -90.0, -90.0 + 360.0 * progress, 48, ENGAGE_GREEN)

  def _detect_state(self, now: float) -> str:
    sm = ui_state.sm
    cs = sm['carState']
    ss = sm['selfdriveState']
    cc = sm['carControl']
    md = sm['modelV2']

    engaged = bool(ss.enabled)
    standstill = bool(cs.standstill)

    if engaged and not self._prev_engaged:
      self._engage_until = now + 2.5
    elif not engaged and self._prev_engaged:
      self._disengage_until = now + 3.5

    if self._prev_standstill and not standstill and cs.vEgo > 0.3:
      self._launch_until = now + 2.5

    self._prev_engaged = engaged
    self._prev_standstill = standstill

    gear = str(cs.gearShifter)
    if gear == 'reverse':
      return 'reverse'
    if gear == 'park':
      return 'disengage'

    # Emergency braking / FCW
    fcw = bool(cc.hudControl.visualAlert == car.CarControl.HUDControl.VisualAlert.fcw)
    if fcw or (cs.vEgo > 1.0 and (cs.aEgo < -3.2 or (engaged and cc.actuators.accel < -3.0))):
      return 'emergency'

    # Hard deceleration
    if cs.vEgo > 2.0 and (cs.aEgo < -1.5 or (engaged and cc.actuators.accel < -1.8)):
      return 'decel'

    # Hard acceleration
    if cs.vEgo > 2.0 and (cs.aEgo > 1.4 or (engaged and cc.actuators.accel > 1.4)):
      return 'accel'

    # Lane change
    lc_state = md.meta.laneChangeState
    if engaged and lc_state in (log.LaneChangeState.laneChangeStarting, log.LaneChangeState.laneChangeFinishing):
      if md.meta.laneChangeDirection == log.LaneChangeDirection.left:
        return 'laneLeft'
      if md.meta.laneChangeDirection == log.LaneChangeDirection.right:
        return 'laneRight'

    # Blinkers (single side only; ignore hazard lights)
    if cs.leftBlinker and not cs.rightBlinker:
      return 'blinkLeft'
    if cs.rightBlinker and not cs.leftBlinker:
      return 'blinkRight'

    # Launch from stop
    if now < self._launch_until and gear == 'drive':
      return 'start'

    # Engage / Disengage transition
    if now < self._engage_until and engaged:
      return 'engage'
    if now < self._disengage_until and not engaged:
      return 'disengage'

    # Waiting at red light / standstill (only when engaged in Drive)
    if engaged and gear == 'drive' and standstill:
      return 'waiting'

    return 'driving' if engaged else 'disengage'

  @staticmethod
  def _motion_for(state: str, t: float) -> tuple[float, float, float]:
    mx, my, ms = 0.0, 0.0, 1.0
    if state == 'decel':
      mx = math.sin(t * 20.0) * 11.0 + math.sin(t * 33.0) * 4.0
      my = math.sin(t * 20.0 + 1.0) * 3.0
      ms = 1.0 + 0.05 * math.sin(t * 6.0)
    elif state == 'emergency':
      mx = math.sin(t * 45.0) * 4.5 + math.sin(t * 29.0) * 3.0
      my = math.cos(t * 39.0) * 4.0
      ms = 1.0 + 0.02 * math.sin(t * 45.0)
    elif state == 'accel':
      my = -abs(math.sin(t * 5.2)) * 17.0
      mx = math.sin(t * 5.2) * 3.0
      ms = 1.0 + 0.05 * abs(math.sin(t * 5.2))
    elif state == 'laneRight':
      mx = 17.0 * math.sin(t * 2.1)
      my = 3.0 * math.sin(t * 4.2)
    elif state == 'laneLeft':
      mx = -17.0 * math.sin(t * 2.1)
      my = 3.0 * math.sin(t * 4.2)
    elif state in ('blinkRight', 'blinkLeft'):
      my = -abs(math.sin(t * 3.4)) * 13.0
    elif state == 'start':
      my = -abs(math.sin(t * 4.4)) * 21.0
      ms = 1.0 + 0.06 * abs(math.sin(t * 4.4))
    elif state == 'engage':
      my = 2.5 * math.sin(t * 2.6)
      ms = 1.0 + 0.045 * math.sin(t * 2.6)
    elif state == 'disengage':
      mx = 4.0 * math.sin(t * 0.6)
      my = 5.0 + 4.0 * math.sin(t * 0.9 + 1.0)
    elif state == 'driving':
      mx = 3.0 * math.sin(t * 1.1)
      my = 5.5 * math.sin(t * 1.8)
    elif state == 'reverse':
      mx = 10.0 + 8.0 * math.sin(t * 1.7)
      my = 3.0 * math.sin(t * 2.3)
    elif state == 'waiting':
      mx = 5.0 * math.sin(t * 0.8)
      my = 4.0 * math.sin(t * 1.5 + 0.7)
    return mx, my, ms

  def _render(self, rect: rl.Rectangle):
    now = time.monotonic() - self._start_time
    rl.draw_rectangle_rec(rect, BLACK)

    state = self._detect_state(now)
    mx, my, ms = self._motion_for(state, now)

    # Map 400x240 virtual viewBox centered inside rect (536x240 on C4) with whole-face motion
    base_scale = min(rect.width / 400.0, rect.height / 240.0)
    scale = base_scale * ms
    ox = rect.x + (rect.width - 400.0 * scale) / 2.0 + mx * base_scale
    oy = rect.y + (rect.height - 240.0 * scale) / 2.0 + my * base_scale

    self._draw_face(ox, oy, scale, state, now)

  @staticmethod
  def _pt(ox: float, oy: float, s: float, x: float, y: float) -> rl.Vector2:
    return rl.Vector2(ox + x * s, oy + y * s)

  @classmethod
  def _draw_quad(cls, ox: float, oy: float, s: float, p0, p1, p2, thick: float, color: rl.Color):
    steps = 12
    prev = None
    for i in range(steps + 1):
      t = i / steps
      u = 1.0 - t
      x = u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0]
      y = u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]
      pt = cls._pt(ox, oy, s, x, y)
      if prev is not None:
        rl.draw_line_ex(prev, pt, thick * s, color)
      prev = pt
    rl.draw_circle_v(cls._pt(ox, oy, s, p0[0], p0[1]), thick * 0.5 * s, color)
    rl.draw_circle_v(cls._pt(ox, oy, s, p2[0], p2[1]), thick * 0.5 * s, color)

  @classmethod
  def _draw_open_eye(cls, ox: float, oy: float, s: float, cx: float, pdx: float, pdy: float, blink: bool = False):
    if blink:
      rl.draw_line_ex(cls._pt(ox, oy, s, cx - 34, 96), cls._pt(ox, oy, s, cx + 34, 96), 10 * s, WHITE)
      return
    rl.draw_ellipse(int(ox + cx * s), int(oy + 96 * s), 38 * s, 46 * s, WHITE)
    rl.draw_circle_v(cls._pt(ox, oy, s, cx + pdx, 96 + pdy), 15 * s, INK)
    rl.draw_circle_v(cls._pt(ox, oy, s, cx + pdx + 5.5, 90 + pdy), 5 * s, WHITE)

  @classmethod
  def _draw_sparkle(cls, ox: float, oy: float, s: float, x: float, y: float, sz: float, alpha: int):
    c = rl.Color(YELLOW.r, YELLOW.g, YELLOW.b, alpha)
    rl.draw_triangle(cls._pt(ox, oy, s, x, y - sz), cls._pt(ox, oy, s, x - sz * 0.28, y), cls._pt(ox, oy, s, x + sz * 0.28, y), c)
    rl.draw_triangle(cls._pt(ox, oy, s, x, y + sz), cls._pt(ox, oy, s, x + sz * 0.28, y), cls._pt(ox, oy, s, x - sz * 0.28, y), c)
    rl.draw_triangle(cls._pt(ox, oy, s, x - sz, y), cls._pt(ox, oy, s, x, y + sz * 0.28), cls._pt(ox, oy, s, x, y - sz * 0.28), c)
    rl.draw_triangle(cls._pt(ox, oy, s, x + sz, y), cls._pt(ox, oy, s, x, y - sz * 0.28), cls._pt(ox, oy, s, x, y + sz * 0.28), c)

  @classmethod
  def _draw_star(cls, ox: float, oy: float, s: float, cx: float, cy: float, ro: float, ri: float, color: rl.Color):
    center = cls._pt(ox, oy, s, cx, cy)
    pts = []
    for i in range(10):
      r = ro if i % 2 == 0 else ri
      a = -math.pi / 2 + i * math.pi / 5
      pts.append(cls._pt(ox, oy, s, cx + r * math.cos(a), cy + r * math.sin(a)))
    for i in range(10):
      p1 = pts[i]
      p2 = pts[(i + 1) % 10]
      rl.draw_triangle(center, p2, p1, color)

  @classmethod
  def _draw_face(cls, ox: float, oy: float, s: float, state: str, now: float):
    blink_cycle = now % 3.4
    is_blink = 3.25 < blink_cycle < 3.38
    sway = math.sin(now * 2.0) * 2.0

    if state == 'decel':
      shake = math.sin(now * 26.0) * 2.0
      for cx in (138, 262):
        rl.draw_ellipse(int(ox + (cx + shake) * s), int(oy + 94 * s), 42 * s, 52 * s, WHITE)
        rl.draw_circle_v(cls._pt(ox, oy, s, cx + shake, 84), 16 * s, INK)
        rl.draw_circle_v(cls._pt(ox, oy, s, cx + shake + 6, 78), 5.5 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 96, 34), cls._pt(ox, oy, s, 86, 18), 6 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 304, 34), cls._pt(ox, oy, s, 314, 18), 6 * s, WHITE)
      rl.draw_circle_lines(int(ox + 200 * s), int(oy + 178 * s), 10 * s, WHITE)

    elif state == 'emergency':
      # Blue sweat drops (k-sweat: -10 -> +46, staggered 0.5s)
      for idx, sx in enumerate((58, 342)):
        ph = (now + idx * 0.5) % 1.0
        drop_y = 52 - 10 + 56 * ph
        alpha = int(255 * (ph / 0.22 if ph < 0.22 else (1.0 - ph) / 0.22 if ph > 0.78 else 1.0))
        c = rl.Color(SWEAT_BLUE.r, SWEAT_BLUE.g, SWEAT_BLUE.b, max(0, min(255, alpha)))
        rl.draw_circle_v(cls._pt(ox, oy, s, sx, drop_y + 7), 10 * s, c)
        rl.draw_triangle(cls._pt(ox, oy, s, sx, drop_y - 14), cls._pt(ox, oy, s, sx - 9, drop_y + 5), cls._pt(ox, oy, s, sx + 9, drop_y + 5), c)
      tremble_x = math.sin(now * 45.0) * 3.5 + math.sin(now * 29.0) * 2.5
      tremble_y = math.cos(now * 39.0) * 3.0
      # > < eyes
      rl.draw_line_ex(cls._pt(ox, oy, s, 170, 62), cls._pt(ox, oy, s, 110, 96), 13 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 110, 96), cls._pt(ox, oy, s, 170, 130), 13 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 230, 62), cls._pt(ox, oy, s, 290, 96), 13 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 290, 96), cls._pt(ox, oy, s, 230, 130), 13 * s, WHITE)
      # Wavy trembling mouth
      prev = None
      for i in range(17):
        x = 168 + i * 4 + tremble_x
        y = 178 + math.sin(i * 1.6) * 6 + tremble_y
        pt = cls._pt(ox, oy, s, x, y)
        if prev is not None:
          rl.draw_line_ex(prev, pt, 7 * s, WHITE)
        prev = pt

    elif state == 'accel':
      for i, y in enumerate((58, 96, 134)):
        ph = (now / 0.6 + i * 0.33) % 1.0
        dx = 24 - 60 * ph
        alpha = int(240 * (ph / 0.3 if ph < 0.3 else (1.0 - ph) / 0.7))
        rl.draw_line_ex(cls._pt(ox, oy, s, 18 + dx, y), cls._pt(ox, oy, s, 58 + dx, y), 6 * s, rl.Color(255, 255, 255, max(0, min(255, alpha))))
      star_scale = 1.16 + 0.16 * math.sin(now * math.pi * 2 / 0.9)
      cls._draw_star(ox, oy, s, 138, 96, 47 * star_scale, 21 * star_scale, YELLOW)
      cls._draw_star(ox, oy, s, 262, 96, 47 * star_scale, 21 * star_scale, YELLOW)
      cls._draw_quad(ox, oy, s, (177, 161), (200, 184), (223, 161), 7, WHITE)
      cls._draw_sparkle(ox, oy, s, 330, 150, 11 * (0.95 + 0.45 * math.sin(now * 5.2)), int(170 + 85 * math.sin(now * 5.2)))
      cls._draw_sparkle(ox, oy, s, 66, 168, 9 * (0.95 + 0.45 * math.cos(now * 5.2)), int(170 + 85 * math.cos(now * 5.2)))

    elif state == 'laneRight':
      cls._draw_open_eye(ox, oy, s, 138, 12, 0, is_blink)
      cls._draw_open_eye(ox, oy, s, 262, 12, 0, is_blink)
      cls._draw_quad(ox, oy, s, (179, 165), (200, 183), (221, 165), 7, WHITE)
      sx = math.sin(now * 5.0) * 6.0
      rl.draw_line_ex(cls._pt(ox, oy, s, 330 + sx, 74), cls._pt(ox, oy, s, 352 + sx, 96), 8 * s, AMBER)
      rl.draw_line_ex(cls._pt(ox, oy, s, 352 + sx, 96), cls._pt(ox, oy, s, 330 + sx, 118), 8 * s, AMBER)

    elif state == 'laneLeft':
      cls._draw_open_eye(ox, oy, s, 138, -12, 0, is_blink)
      cls._draw_open_eye(ox, oy, s, 262, -12, 0, is_blink)
      cls._draw_quad(ox, oy, s, (179, 165), (200, 183), (221, 165), 7, WHITE)
      sx = math.sin(now * 5.0) * 6.0
      rl.draw_line_ex(cls._pt(ox, oy, s, 70 - sx, 74), cls._pt(ox, oy, s, 48 - sx, 96), 8 * s, AMBER)
      rl.draw_line_ex(cls._pt(ox, oy, s, 48 - sx, 96), cls._pt(ox, oy, s, 70 - sx, 118), 8 * s, AMBER)

    elif state == 'blinkRight':
      if int(now * 2.5) % 2 == 0:
        rl.draw_circle_v(cls._pt(ox, oy, s, 356, 42), 13 * s, AMBER)
      cls._draw_open_eye(ox, oy, s, 138, 0, 0, is_blink)
      cls._draw_quad(ox, oy, s, (230, 90), (262, 118), (294, 90), 11, WHITE)
      cls._draw_quad(ox, oy, s, (181, 164), (200, 182), (219, 164), 7, WHITE)

    elif state == 'blinkLeft':
      if int(now * 2.5) % 2 == 0:
        rl.draw_circle_v(cls._pt(ox, oy, s, 44, 42), 13 * s, AMBER)
      cls._draw_quad(ox, oy, s, (106, 90), (138, 118), (170, 90), 11, WHITE)
      cls._draw_open_eye(ox, oy, s, 262, 0, 0, is_blink)
      cls._draw_quad(ox, oy, s, (181, 164), (200, 182), (219, 164), 7, WHITE)

    elif state == 'start':
      b = math.sin(now * 3.0) * 3.0
      cls._draw_sparkle(ox, oy, s, 58, 52, 12, int(170 + 85 * math.sin(now * 3.5)))
      cls._draw_sparkle(ox, oy, s, 342, 52, 12, int(170 + 85 * math.cos(now * 3.0)))
      cls._draw_sparkle(ox, oy, s, 356, 150, 9, int(170 + 85 * math.sin(now * 4.0)))
      cls._draw_quad(ox, oy, s, (100, 108 + b), (138, 56 + b), (176, 108 + b), 12, WHITE)
      cls._draw_quad(ox, oy, s, (224, 108 + b), (262, 56 + b), (300, 108 + b), 12, WHITE)
      cls._draw_quad(ox, oy, s, (173, 156 + b), (200, 196 + b), (227, 156 + b), 7, WHITE)
      rl.draw_ellipse(int(ox + 200 * s), int(oy + (179 + b) * s), 12 * s, 6 * s, TONGUE_PINK)

    elif state == 'engage':
      rl.draw_circle_v(cls._pt(ox, oy, s, 200, 30), 6 * s, ENGAGE_GREEN)
      for cx in (138, 262):
        rl.draw_line_ex(cls._pt(ox, oy, s, cx - 38, 52), cls._pt(ox, oy, s, cx + 38, 52), 10 * s, WHITE)
        rl.draw_ellipse(int(ox + cx * s), int(oy + 98 * s), 36 * s, 30 * s, WHITE)
        rl.draw_circle_v(cls._pt(ox, oy, s, cx, 99), 13 * s, INK)
        rl.draw_circle_v(cls._pt(ox, oy, s, cx + 4.5, 94), 4.5 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 185, 174), cls._pt(ox, oy, s, 215, 174), 7 * s, WHITE)

    elif state == 'disengage':
      cls._draw_quad(ox, oy, s, (102, 88 + sway), (138, 118 + sway), (174, 88 + sway), 11, WHITE)
      cls._draw_quad(ox, oy, s, (226, 88 + sway), (262, 118 + sway), (298, 88 + sway), 11, WHITE)
      yawn_ry = 11 * (0.95 + 0.40 * math.sin(now * math.pi))
      rl.draw_ellipse(int(ox + 200 * s), int(oy + (176 + sway) * s), 9 * s, yawn_ry * s, WHITE)
      for idx, (zx, zy, zsz) in enumerate(((308, 74, 24), (330, 52, 32), (356, 28, 40))):
        ph = (now * 0.42 + idx / 3.0) % 1.0
        za = int(255 * (ph / 0.18 if ph < 0.18 else (1.0 - ph) / 0.28 if ph > 0.72 else 1.0))
        z_scale = 0.6 + 0.75 * ph
        rl.draw_text("Z", int(ox + (zx + 26 * ph) * s), int(oy + (zy + 10 - 58 * ph - zsz * 0.5) * s),
                     int(zsz * z_scale * s), rl.Color(ZZZ_BLUE.r, ZZZ_BLUE.g, ZZZ_BLUE.b, max(0, min(255, za))))

    elif state == 'reverse':
      rl.draw_circle_v(cls._pt(ox, oy, s, 350, 42), 16 * s, AMBER)
      rl.draw_text("R", int(ox + 344 * s), int(oy + 33 * s), int(18 * s), INK)
      rl.draw_line_ex(cls._pt(ox, oy, s, 102, 44), cls._pt(ox, oy, s, 174, 36), 8 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 226, 36), cls._pt(ox, oy, s, 298, 44), 8 * s, WHITE)
      peek_dx = 12 + 8 * math.sin(now * 3.3)
      cls._draw_open_eye(ox, oy, s, 138, peek_dx, -1, is_blink)
      cls._draw_open_eye(ox, oy, s, 262, peek_dx, -1, is_blink)
      rl.draw_circle_lines(int(ox + 200 * s), int(oy + 178 * s), 7 * s, WHITE)

    elif state == 'waiting':
      # Traffic light with glowing red light
      rl.draw_rectangle_rounded(rl.Rectangle(ox + 342 * s, oy + 15 * s, 28 * s, 80 * s), 0.4, 8, TL_BODY)
      rl.draw_circle_v(cls._pt(ox, oy, s, 356, 30), (8.5 + 1.5 * math.sin(now * 7.8)) * s, RED_LIGHT)
      rl.draw_circle_v(cls._pt(ox, oy, s, 356, 55), 8.5 * s, TL_DIM_Y)
      rl.draw_circle_v(cls._pt(ox, oy, s, 356, 80), 8.5 * s, TL_DIM_G)
      rl.draw_line_ex(cls._pt(ox, oy, s, 173, 44), cls._pt(ox, oy, s, 103, 56), 9 * s, WHITE)
      rl.draw_line_ex(cls._pt(ox, oy, s, 227, 44), cls._pt(ox, oy, s, 297, 56), 9 * s, WHITE)
      roll_x = math.sin(now * 2.2) * 10.0
      roll_y = -6.0 - 6.0 * abs(math.cos(now * 2.2))
      cls._draw_open_eye(ox, oy, s, 138, roll_x, roll_y, is_blink)
      cls._draw_open_eye(ox, oy, s, 262, roll_x, roll_y, is_blink)
      cls._draw_quad(ox, oy, s, (185, 177), (200, 171), (215, 177), 7, WHITE)

    else:  # 'driving'
      cls._draw_quad(ox, oy, s, (103, 46 + sway), (138, 35 + sway), (173, 46 + sway), 7, WHITE)
      cls._draw_quad(ox, oy, s, (227, 46 + sway), (262, 35 + sway), (297, 46 + sway), 7, WHITE)
      cls._draw_open_eye(ox, oy, s, 138, 0, 1, is_blink)
      cls._draw_open_eye(ox, oy, s, 262, 0, 1, is_blink)
      cls._draw_quad(ox, oy, s, (181, 166 + sway), (200, 182 + sway), (219, 166 + sway), 7, WHITE)
