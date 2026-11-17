"""Passive Mochi-face DM character shared by C3 and C4. No camera consumer."""
import math
import time

import pyray as rl

from openpilot.selfdrive.ui.dm_preview import PREVIEW_MAX_AGE, eye_expression, preview_rect, preview_state, sample_fresh, smooth_expression
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


PEARL = rl.Color(255, 255, 255, 255)
PUPIL = rl.Color(40, 38, 36, 255)
SHADOW = rl.Color(10, 12, 16, 210)
MOCHI_BG = rl.Color(8, 10, 14, 195)
MOCHI_BORDER = rl.Color(255, 255, 255, 36)
TONGUE = rl.Color(255, 130, 168, 255)
SPARKLE = rl.Color(255, 214, 64, 255)
UNKNOWN = rl.Color(145, 145, 153, 220)
WARNING = rl.Color(255, 151, 105, 255)
HANDS = rl.Color(160, 204, 229, 255)


class DriverPreview(Widget):
  def __init__(self, compact: bool = False):
    super().__init__()
    self.compact = compact
    self.set_enabled(False)  # Display only; touches keep their normal onroad meaning.
    self._font = gui_app.font(FontWeight.BOLD)
    self._closed = False
    self._reset_expression()

  def _reset_expression(self):
    self._started_frame = ui_state.started_frame
    self._expression = None
    self._driver_side = None
    self._draw_time = 0.0
    self._model_frame = None
    self._model_progress = 0.0

  def close(self):
    self._closed = True
    self._reset_expression()

  def draw_onroad(self, parent: rl.Rectangle, alert_visible: bool) -> bool:
    sm = ui_state.sm
    # Parent views may not call us at all while offroad.
    if self._started_frame != ui_state.started_frame:
      self._reset_expression()
    visible = (not self._closed and ui_state.started and
               (sm['selfdriveState'].enabled or ui_state.always_on_dm) and not alert_visible)
    if not visible:
      self._reset_expression()
      return False
    self.render(rl.Rectangle(*preview_rect(parent.x, parent.y, parent.width, parent.height, self.compact)))
    return True

  @staticmethod
  def _fresh(service: str, now: float) -> bool:
    sm = ui_state.sm
    return (sm.valid[service] and sm.alive[service] and sm.recv_frame[service] > ui_state.started_frame and
            sample_fresh(now, sm.logMonoTime[service] / 1e9))

  def _render(self, rect):
    now = time.monotonic()
    sm = ui_state.sm
    dm, model = sm['driverMonitoringState'], sm['driverStateV2']
    monitoring_fresh = self._fresh('driverMonitoringState', now)
    driver_fresh = self._fresh('driverStateV2', now)
    if driver_fresh and model.frameId != self._model_frame:
      self._model_frame, self._model_progress = model.frameId, now
    driver_fresh = driver_fresh and now - self._model_progress < PREVIEW_MAX_AGE
    vision = dm.visionPolicyState
    state = preview_state(monitoring_fresh=monitoring_fresh, camera_unavailable=dm.cameraUnavailable,
                          driver_fresh=driver_fresh, face_detected=vision.faceDetected,
                          distracted=vision.isDistracted, lockout=dm.lockout or dm.alwaysOnLockout,
                          wheel_policy=str(dm.activePolicy) == 'wheeltouch')
    animate = (monitoring_fresh and driver_fresh and not dm.cameraUnavailable and vision.faceDetected and
               state.kind in ('tracking', 'warning'))
    if animate:
      if self._driver_side != dm.isRHD or now - self._draw_time >= PREVIEW_MAX_AGE:
        self._expression = None
      self._driver_side = dm.isRHD
      driver = model.rightDriverData if dm.isRHD else model.leftDriverData
      self._expression = smooth_expression(self._expression, eye_expression(driver), now - self._draw_time)
    else:
      self._expression = None  # Never retain a reassuring pose after lost/stale data.
    self._draw_time = now

    color = {'tracking': PEARL, 'warning': WARNING, 'wheel': HANDS}.get(state.kind, UNKNOWN)
    label_h, font_size = (16, 11) if self.compact else (32, 21)
    scale = min(rect.width / 154, (rect.height - label_h - 8) / 90)
    rl.draw_rectangle_rounded(rect, 0.28, 16, MOCHI_BG)
    rl.draw_rectangle_rounded_lines_ex(rect, 0.28, 16, max(1.0, 1.5 * scale), MOCHI_BORDER)
    cx, cy = rect.x + rect.width / 2, rect.y + (rect.height - label_h) * 0.44
    if state.kind == 'wheel':
      radius = 24 * scale
      rl.draw_circle_lines(int(cx), int(cy), radius, color)
      for dx, dy in ((-0.9, -0.2), (0.9, -0.2), (0, 0.9)):
        rl.draw_line_ex(rl.Vector2(cx, cy), rl.Vector2(cx + dx * radius, cy + dy * radius), max(1, 2 * scale), color)
    else:
      expression = self._expression
      fx = cx + (expression.x * 8 * scale if expression is not None else 0.0)
      fy = cy + (expression.y * 5 * scale if expression is not None else 0.0)
      self._draw_mochi_face(fx, fy, scale, state.kind, expression, color, now)

    label = 'DM EYES?' if state.kind == 'tracking' and (self._expression is None or not (self._expression.left_known or self._expression.right_known)) else state.label
    width = measure_text_cached(self._font, label, font_size).x
    pos = rl.Vector2(rect.x + (rect.width - width) / 2, rect.y + rect.height - label_h)
    rl.draw_text_ex(self._font, label, rl.Vector2(pos.x + 1, pos.y + 1), font_size, 0, SHADOW)
    rl.draw_text_ex(self._font, label, pos, font_size, 0, color)

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
  def _draw_sparkle(sx: float, sy: float, size: float, alpha: int):
    c = rl.Color(SPARKLE.r, SPARKLE.g, SPARKLE.b, alpha)
    rl.draw_triangle(rl.Vector2(sx, sy - size), rl.Vector2(sx - size * 0.28, sy), rl.Vector2(sx + size * 0.28, sy), c)
    rl.draw_triangle(rl.Vector2(sx, sy + size), rl.Vector2(sx + size * 0.28, sy), rl.Vector2(sx - size * 0.28, sy), c)
    rl.draw_triangle(rl.Vector2(sx - size, sy), rl.Vector2(sx, sy + size * 0.28), rl.Vector2(sx, sy - size * 0.28), c)
    rl.draw_triangle(rl.Vector2(sx + size, sy), rl.Vector2(sx, sy - size * 0.28), rl.Vector2(sx, sy + size * 0.28), c)

  @classmethod
  def _draw_mochi_face(cls, cx: float, cy: float, scale: float, kind: str, expression, color: rl.Color, now: float):
    eye_dx = 28 * scale
    eye_y = cy - 4 * scale
    thick = max(2.0, 5.5 * scale)

    if kind == 'tracking':
      # Happy Mochi ^ ^ eyes + smiling mouth with pink tongue + floating yellow sparkles
      bounce = math.sin(now * 2.8) * 1.5 * scale
      for sign in (-1, 1):
        cls._draw_arch_eye(cx + sign * eye_dx, eye_y + bounce, 32 * scale, 16 * scale, thick, color)

      # Smile mouth + pink tongue
      my = cy + 17 * scale + bounce * 0.6
      mw = 20 * scale
      mh = 12 * scale
      steps = 10
      prev = None
      for i in range(steps + 1):
        t = i / steps
        px = cx - mw / 2 + mw * t
        py = my + math.sin(t * math.pi) * mh
        pt = rl.Vector2(px, py)
        if prev is not None:
          rl.draw_line_ex(prev, pt, max(1.8, 3.8 * scale), color)
        prev = pt
      rl.draw_circle_v(rl.Vector2(cx, my + mh * 0.72), 5.2 * scale, TONGUE)

      # Animated sparkles
      a1 = int(165 + 90 * math.sin(now * 3.5))
      a2 = int(165 + 90 * math.cos(now * 2.9))
      cls._draw_sparkle(cx - 52 * scale, eye_y - 16 * scale - bounce, 7.5 * scale, a1)
      cls._draw_sparkle(cx + 50 * scale, eye_y - 12 * scale + bounce, 9.5 * scale, a2)

    elif kind == 'warning':
      # Alert furrowed brows + wide eyes + open 'O' mouth
      for sign in (-1, 1):
        ex = cx + sign * eye_dx
        rl.draw_line_ex(rl.Vector2(ex - 14 * scale, eye_y - (18 - sign * 4) * scale),
                        rl.Vector2(ex + 14 * scale, eye_y - (18 + sign * 4) * scale), max(2.0, 4.0 * scale), color)
        rl.draw_circle_v(rl.Vector2(ex, eye_y), 10 * scale, color)
        rl.draw_circle_v(rl.Vector2(ex, eye_y), 4.5 * scale, PUPIL)
      rl.draw_circle_lines(int(cx), int(cy + 20 * scale), max(3.0, 7.5 * scale), color)

    else:
      # Searching / waiting: sleepy half-closed eyes + small flat mouth
      for sign in (-1, 1):
        ex = cx + sign * eye_dx
        rl.draw_line_ex(rl.Vector2(ex - 13 * scale, eye_y), rl.Vector2(ex + 13 * scale, eye_y), max(2.0, 4.5 * scale), color)
      rl.draw_line_ex(rl.Vector2(cx - 8 * scale, cy + 18 * scale), rl.Vector2(cx + 8 * scale, cy + 18 * scale), max(1.8, 3.2 * scale), color)
