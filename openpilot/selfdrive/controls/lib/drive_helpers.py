import math
import os
import time

import numpy as np
from openpilot.cereal import log
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from opendbc.car.volkswagen.values import VolkswagenFlags


def is_volkswagen_meb(CP) -> bool:
  # VW MEB(ID.4/ID.5) 판별 단일 소스. 플랫폼 플래그 기반이라 향후 다른 VW angle 제어
  # 차종이 추가되어도 MEB 전용 경로(곡률 폐루프, 리드선택, FCW 게이트)가 오활성되지 않음.
  return CP.brand == "volkswagen" and bool(CP.flags & VolkswagenFlags.MEB)
import numpy as np

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED = 4.5  # m/s^2

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, steer_actuator_delay, distances):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    distances = [0.0] * CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = max(0.01, steer_actuator_delay)

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  delayed_curvature_desired = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], curvatures)
  future_curvature_desired = np.interp(1.2, ModelConstants.T_IDXS[:CONTROL_N], curvatures)

  psi = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)

  distance = max(np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], distances), 0.001)
  #average_curvature_desired = psi / (v_ego * delay)

  # curve -> straight or reverse curve
  if v_ego > 5 and abs(current_curvature_desired) > 0.002 and \
     (abs(future_curvature_desired) < 0.001 or np.sign(current_curvature_desired) != np.sign(future_curvature_desired)):
    psis_damping = 0.2
  else:
    psis_damping = 1.0
  #psi *= psis_damping


  average_curvature_desired = psi / distance
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = np.clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature

def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll):
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  # Avoid an abrupt authority drop at 100 km/h. Gradually transition from
  # the low-speed allowance to the high-speed EU-guideline limit.
  max_lateral_accel_no_roll = float(np.interp(
    v_ego,
    [80 / 3.6, 120 / 3.6],
    [MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED, MAX_LATERAL_ACCEL_NO_ROLL],
  ))
  max_lat_accel = max_lateral_accel_no_roll + roll_compensation
  min_lat_accel = -max_lateral_accel_no_roll + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


# 최소거리 클램프의 레인리스 조향 보정 (ClampSteerCorrector)
# 레인리스 모드에서는 controlsd가 모델 곡률만으로 조향하므로, lateral planner의
# 경로 클램프가 조향에 닿지 않는다. 이 보정기는 controlsd에서 모델의 차선·도로경계
# 데이터를 직접 읽어 모델 곡률에 더할 곡률 보정을 계산한다. 경로 클램프와 같은
# 파라미터(ClampEnable/ClampMinDistLane/ClampMinDistEdge)와 같은 가드(확률
# 히스테리시스, 연석 std, 차선변경, 저속 페이드인)를 쓴다.
CLAMP_STEER_PROB_ON = 0.5
CLAMP_STEER_PROB_OFF = 0.3
CLAMP_STEER_EDGE_STD_MAX = 0.5
CLAMP_STEER_LOOKAHEAD_TIME = 1.0  # s, 룩어헤드 거리 = 속도 x 1초
CLAMP_STEER_LOOKAHEAD_MIN = 5.0  # m
CLAMP_STEER_LOOKAHEAD_MAX = 30.0  # m
CLAMP_STEER_MAX_LAT_ACCEL = 1.5  # m/s^2, 보정이 만드는 횡가속 상한
CLAMP_STEER_MAX_CURVATURE = 0.02  # 1/m, 보정 곡률 절대 상한 (저속 과조향 방지)
CLAMP_STEER_GAIN_DEFAULT = 1.0
CLAMP_STEER_GAIN_MAX = 2.0
CLAMP_STEER_MIN_DIST_LANE_DEFAULT = 1.35  # m, 경로 클램프와 동일
CLAMP_STEER_MIN_DIST_EDGE_DEFAULT = 1.15  # m
# 알림(띵) 조건: 보정 곡률 절댓값이 ON 이상이면 보정 중, OFF 아래로 떨어져야
# 종료 (히스테리시스). 종료 후 COOLDOWN 동안은 다시 켜지지 않아 띵 소리가
# 연속으로 울리지 않는다. 0.001 1/m은 60km/h에서 약 0.28 m/s²,
# 100km/h에서 약 0.77 m/s²의 횡가속에 해당 — 운전자가 느낄 만한 보정이다.
CLAMP_STEER_ALERT_ON_THRESHOLD = 0.001  # 1/m
CLAMP_STEER_ALERT_OFF_THRESHOLD = 0.0005  # 1/m
CLAMP_STEER_ALERT_COOLDOWN_SEC = 5.0  # s
# 차선변경 종료 후 홀드오프 + 보정 변화율 제한: 차선변경 중에는 보정이 꺼지고,
# 종료 직후 차가 새 차선에 안착하기 전에는 큰 위반이 남아 있을 수 있다.
# 홀드오프로 재개를 늦추고, 슬루 제한으로 재개 시 보정이 0에서 점진적으로
# 올라가게 해 급조향(확 틀어졌다 바로잡는 현상)을 막는다. 정상 주행에서는
# 위반이 서서히 커지므로 슬루 제한에 걸리지 않는다.
CLAMP_STEER_LANE_CHANGE_HOLDOFF_SEC = 2.0  # s
CLAMP_STEER_SLEW_RATE = 0.01  # 1/m/s, 최대 보정(0.0054)까지 약 0.54초
# 위반 불감대 (히스테리시스): 정상 주행에서 모델 경로가 선 쪽으로 조금씩
# 출렁이는 것만으로 보정이 자꾸 켜지는 것을 막는다. 위반이 20cm를 넘으면
# 켜지고, 10cm 아래로 떨어져야 꺼진다. 문제 지점(위반 약 60cm)에서는 그대로
# 동작하고, 보정 크기 자체는 줄이지 않는다(게이트만 건다).
CLAMP_STEER_DEADBAND_ON = 0.20  # m
CLAMP_STEER_DEADBAND_OFF = 0.10  # m


LANE_CENTER_PROB_ON = 0.5
LANE_CENTER_PROB_OFF = 0.3
LANE_CENTER_LOOKAHEAD_TIME = 1.0
LANE_CENTER_LOOKAHEAD_MIN = 5.0
LANE_CENTER_LOOKAHEAD_MAX = 30.0
LANE_CENTER_MAX_LAT_ACCEL = 1.5
LANE_CENTER_MAX_CURVATURE = 0.02
LANE_CENTER_GAIN_DEFAULT = 1.0
LANE_CENTER_GAIN_MAX = 2.0
LANE_CENTER_THRESHOLD_DEFAULT = 0.20  # m, 중앙에서 이만큼 벗어나면 보정 시작
LANE_CENTER_MIN_SPEED_MAX_KPH = 150
LANE_CENTER_LANE_CHANGE_HOLDOFF_SEC = 2.0  # s
LANE_CENTER_SLEW_RATE = 0.01  # 1/m/s, 클램프와 동일
LANE_CENTER_ENTRY_HOLD_NS = 300_000_000
LANE_CENTER_ENTRY_RAMP_NS = 400_000_000
LANE_CENTER_ENTRY_MAX_FRAME_GAP_NS = 150_000_000
LANE_CENTER_GEOMETRY_WINDOW_NS = 320_000_000
LANE_CENTER_GEOMETRY_HOLD_NS = 1_000_000_000
LANE_CENTER_GEOMETRY_RECOVERY_NS = 200_000_000


class ClampSteerCorrector:
  """레인리스 조향용 최소거리 클램프 보정기.

  update()는 모델 곡률에 더할 곡률 보정(1/m)을 반환한다.
  부호 규약 (controlsd와 동일): 양의 곡률 = 우회전(+y 방향). y는 +가 우측.
  왼쪽 선(중앙선)에 가까우면 양의 보정(우측으로 밀어냄), 오른쪽 선·연석에
  가까우면 음의 보정을 낸다. 위반이 없으면 0.0 (모델 조향을 그대로 둔다).

  모델 데이터가 불완전하거나 파라미터 읽기에 실패하면 0.0을 반환해
  controlsd가 죽지 않는다.
  """

  def __init__(self, params):
    self.params = params
    self.left_active = False
    self.right_active = False
    self.last_correction = 0.0
    # 알림(띵) 상태: 보정이 의미 있는 크기로 걸릴 때 True. controlsd가
    # controlsState.clampSteerActive로 selfdrived에 전달하고, selfdrived가
    # clampSteerActive 이벤트로 prompt 사운드를 울린다.
    self.alert_active = False
    self._alert_cooldown_until = 0.0
    # 차선변경 홀드오프 + 슬루 제한 상태. _prev_time이 None이거나 마지막 호출이
    # 0.5초보다 오래 전이면(보정기가 안 돌던 구간, 예: 레인풀 모드) 이전 출력을
    # 버리고 0에서 다시 시작한다.
    self._lane_change_holdoff_until = 0.0
    self._prev_kappa = 0.0
    self._prev_time = None
    # 불감대 게이트 상태: 위반이 20cm를 넘으면 True, 10cm 아래로 떨어지면 False.
    self._correction_active = False

  def _read_settings(self):
    """(enabled, min_dist_lane_m, min_dist_edge_m, gain)을 읽는다.

    get(..., return_default=True)을 쓴다. get_int/get_bool은 파일이 없으면
    0/False를 반환해서 fresh install에서 클램프가 꺼지거나 거리가 0이 되기
    때문이다. 키 미등록(구 빌드) 등 오류 시 상수 기본값으로 폴백한다.
    get()은 BOOL/INT를 제대로 변환해서 반환하므로 보통 bool/int가 온다.
    """
    try:
      enabled = self.params.get("ClampEnable", return_default=True)
      min_lane_cm = self.params.get("ClampMinDistLane", return_default=True)
      min_edge_cm = self.params.get("ClampMinDistEdge", return_default=True)
      gain_raw = self.params.get("ClampSteerGain", return_default=True)
    except Exception:
      return True, CLAMP_STEER_MIN_DIST_LANE_DEFAULT, CLAMP_STEER_MIN_DIST_EDGE_DEFAULT, CLAMP_STEER_GAIN_DEFAULT
    if enabled is None:
      enabled = True
    elif isinstance(enabled, str):
      enabled = enabled.strip().lower() in ("1", "true", "on", "yes")
    try:
      min_lane = (CLAMP_STEER_MIN_DIST_LANE_DEFAULT if min_lane_cm is None else float(
        np.clip(float(min_lane_cm) * 0.01, 0.0, 3.0)))
      min_edge = (CLAMP_STEER_MIN_DIST_EDGE_DEFAULT if min_edge_cm is None else float(
        np.clip(float(min_edge_cm) * 0.01, 0.0, 3.0)))
      gain = (CLAMP_STEER_GAIN_DEFAULT if gain_raw is None else float(
        np.clip(float(gain_raw) * 0.1, 0.0, CLAMP_STEER_GAIN_MAX)))
    except (TypeError, ValueError):
      return bool(enabled), CLAMP_STEER_MIN_DIST_LANE_DEFAULT, CLAMP_STEER_MIN_DIST_EDGE_DEFAULT, CLAMP_STEER_GAIN_DEFAULT
    if not math.isfinite(min_lane):
      min_lane = CLAMP_STEER_MIN_DIST_LANE_DEFAULT
    if not math.isfinite(min_edge):
      min_edge = CLAMP_STEER_MIN_DIST_EDGE_DEFAULT
    if not math.isfinite(gain):
      gain = CLAMP_STEER_GAIN_DEFAULT
    return bool(enabled), min_lane, min_edge, gain

  def _read_alert_enabled(self):
    """ClampSteerAlert 파라미터를 읽는다. 기본값 True (알림 켜짐).

    get(..., return_default=True)을 쓴다. 키 미등록(구 빌드) 등 오류 시
    True로 폴백해 알림이 계속 울린다.
    """
    try:
      v = self.params.get("ClampSteerAlert", return_default=True)
    except Exception:
      return True
    if v is None:
      return True
    if isinstance(v, str):
      return v.strip().lower() in ("1", "true", "on", "yes")
    return bool(v)

  def _update_alert(self, kappa):
    """보정 크기에 따라 알림 상태(alert_active)를 갱신한다.

    보정 절댓값이 ON 임계값 이상이면 켜고, OFF 임계값 아래로 떨어져야
    끈다 (히스테리시스). 한 번 꺼지면 쿨다운(5초) 동안 다시 켜지지 않아
    띵 소리가 연속으로 울리지 않는다. 알림 파라미터가 꺼져 있으면 항상
    False. 좌/우 방향과 무관하게 절댓값으로 판단한다.
    """
    if not self._read_alert_enabled():
      self.alert_active = False
      return
    now = time.monotonic()
    if self.alert_active:
      if abs(kappa) < CLAMP_STEER_ALERT_OFF_THRESHOLD:
        self.alert_active = False
        self._alert_cooldown_until = now + CLAMP_STEER_ALERT_COOLDOWN_SEC
    elif abs(kappa) >= CLAMP_STEER_ALERT_ON_THRESHOLD and now >= self._alert_cooldown_until:
      self.alert_active = True

  def update(self, md, v_ego, lane_changing):
    """모델 곡률에 더할 곡률 보정(1/m)을 계산한다. 양수 = 우회전.

    반환되는 보정 크기에 따라 알림 상태(alert_active)도 함께 갱신한다.
    보정이 0이면 알림은 꺼지는 방향으로 간다 (쿨다운은 _update_alert에서
    관리). 모든 early return 경로가 이 래퍼를 거치므로 알림 상태가
    stale로 남지 않는다.
    """
    kappa = self._update_impl(md, v_ego, lane_changing)
    kappa = self._slew_limit(kappa)
    self.last_correction = kappa
    self._update_alert(kappa)
    return kappa

  def _slew_limit(self, kappa):
    """보정 곡률의 변화율을 제한한다 (급조향 방지).

    목표 보정과 이전 출력의 차이를 CLAMP_STEER_SLEW_RATE * dt 이내로 자른다.
    차선변경 종료 직후처럼 큰 위반이 갑자기 나타나는 경우에도 보정이 0에서
    최대까지 약 0.5초에 걸쳐 점진적으로 올라간다. 정상 주행에서는 차가 선에
    서서히 다가가므로 목표 보정의 변화가 느려 제한에 걸리지 않는다.
    첫 호출이거나 0.5초 이상 호출이 없었으면(보정기가 안 돌던 구간) 이전
    출력을 버리고 0에서 시작한다. dt는 0.1초로 상한을 두어 긴 간격이
    제한을 무력화하지 못하게 한다.
    """
    now = time.monotonic()
    if self._prev_time is None or (now - self._prev_time) > 0.5:
      self._prev_time = now
      self._prev_kappa = 0.0
      return 0.0
    dt = min(max(now - self._prev_time, 0.0), 0.1)
    self._prev_time = now
    max_delta = CLAMP_STEER_SLEW_RATE * dt
    self._prev_kappa += float(np.clip(kappa - self._prev_kappa, -max_delta, max_delta))
    return self._prev_kappa

  def _update_impl(self, md, v_ego, lane_changing):
    """update()의 본체. 보정 곡률만 계산하고 알림 상태는 건드리지 않는다."""
    self.last_correction = 0.0
    enabled, min_lane, min_edge, gain = self._read_settings()
    if not enabled or gain <= 0.0:
      self.left_active = False
      self.right_active = False
      self._correction_active = False
      return 0.0
    # 차선변경 중에는 선을 넘는 것이 의도된 동작이므로 보정하지 않는다.
    # 차선변경이 끝난 뒤에도 홀드오프(2초) 동안은 보정하지 않는다. desire/
    # laneChangeState가 꺼지는 시점은 타이머 기준이라 차가 새 차선에 안착하기
    # 전일 수 있고, 그때 큰 위반이 남아 있으면 보정이 0에서 최대까지 단번에
    # 뛰며 차를 확 틀어버리기 때문이다. 홀드오프 타이머는 차선변경 중 매
    # 사이클 갱신되므로, 종료 시점부터 정확히 홀드오프 시간만큼 유지된다.
    now = time.monotonic()
    if lane_changing:
      self.left_active = False
      self.right_active = False
      self._correction_active = False
      self._lane_change_holdoff_until = now + CLAMP_STEER_LANE_CHANGE_HOLDOFF_SEC
      return 0.0
    if now < self._lane_change_holdoff_until:
      self._correction_active = False
      return 0.0
    # 저속 페이드인: 5km/h 이하 0, 10km/h 이상 100% (경로 클램프와 동일 구간)
    strength = float(np.interp(v_ego * 3.6, [5.0, 10.0], [0.0, 1.0]))
    if strength <= 0.0:
      self._correction_active = False
      return 0.0
    try:
      return self._compute(md, v_ego, min_lane, min_edge, gain, strength)
    except Exception:
      self._correction_active = False
      return 0.0

  def _compute(self, md, v_ego, min_lane, min_edge, gain, strength):
    lane_lines = md.laneLines
    edges = md.roadEdges
    lines_ok = (len(lane_lines) >= 4 and len(lane_lines[1].y) >= 2
                and len(lane_lines[2].y) >= 2 and len(md.laneLineProbs) >= 3)
    edges_ok = (len(edges) >= 2 and len(edges[0].y) >= 2 and len(edges[1].y) >= 2
                and len(md.roadEdgeStds) >= 2)
    pos_ok = (len(md.position.x) >= 2 and len(md.position.y) >= 2)
    if not pos_ok or (not lines_ok and not edges_ok):
      return 0.0

    # 확률 히스테리시스: 0.5를 넘으면 켜고 0.3 아래로 떨어져야 끈다 (경로 클램프와 동일).
    if lines_ok:
      l_prob = float(md.laneLineProbs[1])
      r_prob = float(md.laneLineProbs[2])
      if l_prob > CLAMP_STEER_PROB_ON:
        self.left_active = True
      elif l_prob < CLAMP_STEER_PROB_OFF:
        self.left_active = False
      if r_prob > CLAMP_STEER_PROB_ON:
        self.right_active = True
      elif r_prob < CLAMP_STEER_PROB_OFF:
        self.right_active = False
    else:
      self.left_active = False
      self.right_active = False

    # 룩어헤드 거리: 1초 앞 지점 (5~30m). 고속일수록 멀리 보고 미리 조향한다.
    lookahead = float(np.clip(v_ego * CLAMP_STEER_LOOKAHEAD_TIME,
                              CLAMP_STEER_LOOKAHEAD_MIN, CLAMP_STEER_LOOKAHEAD_MAX))
    path_y = float(np.interp(lookahead, md.position.x, md.position.y))
    if not math.isfinite(path_y):
      return 0.0

    # 좌측 하한: 경로는 이 값보다 우측(+y)이어야 한다. 차선이 보이면 차선 기준,
    # 안 보이면 도로경계 기준(연석 std가 작을 때만). 경로 클램프와 동일 로직.
    lo = -np.inf
    if self.left_active:
      left_y = float(np.interp(lookahead, lane_lines[1].x, lane_lines[1].y))
      lo = left_y + min_lane
    elif edges_ok:
      le_std = float(md.roadEdgeStds[0])
      if le_std < CLAMP_STEER_EDGE_STD_MAX:
        left_edge_y = float(np.interp(lookahead, edges[0].x, edges[0].y)) + le_std * 0.4
        lo = left_edge_y + min_edge
    # 우측 하한: 경로는 이 값보다 좌측(-y)이어야 한다.
    hi = np.inf
    if self.right_active:
      right_y = float(np.interp(lookahead, lane_lines[2].x, lane_lines[2].y))
      hi = right_y - min_lane
    elif edges_ok:
      re_std = float(md.roadEdgeStds[1])
      if re_std < CLAMP_STEER_EDGE_STD_MAX:
        right_edge_y = float(np.interp(lookahead, edges[1].x, edges[1].y)) - re_std * 0.4
        hi = right_edge_y - min_edge
    if not math.isfinite(lo):
      lo = -np.inf
    if not math.isfinite(hi):
      hi = np.inf
    if lo == -np.inf and hi == np.inf:
      return 0.0

    # 목표 y: 위반한 만큼만 밀어낸 위치. 양쪽 하한이 충돌하면(좁은 차로)
    # 중간 = 차선 중앙을 목표로 한다.
    if lo > hi:
      target_y = (lo + hi) / 2.0
    else:
      target_y = float(np.clip(path_y, lo, hi))
    # 위반 불감대 (히스테리시스): 위반이 20cm를 넘으면 켜지고 10cm 아래로
    # 떨어져야 꺼진다. 작은 위반은 무시해 정상 주행에서 보정이 자꾸 켜지는
    # 것을 막는다. 보정 크기는 줄이지 않는다(게이트만 건다). 켜질 때의
    # 목표값 점프는 슬루 제한이 부드럽게 올려준다.
    violation = abs(target_y - path_y)
    if self._correction_active:
      if violation < CLAMP_STEER_DEADBAND_OFF:
        self._correction_active = False
    elif violation > CLAMP_STEER_DEADBAND_ON:
      self._correction_active = True
    if not self._correction_active:
      return 0.0
    y_corr = (target_y - path_y) * strength
    if y_corr == 0.0:
      return 0.0

    # 곡률 변환 (pure pursuit): κ = 2·y / L². 양의 y_corr(우측 이동)는
    # 양의 곡률(우회전)이다. 게인을 곱하고, 보정이 만드는 횡가속이
    # 1.5 m/s²를 넘지 않도록, 그리고 곡률 자체가 0.02를 넘지 않도록
    # 자른다 (모델 곡률 위의 nudge이므로 과도한 개입을 막고, 최종 한도는
    # 뒤의 clip_curvature가 잡는다).
    kappa = 2.0 * y_corr / (lookahead ** 2) * gain
    max_kappa = min(CLAMP_STEER_MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2,
                    CLAMP_STEER_MAX_CURVATURE)
    kappa = float(np.clip(kappa, -max_kappa, max_kappa))
    self.last_correction = kappa
    return kappa


class LaneCenterCorrector:
  """레인리스 조향용 차선 중앙 유지 보정기.

  update()는 모델 곡률에 더할 곡률 보정(1/m)을 반환한다.
  부호 규약 (controlsd와 동일): 양의 곡률 = 우회전(+y 방향). y는 +가 우측.
  모델 경로가 차선 중앙보다 왼쪽에 있으면 양의 보정(우측으로 이끔),
  오른쪽에 있으면 음의 보정을 낸다. 벗어남이 임계값 이하면 0.0 (모델
  조향을 그대로 둔다).

  클램프와 같은 계산 방식(pure pursuit)과 같은 가드(확률 히스테리시스,
  차선변경 홀드오프, 저속 페이드인, 슬루 제한)를 쓰지만, 양쪽 차선이
  모두 보여야 동작한다. 모델 데이터가 불완전하거나 파라미터 읽기에
  실패하면 0.0을 반환해 controlsd가 죽지 않는다.
  """

  def __init__(self, params):
    self.params = params
    self.left_active = False
    self.right_active = False
    self.last_correction = 0.0
    self._lane_change_holdoff_until = 0.0
    self._prev_kappa = 0.0
    self._prev_time = None
    self._reset_geometry()

  def _reset_entry(self):
    self._correction_active = False
    self._entry_since = None
    self._entry_side = 0
    self._entry_last_frame = None
    self._entry_threshold = None
    self._ramp_since = None

  def _entry_scale(self, stamp, offset, threshold):
    if stamp <= 0:
      self._reset_entry()
      return 0.0
    last = self._entry_last_frame
    if last is not None and (stamp < last or stamp - last > LANE_CENTER_ENTRY_MAX_FRAME_GAP_NS
                             or (not self._correction_active and threshold != self._entry_threshold)):
      self._reset_entry()
    # controlsd runs faster than the model, so repeated samples do not accrue time.
    if stamp != self._entry_last_frame:
      self._entry_last_frame = stamp
      self._entry_threshold = threshold
      magnitude = abs(offset)
      side = 1 if offset > 0.0 else -1
      if self._correction_active and (magnitude < threshold / 2.0 or side != self._entry_side):
        self._correction_active = False
        self._entry_since = None
        self._entry_side = 0
      if not self._correction_active:
        if magnitude > threshold:
          if self._entry_since is None or side != self._entry_side:
            self._entry_since = stamp
            self._entry_side = side
          if stamp - self._entry_since >= LANE_CENTER_ENTRY_HOLD_NS:
            self._correction_active = True
            self._ramp_since = stamp
        else:
          self._entry_since = None
          self._entry_side = 0
    if not self._correction_active:
      return 0.0
    progress = min(1.0, (stamp - self._ramp_since) / LANE_CENTER_ENTRY_RAMP_NS)
    return progress * progress * (3.0 - 2.0 * progress)

  def _reset_geometry(self):
    self._reset_entry()
    self._geometry_history = []
    self._geometry_hold_until = 0
    self._geometry_suppressed = False

  def _geometry_usable(self, stamp, lookahead, path_y, center_y, width, confidence):
    # Each camera frame supplies one sample even though controlsd runs at 100 Hz.
    if stamp <= 0:
      return not self._geometry_suppressed
    if self._geometry_history:
      previous = self._geometry_history[-1][0]
      if stamp == previous:
        return not self._geometry_suppressed
      if stamp < previous or stamp - previous > 500_000_000:
        self._geometry_history.clear()
        if stamp < previous and self._geometry_suppressed:
          self._geometry_hold_until = stamp + LANE_CENTER_GEOMETRY_HOLD_NS

    relative_center = center_y - path_y
    self._geometry_history = [sample for sample in self._geometry_history
                              if stamp - sample[0] <= LANE_CENTER_GEOMETRY_WINDOW_NS]
    for old_stamp, old_lookahead, old_path, old_center, old_offset, old_width, old_conf in self._geometry_history:
      relative_shift = abs(relative_center - old_offset)
      weakest = min(confidence, old_conf)
      if (stamp - old_stamp >= 40_000_000 and abs(lookahead - old_lookahead) <= 1.5
          and max(confidence, old_conf) >= LANE_CENTER_PROB_OFF
          and abs(center_y - old_center) >= 0.18 and relative_shift >= 0.24
          and abs(path_y - old_path) <= 0.18
          and (abs(width - old_width) >= 0.25 or relative_shift >= 0.65
               or (relative_shift >= 0.40 and weakest < 0.80))):
        # A boundary switch moves the inferred center without a matching model-path change.
        self._geometry_suppressed = True
        self._geometry_hold_until = stamp + LANE_CENTER_GEOMETRY_HOLD_NS
        break

    self._geometry_history.append((stamp, lookahead, path_y, center_y, relative_center, width, confidence))
    if self._geometry_suppressed and stamp >= self._geometry_hold_until:
      # Wait for a span of fresh, consistent geometry before restoring the correction.
      offsets = [sample[4] for sample in self._geometry_history]
      widths = [sample[5] for sample in self._geometry_history]
      if (stamp - self._geometry_history[0][0] >= LANE_CENTER_GEOMETRY_RECOVERY_NS
          and min(sample[6] for sample in self._geometry_history) > LANE_CENTER_PROB_ON
          and 2.5 <= min(widths) and max(widths) <= 4.7
          and max(widths) - min(widths) <= 0.22 and max(offsets) - min(offsets) <= 0.16):
        self._geometry_suppressed = False
    return not self._geometry_suppressed

  def _read_settings(self):
    """(enabled, threshold_m, gain)을 읽는다.

    get(..., return_default=True)을 쓴다. get_int/get_bool은 파일이 없으면
    0/False를 반환해서 fresh install에서 보정이 꺼지거나 임계값이 0이 되기
    때문이다. 키 미등록(구 빌드) 등 오류 시 상수 기본값으로 폴백한다.
    """
    try:
      enabled = self.params.get("LaneCenterEnable", return_default=True)
      threshold_cm = self.params.get("LaneCenterThreshold", return_default=True)
      gain_raw = self.params.get("LaneCenterGain", return_default=True)
    except Exception:
      return True, LANE_CENTER_THRESHOLD_DEFAULT, LANE_CENTER_GAIN_DEFAULT
    if enabled is None:
      enabled = True
    elif isinstance(enabled, str):
      enabled = enabled.strip().lower() in ("1", "true", "on", "yes")
    try:
      threshold = (LANE_CENTER_THRESHOLD_DEFAULT if threshold_cm is None else float(
        np.clip(float(threshold_cm) * 0.01, 0.0, 1.0)))
      gain = (LANE_CENTER_GAIN_DEFAULT if gain_raw is None else float(
        np.clip(float(gain_raw) * 0.1, 0.0, LANE_CENTER_GAIN_MAX)))
    except (TypeError, ValueError):
      return bool(enabled), LANE_CENTER_THRESHOLD_DEFAULT, LANE_CENTER_GAIN_DEFAULT
    if not math.isfinite(threshold):
      threshold = LANE_CENTER_THRESHOLD_DEFAULT
    if not math.isfinite(gain):
      gain = LANE_CENTER_GAIN_DEFAULT
    return bool(enabled), threshold, gain

  def update(self, md, v_ego, lane_changing, *, avoidance=None):
    """모델 곡률에 더할 곡률 보정(1/m)을 계산한다. 양수 = 우회전."""
    kappa = self._update_impl(md, v_ego, lane_changing)
    if avoidance is not None and kappa * avoidance.side > 0.0:
      lookahead = float(np.clip(v_ego * LANE_CENTER_LOOKAHEAD_TIME,
                                LANE_CENTER_LOOKAHEAD_MIN, LANE_CENTER_LOOKAHEAD_MAX))
      approach = min(1.0, max(0.0, (lookahead - avoidance.approachStart) / 5.0))
      kappa *= 1.0 - approach * avoidance.strength
    kappa = self._slew_limit(kappa)
    self.last_correction = kappa
    return kappa

  def _slew_limit(self, kappa):
    """보정 곡률의 변화율을 제한한다 (급조향 방지). 클램프와 동일."""
    now = time.monotonic()
    if self._prev_time is None or (now - self._prev_time) > 0.5:
      self._prev_time = now
      self._prev_kappa = 0.0
      return 0.0
    dt = min(max(now - self._prev_time, 0.0), 0.1)
    self._prev_time = now
    max_delta = LANE_CENTER_SLEW_RATE * dt
    self._prev_kappa += float(np.clip(kappa - self._prev_kappa, -max_delta, max_delta))
    return self._prev_kappa

  def _update_impl(self, md, v_ego, lane_changing):
    """update()의 본체. 보정 곡률만 계산한다."""
    self.last_correction = 0.0
    enabled, threshold, gain = self._read_settings()
    if not enabled or gain <= 0.0:
      self.left_active = False
      self.right_active = False
      self._reset_geometry()
      return 0.0
    # 차선변경 중에는 선을 넘는 것이 의도된 동작이므로 보정하지 않는다.
    # 종료 후 홀드오프(2초) 동안도 보정하지 않는다 (클램프와 동일 이유).
    now = time.monotonic()
    if lane_changing:
      self.left_active = False
      self.right_active = False
      self._reset_geometry()
      self._lane_change_holdoff_until = now + LANE_CENTER_LANE_CHANGE_HOLDOFF_SEC
      return 0.0
    if now < self._lane_change_holdoff_until:
      self._reset_entry()
      return 0.0
    # 저속 페이드인: 5km/h 이하 0, 10km/h 이상 100% (클램프와 동일 구간)
    if not math.isfinite(v_ego):
      self._reset_entry()
      return 0.0
    strength = float(np.interp(v_ego * 3.6, [5.0, 10.0], [0.0, 1.0]))
    if strength <= 0.0:
      self._reset_geometry()
      return 0.0
    try:
      min_speed_kph = float(self.params.get("LaneCenterMinSpeed", return_default=True) or 0)
    except Exception:
      min_speed_kph = 0.0
    if not math.isfinite(min_speed_kph):
      min_speed_kph = 0.0
    min_speed = min(max(min_speed_kph, 0.0), LANE_CENTER_MIN_SPEED_MAX_KPH) / 3.6
    try:
      return self._compute(md, v_ego, threshold, gain, strength, speed_eligible=v_ego >= min_speed)
    except Exception:
      self._reset_entry()
      return 0.0

  def _compute(self, md, v_ego, threshold, gain, strength, *, speed_eligible):
    lane_lines = md.laneLines
    lines_ok = (len(lane_lines) >= 4 and len(lane_lines[1].y) >= 2
                and len(lane_lines[2].y) >= 2 and len(md.laneLineProbs) >= 3)
    pos_ok = (len(md.position.x) >= 2 and len(md.position.y) >= 2)
    if not pos_ok or not lines_ok:
      self._reset_entry()
      return 0.0

    # 확률 히스테리시스: 0.5를 넘으면 켜고 0.3 아래로 떨어져야 끈다
    # (클램프와 동일). 양쪽 모두 켜져 있어야 중앙을 계산할 수 있다.
    l_prob = float(md.laneLineProbs[1])
    r_prob = float(md.laneLineProbs[2])
    if not (math.isfinite(l_prob) and math.isfinite(r_prob)):
      self.left_active = self.right_active = False
      self._reset_entry()
      return 0.0
    if l_prob > LANE_CENTER_PROB_ON:
      self.left_active = True
    elif l_prob < LANE_CENTER_PROB_OFF:
      self.left_active = False
    if r_prob > LANE_CENTER_PROB_ON:
      self.right_active = True
    elif r_prob < LANE_CENTER_PROB_OFF:
      self.right_active = False
    # 룩어헤드 거리: 1초 앞 지점 (5~30m). 클램프와 동일.
    lookahead = float(np.clip(v_ego * LANE_CENTER_LOOKAHEAD_TIME,
                              LANE_CENTER_LOOKAHEAD_MIN, LANE_CENTER_LOOKAHEAD_MAX))
    path_y = float(np.interp(lookahead, md.position.x, md.position.y))
    left_y = float(np.interp(lookahead, lane_lines[1].x, lane_lines[1].y))
    right_y = float(np.interp(lookahead, lane_lines[2].x, lane_lines[2].y))
    if not (math.isfinite(path_y) and math.isfinite(left_y) and math.isfinite(right_y)):
      self._reset_entry()
      return 0.0

    # 차선 중앙: 양쪽 차선의 중간. 경로가 중앙에서 벗어난 만큼 보정한다.
    center_y = (left_y + right_y) / 2.0
    stamp = getattr(md, "timestampEof", 0)
    if not self._geometry_usable(stamp, lookahead, path_y, center_y, right_y - left_y, min(l_prob, r_prob)):
      self._reset_entry()
      return 0.0
    # Keep watching for boundary jumps below the selected speed, so crossing
    # the threshold cannot bypass an active geometry holdoff.
    if not speed_eligible or not (self.left_active and self.right_active):
      self._reset_entry()
      return 0.0
    offset = path_y - center_y  # 양수면 경로가 중앙보다 우측
    y_corr = -offset * strength * self._entry_scale(stamp, offset, threshold)
    if y_corr == 0.0:
      return 0.0

    # 곡률 변환 (pure pursuit): κ = 2·y / L². 양의 y_corr(우측 이동)는
    # 양의 곡률(우회전)이다. 게인을 곱하고, 보정이 만드는 횡가속이
    # 1.5 m/s²를 넘지 않도록, 그리고 곡률 자체가 0.02를 넘지 않도록
    # 자른다 (클램프와 동일 상한).
    kappa = 2.0 * y_corr / (lookahead ** 2) * gain
    max_kappa = min(LANE_CENTER_MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2,
                    LANE_CENTER_MAX_CURVATURE)
    kappa = float(np.clip(kappa, -max_kappa, max_kappa))
    self.last_correction = kappa
    return kappa


# 카메라 오프셋 보정 (CameraOffsetCorrector)
# 클램프가 "선을 넘지 말라"는 안전장치고, 중앙 유지가 "차선 가운데에
# 있으라"는 지속 보정이라면, 카메라 오프셋은 "모델 경로 기준으로 차를
# 옆으로 일정량 밀어준다"는 고정 이동이다. 차선 위치를 보지 않고 모델
# 경로 기준으로 밀어주므로 차선이 0개/1개여도 동작한다 — 1차선 도로와
# 차선 하나만 보이는 도로에서 차가 중앙선 쪽에 붙는 문제를 잡는 것이
# 목적이다. 대신 게이트가 있다: (왼쪽 차선만 보임) 또는 (제일 왼쪽
# 차선)일 때만 작동한다. 게이트가 없으면 평소 주행에서도 차가 한쪽으로
# 치우쳐 있기 때문이다. 클램프와 같은 계산 방식(pure pursuit)과 같은
# 가드(확률 히스테리시스, 차선변경 홀드오프, 저속 페이드인, 슬루 제한)를
# 쓴다. 클램프는 안전망으로 남겨둔다: 오프셋이 차를 오른쪽 선 쪽으로
# 너무 밀면 클램프가 막아준다. 중앙 유지와는 방향이 부딪히므로(중앙 유지는
# 가운데로, 오프셋은 옆으로) 함께 켜지 않는 것이 좋다.
CAMERA_OFFSET_PROB_ON = 0.5
CAMERA_OFFSET_PROB_OFF = 0.3
# 먼왼쪽 차선([0])은 검출이 약해서 0.5 기준이면 2차선에서도 확률이
# 0.46~0.54로 경계에 걸려 오판함 (2026-11-24 rlog__5__3 확인 — 1차선에서는
# 0.02~0.03으로 명확히 꺼짐). 먼 차선 존재 판단은 0.35 기준으로 본다.
CAMERA_OFFSET_FAR_PROB_ON = 0.35
CAMERA_OFFSET_FAR_PROB_OFF = 0.25
CAMERA_OFFSET_LOOKAHEAD_TIME = 1.0  # s, 룩어헤드 거리 = 속도 x 1초
CAMERA_OFFSET_LOOKAHEAD_MIN = 5.0  # m
CAMERA_OFFSET_LOOKAHEAD_MAX = 30.0  # m
CAMERA_OFFSET_MAX_LAT_ACCEL = 1.5  # m/s^2, 보정이 만드는 횡가속 상한
CAMERA_OFFSET_MAX_CURVATURE = 0.02  # 1/m, 보정 곡률 절대 상한 (저속 과조향 방지)
CAMERA_OFFSET_DEFAULT_M = 0.0  # m, 기본값 0 = 꺼짐
CAMERA_OFFSET_MAX_M = 0.5  # m, 카탈로그 ±50cm
CAMERA_OFFSET_LANE_CHANGE_HOLDOFF_SEC = 2.0  # s
CAMERA_OFFSET_SLEW_RATE = 0.01  # 1/m/s, 클램프와 동일
CAMERA_OFFSET_GATE_ON_SEC = 1.0  # s, 게이트 조건이 이만큼 유지돼야 켜짐
CAMERA_OFFSET_GATE_OFF_HOLD_SEC = 2.0  # s, 조건 상실 후 이만큼 유지돼야 꺼짐
CAMERA_OFFSET_FILE_REREAD_SEC = 1.0  # s, 백킹 파일 폴백 재읽기 간격


class CameraOffsetCorrector:
  """레인리스 조향용 카메라 오프셋 보정기.

  update()는 모델 곡률에 더할 곡률 보정(1/m)을 반환한다.
  부호 규약 (controlsd와 동일): 양의 곡률 = 우회전(+y 방향). y는 +가 우측.
  오프셋이 양수면 차를 오른쪽으로, 음수면 왼쪽으로 일정량 밀어준다.

  게이트 판정:
  - 왼쪽만 보임: 왼쪽 차선([1]) 활성 AND 오른쪽 차선([2]) 비활성.
    오른쪽만 보일 때는 제외한다 — 오른쪽 끝 차선에서 오른쪽으로 밀면
    연석 쪽으로 가기 때문이다.
  - 제일 왼쪽: 왼쪽 차선 활성 AND 먼왼쪽 차선([0]) 비활성. 왼쪽에 차선이
    더 있으면 [0]이 보인다. 먼 차선은 검출이 약해서 0.35/0.25 기준을 쓴다.
  게이트 조건이 1초 유지돼야 켜지고, 상실 후 2초 유지돼야 꺼진다
  (차선 확률 출렁임에 보정이 껐다 켜졌다 하지 않게).

  모델 데이터가 불완전하거나 파라미터 읽기에 실패하면 0.0을 반환해
  controlsd가 죽지 않는다.
  """

  def __init__(self, params):
    self.params = params
    self.left_active = False
    self.right_active = False
    self.far_left_active = False
    self.last_correction = 0.0
    self._lane_change_holdoff_until = 0.0
    self._prev_kappa = 0.0
    self._prev_time = None
    # 게이트 상태: 조건 유지 시작 시각 / 조건 상실 시작 시각.
    self._gate_on = False
    self._gate_cond_since = None
    self._gate_lost_since = None
    # 백킹 파일 폴백 상태: 키 미등록(구 빌드) 시 typed getter가 예외를
    # 던지므로, 한 번 실패하면 등록 안 됨으로 기억하고 파일만 본다
    # (매 사이클 예외를 던지는 비용 방지). 파일은 1초마다 다시 읽는다.
    # 리빌드 후에는 controlsd가 재시작되며 상태가 초기화된다.
    self._key_registered = None
    self._file_offset_cm = None
    self._file_reread_until = 0.0

  def _read_offset_file_cm(self):
    """백킹 파일(/data/params/d/CameraOffset)을 직접 읽어 cm 값을 반환한다.

    웹 서버는 미등록 키도 파일에 직접 쓰므로(services/params.py), 재빌드
    전에도 이 폴백으로 오프셋 값을 읽을 수 있다. 1초마다 다시 읽는다
    (매 사이클 파일 읽기는 금지). 없거나 읽을 수 없으면 마지막으로 읽은
    값을 유지하고, 한 번도 못 읽었으면 None.
    carrot_param_cache.py의 _read_param_file과 동일한 경로 검증
    (경로 이탈 방지).
    """
    now = time.monotonic()
    if now < self._file_reread_until:
      return self._file_offset_cm
    self._file_reread_until = now + CAMERA_OFFSET_FILE_REREAD_SEC
    try:
      key = "CameraOffset"
      param_dir = os.path.abspath(self.params.get_param_path())
      param_path = os.path.abspath(self.params.get_param_path(key))
      if os.path.normcase(os.path.dirname(param_path)) != os.path.normcase(param_dir):
        return self._file_offset_cm
      with open(param_path, "rb") as f:
        raw = f.read().decode("utf-8", errors="replace").strip()
      self._file_offset_cm = float(raw)
    except Exception:
      pass
    return self._file_offset_cm

  def _read_settings(self):
    """오프셋(m)을 읽는다. 양수 = 우측 이동.

    get(..., return_default=True)을 쓴다. 키 미등록(구 빌드) 시
    UnknownKeyName이 던져지므로, 그때는 백킹 파일을 직접 읽는다.
    파일이 없거나 읽기에 실패하면 0.0 (꺼짐). ±50cm로 자른다.
    """
    raw = None
    if self._key_registered is not False:
      try:
        raw = self.params.get("CameraOffset", return_default=True)
        self._key_registered = True
      except Exception:
        self._key_registered = False
    if self._key_registered is False:
      raw = self._read_offset_file_cm()
    if raw is None:
      return CAMERA_OFFSET_DEFAULT_M
    try:
      offset_m = float(raw) * 0.01
    except (TypeError, ValueError):
      return CAMERA_OFFSET_DEFAULT_M
    if not math.isfinite(offset_m):
      return CAMERA_OFFSET_DEFAULT_M
    return float(np.clip(offset_m, -CAMERA_OFFSET_MAX_M, CAMERA_OFFSET_MAX_M))

  def update(self, md, v_ego, lane_changing):
    """모델 곡률에 더할 곡률 보정(1/m)을 계산한다. 양수 = 우회전."""
    kappa = self._update_impl(md, v_ego, lane_changing)
    kappa = self._slew_limit(kappa)
    self.last_correction = kappa
    return kappa

  def _slew_limit(self, kappa):
    """보정 곡률의 변화율을 제한한다 (급조향 방지). 클램프와 동일."""
    now = time.monotonic()
    if self._prev_time is None or (now - self._prev_time) > 0.5:
      self._prev_time = now
      self._prev_kappa = 0.0
      return 0.0
    dt = min(max(now - self._prev_time, 0.0), 0.1)
    self._prev_time = now
    max_delta = CAMERA_OFFSET_SLEW_RATE * dt
    self._prev_kappa += float(np.clip(kappa - self._prev_kappa, -max_delta, max_delta))
    return self._prev_kappa

  def _reset_gate(self):
    self._gate_on = False
    self._gate_cond_since = None
    self._gate_lost_since = None

  def _update_impl(self, md, v_ego, lane_changing):
    """update()의 본체. 보정 곡률만 계산한다."""
    self.last_correction = 0.0
    offset_m = self._read_settings()
    if offset_m == 0.0:
      self.left_active = False
      self.right_active = False
      self.far_left_active = False
      self._reset_gate()
      return 0.0
    # 차선변경 중에는 선을 넘는 것이 의도된 동작이므로 보정하지 않는다.
    # 종료 후 홀드오프(2초) 동안도 보정하지 않는다 (클램프와 동일 이유).
    now = time.monotonic()
    if lane_changing:
      self.left_active = False
      self.right_active = False
      self.far_left_active = False
      self._reset_gate()
      self._lane_change_holdoff_until = now + CAMERA_OFFSET_LANE_CHANGE_HOLDOFF_SEC
      return 0.0
    if now < self._lane_change_holdoff_until:
      self._reset_gate()
      return 0.0
    # 저속 페이드인: 5km/h 이하 0, 10km/h 이상 100% (클램프와 동일 구간)
    strength = float(np.interp(v_ego * 3.6, [5.0, 10.0], [0.0, 1.0]))
    if strength <= 0.0:
      self._reset_gate()
      return 0.0
    try:
      return self._compute(md, v_ego, offset_m, strength)
    except Exception:
      self._reset_gate()
      return 0.0

  def _compute(self, md, v_ego, offset_m, strength):
    # 오프셋 보정은 차선 위치가 아니라 확률만 본다. 확률 4개가 없으면
    # 게이트를 판단할 수 없으므로 보정하지 않는다.
    if len(md.laneLineProbs) < 4:
      return 0.0

    # 확률 히스테리시스: 인접 차선은 0.5 켜짐 / 0.3 꺼짐 (클램프와 동일),
    # 먼 차선은 검출이 약해서 0.35 켜짐 / 0.25 꺼짐.
    l_prob = float(md.laneLineProbs[1])
    r_prob = float(md.laneLineProbs[2])
    f_prob = float(md.laneLineProbs[0])
    if l_prob > CAMERA_OFFSET_PROB_ON:
      self.left_active = True
    elif l_prob < CAMERA_OFFSET_PROB_OFF:
      self.left_active = False
    if r_prob > CAMERA_OFFSET_PROB_ON:
      self.right_active = True
    elif r_prob < CAMERA_OFFSET_PROB_OFF:
      self.right_active = False
    if f_prob > CAMERA_OFFSET_FAR_PROB_ON:
      self.far_left_active = True
    elif f_prob < CAMERA_OFFSET_FAR_PROB_OFF:
      self.far_left_active = False

    # 게이트 조건: (왼쪽 차선만 보임) 또는 (제일 왼쪽 차선).
    left_only = self.left_active and not self.right_active
    leftmost = self.left_active and not self.far_left_active
    cond = left_only or leftmost
    # 디바운스: 조건이 1초 유지돼야 켜지고, 상실 후 2초 유지돼야 꺼진다.
    # 차선 확률이 출렁여도 보정이 껐다 켜졌다 하지 않게 한다.
    now = time.monotonic()
    if cond:
      if self._gate_cond_since is None:
        self._gate_cond_since = now
      self._gate_lost_since = None
      if now - self._gate_cond_since >= CAMERA_OFFSET_GATE_ON_SEC:
        self._gate_on = True
    else:
      if self._gate_lost_since is None:
        self._gate_lost_since = now
      self._gate_cond_since = None
      if now - self._gate_lost_since >= CAMERA_OFFSET_GATE_OFF_HOLD_SEC:
        self._gate_on = False
    if not self._gate_on:
      return 0.0

    # 룩어헤드 거리: 1초 앞 지점 (5~30m). 클램프·중앙 유지와 동일.
    lookahead = float(np.clip(v_ego * CAMERA_OFFSET_LOOKAHEAD_TIME,
                              CAMERA_OFFSET_LOOKAHEAD_MIN, CAMERA_OFFSET_LOOKAHEAD_MAX))
    # 곡률 변환 (pure pursuit): κ = 2·offset / L². 양의 오프셋(우측 이동)은
    # 양의 곡률(우회전)이다. L = v×1초이므로 보정이 만드는 횡가속은
    # 2·offset으로 속도와 무관하다 (30cm → 0.6 m/s²). 상한은 클램프와 동일.
    kappa = 2.0 * offset_m * strength / (lookahead ** 2)
    max_kappa = min(CAMERA_OFFSET_MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2,
                    CAMERA_OFFSET_MAX_CURVATURE)
    kappa = float(np.clip(kappa, -max_kappa, max_kappa))
    self.last_correction = kappa
    return kappa


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
    v_max = np.max(speeds)
  else:
    v_now = 0.0
    a_now = 0.0
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
    v_max = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop, v_now, v_max #v_target #v_now

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)


# 진입 전용 선행 조향 (LatEntryLead). EV4 커브·램프 진입 11건(2026-11-17/18/21 로그)에서
# 실제 회전이 목표 곡률을 0.26초 늦게 따라가 진입 동안 목표의 약 80%만 돌았다. 꾸준한
# 곡률 구간은 이미 안쪽(11~25cm)이라 조향비를 올리면 완만한 커브에서 파고든다.
# 그래서 |목표 곡률|이 커지는 동안에만 곡률 변화율 × 선행시간만큼 미리 더 꺾는다.
ENTRY_LEAD_RATE_TAU = 0.15     # s, 곡률 변화율 저역 필터 (모델 20Hz 계단을 평탄화)
ENTRY_LEAD_MAX_FRACTION = 0.35  # 선행량 상한: 현재 |목표 곡률|의 35%
ENTRY_LEAD_MAX_CURVATURE = 0.01  # 1/m, 절대 상한
ENTRY_LEAD_SPEED_BP = (8.0, 12.0)  # m/s, 이 구간에서 0 -> 100% 페이드인
ENTRY_LEAD_MAX_SECONDS = 0.5
ENTRY_LEAD_PARAM_PERIOD = 50   # 프레임마다 파라미터 재읽기 (0.5초)


class EntryLeadCorrector:
  """커브 진입(목표 곡률 크기가 커지는 중)에만 선행 곡률을 낸다. 양수/음수 부호는 목표 곡률과 같다.

  LatEntryLead 파라미터: 1/100초 단위 선행시간, 0이면 꺼짐(기본). 꾸준한 곡률과
  탈출(크기가 줄어드는 중)에는 0이며, 상한은 현재 목표 곡률의 일정 비율로 묶어
  직선에서 출렁이지 않게 한다. 결과는 LaC에 넘기는 곡률에만 더하고 목표 곡률
  상태(평활·한도·로그)는 바꾸지 않는다.
  """

  def __init__(self, params, dt=DT_CTRL):
    self.params = params
    self.dt = dt
    self.lead_time = 0.0
    self._frame = 0
    self.reset()

  def reset(self):
    self._prev = None
    self._rate = 0.0
    self.last_lead = 0.0

  def _read_lead_time(self):
    try:
      raw = self.params.get("LatEntryLead", return_default=True)
      value = float(raw) * 0.01 if raw is not None else 0.0
    except Exception:
      value = 0.0
    if not math.isfinite(value):
      value = 0.0
    return float(np.clip(value, 0.0, ENTRY_LEAD_MAX_SECONDS))

  def update(self, desired_curvature, v_ego, active, suppress=False):
    if self._frame % ENTRY_LEAD_PARAM_PERIOD == 0:
      self.lead_time = self._read_lead_time()
    self._frame += 1

    k = float(desired_curvature)
    if not active or not math.isfinite(k) or not math.isfinite(v_ego):
      self.reset()
      return 0.0
    if self._prev is None:
      self._prev = k
    raw_rate = (k - self._prev) / self.dt
    self._prev = k
    alpha = self.dt / (ENTRY_LEAD_RATE_TAU + self.dt)
    self._rate += alpha * (raw_rate - self._rate)

    # 필터된 변화율과 이번 프레임 변화율이 모두 '커지는 중'일 때만 (정점 직후 필터 지연으로 더 꺾지 않게)
    if suppress or self.lead_time <= 0.0 or self._rate * k <= 0.0 or raw_rate * k <= 0.0:
      self.last_lead = 0.0
      return 0.0
    strength = float(np.interp(v_ego, ENTRY_LEAD_SPEED_BP, [0.0, 1.0]))
    limit = min(ENTRY_LEAD_MAX_FRACTION * abs(k), ENTRY_LEAD_MAX_CURVATURE)
    self.last_lead = float(np.clip(self.lead_time * self._rate * strength, -limit, limit))
    return self.last_lead
