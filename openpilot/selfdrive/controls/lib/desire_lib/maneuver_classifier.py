from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from .constants import BLINKER_LEFT, BLINKER_RIGHT

def classify_maneuver_type(blinker_state: int,
                           carstate,
                           side,                 # SideState
                           turn_desire_state: bool,
                           atc_type: str,
                           old_type: str,
                           atc_dist: float = 999.0):
  if blinker_state == 0:
    return "none"

  v_kph = carstate.vEgo * CV.MS_TO_KPH
  accel = carstate.aEgo

  score_turn = 0
  if v_kph < 30.0:
    score_turn += 1
  elif v_kph < 40.0 and accel < -1.0:
    score_turn += 1

  # 차로 없고 edge 여유도 없으면 turn 가산
  if v_kph < 40.0 and (not side.lane_available) and (not side.edge_available):
    score_turn += 1

  # 차선이 잘 안 보이면(교차로 등)
  if v_kph < 40.0 and side.lane_exist_count.counter < int(0.5 / DT_MDL):
    score_turn += 1

  if turn_desire_state:
    score_turn += 1

  if atc_type in ("turn left", "turn right"):
    score_turn += 2
  elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
    score_turn -= 2

  edge_far = side.dist_to_edge_far > 4.0
  # Urban 90-degree corners often keep road_edge <= 2.5 m at t=2 s until the car is already turning.
  # Once within 20 m of an ATC turn at turn speed in the outer lane, allow turn desire directly.
  atc_turn_imminent = (
    atc_type in ("turn left", "turn right")
    and 0.0 <= atc_dist <= 20.0
    and v_kph < 35.0
    and (not side.lane_available)
  )

  if score_turn >= 2:
    if edge_far or atc_turn_imminent:
      return "turn"
    return old_type
  return "lane_change"
