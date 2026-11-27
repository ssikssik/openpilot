import math


DEFAULT_STEER_RATIO_RATE = 100.0
MIN_STEER_RATIO_RATE = 30.0
MAX_STEER_RATIO_RATE = 200.0
MIN_HIGH_SPEED_STEER_RATIO_RATE = 50.0
MAX_HIGH_SPEED_STEER_RATIO_RATE = 150.0
HIGH_SPEED_SR_START_KPH = 40.0
HIGH_SPEED_SR_END_KPH = 150.0
MIN_STEER_RATIO = 0.1


def resolve_vehicle_model_steer_ratio(live_steer_ratio: float, steer_ratio_rate: float,
                                      custom_steer_ratio: float,
                                      high_speed_rate: float = DEFAULT_STEER_RATIO_RATE,
                                      v_ego: float = 0.0) -> float:
  live_steer_ratio = max(live_steer_ratio, MIN_STEER_RATIO)
  custom_steer_ratio *= 0.1
  if math.isfinite(custom_steer_ratio) and custom_steer_ratio > 1.0:
    base_sr = custom_steer_ratio
  else:
    if not math.isfinite(steer_ratio_rate) or not MIN_STEER_RATIO_RATE <= steer_ratio_rate <= MAX_STEER_RATIO_RATE:
      steer_ratio_rate = DEFAULT_STEER_RATIO_RATE
    base_sr = max(live_steer_ratio * steer_ratio_rate * 0.01, MIN_STEER_RATIO)

  if not math.isfinite(high_speed_rate) or not MIN_HIGH_SPEED_STEER_RATIO_RATE <= high_speed_rate <= MAX_HIGH_SPEED_STEER_RATIO_RATE:
    high_speed_rate = DEFAULT_STEER_RATIO_RATE

  if high_speed_rate != DEFAULT_STEER_RATIO_RATE and math.isfinite(v_ego):
    v_kph = abs(v_ego) * 3.6
    if v_kph > HIGH_SPEED_SR_START_KPH:
      alpha = min((v_kph - HIGH_SPEED_SR_START_KPH) / (HIGH_SPEED_SR_END_KPH - HIGH_SPEED_SR_START_KPH), 1.0)
      speed_scale = 1.0 + alpha * (high_speed_rate * 0.01 - 1.0)
      return max(base_sr * speed_scale, MIN_STEER_RATIO)

  return base_sr
