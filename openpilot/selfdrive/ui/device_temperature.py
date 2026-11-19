"""Display-only comma CPU/GPU hotspot temperature from fresh device telemetry."""
import math


def comma_temperature_badge(sm, now):
  if not sm.alive['deviceState'] or not sm.valid['deviceState']:
    return None
  age = now - sm.recv_time['deviceState']
  if not 0 <= age <= 5.0:
    return None
  state = sm['deviceState']
  readings = [float(t) for t in (*state.cpuTempC, *state.gpuTempC)
              if math.isfinite(t) and 0 < t <= 150]
  if not readings:
    return None
  temperature = max(readings)
  # Same 50/65/80/90 degree green/yellow/orange/red gradient as the Jetson badge.
  t = max(50., min(90., temperature))
  red = round(255 * min(1., (t - 50) / 15))
  green = round(255 - 6 * max(0., t - 65)) if t <= 80 else round(165 * (90 - t) / 10)
  return f'C {temperature:.1f}°C', (red, green, 0)
