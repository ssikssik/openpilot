"""Bounded stopping reference for a confirmed stationary radar lead."""
import math


def stopped_lead_profile(speed, accel, distance, lookahead):
  """Return velocity, acceleration, jerk and remaining duration, or no reference."""
  if not all(map(math.isfinite, (speed, accel, distance, lookahead))):
    return None
  if not (0.12 <= speed <= 8.0 and -2.0 <= accel <= 0.10 and distance > 0 and 0 < lookahead <= 1):
    return None

  # Quintic position with the measured initial state and zero terminal v/a.
  nominal_duration = 2.0 * distance / speed
  for scale in (1.0, 0.92, 1.08, 0.84, 1.16, 0.76, 1.24):
    duration = nominal_duration * scale
    if 0.45 <= duration <= 8.0:
      profile = _sample_profile(speed, min(accel, 0.0), distance, duration, lookahead)
      if profile is not None:
        return profile
  return None


def _sample_profile(speed, accel, distance, duration, lookahead):
  b = speed * duration
  c = accel * duration**2 / 2
  p = 10 * distance - 6 * b - 3 * c
  q = -15 * distance + 8 * b + 3 * c
  r = 6 * distance - 3 * b - c

  def sample(u):
    v = (b + u * (2 * c + u * (3 * p + u * (4 * q + u * 5 * r)))) / duration
    a = (2 * c + u * (6 * p + u * (12 * q + u * 20 * r))) / duration**2
    j = (6 * p + u * (24 * q + u * 60 * r)) / duration**3
    return v, a, j

  # Check the extrema of acceleration and jerk over the full profile.
  points = [0.0, 1.0]
  if abs(r) > 1e-9:
    points.append(-q / (5 * r))
    disc = (24 * q)**2 - 4 * 60 * r * 6 * p
    if disc >= 0:
      root = math.sqrt(disc)
      points.extend(((-24 * q - root) / (120 * r), (-24 * q + root) / (120 * r)))
  elif abs(q) > 1e-9:
    points.append(-p / (4 * q))
  for point in points:
    if 0 <= point <= 1:
      _, a, j = sample(point)
      if not (-2.0 <= a <= 1e-6 and abs(j) <= 1.5):
        return None

  v, a, j = sample(min(lookahead / duration, 1.0))
  return max(0.0, v), min(0.0, a), j if lookahead < duration else 0.0, duration
