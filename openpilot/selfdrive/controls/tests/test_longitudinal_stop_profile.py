import math

import pytest

from openpilot.selfdrive.controls.lib.longitudinal_stop_profile import stopped_lead_profile


@pytest.mark.parametrize("speed, accel, distance", [(2.0, 0.0, 6.0), (5.0, -0.5, 12.0), (8.0, -1.0, 25.0), (0.5, 0.0, 0.8)])
def test_profile_is_bounded_braking_only(speed, accel, distance):
  profile = stopped_lead_profile(speed, accel, distance, 0.2)
  assert profile is not None
  v, a, j, duration = profile
  assert 0.0 <= v <= speed + 1e-6
  assert -2.0 <= a <= 0.0
  assert abs(j) <= 1.5
  assert 0.45 <= duration <= 8.0


@pytest.mark.parametrize("speed, accel, distance, lookahead", [
  (0.1, 0.0, 5.0, 0.2),   # too slow: existing stop logic owns it
  (9.0, 0.0, 30.0, 0.2),  # too fast for this profile
  (3.0, -2.5, 6.0, 0.2),  # already braking harder than the limit
  (3.0, 0.5, 6.0, 0.2),   # accelerating
  (3.0, 0.0, 0.0, 0.2),   # no room
  (3.0, 0.0, 6.0, 0.0),   # invalid lookahead
  (8.0, 0.0, 2.0, 0.2),   # would need more than 2 m/s^2
  (math.nan, 0.0, 6.0, 0.2),
  (3.0, 0.0, math.inf, 0.2),
])
def test_profile_refuses_out_of_envelope_inputs(speed, accel, distance, lookahead):
  assert stopped_lead_profile(speed, accel, distance, lookahead) is None


def test_lookahead_past_duration_reports_stop():
  v, a, j, duration = stopped_lead_profile(0.12, 0.0, 0.05, 1.0)
  assert duration < 1.0
  assert v == pytest.approx(0.0, abs=1e-9) and a == pytest.approx(0.0, abs=1e-9) and j == 0.0
