import math

import pytest

from openpilot.selfdrive.controls.lib import drive_helpers as dh


class Params(dict):
  def get(self, key, return_default=False):
    return super().get(key, 0 if return_default else None)


def ramp(lead, start, end, frames, v=12.5, suppress=False, corrector=None):
  """Feed a linear curvature ramp at 100 Hz and return (corrector, leads)."""
  c = corrector or dh.EntryLeadCorrector(Params(LatEntryLead=lead))
  out = []
  for i in range(frames):
    k = start + (end - start) * i / max(frames - 1, 1)
    out.append(c.update(k, v, True, suppress))
  return c, out


def test_default_is_off():
  _, leads = ramp(None, 0.0, 0.02, 300)
  assert all(x == 0.0 for x in leads)


@pytest.mark.parametrize('sign', [1, -1])
def test_entry_leads_in_curvature_direction(sign):
  # Logged EV4 ramp entry: ~0.0012 -> 0.024 1/m over about 6 s at 45 km/h.
  c, leads = ramp(25, sign * 0.0012, sign * 0.024, 600)
  rate = 0.0228 / 6.0
  assert leads[-1] == pytest.approx(sign * 0.25 * rate, rel=0.05)
  assert all(x * sign >= 0 for x in leads)


def test_steady_and_exit_are_untouched():
  c, _ = ramp(35, 0.0, 0.02, 400)
  _, steady = ramp(35, 0.02, 0.02, 200, corrector=c)
  assert steady[-1] == pytest.approx(0.0, abs=1e-7)  # filtered rate decays to ~0 in a steady curve
  _, exit_ = ramp(35, 0.02, 0.0, 300, corrector=c)
  assert all(x == 0.0 for x in exit_)


def test_lead_is_bounded_by_current_curvature():
  # A very fast step-like rise must not exceed 35 % of the current target.
  c = dh.EntryLeadCorrector(Params(LatEntryLead=50))
  for k in (0.0, 0.002, 0.006, 0.012):
    lead = c.update(k, 20.0, True)
    assert abs(lead) <= dh.ENTRY_LEAD_MAX_FRACTION * abs(k) + 1e-12
  assert abs(c.update(0.05, 20.0, True)) <= dh.ENTRY_LEAD_MAX_CURVATURE


def test_low_speed_fades_out():
  _, slow = ramp(35, 0.0, 0.02, 300, v=5.0)
  assert all(x == 0.0 for x in slow)
  _, mid = ramp(35, 0.0, 0.02, 300, v=10.0)
  _, fast = ramp(35, 0.0, 0.02, 300, v=15.0)
  assert 0 < mid[-1] < fast[-1]


def test_suppress_and_inactive_clear_immediately():
  c, leads = ramp(35, 0.0, 0.02, 300)
  assert leads[-1] > 0
  assert c.update(0.0201, 12.5, True, suppress=True) == 0.0
  assert c.update(0.0202, 12.5, False) == 0.0
  # after reset, the first active frame has no history and gives no kick
  assert c.update(0.03, 12.5, True) == 0.0


def test_bad_inputs_and_param_are_safe():
  c = dh.EntryLeadCorrector(Params(LatEntryLead='x'))
  assert c.update(0.01, 12.5, True) == 0.0 and c.lead_time == 0.0
  c = dh.EntryLeadCorrector(Params(LatEntryLead=999))
  c.update(0.0, 12.5, True)
  assert c.lead_time == dh.ENTRY_LEAD_MAX_SECONDS
  assert c.update(math.nan, 12.5, True) == 0.0
  assert c.update(0.01, math.inf, True) == 0.0
