from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.controls.lib import drive_helpers as dh
from openpilot.selfdrive.controls.tests.test_lane_clamp import Settings, model

STEP = .05
SLEW_STEP = dh.LANE_CENTER_SLEW_RATE * STEP


@pytest.fixture
def setup(monkeypatch):
  clock = NS(now=10.)
  monkeypatch.setattr(dh, 'time', NS(monotonic=lambda: clock.now))
  settings = Settings()
  return dh.LaneCenterCorrector(settings), settings, clock


def step(center, clock, md, speed=10., lane_changing=False):
  """One fresh model frame: the entry hold counts model time, not calls."""
  clock.now += STEP
  md.timestampEof = round(clock.now * 1e9)
  return center.update(md, speed, lane_changing)


def engage(center, clock, md, frames=40):
  for _ in range(frames):
    result = step(center, clock, md)
  assert result != 0
  return result


@pytest.mark.parametrize('offset', [-0.4, 0.4])
def test_waits_for_continuous_offset_then_eases_in_with_bounded_slew(setup, offset):
  center, _, clock = setup
  md = model(offset)
  hold_frames = round(dh.LANE_CENTER_ENTRY_HOLD_NS / 1e9 / STEP)
  results = [step(center, clock, md) for _ in range(hold_frames)]
  assert results == [0] * hold_frames
  previous = 0.
  for _ in range(100):
    result = step(center, clock, md)
    assert abs(result - previous) <= SLEW_STEP + 1e-12
    previous = result
  assert result == pytest.approx(-2 * offset / 100)


def test_threshold_hysteresis(setup):
  center, _, clock = setup
  for _ in range(20):
    assert step(center, clock, model(.19)) == 0
  engage(center, clock, model(.21))
  step(center, clock, model(.15))
  assert center._correction_active
  step(center, clock, model(.09))
  assert not center._correction_active


@pytest.mark.parametrize('guard', ['disabled', 'gain_zero', 'lane_change', 'slow', 'missing', 'probability', 'nan_probability', 'nan_path', 'nan_speed'])
def test_guard_releases_existing_correction_at_slew_rate(setup, guard):
  center, settings, clock = setup
  md = model(.4)
  baseline = engage(center, clock, md)
  if guard == 'disabled': settings.values['LaneCenterEnable'] = False
  if guard == 'gain_zero': settings.values['LaneCenterGain'] = 0
  if guard == 'missing': md = NS()
  if guard == 'probability': md.laneLineProbs = [0., .1, .9, 0.]
  if guard == 'nan_probability': md.laneLineProbs = [0., float('nan'), .9, 0.]
  if guard == 'nan_path': md.position.y = [float('nan'), float('nan')]
  speed = float('nan') if guard == 'nan_speed' else 1. if guard == 'slow' else 10.
  first = step(center, clock, md, speed, guard == 'lane_change')
  assert first == pytest.approx(baseline + SLEW_STEP)
  for _ in range(40):
    result = step(center, clock, md, speed, guard == 'lane_change')
  assert result == 0
  assert center.last_correction == 0


def test_lane_change_holdoff_and_probability_hysteresis(setup):
  center, _, clock = setup
  md = model(.4)
  step(center, clock, md, lane_changing=True)
  holdoff_frames = round(dh.LANE_CENTER_LANE_CHANGE_HOLDOFF_SEC / STEP) - 1
  for _ in range(holdoff_frames):
    assert step(center, clock, md) == 0
  # A lane change forgets lane-line confidence: 0.4 alone cannot turn it back on.
  for _ in range(20):
    assert step(center, clock, model(.4, (0., .4, .4, 0.))) == 0
  engage(center, clock, md)
  # Once on, 0.4 stays above the 0.3 off threshold.
  assert step(center, clock, model(.4, (0., .4, .4, 0.))) != 0


@pytest.mark.parametrize('speed', [2., 10., 30.])
def test_correction_caps(setup, speed):
  center, settings, _ = setup
  settings.values['LaneCenterGain'] = 20
  result = center._update_impl(model(100.), speed, False)
  assert abs(result) <= min(.02, 1.5 / speed ** 2) + 1e-12


def test_live_settings_and_missing_defaults(setup):
  center, settings, _ = setup
  assert center._read_settings() == (True, .2, 1.)
  settings.values.update(LaneCenterEnable='0', LaneCenterThreshold=30, LaneCenterGain=5)
  assert center._read_settings() == (False, .3, .5)


def test_real_control_loop_adds_centering_before_existing_limits(monkeypatch):
  from openpilot.selfdrive.controls.tests.test_lateral_startup_control import control_fixture
  clock = NS(now=10.)
  monkeypatch.setattr(dh, 'time', NS(monotonic=lambda: clock.now))
  centered, step_center = control_fixture()
  baseline, step_base = control_fixture()
  for control in (centered, baseline):
    control.sm['modelV2'] = model(.4)
    control.sm['carState'].vEgo = 10.
  original = baseline.params.get
  baseline.params.get = lambda key, **kw: False if key == 'LaneCenterEnable' else original(key, **kw)
  for _ in range(150):
    clock.now += .01
    for control in (centered, baseline):
      control.sm['modelV2'].timestampEof = round(clock.now * 1e9)
    cc_center, _ = step_center()
    cc_base, _ = step_base()
  assert cc_center.latActive and cc_base.latActive
  assert cc_center.actuators.curvature < cc_base.actuators.curvature
  assert abs(cc_center.actuators.curvature) <= dh.MAX_CURVATURE
  assert abs(cc_center.actuators.curvature) * 100 <= 4.5 + 1e-6
