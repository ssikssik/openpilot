from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.controls.lib import drive_helpers as dh
from openpilot.selfdrive.controls.tests.test_lane_clamp import Settings, model


@pytest.fixture
def setup(monkeypatch):
  clock = NS(now=10.)
  monkeypatch.setattr(dh, 'time', NS(monotonic=lambda: clock.now))
  settings = Settings()
  return dh.LaneCenterCorrector(settings), settings, clock


@pytest.mark.parametrize('offset', [-0.4, 0.4])
def test_direction_and_bounded_slew(setup, offset):
  center, _, clock = setup
  md = model(offset)
  assert center.update(md, 10., False) == 0
  clock.now += .05
  assert center.update(md, 10., False) == pytest.approx(-.0005 if offset > 0 else .0005)
  for _ in range(100):
    clock.now += .05
    result = center.update(md, 10., False)
  assert result == pytest.approx(-2 * offset / 100)


def test_threshold_hysteresis(setup):
  center, _, _ = setup
  assert center._update_impl(model(.19), 10., False) == 0
  assert center._update_impl(model(.21), 10., False) < 0
  assert center._update_impl(model(.15), 10., False) < 0
  assert center._update_impl(model(.09), 10., False) == 0


@pytest.mark.parametrize('guard', ['disabled', 'gain_zero', 'lane_change', 'slow', 'missing', 'probability', 'nan_probability', 'nan_path', 'nan_speed'])
def test_guard_clears_existing_correction_immediately(setup, guard):
  center, settings, clock = setup
  md = model(.4)
  center.update(md, 10., False)
  clock.now += .1
  assert center.update(md, 10., False) < 0
  if guard == 'disabled': settings.values['LaneCenterEnable'] = False
  if guard == 'gain_zero': settings.values['LaneCenterGain'] = 0
  if guard == 'missing': md = NS()
  if guard == 'probability': md.laneLineProbs = [0., .1, .9, 0.]
  if guard == 'nan_probability': md.laneLineProbs = [0., float('nan'), .9, 0.]
  if guard == 'nan_path': md.position.y = [float('nan'), float('nan')]
  speed = float('nan') if guard == 'nan_speed' else 1. if guard == 'slow' else 10.
  assert center.update(md, speed, guard == 'lane_change') == 0
  assert center.last_correction == 0


def test_lane_change_holdoff_and_probability_hysteresis(setup):
  center, _, clock = setup
  center.update(model(.4), 10., True)
  clock.now += 1.9
  assert center.update(model(.4), 10., False) == 0
  clock.now += .2
  assert center.update(model(.4), 10., False) == 0  # fresh slew starts from zero
  clock.now += .1
  assert center.update(model(.4, (0., .4, .4, 0.)), 10., False) < 0


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
    cc_center, _ = step_center()
    cc_base, _ = step_base()
  assert cc_center.latActive and cc_base.latActive
  assert cc_center.actuators.curvature < cc_base.actuators.curvature
  assert abs(cc_center.actuators.curvature) <= dh.MAX_CURVATURE
  assert abs(cc_center.actuators.curvature) * 100 <= 4.5 + 1e-6
