from types import SimpleNamespace as NS

import numpy as np
import pytest

from opendbc.car.hyundai.values import CAR
from openpilot.cereal import log
from openpilot.selfdrive.controls.lib import drive_helpers as dh


class Settings:
  def __init__(self, **values):
    self.values = values

  def get(self, key, return_default=False):
    return self.values.get(key)


@pytest.fixture
def corrector(monkeypatch):
  clock = NS(now=10.0)
  monkeypatch.setattr(dh, 'time', NS(monotonic=lambda: clock.now))
  return dh.ClampSteerCorrector(Settings()), clock


def model(y, probs=(0.0, 0.9, 0.9, 0.0)):
  def line(offset):
    return {'x': [0.0, 40.0], 'y': [offset, offset]}
  return log.ModelDataV2.new_message(position=line(y), laneLineProbs=list(probs),
                                     laneLines=[line(x) for x in (-5, -1.7, 1.7, 5)],
                                     roadEdges=[line(-2), line(2)], roadEdgeStds=[0.2, 0.2])


@pytest.mark.parametrize('direction', [-1, 1])
def test_clamp_direction_slew_and_alert(corrector, direction):
  clamp, clock = corrector
  md = model(-direction * 1.0)
  assert clamp.update(md, 10.0, False) == 0.0
  clock.now += 0.05
  assert clamp.update(md, 10.0, False) == pytest.approx(direction * 0.0005)
  assert not clamp.alert_active
  clock.now += 0.1
  assert clamp.update(md, 10.0, False) == pytest.approx(direction * 0.0015)
  assert clamp.alert_active
  assert clamp.update(md, 10.0, True) == 0.0
  assert not clamp.alert_active
  clock.now += 1.99
  assert clamp.update(md, 10.0, False) == 0.0
  clock.now += 0.02
  assert direction * clamp.update(md, 10.0, False) > 0.0
  assert not clamp.alert_active


def test_deadband_and_edge_fallback(corrector):
  clamp, clock = corrector
  assert clamp.update(model(-0.5), 10.0, False) == 0.0  # 15 cm lane violation
  clock.now += 0.1
  assert clamp.update(model(-0.5), 10.0, False) == 0.0
  clock.now += 0.1
  assert clamp.update(model(-1.1, (0.0,) * 4), 10.0, False) > 0.0
  clamp.reset()
  assert clamp.update(model(-1.1), 1.0, False) == 0.0
  clamp.params.values['ClampEnable'] = False
  clock.now += 0.1
  assert clamp.update(model(-1.1), 10.0, False) == 0.0


def test_path_clamp_keeps_configured_distance_and_yields_for_lane_change():
  from openpilot.selfdrive.controls.lib.lane_planner_2 import LanePlanner

  planner = LanePlanner()
  planner.params = Settings(ClampMinDistLane=140)
  planner.ll_x = np.array([0.0, 20.0])
  planner.lll_y = np.array([-1.7, -1.7])
  planner.rll_y = np.array([1.7, 1.7])
  planner.l_prob = planner.r_prob = 0.9
  path = np.array([[0.0, -1.0, 0.0], [20.0, 1.0, 0.0]])
  assert planner.apply_min_distance_clamp(path.copy(), 10.0)[:, 1] == pytest.approx([-0.3, 0.3])
  planner.lane_change_multiplier = 0.0
  assert np.array_equal(planner.apply_min_distance_clamp(path.copy(), 10.0), path)


def test_lateral_limit_settings_preserve_existing_envelope_and_ev4_scale():
  defaults = (6.5, 4.5, 7.0)
  assert dh.read_lat_accel_limits(Settings()) == defaults
  assert dh.read_lat_accel_limits(Settings(LatAccelMaxLow=999, LatAccelMaxHigh=999, LatJerkMax=999)) == defaults
  low, high, jerk = dh.read_lat_accel_limits(Settings(LatAccelMaxLow=30, LatAccelMaxHigh=25, LatJerkMax=40))
  assert (low, high, jerk) == (3.0, 2.5, 4.0)
  v_ego = 20.0
  for fingerprint, scale in [('', 1.0), (CAR.KIA_EV4, 2.0)]:
    curvature, limited = dh.clip_curvature(v_ego, 0.02, 0.02, 0.0, car_fingerprint=fingerprint,
                                           max_lat_accel_low=low, max_lat_accel_high=high, max_lat_jerk=jerk)
    assert limited and curvature * v_ego ** 2 == pytest.approx(scale * low)
  curvature, _ = dh.clip_curvature(v_ego, 0.0, 0.02, 0.0, max_lat_jerk=jerk)
  assert curvature * v_ego ** 2 == pytest.approx(jerk * dh.DT_CTRL)
