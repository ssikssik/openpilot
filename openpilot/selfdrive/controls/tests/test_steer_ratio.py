import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio
from openpilot.selfdrive.controls.lib.lateral_readiness import LateralStartupGate, lateral_vehicle_parameters


@pytest.mark.parametrize("invalid_rate", [-math.inf, math.nan, 0.0, 29.0, 201.0, math.inf])
def test_invalid_steer_ratio_rate_uses_full_live_ratio(invalid_rate):
  assert resolve_vehicle_model_steer_ratio(14.75, invalid_rate, 0.0) == pytest.approx(14.75)


@pytest.mark.parametrize(("rate", "expected"), [(30.0, 4.425), (80.0, 11.8), (100.0, 14.75), (200.0, 29.5)])
def test_valid_steer_ratio_rate_scales_live_ratio(rate, expected):
  assert resolve_vehicle_model_steer_ratio(14.75, rate, 0.0) == pytest.approx(expected)


def test_custom_steer_ratio_overrides_live_ratio_rate():
  assert resolve_vehicle_model_steer_ratio(14.75, 80.0, 150.0) == pytest.approx(15.0)


@pytest.mark.parametrize("invalid_custom", [-math.inf, math.nan, -10.0, 0.0, 10.0, math.inf])
def test_invalid_custom_ratio_uses_live_scaling(invalid_custom):
  assert resolve_vehicle_model_steer_ratio(14.75, 80.0, invalid_custom) == pytest.approx(11.8)


@pytest.mark.parametrize(("v_kph", "expected_scale"), [
  (0.0, 1.0),
  (49.9, 1.0),
  (50.0, 1.0),
  (75.0, 0.95),
  (100.0, 0.90),
  (130.0, 0.90),
])
def test_high_speed_steer_ratio_rate_scales_linearly_from_50_to_100_kph(v_kph, expected_scale):
  v_ego = v_kph / 3.6
  assert resolve_vehicle_model_steer_ratio(15.0, 100.0, 0.0, 90.0, v_ego) == pytest.approx(15.0 * expected_scale)
  assert resolve_vehicle_model_steer_ratio(15.0, 100.0, 160.0, 90.0, v_ego) == pytest.approx(16.0 * expected_scale)


@pytest.mark.parametrize("invalid_hs_rate", [-math.inf, math.nan, 0.0, 49.0, 151.0, math.inf])
def test_invalid_high_speed_steer_ratio_rate_falls_back_to_100(invalid_hs_rate):
  assert resolve_vehicle_model_steer_ratio(15.0, 100.0, 0.0, invalid_hs_rate, 100.0 / 3.6) == pytest.approx(15.0)


@pytest.mark.parametrize("is_vw_meb", [False, True])
def test_controlsd_applies_live_setting_changes_to_vehicle_model(is_vw_meb):
  # Execute the production state_control prefix through VM.update_params.
  # This covers the actual setting reads and wiring without hardware/IPC.
  path = Path(__file__).parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  controls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
  method = next(n for n in controls.body if isinstance(n, ast.FunctionDef) and n.name == "state_control")
  end = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Expr)
             and isinstance(n.value, ast.Call) and ast.unparse(n.value.func) == "self.VM.update_params")
  method.body = method.body[:end + 1]
  module = ast.Module(body=[method], type_ignores=[])
  namespace = {"resolve_vehicle_model_steer_ratio": resolve_vehicle_model_steer_ratio,
               "lateral_vehicle_parameters": lateral_vehicle_parameters}
  exec(compile(module, str(path), "exec"), namespace)
  settings = {"CustomSR": 0.0, "SteerRatioRate": 100.0, "SteerRatioHighSpeedRate": 100.0}
  live = SimpleNamespace(stiffnessFactor=0.69, steerRatio=14.97, valid=True, sensorValid=True,
                         posenetValid=True, angleOffsetDeg=0.0, roll=0.0)
  car_state = SimpleNamespace(vEgo=0.0)
  class Messages(dict):
    seen = {'liveParameters': True}

    def all_checks(self, _services):
      return True
  updates = []
  instance = SimpleNamespace(is_vw_meb=is_vw_meb,
                             lateral_startup=LateralStartupGate(ready=True),
                             CP=SimpleNamespace(steerRatio=15.0),
                             sm=Messages(carState=car_state, liveParameters=live),
                             params=SimpleNamespace(get_float=lambda key: settings[key]),
                             VM=SimpleNamespace(update_geometry=lambda _r: None,
                                                update_params=lambda stiff, sr: updates.append((stiff, sr))))
  for custom, rate, hs_rate, v_kph, learned, expected in [
    (0, 100, 100, 0.0, 14.97, 14.97),
    (159, 30, 100, 0.0, 14.97, 15.9),
    (200, 30, 100, 0.0, 14.97, 20.0),
    (0, 30, 100, 0.0, 14.97, 4.491),
    (0, 100, 100, 0.0, 15.1, 15.1),
    (0, 0, 100, 0.0, 15.1, 15.1),
    (0, 100, 90, 50.0, 15.0, 15.0),
    (0, 100, 90, 75.0, 15.0, 14.25),
    (0, 100, 90, 100.0, 15.0, 13.5),
  ]:
    settings.update(CustomSR=custom, SteerRatioRate=rate, SteerRatioHighSpeedRate=hs_rate)
    car_state.vEgo = v_kph / 3.6
    live.steerRatio = learned
    namespace["state_control"](instance)
    assert updates[-1] == pytest.approx((0.69, expected))
