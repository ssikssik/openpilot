"""Exercise the roadside exception without an MPC, device, or CAN connection."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lane_avoidance import LaneAvoidanceGuard


EGO_SPEED = 12.0
STEP = 0.05
START_TIME = 100.0
X = np.linspace(0.0, 80.0, 33)


def line(y):
  return SimpleNamespace(x=X.copy(), y=np.broadcast_to(y, X.shape).copy())


def model(side=1):
  return SimpleNamespace(
    position=line(-0.5 * side),
    laneLines=[line(-5.0), line(-1.9), line(1.9), line(5.0)],
    laneLineProbs=[0.0, 0.99, 0.99, 0.0],
    laneLineStds=[1.0, 0.10, 0.10, 1.0],
    roadEdges=[line(-3.0), line(3.0)],
    roadEdgeStds=[0.10, 0.10],
    velocity=SimpleNamespace(x=np.full(X.shape, EGO_SPEED)),
  )


def point(track_id, source, distance, model_y, *, speed=0.0, lateral_speed=0.0, measured=True):
  return SimpleNamespace(track_id=track_id, source=source, d_rel=distance, y_rel=-model_y,
                         v_rel=speed - EGO_SPEED, v_lead=speed, yv_rel=lateral_speed, measured=measured,
                         radar_track_state=2)


def pair(distance, side=1, *, speed=0.0):
  return (point(1, 'frontRadar', distance, side * 2.15, speed=speed),
          point(2, 'corner235', distance + 0.25, side * 2.20, speed=speed))


def step(guard, md, points, frame, *, radar_time=None, speed=EGO_SPEED, curvature=0.0, **kwargs):
  now = START_TIME + frame * STEP
  return guard.update(now, md, points, now if radar_time is None else radar_time, speed, curvature, **kwargs)


def drive(guard, md, side=1, *, start_distance=18.0, frames=16, object_speed=0.0):
  return [step(guard, md, pair(start_distance - (EGO_SPEED - object_speed) * STEP * n, side,
                               speed=object_speed), n)
          for n in range(frames)]


@pytest.mark.parametrize('fingerprint', ['KIA_EV4', 'KIA_K8_HEV_1ST_GEN'])
@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('object_speed', [0.0, 1.5])
def test_stationary_and_creeping_roadside_vehicles_are_mirrored(side, object_speed, fingerprint):
  results = drive(LaneAvoidanceGuard.for_vehicle(fingerprint), model(side), side, object_speed=object_speed)
  assert all(r.status == 'confirming' and r.strength == 0.0 for r in results[:6])
  assert results[-1].status == 'tracked'
  assert results[-1].side == side
  assert results[-1].strength == 1.0
  assert all(a.strength <= b.strength for a, b in zip(results, results[1:]))


@pytest.mark.parametrize('fingerprint', ['KIA_EV4', 'KIA_K8_HEV_1ST_GEN'])
@pytest.mark.parametrize('scenario', ['stopped_queue', 'front_only', 'corner_only', 'scc_only', 'unmeasured', 'centered'])
def test_rejects_unsupported_or_non_avoiding_paths(scenario, fingerprint):
  guard, md = LaneAvoidanceGuard.for_vehicle(fingerprint), model()
  if scenario == 'stopped_queue':
    md.velocity.x[4:] = 0.0
  if scenario == 'centered':
    md.position = line(0.0)
  for n in range(16):
    points = pair(18.0 - EGO_SPEED * n * STEP)
    if scenario == 'front_only':
      points = points[:1]
    elif scenario == 'corner_only':
      points = points[1:]
    elif scenario == 'scc_only':
      points = (point(1, 'scc', points[0].d_rel, 2.15), points[1])
    elif scenario == 'unmeasured':
      points[0].measured = False
    assert step(guard, md, points, n).strength == 0.0


def test_repeated_radar_sample_cannot_build_confirmation():
  guard, md = LaneAvoidanceGuard(), model()
  for n in range(8):
    sample_at = START_TIME if n < 4 else START_TIME + 4 * STEP
    assert step(guard, md, pair(18.0 - EGO_SPEED * STEP * n), n, radar_time=sample_at).strength == 0.0
  assert guard.result.status == 'confirming'


def test_near_track_loss_holds_then_returns_smoothly_after_passing():
  guard, md = LaneAvoidanceGuard(), model()
  assert drive(guard, md, start_distance=14.0)[-1].strength == 1.0
  results = [step(guard, md, (), n) for n in range(16, 62)]
  assert results[0].status == 'passing' and results[0].strength == 1.0
  release = [r.strength for r in results if r.status == 'releasing']
  assert len(release) >= 10 and release[0] < 1.0
  assert all(0.0 < a - b <= 0.08 for a, b in zip(release, release[1:]))
  assert results[-1].strength == 0.0


@pytest.mark.parametrize('stale,status', [(False, 'inactive'), (True, 'stale')])
def test_far_or_stale_track_loss_does_not_hold(stale, status):
  guard, md = LaneAvoidanceGuard(), model()
  assert drive(guard, md)[-1].strength == 1.0
  result = step(guard, md, (), 16, radar_time=START_TIME if stale else None)
  assert result.status == status
  assert result.strength == 0.0


@pytest.mark.parametrize('scenario,status', [
  ('driver', 'inhibited'), ('curve', 'speedOrCurve'), ('blindspot', 'traffic'), ('oncoming', 'traffic'),
])
def test_driver_sharp_curve_and_traffic_cancel_existing_exception(scenario, status):
  guard, md = LaneAvoidanceGuard(), model()
  assert drive(guard, md)[-1].strength == 1.0
  points = pair(18.0 - EGO_SPEED * 16 * STEP)
  if scenario == 'oncoming':
    points += (point(3, 'frontRadar', 16.0, -1.75, speed=-EGO_SPEED),)
  result = step(guard, md, points, 16, allowed=scenario != 'driver',
                left_occupied=scenario == 'blindspot', curvature=0.02 if scenario == 'curve' else 0.0)
  assert result.status == status
  assert result.strength == 0.0


@pytest.mark.parametrize('scenario', ['narrow', 'split', 'edge', 'weak_line', 'crossed', 'nonmonotonic', 'short'])
def test_rejects_unreliable_geometry_or_paths_outside_vehicle_envelope(scenario):
  guard, md = LaneAvoidanceGuard(), model()
  if scenario == 'narrow':
    md.laneLines[2] = line(1.05)
  elif scenario == 'split':
    md.laneLines[2] = line(np.interp(X, (0.0, 35.0, 80.0), (1.8, 2.6, 2.6)))
  elif scenario == 'edge':
    md.roadEdges[0] = line(-1.45)
  elif scenario == 'weak_line':
    md.laneLineProbs[1] = 0.65
  elif scenario == 'crossed':
    md.laneLines[1] = line(2.0)
  elif scenario == 'nonmonotonic':
    md.position.x[7] = md.position.x[6]
  elif scenario == 'short':
    md.position.x *= 0.15
  assert all(r.strength == 0.0 for r in drive(guard, md))


def test_uncertain_edge_requires_room_for_its_inward_uncertainty_margin():
  md = model()
  md.roadEdgeStds = [1.0, 0.1]
  assert drive(LaneAvoidanceGuard(), md)[-1].strength == 0.0
  md.roadEdges[0] = line(-4.0)
  assert drive(LaneAvoidanceGuard(), md)[-1].strength == 1.0


def make_lane_center_corrector(params=None):
  source = Path(__file__).parents[1] / 'lib/drive_helpers.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  nodes = [n for n in tree.body if (
    isinstance(n, ast.ClassDef) and n.name == 'LaneCenterCorrector' or
    isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) and
    (n.targets[0].id.startswith('LANE_CENTER_') or n.targets[0].id == 'MIN_SPEED')
  )]
  clock = SimpleNamespace(now=0.0)
  namespace = {'np': np, 'math': math, 'time': SimpleNamespace(monotonic=lambda: clock.now)}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), namespace)
  settings = {} if params is None else params
  return namespace['LaneCenterCorrector'](SimpleNamespace(get=lambda key, **_: settings.get(key))), clock


def center_step(corrector, clock, md, *, fresh=True, lane_changing=False, speed=EGO_SPEED, **kwargs):
  clock.now += STEP
  if fresh:
    md.timestampEof = round(clock.now * 1e9)
  return corrector.update(md, speed, lane_changing, **kwargs)


def shift_center(md, delta, *, width_delta=0.0):
  md.laneLines[1].y += delta - width_delta / 2.0
  md.laneLines[2].y += delta + width_delta / 2.0


def test_lane_center_waits_for_continuous_offset_and_eases_in():
  corrector, clock = make_lane_center_corrector({'LaneCenterThreshold': 15, 'LaneCenterGain': 3})
  md = model()
  md.position = line(-0.18)
  for _ in range(7):
    assert center_step(corrector, clock, md) == 0.0
  ramp = [center_step(corrector, clock, md) for _ in range(8)]
  target = 2 * 0.18 * 0.3 / EGO_SPEED ** 2
  assert 0 < ramp[0] < ramp[3] < ramp[-1] == pytest.approx(target)
  md.position = line(-0.07)
  assert center_step(corrector, clock, md) < target
  assert center_step(corrector, clock, md) == 0.0


@pytest.mark.parametrize('interruption', ['distance', 'side', 'confidence', 'setting', 'gap', 'repeated'])
def test_lane_center_entry_does_not_accumulate_across_interruptions(interruption):
  params = {'LaneCenterThreshold': 15}
  corrector, clock = make_lane_center_corrector(params)
  md = model()
  md.position = line(-0.18)
  for _ in range(5):
    assert center_step(corrector, clock, md) == 0.0

  if interruption == 'distance':
    md.position = line(-0.15)
    assert center_step(corrector, clock, md) == 0.0
    md.position = line(-0.18)
  elif interruption == 'side':
    md.position = line(0.18)
  elif interruption == 'confidence':
    md.laneLineProbs[1] = float('nan')
    for _ in range(8):
      assert center_step(corrector, clock, md) == 0.0
    md.laneLineProbs[1] = 0.99
  elif interruption == 'setting':
    params['LaneCenterThreshold'] = 17
  elif interruption == 'gap':
    clock.now += 0.2
  elif interruption == 'repeated':
    for _ in range(20):
      assert center_step(corrector, clock, md, fresh=False) == 0.0

  for _ in range(7):
    assert center_step(corrector, clock, md) == 0.0
  assert abs(center_step(corrector, clock, md)) > 0


def test_lane_center_min_speed_is_inclusive_and_live():
  params = {'LaneCenterMinSpeed': 45}
  corrector, clock = make_lane_center_corrector(params)
  md = model()
  threshold = 45 / 3.6
  for _ in range(20):
    assert center_step(corrector, clock, md, speed=math.nextafter(threshold, 0.0)) == 0.0
  for _ in range(7):
    assert center_step(corrector, clock, md, speed=threshold) == 0.0
  assert center_step(corrector, clock, md, speed=threshold) > 0.0

  # Raising the limit while engaged releases the current correction at the
  # usual slew rate. Decelerating below it takes the same path.
  for _ in range(20):
    baseline = center_step(corrector, clock, md, speed=threshold)
  params['LaneCenterMinSpeed'] = 50
  first = center_step(corrector, clock, md, speed=threshold)
  assert baseline - first == pytest.approx(0.01 * STEP)
  for _ in range(20):
    last = center_step(corrector, clock, md, speed=threshold)
  assert last == 0.0
  for _ in range(7):
    assert center_step(corrector, clock, md, speed=50 / 3.6) == 0.0
  assert center_step(corrector, clock, md, speed=50 / 3.6) > 0.0
  assert center_step(corrector, clock, md, speed=threshold) == pytest.approx(0.0)


def test_lane_center_min_speed_keeps_geometry_jump_protection():
  corrector, clock = make_lane_center_corrector({'LaneCenterMinSpeed': 45})
  md = model()
  for _ in range(12):
    assert center_step(corrector, clock, md, speed=12.0) == 0.0
  shift_center(md, 1.0)
  assert center_step(corrector, clock, md, speed=12.0) == 0.0
  for _ in range(19):
    assert center_step(corrector, clock, md, speed=45 / 3.6) == 0.0
  for _ in range(22):
    result = center_step(corrector, clock, md, speed=45 / 3.6)
  assert result > 0.0


@pytest.mark.parametrize('setting', [0, 'invalid', float('nan')])
def test_lane_center_min_speed_defaults_preserve_the_low_speed_fade(setting):
  default, default_clock = make_lane_center_corrector()
  configured, clock = make_lane_center_corrector({'LaneCenterMinSpeed': setting})
  md, configured_md = model(), model()
  for _ in range(20):
    expected = center_step(default, default_clock, md, speed=7.5 / 3.6)
    assert center_step(configured, clock, configured_md, speed=7.5 / 3.6) == expected
  assert expected > 0.0


@pytest.mark.parametrize('side', [-1, 1])
def test_actual_lane_center_gives_way_before_slew_limiting(side):
  corrector, clock = make_lane_center_corrector()
  md = model(side)
  opposing = SimpleNamespace(side=side, strength=1.0, approachStart=0.0)
  same_direction = SimpleNamespace(side=-side, strength=1.0, approachStart=0.0)
  far_away = SimpleNamespace(side=side, strength=1.0, approachStart=30.0)

  assert center_step(corrector, clock, md) == 0.0
  for _ in range(30):
    baseline = center_step(corrector, clock, md)
  assert side * baseline > 0.0
  assert center_step(corrector, clock, md, avoidance=same_direction) == baseline
  assert center_step(corrector, clock, md, avoidance=far_away) == baseline
  first = center_step(corrector, clock, md, avoidance=opposing)
  assert side * (baseline - first) == pytest.approx(0.01 * STEP)
  for _ in range(20):
    last = center_step(corrector, clock, md, avoidance=opposing)
  assert last == 0.0


def test_lane_center_preserves_stable_offset_slow_drift_and_matching_curve():
  corrector, clock = make_lane_center_corrector()
  md = model()
  for _ in range(30):
    baseline = center_step(corrector, clock, md)
  assert baseline > 0.0
  for n in range(30):
    shift_center(md, 0.0075)
    previous = center_step(corrector, clock, md)
    assert previous == pytest.approx(baseline * (0.50 + (n + 1) * 0.0075) / 0.50)
  for _ in range(16):
    # A changing road curvature shifts both model and lane center together.
    curve = 0.0006 * X * X
    md.position.y += curve
    md.laneLines[1].y += curve
    md.laneLines[2].y += curve
    assert center_step(corrector, clock, md) == pytest.approx(previous)
  # A symmetric width change leaves the inferred center in the same place.
  shift_center(md, 0.0, width_delta=0.60)
  assert center_step(corrector, clock, md) == pytest.approx(previous)


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('jump', ['midpoint', 'one_boundary'])
def test_lane_center_suppresses_abrupt_geometry_before_slew_and_recovers(side, jump):
  corrector, clock = make_lane_center_corrector()
  md = model(side)
  for _ in range(21):
    baseline = center_step(corrector, clock, md)
  assert side * baseline > 0.0
  if jump == 'midpoint':
    shift_center(md, side * 1.0)
  else:
    shift_center(md, side * 0.30, width_delta=-0.60)

  first = center_step(corrector, clock, md)
  assert side * (baseline - first) == pytest.approx(0.01 * STEP)
  for _ in range(19):
    previous, first = first, center_step(corrector, clock, md)
    assert -(0.01 * STEP + 1e-12) <= side * (first - previous) <= 0.0
  assert first == pytest.approx(0.0)
  for _ in range(28):
    previous, first = first, center_step(corrector, clock, md)
    assert 0.0 <= side * (first - previous) <= 0.01 * STEP + 1e-12
  assert side * first > 0.0


def test_lane_center_recovery_requires_fresh_stable_model_frames():
  corrector, clock = make_lane_center_corrector()
  md = model()
  for _ in range(21):
    center_step(corrector, clock, md)
  shift_center(md, 1.0)
  center_step(corrector, clock, md)
  for _ in range(50):
    result = center_step(corrector, clock, md, fresh=False)
  assert result == 0.0
  for _ in range(4):
    assert center_step(corrector, clock, md) == 0.0
  for _ in range(30):
    result = center_step(corrector, clock, md)
  assert result > 0.0


def test_lane_center_settings_and_lane_change_discard_old_midpoint_history():
  params = {'LaneCenterEnable': True}
  corrector, clock = make_lane_center_corrector(params)
  md = model()
  for _ in range(21):
    baseline = center_step(corrector, clock, md)
  assert baseline > 0.0

  params['LaneCenterEnable'] = False
  shift_center(md, 0.85)
  assert center_step(corrector, clock, md) < baseline
  params['LaneCenterEnable'] = True
  before = center_step(corrector, clock, md)
  assert center_step(corrector, clock, md) < before
  for _ in range(30):
    result = center_step(corrector, clock, md)
  assert result > before

  shift_center(md, -0.85)
  assert center_step(corrector, clock, md, lane_changing=True) < result
  for _ in range(37):
    result = center_step(corrector, clock, md)
  assert result == 0.0
  for _ in range(14):
    result = center_step(corrector, clock, md)
  assert result > 0.0


@pytest.mark.parametrize('fingerprint', ['', 'KIA_EV9', 'HYUNDAI_IONIQ_5_PE'])
def test_unrequested_vehicles_do_not_construct_avoidance(fingerprint):
  assert LaneAvoidanceGuard.for_vehicle(fingerprint) is None
