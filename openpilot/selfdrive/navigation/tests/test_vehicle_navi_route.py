import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.navd import navd
from openpilot.selfdrive.navigation.helpers import NavigationPosition
from openpilot.selfdrive.navigation.map_client import MapInput
from openpilot.selfdrive.navigation.mapsd import stock_navi_map_active
from openpilot.selfdrive.navigation.renderer import (
  EARTH_CIRCUMFERENCE, MapRenderer, Position, ROUTE_INTENSITY, SIZE, mercator,
)
from openpilot.selfdrive.navigation.source import RouteSource
from openpilot.selfdrive.navigation.tests.test_rendering import field, varint
from openpilot.selfdrive.navigation.vehicle_navi_route import VehicleNaviRouteSynthesizer


def zigzag(value: int) -> int:
  return (value << 1) ^ (value >> 31)


ORIGIN_LAT = 37.6611
ORIGIN_LON = 126.8412
ZOOM = 15
EXTENT = 4096


def _encode_mvt_line(points_xy):
  cmds = []
  x = y = 0
  for i, (px, py) in enumerate(points_xy):
    ix, iy = int(round(px)), int(round(py))
    if i == 0:
      cmds.extend([9, zigzag(ix - x), zigzag(iy - y)])
    else:
      if i == 1:
        cmds.append(((len(points_xy) - 1) << 3) | 2)
      cmds.extend([zigzag(ix - x), zigzag(iy - y)])
    x, y = ix, iy
  return b''.join(varint(v) for v in cmds)


def _enu_to_tile_xy(points_enu, tile_x, tile_y):
  origin_m = mercator(np.array([[ORIGIN_LON, ORIGIN_LAT]], dtype=np.float64))[0]
  scale = EARTH_CIRCUMFERENCE * math.cos(math.radians(ORIGIN_LAT))
  n = 1 << ZOOM
  pts = np.asarray(points_enu, dtype=np.float64)
  mx = origin_m[0] + pts[:, 0] / scale
  my = origin_m[1] - pts[:, 1] / scale
  tx = (mx * n - tile_x) * EXTENT
  ty = (my * n - tile_y) * EXTENT
  return np.column_stack((tx, ty))


def _enu_point_to_lonlat(east, north):
  origin_m = mercator(np.array([[ORIGIN_LON, ORIGIN_LAT]], dtype=np.float64))[0]
  scale = EARTH_CIRCUMFERENCE * math.cos(math.radians(ORIGIN_LAT))
  mx = origin_m[0] + east / scale
  my = origin_m[1] - north / scale
  lon = mx * 360.0 - 180.0
  lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * my))))
  return lon, lat


def _build_mapbox_tile(tile_x, tile_y, road_specs):
  """Encode a Mapbox Streets v8 'road' MVT tile from ENU road specs."""
  keys = ['class', 'structure', 'oneway']
  values = []

  def val_idx(v):
    if v not in values:
      values.append(v)
    return values.index(v)

  features = b''
  for pts_enu, road_class, structure, oneway in road_specs:
    txy = _enu_to_tile_xy(pts_enu, tile_x, tile_y)
    tags = [0, val_idx(road_class), 1, val_idx(structure), 2, val_idx('true' if oneway else 'false')]
    tag_bytes = b''.join(varint(v) for v in tags)
    geom = _encode_mvt_line(txy)
    features += field(2, field(2, tag_bytes) + field(3, 2) + field(4, geom))

  layer = field(15, 2) + field(1, b'road') + features
  for k in keys:
    layer += field(3, k.encode())
  for v in values:
    layer += field(4, field(1, v.encode()))
  layer += field(5, EXTENT)
  return field(3, layer)


class _FakeMapboxArchive:
  min_zoom, max_zoom = 0, 16

  def __init__(self, road_specs):
    self.road_specs = road_specs

  def get(self, zoom, x, y):
    assert zoom == ZOOM
    return _build_mapbox_tile(x, y, self.road_specs)


def _gps(east=0.0, north=0.0, bearing=0.0, speed=10.0, t=10.0):
  lon, lat = _enu_point_to_lonlat(east, north)
  return NavigationPosition(lat, lon, bearing, speed, t, t)


def _inst(mtype, modifier, distance):
  return NS(
    maneuverType=mtype,
    maneuverModifier=modifier,
    maneuverDistance=distance,
    allManeuvers=[NS(type=mtype, modifier=modifier, distance=distance)],
  )


def _cross_roads():
  """North-south primary avenue intersecting an east-west secondary street at (0, 80)."""
  return [
    ([(0, -150), (0, 80), (0, 320)], 'primary', 'none', False),
    ([(-220, 80), (0, 80), (220, 80)], 'secondary', 'none', False),
  ]


def test_straight_distant_maneuver_renders_valid_nav_map():
  renderer = MapRenderer(_FakeMapboxArchive(_cross_roads()))
  synth = VehicleNaviRouteSynthesizer()
  res = synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'right', 450.0), renderer, now=10.0)
  assert res is not None
  route, (lon, lat, brg) = res
  assert brg == pytest.approx(0.0, abs=2.0)
  # Route stays straight north (east stays near 0).
  assert np.max(np.abs(route[:, 0] - ORIGIN_LON)) < 0.0002
  rendered = renderer.render(Position(lat, lon, brg), route)
  assert rendered.valid
  assert np.any(rendered.image == ROUTE_INTENSITY)


def test_right_turn_selects_east_branch_and_locks_through_intersection():
  renderer = MapRenderer(_FakeMapboxArchive(_cross_roads()))
  synth = VehicleNaviRouteSynthesizer()

  # Approaching right turn at 80 m ahead.
  res = synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'right', 80.0), renderer, now=10.0)
  assert res is not None
  route, (lon, lat, brg) = res
  # Route must turn east (longitude increases well beyond origin).
  assert route[-1, 0] > ORIGIN_LON + 0.0012
  rendered = renderer.render(Position(lat, lon, brg), route)
  assert rendered.valid

  # Mid-intersection turn-through hold (dist = -10 m, heading turning right): stays locked!
  res_hold = synth.synthesize(_gps(5, 80, bearing=45.0, t=14.0), _inst('turn', 'right', -10.0), renderer, now=14.0)
  assert res_hold is not None
  route_hold, pose_hold = res_hold
  assert np.array_equal(route_hold, route)
  assert renderer.render(Position(pose_hold[1], pose_hold[0], pose_hold[2]), route_hold).valid


def test_left_turn_selects_west_branch():
  renderer = MapRenderer(_FakeMapboxArchive(_cross_roads()))
  synth = VehicleNaviRouteSynthesizer()
  res = synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'left', 80.0), renderer, now=10.0)
  assert res is not None
  route, _ = res
  assert route[-1, 0] < ORIGIN_LON - 0.0012


def test_highway_off_ramp_right_prefers_motorway_link():
  roads = [
    ([(0, -150), (0, 100), (0, 350)], 'motorway', 'none', True),
    ([(0, 100), (25, 160), (70, 220), (140, 260)], 'motorway_link', 'none', True),
  ]
  renderer = MapRenderer(_FakeMapboxArchive(roads))
  synth = VehicleNaviRouteSynthesizer()
  res = synth.synthesize(_gps(0, 0, bearing=0.0, speed=25.0), _inst('off ramp', 'right', 100.0), renderer, now=10.0)
  assert res is not None
  route, (lon, lat, brg) = res
  assert route[-1, 0] > ORIGIN_LON + 0.0008
  assert renderer.render(Position(lat, lon, brg), route).valid


def test_bridge_overpass_never_turns_onto_road_underneath():
  roads = [
    ([(0, -150), (0, 80), (0, 320)], 'motorway', 'bridge', True),
    ([(-200, 80), (0, 80), (200, 80)], 'secondary', 'none', False),
  ]
  renderer = MapRenderer(_FakeMapboxArchive(roads))
  synth = VehicleNaviRouteSynthesizer()
  # At 80 m before an alleged right turn, the crossing road is below the bridge (level 0 != level 1), so fail closed!
  assert synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'right', 80.0), renderer, now=10.0) is None


def test_oneway_street_blocks_wrong_way_turn():
  roads = [
    ([(0, -150), (0, 80), (0, 320)], 'primary', 'none', False),
    # One-way going WEST only (east -> west): turning right (east) is illegal!
    ([(220, 80), (0, 80), (-220, 80)], 'secondary', 'none', True),
  ]
  renderer = MapRenderer(_FakeMapboxArchive(roads))
  synth = VehicleNaviRouteSynthesizer()
  assert synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'right', 80.0), renderer, now=10.0) is None
  # Turning left (west, legal oneway direction) succeeds!
  res_left = synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'left', 80.0), renderer, now=10.0)
  assert res_left is not None and res_left[0][-1, 0] < ORIGIN_LON - 0.0012


def test_fail_closed_when_off_road_or_missing_turn():
  roads = [([(0, -150), (0, 320)], 'primary', 'none', False)]
  renderer = MapRenderer(_FakeMapboxArchive(roads))
  synth = VehicleNaviRouteSynthesizer()
  # Off-road by 50 m -> None.
  assert synth.synthesize(_gps(50, 0, bearing=0.0), _inst('turn', 'right', 400.0), renderer, now=10.0) is None
  # Imminent right turn on a road with NO right branch -> fail closed (None).
  assert synth.synthesize(_gps(0, 0, bearing=0.0), _inst('turn', 'right', 90.0), renderer, now=10.0) is None


def test_route_source_stock_navi_priority_and_map_input_handshake(monkeypatch):
  from openpilot.selfdrive.navigation import map_client

  src = RouteSource()
  # Stock navi takes ownership when no external nav and no Mapbox destination.
  assert src.select(False, vehicle_navi_active=True) == []
  assert src.selected == ('vehicle_navi', ())
  # No redundant bump while staying in stock navi.
  assert src.select(False, vehicle_navi_active=True) is None
  # Mapbox destination overrides stock navi.
  src.mapbox = ((37.0, 127.0), (37.1, 127.1))
  selected = src.select(False, vehicle_navi_active=True)
  assert len(selected) == 2 and src.selected[0] == 'mapbox'
  # Clearing Mapbox destination cleanly returns to stock navi (bumps route id).
  src.mapbox = ()
  assert src.select(False, vehicle_navi_active=True) == []
  assert src.selected == ('vehicle_navi', ())
  # External nav overrides stock navi.
  assert src.select(True, vehicle_navi_active=True) == []
  assert src.selected == ('external', ())

  # Verify stock_navi_map_active gating.
  class SM(dict):
    pass

  sm = SM(
    carrotMan=NS(vehicleNaviAvailable=True, xTurnInfo=2, remote='', nGoPosDist=0, nGoPosTime=0, activeCarrot=5),
    carrotNavi=NS(),
    navInstructionCarrot=_inst('turn', 'right', 80.0),
    navRoute=NS(carrotRouteId=777),
  )
  sm.valid = {'carrotMan': True, 'carrotNavi': False, 'navInstructionCarrot': True, 'navRoute': True}
  sm.alive = {'carrotMan': True, 'carrotNavi': False}
  sm.recv_time = {'navInstructionCarrot': 9.9}

  assert stock_navi_map_active(sm, route_id=777, tracker=None, fresh_tbt=True, enabled=True)
  assert not stock_navi_map_active(sm, route_id=777, tracker=None, fresh_tbt=True, enabled=False)
  sm['carrotMan'].remote = '192.168.1.10'
  assert not stock_navi_map_active(sm, route_id=777, tracker=None, fresh_tbt=True, enabled=True)
  sm['carrotMan'].remote = ''
  sm.alive['carrotNavi'] = sm.valid['carrotNavi'] = True
  assert not stock_navi_map_active(sm, route_id=777, tracker=None, fresh_tbt=True, enabled=True)
  sm.alive['carrotNavi'] = sm.valid['carrotNavi'] = False

  # Verify MapInput accepts stock-navi nav_map + navigation when route_id matches.
  monkeypatch.setattr(map_client.time, 'monotonic', lambda: 10.0)
  client = MapInput()
  client.client = NS(is_connected=lambda: True, recv=lambda _: None)
  client.image = np.full((SIZE, SIZE), 128, np.uint8)
  client.image_time = 9.9
  client.route_time = 777
  out = client.update(NS(views={'nav_map': None}), sm, 10.0)
  assert 'nav_map' in out and 'navigation' in out
  assert out['navigation'][10] == 1.0  # Valhalla 'right' turn


def test_navd_ignores_vehicle_navi_bump_and_curve_active_carrot():
  class SM(dict):
    pass

  sm = SM(
    carrotMan=NS(activeCarrot=5, remote='', vehicleNaviAvailable=True),
    carrotNavi=NS(),
  )
  sm.valid = {'carrotMan': True, 'carrotNavi': False}
  sm.alive = {'carrotMan': True, 'carrotNavi': False}
  engine = navd.RouteEngine.__new__(navd.RouteEngine)
  engine.sm = sm
  assert not engine._external_nav_active()

  sm['carrotMan'].activeCarrot = 6
  assert not engine._external_nav_active()

  sm['carrotMan'].activeCarrot = 2
  assert engine._external_nav_active()

  sm['carrotMan'].activeCarrot = 5
  sm['carrotMan'].remote = '192.168.1.50'
  assert engine._external_nav_active()

  sm['carrotMan'].remote = ''
  sm.alive['carrotNavi'] = sm.valid['carrotNavi'] = True
  assert engine._external_nav_active()


def test_carplay_tmap_straight_guidance_enables_mapsd_noo_without_atc():
  class SM(dict):
    pass

  sm = SM(
    carrotMan=NS(vehicleNaviAvailable=True, xTurnInfo=-1, remote='', nGoPosDist=0, nGoPosTime=0, activeCarrot=0),
    carrotNavi=NS(),
    carState=NS(vehicleNaviTurnInfo=9),
  )
  sm.valid = {'carrotMan': True, 'carrotNavi': False, 'carState': True}
  sm.alive = {'carrotMan': True, 'carrotNavi': False, 'carState': True}
  assert stock_navi_map_active(sm, route_id=777, tracker=None, fresh_tbt=True, enabled=True)
  sm['carState'].vehicleNaviTurnInfo = -1
  assert not stock_navi_map_active(sm, route_id=777, tracker=None, fresh_tbt=True, enabled=True)
