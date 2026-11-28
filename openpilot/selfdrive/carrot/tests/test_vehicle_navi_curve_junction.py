from pathlib import Path
import re
import sys
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
CLUSTER_DIR = REPO_ROOT / "openpilot" / "selfdrive" / "carrot" / "cluster"
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from opendbc.car import Bus, structs
from opendbc.can.packer import CANPacker
from opendbc.car.hyundai import carstate as hyundai_carstate
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner
from openpilot.selfdrive.carrot.carrot_serv import CarrotServ
from openpilot.selfdrive.carrot.deceleration_source import (
  deceleration_source_presentation,
  format_vehicle_navi_junction,
)
from openpilot.selfdrive.carrot.realtime.compact_state import SERVICE_SCHEMAS
from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _car_state as _serv_cs
from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _serv as _make_serv
from openpilot.selfdrive.carrot.tests.test_cluster_scene import LIGHT_CLUSTER_THEME, _cluster_state
import cluster_renderer


class _FakeNaviParams:
  def __init__(self, can_control=1, school_zone=False, ramp_control=True, curve_factor=100, ctrl_end=3):
    self.can_control = can_control
    self.school_zone = school_zone
    self.ramp_control = ramp_control
    self.curve_factor = curve_factor
    self.ctrl_end = ctrl_end

  def get_int(self, key):
    return {
      "HyundaiCameraSCC": 0,
      "AutoEngage": 0,
      "VehicleSpeedCameraDistanceTime": 60,
      "VehicleNaviCanControl": int(self.can_control),
      "AutoNaviSpeedCtrlMode": 2,
      "AutoNaviSpeedSafetyFactor": 100,
      "AutoNaviSpeedBumpSpeed": 35,
      "AutoNaviSpeedBumpTime": 1,
      "AutoNaviSpeedBumpEndDistance": 200,
      "AutoNaviSpeedDecelRate": 120,
      "CruiseButtonLongDelay": 40,
      "VehicleNaviCurveSpeedFactor": self.curve_factor,
      "AutoCurveSpeedLowerLimit": 30,
      "VehicleNaviCurveCtrlEnd": self.ctrl_end,
    }.get(key, 0)

  def get_bool(self, key):
    if key == "VehicleNaviRampCurveControl":
      return self.ramp_control
    if key == "VehicleNaviSchoolZoneControl":
      return self.school_zone
    return key in ("ControlsReady", "RoundaboutLaneChangeHold", "TunnelLaneChangeHold")

  def put_bool(self, key, value):
    pass

  def get(self, key):
    return repr({k: [] for k in range(-4, 8)}) if key == "FingerPrints" else ""


def _make_hyundai_state(monkeypatch, fingerprint, *, ramp_control=True, can_control=1):
  fake_params = _FakeNaviParams(can_control=can_control, ramp_control=ramp_control)
  monkeypatch.setattr(hyundai_carstate, "Params", lambda: fake_params)
  CP = structs.CarParams()
  CP.carFingerprint = fingerprint
  CP.flags = int(HyundaiFlags.CANFD)
  CS = CarState(CP)
  CS.controls_ready_count = 122
  parsers = CS.get_can_parsers_canfd(CP)
  CS.cp = parsers[Bus.pt]
  CS.cp_cam = parsers[Bus.cam]
  CS.cp_alt = parsers.get(Bus.alt)
  return CS, parsers, CANPacker("hyundai_canfd_generated")


def _segment_values(calc_route, path_idx=8, road_class=1):
  raw = (800 & 0x1FFF) | ((path_idx & 0x3F) << 13) | ((calc_route & 0x3) << 22) | ((road_class & 0x7) << 24)
  return {f"BYTE_{i + 1}": (raw >> (i * 8)) & 0xFF for i in range(8)}


def _step(CS, parsers, t_s, frames=(), *, v_ego=25.0, left_blinker=False, right_blinker=False):
  ts_ns = int(round(t_s * 1e9))
  bus_msgs = {bus: [] for bus in parsers}
  for bus, dat in frames:
    bus_msgs[bus].append(dat)
  for bus, parser in parsers.items():
    parser.update([(ts_ns, bus_msgs[bus])])
  CS.monitor_fingerprint(parsers, True)
  CS.totalDistance += v_ego * 0.01
  ret = SimpleNamespace(
    vEgo=v_ego,
    speedLimit=100.0,
    leftBlinker=left_blinker,
    rightBlinker=right_blinker,
  )
  CS._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers.get(Bus.alt))
  return ret


def test_compact_state_carrot_man_schema_synced_across_python_cpp_js():
  cpp_text = (REPO_ROOT / "openpilot" / "selfdrive" / "carrot" / "realtime" / "compact_state_native.cc").read_text()
  js_text = (REPO_ROOT / "openpilot" / "selfdrive" / "carrot" / "web" / "js" / "realtime" / "vision_compact.js").read_text()

  cpp_match = re.search(r"void encode_carrot_man\([^\)]*\)\s*\{(.*?)\n\}", cpp_text, re.DOTALL)
  assert cpp_match is not None
  cpp_fields = re.findall(r'append_(\w+)\(out,\s*value,\s*"([^"]+)"\)', cpp_match.group(1))
  cpp_norm = [(name, typ) for typ, name in cpp_fields]

  js_match = re.search(r'\[\s*5\s*,\s*\["carrotMan"\s*,\s*\[(.*?)\]\s*\]\s*\]', js_text, re.DOTALL)
  assert js_match is not None
  js_fields = re.findall(r'\["([^"]+)"\s*,\s*"([^"]+)"\]', js_match.group(1))

  py_fields = list(SERVICE_SCHEMAS["carrotMan"][1])
  assert py_fields == cpp_norm == js_fields
  assert ("vehicleNaviCurveCurrentSpeed", "f32") in py_fields
  assert ("vehicleNaviCurveIsRamp", "bool") in py_fields
  assert ("vehicleNaviJunctionType", "u8") in py_fields
  assert ("vehicleNaviJunctionSide", "u8") in py_fields
  assert ("vehicleNaviJunctionDistance", "i32") in py_fields


@pytest.mark.parametrize(
  ("raw_v0", "expected_sign", "expected_radius"),
  (
    (611, 1.0, 380.0),
    (411, -1.0, 380.0),
    (521, 1.0, 38.0),
    (501, -1.0, 38.0),
  ),
)
def test_decode_hyundai_navi_curvature_scale_3_8_m_per_count(raw_v0, expected_sign, expected_radius):
  curv = CarState._decode_hyundai_navi_curvature(raw_v0)
  assert curv == pytest.approx(expected_sign / expected_radius)


@pytest.mark.parametrize("sentinel", (-1, 511, 1023, 1024))
def test_decode_hyundai_navi_curvature_rejects_sentinels(sentinel):
  assert CarState._decode_hyundai_navi_curvature(sentinel) is None


def test_ev4_curve_linkclass_gate_and_recalculation_reset(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4)
  bus = parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=0, path_idx=8)))
  hda_hwy = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 0}))
  curve_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 200, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  ret = _step(CS, parsers, 1.0, [seg_frame, hda_hwy, curve_frame])
  assert ret.vehicleNaviCurveRoadAllowed is True
  assert ret.vehicleNaviCurveRouteState == 0
  assert ret.vehicleNaviCurveIsRamp is False
  assert ret.vehicleNaviCurveCurvature == pytest.approx(1.0 / 38.0)
  assert ret.vehicleNaviCurveSpeed == pytest.approx(CarState._vehicle_navi_curve_reference_speed(1.0 / 38.0))

  # Switching to local road (LinkClass=0) must block curve control immediately.
  hda_local = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 0, "Frwinfo": 0}))
  ret_local = _step(CS, parsers, 1.1, [hda_local])
  assert ret_local.vehicleNaviCurveRoadAllowed is False
  assert ret_local.vehicleNaviCurveSpeed == 0.0

  # JC loop connector (LinkClass=3) must stay allowed while cached curve is valid.
  hda_jc = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 3, "Frwinfo": 0}))
  ret_jc = _step(CS, parsers, 1.15, [hda_jc])
  assert ret_jc.vehicleNaviCurveRoadAllowed is True
  assert ret_jc.vehicleNaviCurveSpeed > 0.0

  # Inside the curve span (start < totalDistance <= start + span), hold distance=0 and curve speed until exit.
  CS.totalDistance = CS.vehicleNaviCurves[0]["target"] + 20.0
  ret_mid = _step(CS, parsers, 1.17, [hda_jc], v_ego=0.0)
  assert ret_mid.vehicleNaviCurveDistance == 0.0
  assert ret_mid.vehicleNaviCurveSpeed > 0.0
  CS.totalDistance = CS.vehicleNaviCurves[0]["target"] - 50.0

  # Back on highway -> cached curve still valid until CalculatedRoute == 2 resets it.
  ret_back = _step(CS, parsers, 1.2, [hda_hwy])
  assert ret_back.vehicleNaviCurveSpeed > 0.0

  seg_recalc = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=2, path_idx=8)))
  ret_reset = _step(CS, parsers, 1.3, [seg_recalc, hda_hwy])
  assert ret_reset.vehicleNaviCurveRouteState == 2
  assert ret_reset.vehicleNaviCurveSpeed == 0.0
  assert CS.vehicleNaviCurves == []


def test_ev4_094_simultaneous_section_taper_curve_and_conditional_098_ramp(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4, can_control=1)
  bus = parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=0, path_idx=8)))
  hda_fork = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 2}))
  ramp_curve = (Bus.pt, packer.make_can_msg("CANFD_NAVI_RAMP_098", bus, {
    "PROSHORT_OFFSET": 250, "PROSHORT_PATH_INDEX": 9, "PROSHORT_DISTANCE": 5,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  stub_right = (Bus.pt, packer.make_can_msg("CANFD_NAVI_STUB_09A", bus, {
    "STUB_OFFSET": 240, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x49, "STUB_TURN_ANGLE": 16.875,
  }))
  taper_diverge = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 150, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 293,
    "PROSHORT_VALUE_0": 1, "PROSHORT_VALUE_1": (6 << 1), "PROSHORT_PROFILE_TYPE": 16,
  }))

  # 1. Driving straight past exit with no blinker -> 0x098 MUST NOT arm.
  ret_straight = _step(CS, parsers, 1.0, [seg_frame, hda_fork, ramp_curve, stub_right, taper_diverge])
  assert ret_straight.vehicleNaviCurveSpeed == 0.0
  assert ret_straight.vehicleNaviCurveIsRamp is False
  assert ret_straight.vehicleNaviJunctionType == 1
  assert ret_straight.vehicleNaviJunctionSide == 2
  assert ret_straight.vehicleNaviJunctionLane == 6

  # 2. Hazard flashers (both blinkers) or left overtake blinker -> 0x098 MUST NOT arm.
  assert _step(CS, parsers, 1.1, left_blinker=True, right_blinker=True).vehicleNaviCurveSpeed == 0.0
  assert _step(CS, parsers, 1.2, left_blinker=True, right_blinker=False).vehicleNaviCurveSpeed == 0.0

  # 3. Single right blinker matching STUB right exit + Frwinfo=2 -> 0x098 arms cleanly!
  ret_right = _step(CS, parsers, 1.3, left_blinker=False, right_blinker=True)
  assert ret_right.vehicleNaviCurveIsRamp is True
  assert ret_right.vehicleNaviCurveSpeed > 0.0

  # 4. Left-side exit (negative turn angle) -> left blinker arms, right does not.
  stub_left = (Bus.pt, packer.make_can_msg("CANFD_NAVI_STUB_09A", bus, {
    "STUB_OFFSET": 220, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x49, "STUB_TURN_ANGLE": -19.6875,
  }))
  assert _step(CS, parsers, 1.4, [stub_left], left_blinker=False, right_blinker=True).vehicleNaviCurveIsRamp is False
  assert _step(CS, parsers, 1.5, left_blinker=True, right_blinker=False).vehicleNaviCurveIsRamp is True

  # 5. EV4 section (ptype=16 v0=0) still latches cleanly right alongside curve/junction traffic.
  section_msg = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 100, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 50,
    "PROSHORT_VALUE_0": 0, "PROSHORT_VALUE_1": (19 << 5), "PROSHORT_PROFILE_TYPE": 16,
  }))
  _step(CS, parsers, 1.6, [section_msg])
  assert CS.ev4Section is not None and CS.ev4Section["speed"] == 90


def test_pv5_curve_keeps_camera_route_exclusion_and_legacy_4ba_4bf_works(monkeypatch):
  pv5_cs, pv5_parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_PV5)
  bus = pv5_parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=1, path_idx=8)))
  hda_hwy = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 0}))
  curve_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 200, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  ret_pv5 = _step(pv5_cs, pv5_parsers, 1.0, [seg_frame, hda_hwy, curve_frame])
  assert pv5_cs.navi_segment_4b9 is None
  assert ret_pv5.speedBumpDistance == 0.0
  assert ret_pv5.vehicleNaviCurveSpeed > 0.0

  # Legacy CAN-FD (Ioniq 5): 0x4BA + 0x4B9 + 0x4BF + 0x4A3
  leg_cs, leg_parsers, _ = _make_hyundai_state(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  leg_bus = leg_parsers[Bus.pt].bus
  leg_parsers[Bus.pt]._add_message("HDA_INFO_4A3", float("nan"))
  leg_cs.hda_info_4a3 = leg_parsers[Bus.pt].vl["HDA_INFO_4A3"]
  ret_leg = _step(leg_cs, leg_parsers, 1.0, [
    (Bus.pt, packer.make_can_msg("NEW_MSG_4B9", leg_bus, _segment_values(calc_route=1, path_idx=8))),
    (Bus.pt, packer.make_can_msg("HDA_INFO_4A3", leg_bus, {"LinkClass": 1, "Frwinfo": 2})),
    (Bus.pt, packer.make_can_msg("NEW_MSG_4BA", leg_bus, {
      "PROSHORT_OFFSET": 220, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
      "PROSHORT_VALUE_0": 526, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
    })),
    (Bus.pt, packer.make_can_msg("NEW_MSG_4BF", leg_bus, {
      "STUB_OFFSET": 300, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x01, "STUB_TURN_ANGLE": -158.9,
    })),
  ])
  assert ret_leg.vehicleNaviCurveSpeed > 0.0
  assert ret_leg.vehicleNaviJunctionType == 2
  assert ret_leg.vehicleNaviJunctionSide == 1


def test_carrot_serv_curve_gates_scaling_and_external_nav_priority():
  serv = _make_serv(1)
  serv.vehicleNaviCurveControl = False
  serv.vehicleNaviCurveMppControl = True
  serv.vehicleNaviRampCurveControl = True
  serv.vehicleNaviCurveSpeedFactor = 1.0
  serv.vehicleNaviCurveControlEnd = 3.0
  serv.autoCurveSpeedLowerLimit = 30
  serv.external_navigation_active = False

  CS = _serv_cs()
  CS.vehicleNaviCurveRoadAllowed = True
  CS.vehicleNaviCurveRouteState = 0
  CS.vehicleNaviCurveRouteActive = False
  CS.vehicleNaviCurveDistance = 120.0
  CS.vehicleNaviCurveSpeed = 55.0
  CS.vehicleNaviCurveCurvature = 1.0 / 60.0
  CS.vehicleNaviCurveIsRamp = True
  CS.vehicleNaviJunctionType = 1
  CS.vehicleNaviJunctionSide = 2
  CS.vehicleNaviJunctionDistance = 180.0

  # Regular curve opt-in off, ramp opt-in on: regular curve stays off (250), ramp curve still activates independently
  expected = serv.calculate_current_speed(120.0, 55.0, 3.0, serv.autoNaviSpeedDecelRate)
  CS.vehicleNaviCurveIsRamp = False
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  assert 55.0 <= serv._vehicle_navi_curve_display_speed(CS) < 250.0
  CS.vehicleNaviCurveIsRamp = True
  assert serv._vehicle_navi_curve_speed(CS) == pytest.approx(expected)

  # Both opt-ins off: ramp curve also inactive (250)
  serv.vehicleNaviRampCurveControl = False
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  serv.vehicleNaviRampCurveControl = True

  # Far-away curve preview (> vEgo + 20 kph) is suppressed on HUD to avoid high-speed noise
  CS.vehicleNaviCurveDistance = 1200.0
  assert serv._vehicle_navi_curve_display_speed(CS) == 0.0
  CS.vehicleNaviCurveDistance = 120.0

  # Opt-in on + scaling + ramp source label
  serv.vehicleNaviCurveControl = True
  assert serv._vehicle_navi_curve_speed(CS) == pytest.approx(expected)
  assert deceleration_source_presentation("hda_ramp") == ("ramp", 3)
  assert deceleration_source_presentation("hda_curve") == ("curve", 3)
  assert deceleration_source_presentation("hda_atc") == ("atc", 3)
  assert deceleration_source_presentation("gas") == ("gas", 2)
  assert deceleration_source_presentation("gas", vehicle_navi_active=True) == ("gas", 3)

  # Ramp gate off with regular curve gate on -> ignores ramp curve, keeps regular curve active
  serv.vehicleNaviRampCurveControl = False
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  CS.vehicleNaviCurveIsRamp = False
  assert serv._vehicle_navi_curve_speed(CS) == pytest.approx(expected)
  CS.vehicleNaviCurveIsRamp = True
  serv.vehicleNaviRampCurveControl = True

  # Local road gate (RoadAllowed=False) -> blocked
  CS.vehicleNaviCurveRoadAllowed = False
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  assert serv._vehicle_navi_curve_display_speed(CS) == 0.0
  CS.vehicleNaviCurveRoadAllowed = True

  # External navigation active -> supersedes both curve and junction display
  serv.external_navigation_active = True
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  assert serv._vehicle_navi_curve_display_speed(CS) == 0.0
  assert serv._vehicle_navi_junction_display(CS) == (0, 0, 0)


def test_carrot_planner_junction_accel_smoothing():
  planner = object.__new__(CarrotPlanner)
  planner.cruiseMaxVals0 = 1.6
  planner.cruiseMaxVals1 = 1.4
  planner.cruiseMaxVals2 = 1.2
  planner.cruiseMaxVals3 = 1.1
  planner.cruiseMaxVals4 = 1.0
  planner.cruiseMaxVals5 = 0.9
  planner.cruiseMaxVals6 = 0.8
  planner.myDrivingMode = 3
  planner.myHighModeFactor = 1.0
  planner.mySafeFactor = 1.0

  planner.vehicle_navi_junction_assist = False
  planner.vehicle_navi_junction_accel_limited = False
  assert planner.get_carrot_accel(25.0) > 0.8

  planner.vehicle_navi_junction_accel_limited = True
  assert planner.get_carrot_accel(25.0) == pytest.approx(0.8)


def test_hud_and_cluster_render_ramp_and_junction_badges(monkeypatch):
  assert format_vehicle_navi_junction(1, 2, 220) == "FORK -> 220m"
  assert format_vehicle_navi_junction(2, 1, 0) == "<- MERGE NOW"
  assert format_vehicle_navi_junction(0, 2, 100) == ""

  renderer = object.__new__(cluster_renderer.ClusterUiRenderer)
  renderer._current_theme = lambda: LIGHT_CLUSTER_THEME
  renderer._speed_bg_texture = None
  renderer.is_metric = True
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append(args)

  renderer._draw_speed_block(_cluster_state(
    vehicle_navi_curve_speed_kph=72.0,
    vehicle_navi_curve_is_ramp=True,
    vehicle_navi_junction_badge="FORK -> 220m",
  ))
  texts = [d[0] for d in draws]
  assert "ramp" in texts
  assert "72" in texts
  assert "FORK -> 220m" in texts


def test_ev4_099_curve_segment_survives_gaps_and_expires_at_600s(monkeypatch):
  cs, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4)
  bus = parsers[Bus.pt].bus
  hda = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1}))
  frames = [
    hda,
    (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(1))),
    (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
      "PROSHORT_OFFSET": 250, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
      "PROSHORT_VALUE_0": 521, "PROSHORT_PROFILE_TYPE": 1,
    })),
  ]
  assert _step(cs, parsers, 1.0, frames).vehicleNaviCurveSpeed > 0
  # Event-driven 0x099 survives normal multi-second gaps while 0x364 stays fresh
  assert _step(cs, parsers, 45.0, [hda]).vehicleNaviCurveSpeed > 0
  # Expires after the 600s stream backstop
  assert _step(cs, parsers, 602.0, [hda]).vehicleNaviCurveSpeed == 0.0


@pytest.mark.parametrize(("display_type", "direction", "bar_level", "link_class", "expected"), (
  (0x46, 0, 0, 5, 1),   # city turn left
  (0x47, 0, 0, 5, 1),   # city slight left
  (0x47, 0, 0, 1, 3),   # highway slight left -> fork left
  (0x43, 0, 0, 5, 2),   # city turn right
  (0x41, 0, 0, 1, 3),   # highway fork left
  (0x42, 0, 0, 1, 4),   # JC fork right
  (0xA1, 0, 0, 2, 4),   # highway entry right
  (0xA4, 0, 0, 1, 4),   # IC exit right
  (0x11, 0, 0, 1, 4),   # IC exit right close
  (0x0D, 0x00, 0, 1, 3),  # close branch left
  (0x0D, 0x03, 1, 1, 4),  # close branch right
  (0x0D, 0x03, 3, 1, 4),  # close branch right (JC)
  (0x0D, 0x01, 0, 1, None),  # ambiguous/straight close graphic -> inactive
  (0xB2, 0, 0, 1, 6),   # tollgate
  (0x30, 0, 0, 1, 6),   # tollgate close
  (0x48, 0, 0, 5, 7),   # U-turn -> Carrot xTurnInfo=7 (straight + stop speed)
  (0x70, 0, 0, 5, 8),   # destination approach (variant) -> Carrot xTurnInfo=8 (arrive)
  (0x72, 0, 0, 5, 8),   # destination approach -> Carrot xTurnInfo=8 (arrive)
  (0x54, 0, 0, 5, None),  # underpass straight -> inactive
  (0x73, 0, 0, 5, None),  # waypoint -> inactive
  (0x08, 0, 0, 5, None),  # compass / no route
  (0x09, 0, 0, 5, None),  # recalculating
  (0x03, 0, 0, 5, None),  # arrived
  (0x02, 0, 0, 5, None),  # off-route / unmapped
  (0x99, 0, 0, 5, None),  # unknown code -> fail-closed
))
def test_ev4_2a3_tbt_decode_mapping(display_type, direction, bar_level, link_class, expected):
  decoded = CarState._decode_vehicle_navi_tbt({
    "TBT_DISPLAY_TYPE": display_type,
    "TBT_DIRECTION": direction,
    "TBT_BAR_GRAPH_LEVEL": bar_level,
    "TBT_DISTANCE_TURN_POINT": 180,
    "TBT_SCALE": 0,
    "TBT_DIST_DECIMAL": 0,
    "TBT_NEXT_DISTANCE": 2,
    "TBT_NEXT_SCALE": 1,
    "TBT_NEXT_DIST_DECIMAL": 4,
  }, link_class=link_class)
  if expected is None:
    assert decoded is None
  else:
    assert decoded["turn_info"] == expected
    assert decoded["dist_m"] == pytest.approx(180.0)
    assert decoded["next_dist_m"] == pytest.approx(2400.0)


def test_ev4_2a3_dead_reckoning_counter_burst_guards_and_timeout(monkeypatch):
  cs, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4)
  bus = parsers[Bus.pt].bus
  hda = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1}))

  # Repeated & jumping ALIVE_COUNTER values (matching real EV4 bursts) must pass CRC-16 and update smoothly.
  msg1 = packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 129, "TBT_DISPLAY_TYPE": 0x42, "TBT_DISTANCE_TURN_POINT": 200, "TBT_SCALE": 0,
    "TBT_NEXT_DISTANCE": 350, "TBT_NEXT_SCALE": 0,
  })
  ret1 = _step(cs, parsers, 1.0, [hda, (Bus.pt, msg1)], v_ego=20.0)
  assert ret1.vehicleNaviTurnInfo == 4
  assert ret1.vehicleNaviTurnDistance == pytest.approx(200.0)
  assert ret1.vehicleNaviTurnNextDistance == pytest.approx(350.0)
  assert ret1.vehicleNaviTurnDisplayType == 0x42

  # Same 200m anchor + repeated ALIVE_COUNTER=129: dead-reckons smoothly down to raw - step (190m floor).
  for i in range(1, 20):
    ret_mid = _step(cs, parsers, 1.0 + 0.05 * i, [hda, (Bus.pt, msg1)], v_ego=20.0)
  assert 190.0 <= ret_mid.vehicleNaviTurnDistance < 200.0
  # Floor clamp prevents drifting below raw_dist - step_m (200 - 10 = 190m)
  for i in range(20, 80):
    ret_floor = _step(cs, parsers, 1.0 + 0.05 * i, [hda, (Bus.pt, msg1)], v_ego=20.0)
  assert ret_floor.vehicleNaviTurnDistance == pytest.approx(190.0)

  # Counter jump (129 -> 143) + new distance step (190m) re-anchors cleanly.
  msg2 = packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 143, "TBT_DISPLAY_TYPE": 0x42, "TBT_DISTANCE_TURN_POINT": 190, "TBT_SCALE": 0,
  })
  ret2 = _step(cs, parsers, 5.1, [hda, (Bus.pt, msg2)], v_ego=20.0)
  assert ret2.vehicleNaviTurnInfo == 4
  assert ret2.vehicleNaviTurnDistance == pytest.approx(190.0)

  # Recalculating (0x09 / 0xFFFF) clears immediately.
  msg_reroute = packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 144, "TBT_DISPLAY_TYPE": 0x09, "TBT_DISTANCE_TURN_POINT": 0xFFFF, "TBT_SCALE": 0,
  })
  ret_reroute = _step(cs, parsers, 5.3, [hda, (Bus.pt, msg_reroute)], v_ego=20.0)
  assert ret_reroute.vehicleNaviTurnInfo == -1
  assert ret_reroute.vehicleNaviTurnDistance == 0.0

  # Freshness timeout (> 2.0 s) clears TBT.
  ret_fresh = _step(cs, parsers, 6.0, [hda, (Bus.pt, msg2)], v_ego=20.0)
  assert ret_fresh.vehicleNaviTurnInfo == 4
  ret_stale = _step(cs, parsers, 8.5, [hda], v_ego=20.0)
  assert ret_stale.vehicleNaviTurnInfo == -1

  # Non-zero reserved bytes (byte 3..7 or 22..23) or wrong length (32B CAM_0x2a3) fail closed.
  addr, dat, src = msg2
  bad_pad = bytearray(dat)
  bad_pad[4] = 0x01
  assert _step(cs, parsers, 9.0, [hda, (Bus.pt, (addr, bytes(bad_pad), src))], v_ego=20.0).vehicleNaviTurnInfo == -1
  assert _step(cs, parsers, 9.2, [hda, (Bus.pt, (addr, dat + bytes(8), src))], v_ego=20.0).vehicleNaviTurnInfo == -1


def test_carrot_serv_vehicle_navi_atc_gating_and_priority():
  serv = _make_serv(1)
  serv.lang = "ko"
  serv.nRoadLimitSpeed = 100
  serv.nTBTNextRoadWidth = 5
  serv.autoTurnControl = 2
  serv.autoTurnControlSpeedTurn = 25
  serv.autoTurnControlTurnEnd = 6
  serv.autoTurnMapChange = 0
  serv.atc_paused = False
  serv.atc_activate_count = 0
  serv.xTurnInfo = -1
  serv.xDistToTurn = 0.0
  serv.szTBTMainText = ""
  serv.vehicleNaviAtcControl = False
  serv.vehicle_navi_tbt_active = False

  cs = _serv_cs(speed_limit=100, v_ego=25.0)
  cs.vehicleNaviTurnInfo = 4
  cs.vehicleNaviTurnDistance = 220.0
  cs.vehicleNaviTurnDisplayType = 0xA4
  sm = {"carState": SimpleNamespace(steeringPressed=False, steeringTorque=0.0)}

  # Default opt-in off -> stays inactive
  serv._apply_vehicle_navi_tbt(cs)
  assert serv.xTurnInfo == -1
  assert serv.szTBTMainText == ""

  # Opt-in on -> populates xTurnInfo/xDistToTurn/szTBTMainText and drives ATC fork right
  serv.vehicleNaviAtcControl = True
  serv._apply_vehicle_navi_tbt(cs)
  assert (serv.xTurnInfo, serv.xDistToTurn, serv.szTBTMainText) == (4, pytest.approx(220.0), "우측 출구")
  atc_desired, atc_type, _, _ = serv.update_auto_turn(90.0, sm, serv.xTurnInfo, serv.xDistToTurn, True)
  assert atc_type == "fork right"
  assert atc_desired >= 100.0

  # U-turn (xTurnInfo=7) produces straight + stop-speed (no dangerous left-turn steering)
  serv.xTurnInfo = -1
  cs.vehicleNaviTurnInfo = 7
  cs.vehicleNaviTurnDistance = 35.0
  cs.vehicleNaviTurnDisplayType = 0x48
  serv._apply_vehicle_navi_tbt(cs)
  assert (serv.xTurnInfo, serv.szTBTMainText) == (7, "유턴")
  atc_desired_u, atc_type_u, _, _ = serv.update_auto_turn(50.0, sm, serv.xTurnInfo, serv.xDistToTurn, True)
  assert atc_type_u == "straight"
  assert atc_desired_u < 45.0

  # Openpilot navInstruction already set (xTurnInfo >= 0) -> stock navi must not overwrite
  serv.xTurnInfo = 1
  serv.xDistToTurn = 90.0
  serv.szTBTMainText = "NavInstruction Left"
  serv._apply_vehicle_navi_tbt(cs)
  assert (serv.xTurnInfo, serv.xDistToTurn, serv.szTBTMainText) == (1, 90.0, "NavInstruction Left")

  # External navigation active -> stock navi must not override
  serv.external_navigation_active = True
  serv.xTurnInfo = -1
  serv.szTBTMainText = "External Tmap"
  serv._apply_vehicle_navi_tbt(cs)
  assert serv.xTurnInfo == -1
  assert serv.szTBTMainText == "External Tmap"

  # Clearing TBT clears stock-navi szTBTMainText cleanly
  serv.external_navigation_active = False
  serv.szTBTMainText = ""
  cs.vehicleNaviTurnInfo = 2
  cs.vehicleNaviTurnDistance = 120.0
  cs.vehicleNaviTurnDisplayType = 0x43
  serv._apply_vehicle_navi_tbt(cs)
  assert serv.szTBTMainText == "우회전"
  serv.xTurnInfo = -1
  cs.vehicleNaviTurnInfo = -1
  cs.vehicleNaviTurnDistance = 0.0
  cs.vehicleNaviTurnDisplayType = 0x08
  serv._apply_vehicle_navi_tbt(cs)
  assert serv.szTBTMainText == ""
  assert serv.vehicle_navi_tbt_active is False

  # Close city turn holds through intersection when 0x2A3 advances early to next far maneuver,
  # and drops immediately on reroute/compass (0x09/0x08) or after -35m.
  cs.vehicleNaviTurnInfo = 2
  cs.vehicleNaviTurnDistance = 15.0
  cs.vehicleNaviTurnDisplayType = 0x43
  serv._apply_vehicle_navi_tbt(cs, delta_dist=1.0)
  assert (serv.xTurnInfo, serv.xDistToTurn) == (2, pytest.approx(15.0))
  # 0x2A3 jumps early to next maneuver (U-turn 900m) -> keep holding turn right through intersection
  serv.xTurnInfo = -1
  cs.vehicleNaviTurnInfo = 7
  cs.vehicleNaviTurnDistance = 900.0
  cs.vehicleNaviTurnDisplayType = 0x48
  serv._apply_vehicle_navi_tbt(cs, delta_dist=10.0)
  assert (serv.xTurnInfo, serv.xDistToTurn, serv.szTBTMainText) == (2, pytest.approx(5.0), "우회전")
  serv.xTurnInfo = -1
  serv._apply_vehicle_navi_tbt(cs, delta_dist=15.0)
  assert (serv.xTurnInfo, serv.xDistToTurn) == (2, pytest.approx(-10.0))
  # Past -35m -> releases to next maneuver
  serv.xTurnInfo = -1
  serv._apply_vehicle_navi_tbt(cs, delta_dist=30.0)
  assert (serv.xTurnInfo, serv.xDistToTurn, serv.szTBTMainText) == (7, pytest.approx(900.0), "유턴")
  # Explicit reroute (0x09) cancels hold immediately
  serv.xTurnInfo = -1
  cs.vehicleNaviTurnInfo = 1
  cs.vehicleNaviTurnDistance = 10.0
  cs.vehicleNaviTurnDisplayType = 0x46
  serv._apply_vehicle_navi_tbt(cs, delta_dist=1.0)
  serv.xTurnInfo = -1
  cs.vehicleNaviTurnInfo = -1
  cs.vehicleNaviTurnDistance = 0.0
  cs.vehicleNaviTurnDisplayType = 0x09
  serv._apply_vehicle_navi_tbt(cs, delta_dist=2.0)
  assert serv.xTurnInfo == -1
  assert serv.vehicle_navi_tbt_hold is None
  # Valhalla maneuver mapping for navInstructionCarrot
  assert serv._vehicle_navi_tbt_maneuver(0x46, 1) == ("turn", "left")
  assert serv._vehicle_navi_tbt_maneuver(0x43, 2) == ("turn", "right")
  assert serv._vehicle_navi_tbt_maneuver(0xA4, 4) == ("off ramp", "right")
  assert serv._vehicle_navi_tbt_maneuver(0x41, 3) == ("fork", "left")


def test_carrot_planner_atc_active_accepts_stock_navi_tbt():
  import time
  class SM(dict):
    alive = {"carrotMan": True}
    valid = {"carrotMan": True}
    seen = {"carrotMan": True}
    recv_time = {"carrotMan": time.monotonic()}
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.signal_detection_suspended = False
  planner.trafficState_carrot = 0
  planner.carrot_stay_stop = False
  planner.soft_hold_active = 0
  planner.xState = 0
  planner.traffic_starting_count = 0
  sm = SM(
    carState=SimpleNamespace(leftBlinker=False),
    carrotMan=SimpleNamespace(
      activeCarrot=0,
      xTurnInfo=1,
      xDistToTurn=60,
      atcType="atc left",
      trafficState=0,
      desiredSpeed=45,
    ),
  )
  v_cruise, atc_active = planner._update_carrot_man(sm, 40.0, 60.0)
  assert v_cruise == 45.0
  assert atc_active is True


def test_maneuver_classifier_imminent_urban_atc_turn_without_wide_edge():
  from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type
  cs = SimpleNamespace(vEgo=26.0 / 3.6, aEgo=-0.5)
  side = SimpleNamespace(
    lane_available=False,
    edge_available=False,
    dist_to_edge_far=1.8,  # urban curb <= 4.0m
    lane_exist_count=SimpleNamespace(counter=0),
  )
  # Beyond 20m -> still waits for edge_far > 4.0
  assert classify_maneuver_type(2, cs, side, False, "turn right", "lane_change", atc_dist=30.0) == "lane_change"
  # Within 20m in outer turn lane at turn speed -> transitions directly to "turn"
  assert classify_maneuver_type(2, cs, side, False, "turn right", "lane_change", atc_dist=18.0) == "turn"
  # If another lane still exists on the turn side -> does not skip lane change
  side.lane_available = True
  assert classify_maneuver_type(2, cs, side, False, "turn right", "lane_change", atc_dist=18.0) == "lane_change"


def test_vehicle_bump_active_carrot_does_not_clobber_stock_navi_instruction(monkeypatch):
  from openpilot.selfdrive.carrot import carrot_serv
  from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _car_state

  values = {
    "AutoNaviSpeedBumpSpeed": 25, "AutoNaviSpeedBumpTime": 4,
    "AutoNaviSpeedBumpEndDistance": 200, "AutoNaviSpeedCtrlMode": 2,
    "AutoNaviSpeedCtrlEnd": 10, "AutoNaviSpeedDecelRate": 120,
    "AutoNaviRearCameraHoldDistance": 100, "AutoNaviSpeedSafetyFactor": 100,
    "AutoNaviCountDownMode": 0, "VehicleNaviCanControl": 1,
    "VehicleNaviAtcControl": 1, "AutoTurnControl": 2,
    "AutoTurnControlSpeedTurn": 25, "AutoTurnControlTurnEnd": 5,
    "IsMetric": 1, "AutoRoadSpeedLimitOffset": -1,
  }

  class _Params:
    def __init__(self, *_args):
      pass
    def get(self, key):
      return str(values.get(key, 0))
    def get_int(self, key):
      return int(self.get(key))
    def get_float(self, key):
      return float(self.get(key))
    def get_bool(self, key):
      return bool(self.get_int(key))

  monkeypatch.setattr(carrot_serv, "Params", _Params)
  serv = carrot_serv.CarrotServ()
  monkeypatch.setattr(serv, "_update_carrot_navi", lambda _sm: False)
  monkeypatch.setattr(serv, "_update_gps", lambda *_args: 0.)

  cs = _car_state(speed_limit=50, distance=0, v_ego=30 / 3.6)
  cs.vehicleNaviActive = cs.vehicleNaviAvailable = True
  cs.speedBumpDistance = 20.0  # triggers self.active_carrot = 5 without external nav
  cs.vehicleNaviTurnInfo = 2
  cs.vehicleNaviTurnDistance = 80.0
  cs.vehicleNaviTurnDisplayType = 0x10

  class _SM(dict):
    alive = {"carState": True, "selfdriveState": True, "navInstruction": False}
    valid = {"carState": True, "selfdriveState": True, "navInstruction": False}

  sm = _SM(carState=cs, selfdriveState=SimpleNamespace(distanceTraveled=0.))
  sent = {}
  pm = SimpleNamespace(send=lambda name, msg: sent.update({name: msg}))
  serv.update_navi("", sm, pm, 200., [], [], 200., "gpsLocationExternal")

  nic = sent["navInstructionCarrot"]
  assert sent["carrotMan"].carrotMan.activeCarrot == 5
  assert nic.valid
  assert nic.navInstructionCarrot.maneuverType == "turn"
  assert nic.navInstructionCarrot.maneuverModifier == "right"
  assert nic.navInstructionCarrot.maneuverDistance == pytest.approx(80.0)
  assert len(nic.navInstructionCarrot.allManeuvers) == 1


def test_carplay_tmap_0x2a3_0x41_straight_and_0x93_0x95_ramps(monkeypatch):
  from opendbc.car.hyundai.carstate import CarState
  from openpilot.selfdrive.carrot import carrot_serv
  from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _car_state

  # Stock navi 0x41 (side_street=0) -> left fork (3)
  stock_41 = CarState._decode_vehicle_navi_tbt({
    "TBT_DISPLAY_TYPE": 0x41, "TBT_SIDE_STREET": 0x0000,
    "TBT_DISTANCE_TURN_POINT": 300, "TBT_SCALE": 0, "TBT_DIST_DECIMAL": 0,
  })
  assert stock_41["turn_info"] == 3

  # CarPlay TMap 0x41 (side_street=0xFFFF) -> straight NOO-only guidance (9), never left fork
  carplay_41 = CarState._decode_vehicle_navi_tbt({
    "TBT_DISPLAY_TYPE": 0x41, "TBT_SIDE_STREET": 0xFFFF,
    "TBT_DISTANCE_TURN_POINT": 2, "TBT_SCALE": 1, "TBT_DIST_DECIMAL": 4,
  })
  assert carplay_41["turn_info"] == 9
  assert carplay_41["dist_m"] == pytest.approx(2400.0)

  # CarPlay TMap 0x93 & 0x95 -> right IC/JC exit ramp (4)
  for code in (0x93, 0x95):
    dec = CarState._decode_vehicle_navi_tbt({
      "TBT_DISPLAY_TYPE": code, "TBT_SIDE_STREET": 0xFFFF,
      "TBT_DISTANCE_TURN_POINT": 120, "TBT_SCALE": 0, "TBT_DIST_DECIMAL": 0,
    })
    assert dec["turn_info"] == 4
    assert dec["step_m"] == 5.0
    assert carrot_serv.CarrotServ._vehicle_navi_tbt_maneuver(code, 4) == ("off ramp", "right")

  # Verify turn_info=9 keeps ATC off (xTurnInfo=-1) while still emitting continue/straight navInstructionCarrot
  values = {
    "AutoNaviSpeedBumpSpeed": 25, "AutoNaviSpeedBumpTime": 4,
    "AutoNaviSpeedBumpEndDistance": 200, "AutoNaviSpeedCtrlMode": 2,
    "AutoNaviSpeedCtrlEnd": 10, "AutoNaviSpeedDecelRate": 120,
    "AutoNaviRearCameraHoldDistance": 100, "AutoNaviSpeedSafetyFactor": 100,
    "AutoNaviCountDownMode": 0, "VehicleNaviCanControl": 1,
    "VehicleNaviAtcControl": 1, "AutoTurnControl": 2,
    "AutoTurnControlSpeedTurn": 25, "AutoTurnControlTurnEnd": 5,
    "IsMetric": 1, "AutoRoadSpeedLimitOffset": -1,
  }
  class _Params:
    def __init__(self, *_args): pass
    def get(self, key): return str(values.get(key, 0))
    def get_int(self, key): return int(self.get(key))
    def get_float(self, key): return float(self.get(key))
    def get_bool(self, key): return bool(self.get_int(key))

  monkeypatch.setattr(carrot_serv, "Params", _Params)
  serv = carrot_serv.CarrotServ()
  monkeypatch.setattr(serv, "_update_carrot_navi", lambda _sm: False)
  monkeypatch.setattr(serv, "_update_gps", lambda *_args: 0.)

  cs = _car_state(speed_limit=80, distance=0, v_ego=75 / 3.6)
  cs.vehicleNaviActive = cs.vehicleNaviAvailable = True
  cs.vehicleNaviTurnInfo = 9
  cs.vehicleNaviTurnDistance = 1200.0
  cs.vehicleNaviTurnDisplayType = 0x41

  class _SM(dict):
    alive = {"carState": True, "selfdriveState": True, "navInstruction": False}
    valid = {"carState": True, "selfdriveState": True, "navInstruction": False}

  sm = _SM(carState=cs, selfdriveState=SimpleNamespace(distanceTraveled=0.))
  sent = {}
  pm = SimpleNamespace(send=lambda name, msg: sent.update({name: msg}))
  serv.update_navi("", sm, pm, 200., [], [], 200., "gpsLocationExternal")

  assert sent["carrotMan"].carrotMan.xTurnInfo == -1
  assert sent["carrotMan"].carrotMan.atcType == "none"
  assert serv.vehicle_navi_straight_active
  nic = sent["navInstructionCarrot"]
  assert nic.valid
  assert nic.navInstructionCarrot.maneuverType == "continue"
  assert nic.navInstructionCarrot.maneuverModifier == "straight"


def test_ev4_098_ramp_tbt_auto_arm_suppression_and_6s_hold(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4, can_control=1, ramp_control=True)
  bus = parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=0, path_idx=8)))
  hda_fork = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 3}))
  hda_clear = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 0}))
  hda_local = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 0, "Frwinfo": 0}))
  ramp_curve = (Bus.pt, packer.make_can_msg("CANFD_NAVI_RAMP_098", bus, {
    "PROSHORT_OFFSET": 400, "PROSHORT_PATH_INDEX": 9, "PROSHORT_DISTANCE": 5,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  stub_right = (Bus.pt, packer.make_can_msg("CANFD_NAVI_STUB_09A", bus, {
    "STUB_OFFSET": 350, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x49, "STUB_TURN_ANGLE": 16.875,
  }))
  tbt_straight = (Bus.pt, packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 1, "TBT_DISPLAY_TYPE": 0x41, "TBT_SIDE_STREET": 0xFFFF,
    "TBT_DISTANCE_TURN_POINT": 1, "TBT_SCALE": 1, "TBT_DIST_DECIMAL": 2,
  }))
  tbt_left_city = (Bus.pt, packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 2, "TBT_DISPLAY_TYPE": 0x46, "TBT_SIDE_STREET": 0x0000,
    "TBT_DISTANCE_TURN_POINT": 65, "TBT_SCALE": 0, "TBT_DIST_DECIMAL": 0,
  }))
  tbt_exit_far = (Bus.pt, packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 3, "TBT_DISPLAY_TYPE": 0x42, "TBT_SIDE_STREET": 0x0000,
    "TBT_DISTANCE_TURN_POINT": 700, "TBT_SCALE": 0, "TBT_DIST_DECIMAL": 0,
  }))
  tbt_exit_right = (Bus.pt, packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 4, "TBT_DISPLAY_TYPE": 0x93, "TBT_SIDE_STREET": 0xFFFF,
    "TBT_DISTANCE_TURN_POINT": 250, "TBT_SCALE": 0, "TBT_DIST_DECIMAL": 0,
  }))
  tbt_no_route = (Bus.pt, packer.make_can_msg("CANFD_NAVI_TBT_2A3", bus, {
    "ALIVE_COUNTER": 5, "TBT_DISPLAY_TYPE": 0x08, "TBT_SIDE_STREET": 0x0000,
    "TBT_DISTANCE_TURN_POINT": 0xFFFF, "TBT_SCALE": 0, "TBT_DIST_DECIMAL": 0,
  }))

  # 1. Active 0x2A3 TBT straight (CarPlay 0x41 -> turn_info=9) suppresses right lane-change blinker false trigger.
  ret = _step(CS, parsers, 1.0, [seg_frame, hda_fork, ramp_curve, stub_right, tbt_straight], v_ego=20.0, right_blinker=True)
  assert ret.vehicleNaviTurnInfo == 9
  assert ret.vehicleNaviCurveIsRamp is False
  assert ret.vehicleNaviCurveSpeed == 0.0

  # 2. Active 0x2A3 TBT non-exit (city left 0x46 @ 65m, or far exit > 500m) also suppresses right blinker.
  ret = _step(CS, parsers, 1.5, [hda_fork, tbt_left_city], v_ego=20.0, right_blinker=True)
  assert ret.vehicleNaviCurveIsRamp is False
  ret = _step(CS, parsers, 2.0, [hda_fork, tbt_exit_far], v_ego=20.0, right_blinker=True)
  assert ret.vehicleNaviCurveIsRamp is False

  # 3. Matching 0x2A3 TBT right exit within 500m arms 0x098 automatically with NO blinker!
  ret = _step(CS, parsers, 2.5, [hda_fork, tbt_exit_right], v_ego=20.0, right_blinker=False)
  assert ret.vehicleNaviTurnInfo == 4
  assert ret.vehicleNaviCurveIsRamp is True
  assert ret.vehicleNaviCurveSpeed > 0.0

  # 4. At ramp entry, Frwinfo drops (3 -> 0) and 0x2A3 advances to next straight maneuver: 6s hold bridges cleanly.
  ret = _step(CS, parsers, 5.5, [hda_clear, tbt_straight], v_ego=20.0, right_blinker=False)
  assert ret.vehicleNaviCurveIsRamp is True
  ret = _step(CS, parsers, 8.4, [hda_clear, tbt_straight], v_ego=20.0, right_blinker=False)
  assert ret.vehicleNaviCurveIsRamp is True
  # After 6s hold expires (2.5 + 6.0 = 8.5s), drops cleanly.
  ret = _step(CS, parsers, 8.6, [hda_clear, tbt_straight], v_ego=20.0, right_blinker=False)
  assert ret.vehicleNaviCurveIsRamp is False

  # 5. Free driving (0x08 compass / no TBT): right blinker arms, blinker-off holds for 6s, opposite blinker cancels immediately.
  ret = _step(CS, parsers, 10.0, [hda_fork, tbt_no_route], v_ego=20.0, right_blinker=True)
  assert ret.vehicleNaviCurveIsRamp is True
  ret = _step(CS, parsers, 13.0, [hda_clear, tbt_no_route], v_ego=20.0, right_blinker=False)
  assert ret.vehicleNaviCurveIsRamp is True
  # Opposite blinker immediately cancels hold.
  ret = _step(CS, parsers, 13.5, [hda_clear, tbt_no_route], v_ego=20.0, left_blinker=True)
  assert ret.vehicleNaviCurveIsRamp is False

  # 6. Re-arm and verify hazards or leaving controlled-access road cancels hold immediately.
  assert _step(CS, parsers, 14.0, [hda_fork, tbt_no_route], v_ego=20.0, right_blinker=True).vehicleNaviCurveIsRamp is True
  assert _step(CS, parsers, 14.5, [hda_clear, tbt_no_route], v_ego=20.0, left_blinker=True, right_blinker=True).vehicleNaviCurveIsRamp is False
  assert _step(CS, parsers, 15.0, [hda_fork, tbt_no_route], v_ego=20.0, right_blinker=True).vehicleNaviCurveIsRamp is True
  assert _step(CS, parsers, 15.5, [hda_local, tbt_no_route], v_ego=20.0).vehicleNaviCurveIsRamp is False
  assert _step(CS, parsers, 16.0, [hda_clear, tbt_no_route], v_ego=20.0).vehicleNaviCurveIsRamp is False


def test_ev4_093_ignores_branch_path_9_camera_alongside_mainline_path_8_camera(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4, can_control=1)
  bus = parsers[Bus.pt].bus
  hda = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "SPEED_LIMIT": 90, "MapSource": 1}))
  seg = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=0, path_idx=8)))
  # Real EV4 00000371--4 t=294.66 0x093 frame:
  #   slot 0: path_index=8 (mainline), 90 km/h @ 815m
  #   slot 1: path_index=9 (off-ramp branch), 40 km/h @ 810m
  raw_093 = bytes.fromhex("cdb82effffffffff320100002fe3081c920000002ae30904")
  ret = _step(CS, parsers, 1.0, [hda, seg, (Bus.pt, (0x093, raw_093, bus))], v_ego=26.5)

  cameras = [e for e in CS.vehicleNaviEvents if e["type"] == "camera"]
  assert len(cameras) == 1
  assert cameras[0]["speed"] == 90
  assert cameras[0]["target"] - CS.totalDistance == pytest.approx(815.0)
  assert ret.speedLimit == pytest.approx(90.0)
  assert CS.vehicleNaviCameraTarget - CS.totalDistance == pytest.approx(815.0)

  # Even before 0x099 is received, EV4 must reject path_index=9 branch cameras
  CS2, parsers2, packer2 = _make_hyundai_state(monkeypatch, CAR.KIA_EV4, can_control=1)
  bus2 = parsers2[Bus.pt].bus
  hda2 = (Bus.pt, packer2.make_can_msg("CANFD_HDA_INFO_364", bus2, {"LinkClass": 1, "SPEED_LIMIT": 90, "MapSource": 1}))
  ret2 = _step(CS2, parsers2, 1.0, [hda2, (Bus.pt, (0x093, raw_093, bus2))], v_ego=26.5)
  cameras2 = [e for e in CS2.vehicleNaviEvents if e["type"] == "camera"]
  assert len(cameras2) == 1
  assert cameras2[0]["speed"] == 90
  assert ret2.speedLimit == pytest.approx(90.0)
  assert CS2.vehicleNaviCameraTarget - CS2.totalDistance == pytest.approx(815.0)


@pytest.mark.parametrize(
  ("radius_m", "expected_kph"),
  (
    (800.0, 150.0),
    (440.0, 100.0),
    (190.0, 70.0),
    (135.0, 60.0),
    (85.0, 50.0),
    (55.0, 40.0),
    (30.0, 15.0),
    (25.0, 5.0),
  ),
)
def test_vnavi_curve_reference_speed_matches_external_navi_lookup(radius_m, expected_kph):
  assert CarState._vehicle_navi_curve_reference_speed(1.0 / radius_m) == pytest.approx(expected_kph)
  assert CarState._vehicle_navi_curve_reference_speed(-1.0 / radius_m) == pytest.approx(expected_kph)
