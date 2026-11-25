from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


@pytest.fixture(params=[False, True], ids=['hda1', 'hda2'])
def ev4(request, monkeypatch):
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.side_effect = lambda key: key == 'ControlsReady'
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, 'Params', lambda: params)
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: params)
  flags = HyundaiFlags.CANFD | HyundaiFlags.EV | HyundaiFlags.CANFD_CAMERA_SCC
  if request.param:
    flags |= HyundaiFlags.CANFD_HDA2
  cp = structs.CarParams(carFingerprint=CAR.KIA_EV4, flags=int(flags), safetyConfigs=[{}])
  state = carstate.CarState(cp)
  state.is_metric = True
  state.vehicleNaviCanControl = 1
  parsers = state.get_can_parsers(cp)
  for parser in parsers.values():
    parser.controls_ready = True
  return state, parsers, CANPacker('hyundai_canfd_generated')


def receive(state, parsers, time_s, frames=()):
  for parser in parsers.values():
    parser.update([int(time_s * 1e9), list(frames)])
  state.monitor_fingerprint(parsers, True)


def output(state, parser, speed=0.):
  ret = SimpleNamespace(speedLimit=speed, vEgo=0.)
  state._update_vehicle_navi_events(parser, ret, False)
  return ret


def test_ev4_logged_camera_is_cached_before_fingerprint_registration(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  # EV4 0000030c/8, camera preview 60 km/h at 2000 m.
  receive(state, parsers, 1., [(0x93, bytes.fromhex('89090cffffffffffd1000000d0e70814ffffffffffffffff'), parser.bus)])
  state.controls_ready_count = 122
  receive(state, parsers, 1.2)
  assert state.navi_profile_4be is parser.vl[carstate.CANFD_NAVI_PROFILE_MSG]
  ret = output(state, parser)
  assert ret.vehicleNaviAvailable and ret.vehicleNaviActive
  assert ret.speedLimit == 60.
  assert state.vehicleNaviCameraTarget == 2000.


@pytest.mark.parametrize('mode,route,expected', [(0, 1, False), (1, 0, True), (2, 0, False), (2, 1, True), (3, 1, True)])
def test_ev4_bump_uses_existing_mode_and_route_admission(ev4, mode, route, expected):
  state, parsers, packer = ev4
  parser = parsers[Bus.pt]
  state.vehicleNaviCanControl = mode
  state.controls_ready_count = 122
  raw = (8 << 13) | (route << 22) | (3 << 24)
  frames = [
    packer.make_can_msg(carstate.CANFD_NAVI_SEGMENT_MSG, parser.bus,
                       {f'BYTE_{i+1}': (raw >> (8*i)) & 255 for i in range(8)}),
    packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, parser.bus,
                       {'PROLONG_VALUE': 6, 'PROLONG_OFFSET': 150, 'PROLONG_PATH_INDEX': 8, 'PROLONG_PROFILE_TYPE': 16}),
  ]
  receive(state, parsers, 1., frames)
  ret = output(state, parser)
  assert ret.vehicleNaviActive == expected
  assert ret.speedBumpDistance == (150 if expected else 0)


def test_ev4_padding_does_not_refresh_route_or_keep_bump_after_timeout(ev4):
  state, parsers, packer = ev4
  parser = parsers[Bus.pt]
  state.vehicleNaviCanControl = 2
  state.controls_ready_count = 122
  raw = (8 << 13) | (1 << 22) | (3 << 24)
  receive(state, parsers, 1., [packer.make_can_msg(carstate.CANFD_NAVI_SEGMENT_MSG, parser.bus,
                             {f'BYTE_{i+1}': (raw >> (8*i)) & 255 for i in range(8)})])
  output(state, parser)
  state._add_vehicle_navi_event('bump', 0, 6, 100.)
  padding = (0x99, bytes.fromhex('502627ffffffffffffffffffffffffffffffffffffffffff'), parser.bus)
  receive(state, parsers, 1.1, [padding])
  assert output(state, parser).speedBumpDistance == 100.
  assert state.vehicleNaviSegmentTimestamp == 1_000_000_000
  receive(state, parsers, 3.1, [padding])
  assert output(state, parser).speedBumpDistance == 0.
  assert state.vehicleNaviRouteState == 0


def test_ev4_unvalidated_position_cannot_activate_section(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  state.controls_ready_count = 122
  # Real EV4 0x092 payload. The proposed legacy-position mapping decodes an
  # impossible 384 km/h average and would incorrectly establish a section cap.
  receive(state, parsers, 1., [(0x92, bytes.fromhex('24da2affffffffffd40c0070113dffff'), parser.bus)])
  assert not output(state, parser, 100.).vehicleNaviSectionActive


def test_ev4_logged_hda_marks_availability_without_profile(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  state.controls_ready_count = 122
  receive(state, parsers, 1., [(0x364, bytes.fromhex('a4517e0000000000416e9a0900000100'), parser.bus)])
  ret = output(state, parser)
  assert ret.vehicleNaviAvailable
  assert not ret.vehicleNaviActive
  assert state.hda_info_4a3['SPEED_LIMIT'] == 110
  assert state.hda_info_4a3['CountryCode'] == 410


def test_ev4_logged_bump_uses_profile_distance(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  state.controls_ready_count = 122
  receive(state, parsers, 1., [(0x93, bytes.fromhex('7b04ecffffffffff060000005de40804ffffffffffffffff'), parser.bus)])
  ret = output(state, parser)
  assert ret.vehicleNaviActive
  assert ret.speedBumpDistance == 1117.


def test_ev4_missing_optional_navigation_does_not_add_timeouts(ev4):
  state, parsers, packer = ev4
  parser = parsers[Bus.pt]
  for frame in range(1, 301):
    receive(state, parsers, frame * .02, [packer.make_can_msg('CRUISE_BUTTONS', parser.bus, {'COUNTER': frame % 16})])
  assert parser.can_valid
  assert not output(state, parser).vehicleNaviAvailable


# Logged EV4 frames, 0000033a--5e9dbab0d4 (2026-11-21 school-zone passes).
EV4_PROFILE_77 = bytes.fromhex('f00c8affffffffff7700000000e0080cffffffffffffffff')
EV4_PROFILE_D7 = bytes.fromhex('80d412ffffffffffd700000000e0080cffffffffffffffff')
EV4_STATUS = {  # (warning byte 10, limit byte 12) -> A-CAN 0x380/24B
  (0, 0): bytes.fromhex('a4d3a9000000200000000012001071000000000000000000'),
  (0, 30): bytes.fromhex('4ce6120000002000000000121e1071000000000000000000'),
  (0, 60): bytes.fromhex('5567f10000002000000000123c1071000000000000000000'),
  (1, 0): bytes.fromhex('2102d1000000200000000112001071000000000000000000'),
  (1, 30): bytes.fromhex('4d1ea40000002000000001121e1071000000000000000000'),
  # 00000320--5f1dce4517 (2026-11-18 motorway, three confirmed section-enforcement entries)
  (0, 100): bytes.fromhex('a6f177000000200000000012641071000000000000000000'),
  (0x10, 100): bytes.fromhex('3d8600000000200000001012641071000000000000000000'),
  (0x1b, 100): bytes.fromhex('6c392a000000200000001b12641071000000000000000000'),
  (0x10, 60): bytes.fromhex('b3257c0000002000000010123c1071000000000000000000'),
}


def school_output(state, parsers):
  ret = SimpleNamespace(speedLimit=0., vEgo=0.)
  state._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers[Bus.alt])
  return ret


def school_receive(state, parsers, time_s, status=None, profile=None):
  frames = []
  if status is not None:
    frames.append((0x380, EV4_STATUS[status], parsers[Bus.alt].bus))
  if profile is not None:
    frames.append((0x93, profile, parsers[Bus.pt].bus))
  receive(state, parsers, time_s, frames)
  return school_output(state, parsers)


def school_zone_entered(ev4):
  state, parsers, _ = ev4
  state.vehicleNaviSchoolZoneControl = True
  state.controls_ready_count = 122
  # 0x380 shows 30 about 0.6 s before 0x77 (5/5 logged school-zone passes).
  school_receive(state, parsers, 1.0, status=(0, 30))
  assert state.navi_status_380 is parsers[Bus.alt].vl[carstate.CANFD_NAVI_STATUS_MSG]
  ret = school_receive(state, parsers, 1.6, status=(0, 30), profile=EV4_PROFILE_77)
  return state, parsers, ret


def test_ev4_school_zone_confirmed_by_acan_limit(ev4):
  state, parsers, ret = school_zone_entered(ev4)
  # 0x364 stays MapSource=1 on EV4; the A-CAN 30 confirms instead.
  assert ret.schoolZoneActive
  assert ret.speedLimit == 30
  assert state.vehicleNaviSchoolZoneUsesNaviStatus
  assert not state.vehicleNaviSchoolZoneUsesCameraStatus


@pytest.mark.parametrize('status', [None, (0, 0), (0, 60)])
def test_ev4_school_zone_needs_acan_30(ev4, status):
  state, parsers, _ = ev4
  state.vehicleNaviSchoolZoneControl = True
  state.controls_ready_count = 122
  school_receive(state, parsers, 1.0, status=status)
  ret = school_receive(state, parsers, 1.6, status=status, profile=EV4_PROFILE_77)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


def test_ev4_acan_30_alone_does_not_activate_school_zone(ev4):
  state, parsers, _ = ev4
  state.vehicleNaviSchoolZoneControl = True
  state.controls_ready_count = 122
  for t in (1.0, 1.5, 2.0):
    ret = school_receive(state, parsers, t, status=(0, 30))
  assert not ret.schoolZoneActive


def test_ev4_school_zone_holds_through_logged_gaps(ev4):
  state, parsers, _ = school_zone_entered(ev4)
  t = 1.6
  # Logged inside-zone gaps: 30 hidden ~4 s without warning, ~7.7 s while
  # the pedestrian warning (code 1) stays on.
  for status, duration in (((0, 0), 4.0), ((1, 30), 5.0), ((1, 0), 7.7), ((0, 30), 2.0)):
    end = t + duration
    while t < end:
      t = round(t + 0.5, 3)
      ret = school_receive(state, parsers, t, status=status)
      assert ret.schoolZoneActive, (status, t)


def test_ev4_school_zone_releases_after_sustained_loss(ev4):
  state, parsers, _ = school_zone_entered(ev4)
  # The release timer starts at the first frame showing limit 0 / no warning.
  lost_start = t = 2.1
  while t < lost_start + 7.5:
    assert school_receive(state, parsers, t, status=(0, 0)).schoolZoneActive, t
    t = round(t + 0.5, 3)
  t = round(lost_start + 8.0, 3)
  assert not school_receive(state, parsers, t, status=(0, 0)).schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


def test_ev4_school_zone_releases_on_different_limit(ev4):
  state, parsers, _ = school_zone_entered(ev4)
  ret = school_receive(state, parsers, 2.0, status=(0, 60))
  assert not ret.schoolZoneActive


def test_ev4_school_zone_releases_when_acan_goes_stale(ev4):
  state, parsers, _ = school_zone_entered(ev4)
  ret = school_receive(state, parsers, 1.6 + 1.5)  # no 0x380 for 1.5 s
  assert not ret.schoolZoneActive


def test_ev4_school_zone_new_non_30_zone_profile_clears(ev4):
  state, parsers, _ = school_zone_entered(ev4)
  ret = school_receive(state, parsers, 2.0, status=(0, 60), profile=EV4_PROFILE_D7)
  assert not ret.schoolZoneActive


def test_ev4_school_zone_disabled_by_setting(ev4):
  state, parsers, _ = ev4
  state.vehicleNaviSchoolZoneControl = False
  state.controls_ready_count = 122
  school_receive(state, parsers, 1.0, status=(0, 30))
  ret = school_receive(state, parsers, 1.6, status=(0, 30), profile=EV4_PROFILE_77)
  assert not ret.schoolZoneActive


# 17:50:33 section entry: kind 7, offset 0, 100 km/h (0x157). The identical
# profile also arrives without a 0x380 section alert (17:28:40, not a section).
EV4_PROFILE_SECTION_100 = bytes.fromhex('e20b3affffffffff5701000000e0080cffffffffffffffff')


def section_entered(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  # 0x380 shows 0x10 at the section limit ~0.2-0.5 s before kind 7 (3/3 confirmed).
  school_receive(state, parsers, 1.0, status=(0x10, 100))
  ret = school_receive(state, parsers, 1.4, status=(0x10, 100), profile=EV4_PROFILE_SECTION_100)
  return state, parsers, ret


def test_ev4_section_confirmed_by_acan_alert(ev4):
  state, parsers, ret = section_entered(ev4)
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 100
  assert state.vehicleNaviSpeedZoneUsesNaviStatus


@pytest.mark.parametrize('status', [None, (0, 100), (0x10, 60), (0x1b, 100)])
def test_ev4_section_needs_alert_at_same_limit(ev4, status):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  school_receive(state, parsers, 1.0, status=status)
  ret = school_receive(state, parsers, 1.4, status=status, profile=EV4_PROFILE_SECTION_100)
  assert not ret.vehicleNaviSectionActive
  assert not state.vehicleNaviSpeedZoneActive


def test_ev4_section_alert_alone_does_not_activate(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  for t in (1.0, 1.5, 2.0):
    ret = school_receive(state, parsers, t, status=(0x10, 100))
  assert not ret.vehicleNaviSectionActive


def test_ev4_section_holds_while_limit_agrees(ev4):
  state, parsers, _ = section_entered(ev4)
  t = 1.4
  # The alert drops after ~5 s, a later 0x1B notice and an alert-less repeat
  # of the same profile must not end the section (17:50 pass).
  for status, duration, profile in (((0, 100), 10.0, None), ((0x1b, 100), 5.0, None),
                                    ((0, 100), 0.5, EV4_PROFILE_SECTION_100), ((0, 100), 60.0, None)):
    end = t + duration
    while t < end:
      t = round(t + 0.5, 3)
      ret = school_receive(state, parsers, t, status=status, profile=profile)
      profile = None
      assert ret.vehicleNaviSectionActive, (status, t)
      assert ret.vehicleNaviSpeed == 100


def test_ev4_section_releases_on_different_limit(ev4):
  state, parsers, _ = section_entered(ev4)
  ret = school_receive(state, parsers, 2.0, status=(0, 60))
  assert not ret.vehicleNaviSectionActive
  assert not state.vehicleNaviSpeedZoneActive


def test_ev4_section_releases_when_acan_goes_stale(ev4):
  state, parsers, _ = section_entered(ev4)
  ret = school_receive(state, parsers, 1.4 + 1.5)
  assert not ret.vehicleNaviSectionActive


def test_ev4_section_releases_after_sustained_blank_limit(ev4):
  state, parsers, _ = section_entered(ev4)
  lost_start = t = 2.0
  while t < lost_start + 7.5:
    assert school_receive(state, parsers, t, status=(0, 0)).vehicleNaviSectionActive, t
    t = round(t + 0.5, 3)
  assert not school_receive(state, parsers, round(lost_start + 8.0, 3), status=(0, 0)).vehicleNaviSectionActive


# 00000348--52a1230766 (2026-11-22). 0x094 announces the section once, with the
# start camera's 0x093 profile: 16:41:51 offset 1990 m, 397 x 20 m, 90 km/h;
# the driver confirmed the 16:43-16:51 section (7.9 km by odometer).
EV4_SECTION_094_7940 = bytes.fromhex('34a1c8ffffffffffc607f9c60000136e' + 'ff' * 16)
EV4_CURVE_094 = bytes.fromhex('7db8aaffffffffffce07f94f0460006a' + 'ff' * 16)  # type 1 curve slot
EV4_STATUS[(0, 90)] = bytes.fromhex('f210690000002000000000125a1071000000000000000000')


def section_094_receive(state, parsers, time_s, distance, status=(0, 90), proshort=None):
  state.totalDistance = distance
  frames = [(0x380, EV4_STATUS[status], parsers[Bus.alt].bus)] if status is not None else []
  if proshort is not None:
    frames.append((0x94, proshort, parsers[Bus.pt].bus))
  receive(state, parsers, time_s, frames)
  return school_output(state, parsers)


def section_094_announced(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  ret = section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)
  assert state.navi_proshort_094 is parsers[Bus.pt].vl[carstate.CANFD_NAVI_PROSHORT_MSG]
  return state, parsers, ret


def test_ev4_094_decodes_logged_section():
  values = {'PROSHORT_OFFSET': 1990, 'PROSHORT_PATH_INDEX': 8, 'PROSHORT_DISTANCE': 397,
            'PROSHORT_VALUE_0': 0, 'PROSHORT_VALUE_1': 608, 'PROSHORT_PROFILE_TYPE': 16}
  assert carstate.CarState._decode_ev4_navi_section(values) == (1990, 7940, 90, 8)
  assert carstate.CarState._decode_ev4_navi_section({**values, 'PROSHORT_VALUE_1': 544})[2] == 80


@pytest.mark.parametrize('change', [{'PROSHORT_PROFILE_TYPE': 1}, {'PROSHORT_VALUE_0': 2},
                                    {'PROSHORT_VALUE_1': 10}, {'PROSHORT_VALUE_1': 0x80},
                                    {'PROSHORT_DISTANCE': 0}, {'PROSHORT_OFFSET': 0}])
def test_ev4_094_rejects_non_section_profiles(change):
  values = {'PROSHORT_OFFSET': 1990, 'PROSHORT_PATH_INDEX': 8, 'PROSHORT_DISTANCE': 397,
            'PROSHORT_VALUE_0': 0, 'PROSHORT_VALUE_1': 608, 'PROSHORT_PROFILE_TYPE': 16}
  assert carstate.CarState._decode_ev4_navi_section({**values, **change}) is None


def test_ev4_094_section_spans_start_to_end(ev4):
  state, parsers, ret = section_094_announced(ev4)
  assert not ret.vehicleNaviSectionActive  # start camera is still 1990 m ahead
  t = 1.0
  for distance, inside in ((2989., False), (2990., True), (6000., True), (10929., True), (10930., False)):
    t = round(t + 0.5, 3)
    ret = section_094_receive(state, parsers, t, distance)
    assert ret.vehicleNaviSectionActive == inside, distance
    if inside:
      assert ret.vehicleNaviSpeed == 90
  assert state.ev4Section is None


def test_ev4_094_curve_slot_is_not_a_section(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_CURVE_094)
  assert state.ev4Section is None
  assert not section_094_receive(state, parsers, 1.5, 3000.).vehicleNaviSectionActive


def test_ev4_094_section_kept_through_different_approach_limit(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  # 16:11:03 / 16:57:59 / 17:31:58: announced while still on a 40/90/70 road.
  section_094_receive(state, parsers, 1.0, 1000., status=(0, 60), proshort=EV4_SECTION_094_7940)
  t = 1.0
  for _ in range(20):
    t = round(t + 0.5, 3)
    assert not section_094_receive(state, parsers, t, 2000., status=(0, 60)).vehicleNaviSectionActive
  assert state.ev4Section is not None
  assert section_094_receive(state, parsers, t + 0.5, 3000.).vehicleNaviSectionActive


def test_ev4_094_section_survives_blank_and_brief_other_limit(ev4):
  state, parsers, _ = section_094_announced(ev4)
  # Inside: 0x380 blanks its limit, and a ramp limit flickers for <3 s
  # (suspended meanwhile, never capped at the wrong road's limit).
  t, distance = 1.0, 3000.
  for status in [(0, 0)] * 10 + [(0, 60)] * 5 + [(0, 90)] * 2:
    t = round(t + 0.5, 3)
    distance += 10.
    ret = section_094_receive(state, parsers, t, distance, status=status)
    assert ret.vehicleNaviSectionActive == (status != (0, 60)), (t, status)
  assert state.ev4Section is not None


def test_ev4_094_section_drops_when_leaving_the_road(ev4):
  state, parsers, _ = section_094_announced(ev4)
  # 16:15:17: the driver took a 60 km/h exit; the start point is then reached
  # on that other road, which suspends and then drops the section.
  t = 1.0
  for _ in range(6):  # first 60 past the start at 1.5 s; still held at 4.0 s
    t = round(t + 0.5, 3)
    assert not section_094_receive(state, parsers, t, 3000., status=(0, 60)).vehicleNaviSectionActive
  assert state.ev4Section is not None
  section_094_receive(state, parsers, 4.5, 3000., status=(0, 60))
  assert state.ev4Section is None
  assert not section_094_receive(state, parsers, 5.0, 4000.).vehicleNaviSectionActive


def test_ev4_094_section_needs_navi_can_control(ev4):
  state, parsers, _ = section_094_announced(ev4)
  state.vehicleNaviCanControl = 0
  assert not section_094_receive(state, parsers, 1.5, 3000.).vehicleNaviSectionActive


def test_ev4_094_section_cleared_by_route_reset(ev4):
  state, parsers, _ = section_094_announced(ev4)
  state._clear_vehicle_navi_events()
  assert not section_094_receive(state, parsers, 1.5, 3000.).vehicleNaviSectionActive


# Real logged EV4 bus 0 0x363 frames (16B):
# - avg=0 during Route 348 S2 (POS_OFFSET=12, POS_RANGE_AVG_SPEED=0)
# - avg=510 (0x1FE) cold-boot invalid from 0000031f--0
EV4_POS_363_ZERO = bytes.fromhex('10f88600000000000c0009403092d004')
EV4_POS_363_BOOT_510 = bytes.fromhex('32b20000000000000000c03f00000000')
# EV9 0x4B4 active payload (0c00454b3092d08c, POS_RANGE_AVG_SPEED=90) in CANFD 0x363 wrapper
EV4_POS_363_AVG_90 = bytes.fromhex('00000000000000000c00454b3092d08c')
# Real EV4 bus 0 0x093 frames:
# - end90: Route 348 seg 49 t=3279.81, kind=0 90 km/h @ 1998 m (section end camera)
# - slot1: Route 348 seg 53 t=3523.60, slot 0 = ff*8, slot 1 = kind=2 40 km/h @ 840 m
EV4_PROFILE_093_END90 = bytes.fromhex('e61a72ffffffffff30010000cee7080cffffffffffffffff')
EV4_PROFILE_093_SLOT1_ONLY = bytes.fromhex('62c676ffffffffffffffffffffffffff9200000048e30904')


def _hda_frame(packer, bus, limit_kph, map_source=1):
  return packer.make_can_msg(carstate.CANFD_HDA_INFO_MSG, bus, {
    'LinkClass': 1,
    'SPEED_LIMIT': limit_kph,
    'CountryCode': 410,
    'MapSource': map_source,
  })


def _pos_363_receive(state, parsers, time_s, distance, frame_363, *, speed=90., status=None):
  state.totalDistance = distance
  frames = [(0x363, frame_363, parsers[Bus.pt].bus)] if frame_363 is not None else []
  if status is not None:
    frames.append((0x380, EV4_STATUS[status], parsers[Bus.alt].bus))
  receive(state, parsers, time_s, frames)
  ret = SimpleNamespace(speedLimit=speed, vEgo=20.)
  state._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers[Bus.alt])
  return ret


def test_ev4_363_position_dbc_ignores_zero_and_boot_invalid_and_keeps_094_and_380(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  parser = parsers[Bus.pt]

  ret = _pos_363_receive(state, parsers, 1.0, 100., EV4_POS_363_ZERO)
  assert state.navi_position_4b4 is parser.vl[carstate.CANFD_NAVI_POSITION_MSG]
  assert state.navi_position_4b4['POS_OFFSET'] == 12
  assert state.navi_position_4b4['POS_RANGE_AVG_SPEED'] == 0
  assert not ret.vehicleNaviSectionActive

  ret = _pos_363_receive(state, parsers, 1.2, 105., EV4_POS_363_BOOT_510)
  assert state.navi_position_4b4['POS_RANGE_AVG_SPEED'] == 510
  assert not ret.vehicleNaviSectionActive

  # Streaming avg=0 on 0x363 must never clear an active 0x094 section.
  section_094_receive(state, parsers, 1.5, 1000., proshort=EV4_SECTION_094_7940)
  ret = _pos_363_receive(state, parsers, 2.0, 3500., EV4_POS_363_ZERO, speed=0., status=(0, 90))
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 90
  assert state.ev4Section is not None

  # Nor may avg=0 clear an active A-CAN 0x380 kind-7 section.
  state.ev4Section = None
  school_receive(state, parsers, 2.2, status=(0x10, 100))
  school_receive(state, parsers, 2.6, status=(0x10, 100), profile=EV4_PROFILE_SECTION_100)
  ret = _pos_363_receive(state, parsers, 3.0, 3520., EV4_POS_363_ZERO, speed=0., status=(0, 100))
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 100


def test_ev4_363_range_average_activates_overrides_and_releases(ev4, monkeypatch):
  state, parsers, _ = ev4
  state.controls_ready_count = 122

  # Active 0x094 section takes backseat when 0x363 provides authoritative range average,
  # and both release cleanly when 0x363 drops back to 0 near section end.
  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)
  assert _pos_363_receive(state, parsers, 1.5, 3000., EV4_POS_363_ZERO, speed=0., status=(0, 90)).vehicleNaviSectionActive

  ret = _pos_363_receive(state, parsers, 2.0, 3050., EV4_POS_363_AVG_90, speed=90.)
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 90
  assert state.vehicleNaviSpeedZoneUsesPosition

  ret = _pos_363_receive(state, parsers, 2.5, 10880., EV4_POS_363_ZERO, speed=90.)
  assert not ret.vehicleNaviSectionActive
  assert not state.vehicleNaviSpeedZoneUsesPosition
  assert state.ev4Section is None

  # 0x363 timeout releases the position-latched section.
  ret = _pos_363_receive(state, parsers, 3.0, 12000., EV4_POS_363_AVG_90, speed=90.)
  assert ret.vehicleNaviSectionActive
  assert _pos_363_receive(state, parsers, 3.8, 12020., None, speed=90.).vehicleNaviSectionActive
  assert not _pos_363_receive(state, parsers, 4.1, 12030., None, speed=90.).vehicleNaviSectionActive

  # PV5 (canfd_wrapped_navi) also registers CANFD_NAVI_POSITION_363 and honors avg > 0.
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.side_effect = lambda key: key == 'ControlsReady'
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, 'Params', lambda: params)
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: params)
  pv5_cp = structs.CarParams(carFingerprint=CAR.KIA_PV5,
                             flags=int(HyundaiFlags.CANFD | HyundaiFlags.EV | HyundaiFlags.CANFD_CAMERA_SCC),
                             safetyConfigs=[{}])
  pv5_state = carstate.CarState(pv5_cp)
  pv5_state.is_metric = True
  pv5_state.vehicleNaviCanControl = 1
  pv5_parsers = pv5_state.get_can_parsers(pv5_cp)
  for p in pv5_parsers.values():
    p.controls_ready = True
  pv5_state.controls_ready_count = 122
  assert _pos_363_receive(pv5_state, pv5_parsers, 1.0, 100., EV4_POS_363_AVG_90, speed=90.).vehicleNaviSectionActive
  assert not _pos_363_receive(pv5_state, pv5_parsers, 1.5, 120., EV4_POS_363_ZERO, speed=90.).vehicleNaviSectionActive


def test_ev4_093_end_camera_snap_and_filters(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus

  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)
  assert state.ev4Section['end'] == 10930.

  # Mid-section 90 km/h kind=3 mobile camera near end, wrong-speed kind=0, and far kind=0 must NOT snap.
  state.totalDistance = 10000.
  ignored_frames = [
    packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, bus,
                        {'PROLONG_VALUE': 0x133, 'PROLONG_OFFSET': 880, 'PROLONG_CYCLIC_COUNTER': 1, 'PROLONG_PROFILE_TYPE': 16}),
    packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, bus,
                        {'PROLONG_VALUE': 0x110, 'PROLONG_OFFSET': 890, 'PROLONG_CYCLIC_COUNTER': 2, 'PROLONG_PROFILE_TYPE': 16}),
    packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, bus,
                        {'PROLONG_VALUE': 0x130, 'PROLONG_OFFSET': 400, 'PROLONG_CYCLIC_COUNTER': 3, 'PROLONG_PROFILE_TYPE': 16}),
  ]
  for i, frame in enumerate(ignored_frames):
    receive(state, parsers, 2.0 + i * 0.2, [frame])
    school_output(state, parsers)
  assert state.ev4Section['end'] == 10930.
  assert not state.ev4Section['end_snapped']

  # Too close to start (<= start + 500 m) cannot snap either.
  state.totalDistance = 3000.
  receive(state, parsers, 3.0, [packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, bus, {
    'PROLONG_VALUE': 0x130, 'PROLONG_OFFSET': 400, 'PROLONG_CYCLIC_COUNTER': 0, 'PROLONG_PROFILE_TYPE': 16})])
  school_output(state, parsers)
  assert state.ev4Section['end'] == 10930.

  # Real logged 0x093 kind=0 90 km/h @ 1998 m from d=8880 -> target=10878 (52 m before 10930).
  state.totalDistance = 8880.
  receive(state, parsers, 4.0, [(0x93, EV4_PROFILE_093_END90, bus)])
  school_output(state, parsers)
  assert state.ev4Section['end'] == 10878.
  assert state.ev4Section['end_snapped']
  assert section_094_receive(state, parsers, 4.5, 10877.).vehicleNaviSectionActive
  assert not section_094_receive(state, parsers, 5.0, 10878.).vehicleNaviSectionActive


def test_ev4_364_mapsource_falling_syncs_start_and_end(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus

  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)  # start=2990, end=10930

  # Start approach: MapSource=2 at 2960, drops 2->1 at 2975 (15 m before 2990).
  state.totalDistance = 2960.
  receive(state, parsers, 1.5, [_hda_frame(packer, bus, 90, map_source=2)])
  assert not state._update_vehicle_navi_events(parsers[Bus.pt], SimpleNamespace(speedLimit=90., vEgo=20.), True, parsers[Bus.alt])
  state.totalDistance = 2975.
  receive(state, parsers, 2.0, [_hda_frame(packer, bus, 90, map_source=1)])
  ret = SimpleNamespace(speedLimit=90., vEgo=20.)
  state._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers[Bus.alt])
  assert state.ev4Section['start'] == 2975.
  assert state.ev4Section['end'] == 10915.
  assert state.ev4Section['entered']
  assert ret.vehicleNaviSectionActive

  # Mid-section MapSource 2->1 at d=5000 must NOT end the section.
  state.totalDistance = 4900.
  receive(state, parsers, 2.5, [_hda_frame(packer, bus, 90, map_source=2)])
  state._update_vehicle_navi_events(parsers[Bus.pt], SimpleNamespace(speedLimit=90., vEgo=20.), True, parsers[Bus.alt])
  state.totalDistance = 5000.
  receive(state, parsers, 3.0, [_hda_frame(packer, bus, 90, map_source=1)])
  ret = SimpleNamespace(speedLimit=90., vEgo=20.)
  state._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers[Bus.alt])
  assert ret.vehicleNaviSectionActive
  assert state.ev4Section is not None

  # End camera MapSource 2->1 at d=10865 (50 m before 10915) finishes immediately.
  state.totalDistance = 10700.
  receive(state, parsers, 3.5, [_hda_frame(packer, bus, 90, map_source=2)])
  state._update_vehicle_navi_events(parsers[Bus.pt], SimpleNamespace(speedLimit=90., vEgo=20.), True, parsers[Bus.alt])
  state.totalDistance = 10865.
  receive(state, parsers, 4.0, [_hda_frame(packer, bus, 90, map_source=1)])
  ret = SimpleNamespace(speedLimit=90., vEgo=20.)
  state._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers[Bus.alt])
  assert not ret.vehicleNaviSectionActive
  assert state.ev4Section is None


def test_ev4_093_slot1_decodes_logged_frame_and_queues_camera(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus

  state.totalDistance = 200.
  receive(state, parsers, 1.0, [(0x93, EV4_PROFILE_093_SLOT1_ONLY, bus)])
  school_output(state, parsers)
  assert len(state.vehicleNaviEvents) == 1
  assert state.vehicleNaviEvents[0] == {'type': 'camera', 'speed': 40, 'kind': 2, 'target': 1040.0}

  # Zero-padded slot 1 (from CANPacker single-slot msg) is ignored cleanly.
  state.vehicleNaviEvents.clear()
  receive(state, parsers, 2.0, [packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, bus, {
    'PROLONG_VALUE': 0xffffffff, 'PROLONG_OFFSET': 8191, 'PROLONG_PROFILE_TYPE': 31})])
  school_output(state, parsers)
  assert state.vehicleNaviEvents == []


def test_ev4_094_queues_back_to_back_next_section(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus

  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)  # A: 2990..10930, 90 kph
  assert section_094_receive(state, parsers, 1.5, 3000.).vehicleNaviSectionActive

  # Approaching A's end at d=9000, announce B starting at 9000 + 1990 = 10990 (80 kph, 6520 m).
  next_094 = packer.make_can_msg(carstate.CANFD_NAVI_PROSHORT_MSG, bus, {
    'PROSHORT_OFFSET': 1990, 'PROSHORT_PATH_INDEX': 8, 'PROSHORT_DISTANCE': 326,
    'PROSHORT_VALUE_0': 0, 'PROSHORT_VALUE_1': 17 << 5, 'PROSHORT_PROFILE_TYPE': 16})[1]
  ret = section_094_receive(state, parsers, 2.0, 9000., proshort=next_094)
  assert ret.vehicleNaviSectionActive and ret.vehicleNaviSpeed == 90
  assert state.ev4NextSection is not None and state.ev4NextSection['speed'] == 80

  # Exiting A promotes B without activating early before 10990.
  ret = section_094_receive(state, parsers, 2.5, 10930.)
  assert not ret.vehicleNaviSectionActive
  assert state.ev4Section is not None and state.ev4Section['speed'] == 80
  assert state.ev4NextSection is None

  # Entering B at 10990 activates 80 kph cap.
  ret = section_094_receive(state, parsers, 3.0, 10990., status=(0, 0))
  assert ret.vehicleNaviSectionActive and ret.vehicleNaviSpeed == 80


def test_ev4_364_limit_mismatch_fallback_drops_section_when_380_blank(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus

  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)
  assert section_094_receive(state, parsers, 1.5, 3000., status=(0, 90)).vehicleNaviSectionActive

  # 0x380 blanks limit to 0 on exit ramp, while fresh 0x364 switches SPEED_LIMIT to 60.
  state.totalDistance = 3100.
  t = 2.0
  for step in range(6):
    t = round(2.0 + step * 0.5, 2)
    receive(state, parsers, t, [(0x380, EV4_STATUS[(0, 0)], parsers[Bus.alt].bus), _hda_frame(packer, bus, 60)])
    assert not school_output(state, parsers).vehicleNaviSectionActive
  assert state.ev4Section is not None  # < 3 s elapsed (2.0..4.5 = 2.5 s)

  receive(state, parsers, 5.0, [(0x380, EV4_STATUS[(0, 0)], parsers[Bus.alt].bus), _hda_frame(packer, bus, 60)])
  assert not school_output(state, parsers).vehicleNaviSectionActive
  assert state.ev4Section is None


def test_ev4_363_zero_does_not_latch_unconfirmed_kind7(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122

  _pos_363_receive(state, parsers, 1.0, 100., EV4_POS_363_ZERO, speed=100.)
  for time_s, cam_active in ((1.4, False), (1.8, True)):
    receive(state, parsers, time_s, [(0x93, EV4_PROFILE_SECTION_100, parsers[Bus.pt].bus),
                                     (0x363, EV4_POS_363_ZERO, parsers[Bus.pt].bus)])
    ret = SimpleNamespace(speedLimit=100. if cam_active else 0., vEgo=20.)
    state._update_vehicle_navi_events(parsers[Bus.pt], ret, cam_active, parsers[Bus.alt])
    assert not ret.vehicleNaviSectionActive
    assert not state.vehicleNaviSpeedZoneActive


def test_ev4_route_reset_keeps_entered_section_but_clears_unentered_and_next(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus
  reset_raw = (2 << 22) | (1 << 24)
  reset_msg = lambda: packer.make_can_msg(
    carstate.CANFD_NAVI_SEGMENT_MSG, bus, {f'BYTE_{i + 1}': (reset_raw >> (8 * i)) & 255 for i in range(8)})

  # Before entry, CalculatedRoute == 2 still clears the pending announcement.
  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_SECTION_094_7940)
  assert state.ev4Section is not None and not state.ev4Section['entered']
  receive(state, parsers, 1.2, [reset_msg()])
  assert not school_output(state, parsers).vehicleNaviSectionActive
  assert state.ev4Section is None

  # Once entered (e.g., tunnel GPS re-anchor), CalculatedRoute == 2 preserves ev4Section
  # while still dropping any queued ev4NextSection.
  section_094_receive(state, parsers, 2.0, 1000., proshort=EV4_SECTION_094_7940)
  assert section_094_receive(state, parsers, 2.5, 3000.).vehicleNaviSectionActive
  next_094 = packer.make_can_msg(carstate.CANFD_NAVI_PROSHORT_MSG, bus, {
    'PROSHORT_OFFSET': 1990, 'PROSHORT_PATH_INDEX': 8, 'PROSHORT_DISTANCE': 326,
    'PROSHORT_VALUE_0': 0, 'PROSHORT_VALUE_1': 17 << 5, 'PROSHORT_PROFILE_TYPE': 16})[1]
  assert section_094_receive(state, parsers, 3.0, 9000., proshort=next_094).vehicleNaviSectionActive
  assert state.ev4NextSection is not None

  state.totalDistance = 9050.
  receive(state, parsers, 3.5, [reset_msg(), (0x380, EV4_STATUS[(0, 90)], parsers[Bus.alt].bus)])
  ret = school_output(state, parsers)
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 90
  assert state.ev4Section is not None and state.ev4Section['entered']
  assert state.ev4NextSection is None


def test_ev4_section_end_clears_matching_380_kind7_zone(ev4):
  state, parsers, packer = ev4
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus

  sec_100 = packer.make_can_msg(carstate.CANFD_NAVI_PROSHORT_MSG, bus, {
    'PROSHORT_OFFSET': 1990, 'PROSHORT_PATH_INDEX': 8, 'PROSHORT_DISTANCE': 397,
    'PROSHORT_VALUE_0': 0, 'PROSHORT_VALUE_1': 21 << 5, 'PROSHORT_PROFILE_TYPE': 16})[1]
  section_094_receive(state, parsers, 1.0, 1000., status=(0, 100), proshort=sec_100)  # 2990..10930
  assert section_094_receive(state, parsers, 1.5, 3000., status=(0, 100)).vehicleNaviSectionActive

  # Late-section repeat of 0x380 0x10 + 0x093 kind=7 at the same 100 km/h limit.
  state.totalDistance = 10500.
  receive(state, parsers, 2.0, [(0x380, EV4_STATUS[(0x10, 100)], parsers[Bus.alt].bus)])
  school_output(state, parsers)
  receive(state, parsers, 2.2, [(0x380, EV4_STATUS[(0x10, 100)], parsers[Bus.alt].bus),
                                (0x93, EV4_PROFILE_SECTION_100, bus)])
  ret = school_output(state, parsers)
  assert ret.vehicleNaviSectionActive and ret.vehicleNaviSpeed == 100
  assert state.vehicleNaviSpeedZoneUsesNaviStatus

  # Advancing past the section endpoint releases both ev4Section and matching kind-7 latch
  # even if 0x380 stays at highway limit (0, 100).
  ret = section_094_receive(state, parsers, 2.5, 10930., status=(0, 100))
  assert state.ev4Section is None
  assert not ret.vehicleNaviSectionActive
  assert not state.vehicleNaviSpeedZoneActive
