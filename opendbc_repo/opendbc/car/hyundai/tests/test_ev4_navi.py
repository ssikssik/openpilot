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


def test_ev4_section_releases_on_different_limit(ev4):
  state, parsers, _ = section_entered(ev4)
  ret = school_receive(state, parsers, 2.0, status=(0, 60))
  assert not ret.vehicleNaviSectionActive
  assert not state.vehicleNaviSpeedZoneActive


def test_ev4_section_releases_when_acan_goes_stale(ev4):
  state, parsers, _ = section_entered(ev4)
  ret = school_receive(state, parsers, 1.4 + 1.5)
  assert not ret.vehicleNaviSectionActive


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


def test_ev4_094_curve_slot_is_not_a_section(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  section_094_receive(state, parsers, 1.0, 1000., proshort=EV4_CURVE_094)
  assert state.ev4Section is None
  assert not section_094_receive(state, parsers, 1.5, 3000.).vehicleNaviSectionActive


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


@pytest.mark.parametrize('status', [(0x10, 100), (0, 100), (0x1b, 100)])
def test_ev4_status_profile_does_not_replace_valid_position(ev4, status):
  """The new upstream policy does not establish a section from 0x380 alone."""
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  for t in (1.0, 1.5, 2.0, 9.0):
    ret = school_receive(state, parsers, t, status=status, profile=EV4_PROFILE_SECTION_100)
    assert not ret.vehicleNaviSectionActive
    assert not state.vehicleNaviSpeedZoneActive


def test_ev4_094_without_position_does_not_control_approach_or_inside_section(ev4):
  """Keep the decoder, but use upstream admission rather than the old fallback."""
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  for time_s, distance in ((1.0, 1000.), (1.5, 2990.), (2.0, 6000.), (2.5, 10930.)):
    ret = section_094_receive(state, parsers, time_s, distance, proshort=EV4_SECTION_094_7940)
    assert not ret.vehicleNaviSectionActive
    assert ret.vehicleNaviSpeed == 0


def test_ev4_status_does_not_confirm_school_zone_in_new_upstream_policy(ev4):
  state, parsers, _ = ev4
  state.controls_ready_count = 122
  state.vehicleNaviSchoolZoneControl = True
  for t in (1., 1.5, 2., 10.):
    ret = school_receive(state, parsers, t, status=(0, 30), profile=EV4_PROFILE_77)
    assert not ret.schoolZoneActive
