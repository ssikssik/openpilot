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
