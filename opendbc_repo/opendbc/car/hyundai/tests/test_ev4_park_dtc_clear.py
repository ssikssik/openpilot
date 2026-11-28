from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def _make_cc(fingerprint, flags):
  cc = object.__new__(CarController)
  cc.car_fingerprint = fingerprint
  cc.CAN = SimpleNamespace(ECAN=0, ACAN=1, CAM=2)
  cc.ev4_park_dtc_clear_enabled = (
    fingerprint == CAR.KIA_EV4 and
    bool(flags & HyundaiFlags.CANFD_HDA2) and
    bool(flags & HyundaiFlags.CAMERA_SCC)
  )
  cc.ev4_park_dtc_dirty = cc.ev4_park_dtc_clear_enabled
  cc.ev4_park_dtc_start_frame = None
  cc.frame = 0
  return cc


def _state(*, gear=structs.CarState.GearShifter.park, standstill=True, enabled=False, lat_active=False):
  CC = SimpleNamespace(enabled=enabled, latActive=lat_active)
  CS = SimpleNamespace(out=SimpleNamespace(gearShifter=gear, standstill=standstill))
  return CC, CS


def test_ev4_park_dtc_clear_sequence_and_rearm():
  cc = _make_cc(CAR.KIA_EV4, HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2 | HyundaiFlags.CAMERA_SCC)

  # Drive first without clearing -> stays dirty and emits nothing while in D
  CC, CS = _state(gear=structs.CarState.GearShifter.drive, standstill=False, enabled=True, lat_active=True)
  for f in range(30):
    cc.frame = f
    assert cc._ev4_park_dtc_clear_msgs(CC, CS) == []

  # Shift to P while stopped & disengaged -> emits exact 4-step UDS sequence to 0x7C4(bus 0) & 0x730(bus 2)
  CC, CS = _state(gear=structs.CarState.GearShifter.park, standstill=True, enabled=False, lat_active=False)
  emitted = []
  for f in range(100, 140):
    cc.frame = f
    msgs = cc._ev4_park_dtc_clear_msgs(CC, CS)
    if msgs:
      emitted.append((f - 100, msgs))

  assert [t for t, _ in emitted] == [5, 11, 16, 21]
  expected_payloads = [
    b"\x04\x14\xFF\xFF\xFF\x00\x00\x00",
    b"\x02\x10\x03\x00\x00\x00\x00\x00",
    b"\x04\x14\xFF\xFF\xFF\x00\x00\x00",
    b"\x02\x10\x01\x00\x00\x00\x00\x00",
  ]
  for (_, msgs), payload in zip(emitted, expected_payloads, strict=True):
    assert msgs == [(0x7C4, payload, 0), (0x730, payload, 2)]

  # Once cleared, stays silent in P until openpilot engages steering again
  assert cc.ev4_park_dtc_dirty is False
  for f in range(140, 200):
    cc.frame = f
    assert cc._ev4_park_dtc_clear_msgs(CC, CS) == []

  # Re-engaging steering re-arms clear for the next park
  CC_eng, CS_drv = _state(gear=structs.CarState.GearShifter.drive, standstill=False, enabled=True, lat_active=True)
  cc.frame = 200
  assert cc._ev4_park_dtc_clear_msgs(CC_eng, CS_drv) == []
  assert cc.ev4_park_dtc_dirty is True

  # Leaving P mid-sequence aborts cleanly and retries on next P
  for f in range(210, 222):  # runs steps 5 & 11 only
    cc.frame = f
    cc._ev4_park_dtc_clear_msgs(CC, CS)
  assert cc.ev4_park_dtc_dirty is True
  cc.frame = 222
  assert cc._ev4_park_dtc_clear_msgs(*_state(gear=structs.CarState.GearShifter.drive, standstill=True)) == []
  assert cc.ev4_park_dtc_start_frame is None
  assert cc.ev4_park_dtc_dirty is True


def test_ev4_park_dtc_clear_disabled_for_other_cars_or_non_camera_scc():
  cc_other = _make_cc(CAR.HYUNDAI_IONIQ_5, HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2 | HyundaiFlags.CAMERA_SCC)
  cc_no_cam = _make_cc(CAR.KIA_EV4, HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2)
  CC, CS = _state(gear=structs.CarState.GearShifter.park, standstill=True, enabled=False, lat_active=False)
  for cc in (cc_other, cc_no_cam):
    for f in range(30):
      cc.frame = f
      assert cc._ev4_park_dtc_clear_msgs(CC, CS) == []
