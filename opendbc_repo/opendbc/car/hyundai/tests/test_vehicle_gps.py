import math

import pytest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CANFD_CAR, CAR
from opendbc.car.hyundai.vehicle_gps import (
  CANFD_COORD_SCALE, CANFD_HEADING_SCALE, CANFD_MESSAGE, MESSAGE, VehicleGPS, register_vehicle_gps,
)


def frame(lat=37., lon=127.):
  return int(lat * 1e6).to_bytes(4, 'little') + int(lon * 1e6).to_bytes(4, 'little')


def canfd_frame(lat=37.6611, lon=126.8412, heading=222.0, speed=10.0, counter=0):
  import struct
  from opendbc.can.dbc import DBC
  dbc = DBC('hyundai_canfd_generated')
  sig = dbc.name_to_msg[CANFD_MESSAGE].sigs['CHECKSUM']
  payload = bytearray(32)
  payload[2] = counter & 0xFF
  struct.pack_into('<ii', payload, 16, int(round(lat / CANFD_COORD_SCALE)), int(round(lon / CANFD_COORD_SCALE)))
  raw_hdg = 0xFFFF if heading is None else int(round((heading % 360.0) / CANFD_HEADING_SCALE)) & 0xFFFF
  struct.pack_into('<HH', payload, 24, raw_hdg, int(round(speed * 100.0)) & 0xFFFF)
  crc = sig.calc_checksum(0x206, sig, payload)
  struct.pack_into('<H', payload, 0, crc)
  return bytes(payload)


def setup():
  return CANParser('hyundai_canfd_generated', [(MESSAGE, math.nan)], 0), VehicleGPS()


def tick(cp, gps, t, data=None, speed=0., bus=0, reverse=False):
  cp.update([[int(t * 1e9), [] if data is None else [(0x374, data, bus)]]])
  return gps.update(cp, speed, reverse)


def test_acquisition_optional_absence_and_schema():
  cp, gps = setup()
  assert not tick(cp, gps, 1)['valid']
  assert cp.can_valid  # optional position must never create a controls CAN fault
  assert not tick(cp, gps, 2, frame())['valid']
  assert not tick(cp, gps, 3, frame())['valid']
  state = tick(cp, gps, 4, frame())
  assert state['valid'] and not state['headingValid']
  decoded = structs.CarState(vehicleGps=state).vehicleGps
  assert decoded.latitude == 37. and decoded.longitude == 127.
  assert decoded.sampleMonoTime == 4_000_000_000


def test_course_is_from_motion_not_assumed_north_and_reverse_disables_it():
  cp, gps = setup()
  for t in (1, 2, 3):
    state = tick(cp, gps, t, frame(37., 127. + t * .0001), speed=9)
  assert state['valid'] and state['headingValid']
  assert state['headingDeg'] == pytest.approx(90.)
  assert not tick(cp, gps, 4, frame(37., 127.0004), speed=9, reverse=True)['headingValid']


@pytest.mark.parametrize('bus', [1, 2, 128, 130])
def test_other_buses_and_transmit_receipts_do_not_acquire(bus):
  cp, gps = setup()
  for t in (1, 2, 3):
    assert not tick(cp, gps, t, frame(), bus=bus)['valid']


@pytest.mark.parametrize('bad', [b'', b'\x00' * 8, b'\xff' * 8, frame()[:-1], frame() + b'\x00', frame(86, 127)])
def test_malformed_rejects_and_needs_reacquisition(bad):
  cp, gps = setup()
  for t in (1, 2, 3):
    tick(cp, gps, t, frame())
  assert not tick(cp, gps, 4, bad)['valid']
  assert not tick(cp, gps, 5, frame())['valid']
  assert not tick(cp, gps, 6, frame())['valid']
  assert tick(cp, gps, 7, frame())['valid']


def test_position_jump_stale_and_moving_frozen_coordinates():
  cp, gps = setup()
  for t in (1, 2, 3):
    tick(cp, gps, t, frame())
  assert not tick(cp, gps, 4, frame(38, 128))['valid']
  cp, gps = setup()
  for t in (1, 2, 3):
    tick(cp, gps, t, frame())
  assert not tick(cp, gps, 6)['valid']
  cp, gps = setup()
  for t in (1, 2, 3):
    tick(cp, gps, t, frame())
  assert tick(cp, gps, 4, frame(), speed=10)['valid']
  assert not tick(cp, gps, 5, frame(), speed=10)['valid']


def test_unchanged_message_timestamp_and_long_park():
  cp, gps = setup()
  for t in range(1, 101):
    state = tick(cp, gps, t, frame())
  assert state['valid'] and not state['headingValid']
  cp.update([[100_100_000_000, [(123, b'12345678', 0)]]])
  assert gps.update(cp, 0)['sampleMonoTime'] == 100_000_000_000


def test_highway_travel_between_one_hz_fixes_is_not_a_frozen_coordinate():
  cp, gps = setup()
  for t in range(1, 5):
    state = tick(cp, gps, t, frame(37., 127. + t * .0004), speed=35)
    if t >= 3:
      assert state['valid']
      for step in range(1, 10):
        cp.update([[int((t + step / 10) * 1e9), [(123, b'12345678', 0)]]])
        assert gps.update(cp, 35)['valid']


@pytest.mark.parametrize('candidate', sorted(CANFD_CAR) + [CAR.HYUNDAI_SONATA])
def test_dbc_profile_registers_across_platforms_without_car_name_gate(monkeypatch, candidate):
  from unittest.mock import Mock
  from opendbc.car import Bus
  from opendbc.car.hyundai import carstate, hyundaicanfd
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.return_value = False
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, 'Params', lambda: params)
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: params)
  config = structs.CarParams(carFingerprint=candidate, flags=int(candidate.config.flags), safetyConfigs=[{}])
  state = carstate.CarState(config)
  parser = state.get_can_parsers(config)[Bus.pt]
  supported = candidate in CANFD_CAR
  assert (0x374 in parser.addresses) == supported
  if supported:
    assert parser.message_states[0x374].ignore_alive
    parser.controls_ready = True  # late GPS arrival after startup is supported
    for t in (1, 2, 3):
      result = tick(parser, state.vehicle_gps, t, frame(), bus=parser.bus)
    assert result['valid']
  else:
    assert state.vehicle_gps.update(parser, 0.) == {}


@pytest.mark.parametrize('field,value', [('address', 0x375), ('size', 16), ('missing', None),
                                        ('start_bit', 1), ('size_signal', 16), ('is_little_endian', False),
                                        ('is_signed', True), ('factor', 1e-7), ('offset', -180.)])
def test_same_id_or_name_with_incompatible_dbc_does_not_enable(field, value):
  from copy import deepcopy
  from types import SimpleNamespace
  cp = CANParser('hyundai_canfd_generated', [], 0)
  message = deepcopy(cp.dbc.name_to_msg[MESSAGE])
  if field in ('address', 'size'):
    setattr(message, field, value)
  elif field == 'missing':
    del message.sigs['LONGITUDE']
  else:
    setattr(message.sigs['LATITUDE'], 'size' if field == 'size_signal' else field, value)
  cp.dbc = SimpleNamespace(name_to_msg={MESSAGE: message})
  register_vehicle_gps(cp)
  assert 0x374 not in cp.addresses
  assert VehicleGPS().update(cp, 0.) == {}


def test_canfd_206_acquisition_native_heading_at_standstill_and_moving():
  cp = register_vehicle_gps(CANParser('hyundai_canfd_generated', [], 0))
  gps = VehicleGPS()
  assert 0x206 in cp.addresses
  assert cp.message_states[0x206].ignore_alive
  # 5 Hz frames at 0 km/h standstill still provide valid native heading after 3 consistent frames.
  for i in range(3):
    t = 1.0 + i * 0.2
    cp.update([[int(t * 1e9), [(0x206, canfd_frame(37.6611, 126.8412, heading=222.0, speed=0.0, counter=i * 2), 0)]]])
    state = gps.update(cp, 0.0)
  assert state['valid'] and state['headingValid']
  assert state['latitude'] == pytest.approx(37.6611, abs=1e-5)
  assert state['longitude'] == pytest.approx(126.8412, abs=1e-5)
  assert state['headingDeg'] == pytest.approx(222.0, abs=0.05)

  # Reverse disables headingValid; invalid heading sentinel (0xFFFF) does not produce headingValid.
  cp.update([[int(1.6 * 1e9), [(0x206, canfd_frame(37.6611, 126.8412, heading=222.0, speed=0.0, counter=6), 0)]]])
  assert not gps.update(cp, 0.0, reverse=True)['headingValid']

  # Zero coordinates or corrupted CRC are rejected.
  cp.update([[int(1.8 * 1e9), [(0x206, canfd_frame(0.0, 0.0, heading=0.0, speed=0.0, counter=8), 0)]]])
  assert not gps.update(cp, 0.0)['valid']
