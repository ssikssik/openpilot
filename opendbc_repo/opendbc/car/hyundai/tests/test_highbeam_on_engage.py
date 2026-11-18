from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.hyundai.carcontroller import CarController, stalk_checksum

# Ioniq 5 PE lever neutral, light knob 3 (byte2 0x30), COUNTER_ALT 8 (05_can/hba_ifs.md, blinker_route19)
STOCK = {"CHECKSUM_MAYBE": 0, "COUNTER_ALT": 8, "HIGHBEAM_FORWARD": 0, "LIGHT_KNOB_POSITION": 3, "HIGHBEAM_BACKWARD": 0,
         "LEFT_BLINKER": 0, "RIGHT_BLINKER": 0, "LEFT_BLINKER_TAP": 0, "RIGHT_BLINKER_TAP": 0}


def make_cc(on=True):
  return SimpleNamespace(highbeam_on_engage=on, highbeam_frame=None, highbeam_prev_enabled=False, frame=0,
                         CP=SimpleNamespace(flags=0), blinker_stalk=0, CAN=SimpleNamespace(ECAN=0), packer=CANPacker('hyundai_canfd_generated'))


def drive(cc, enabled, frames, stalks=STOCK, fault=None):
  sent = []
  for _ in range(frames):
    now = 1_000_000_000 + cc.frame * 10_000_000
    raw = bytes.fromhex('f880300000000000')
    if fault == 'length':
      raw += bytes(16)
    elif fault == 'crc':
      raw = bytes(8)
    stamp = now - 500_000_000 if fault == 'stale' else now
    cs = SimpleNamespace(blinker_stalks=stalks, out=SimpleNamespace(canValid=fault != 'can'),
                         cp=SimpleNamespace(dat={0x3C1: raw}, ts_nanos={0x3C1: {'COUNTER_ALT': stamp}}))
    sent += [(cc.frame, m) for m in CarController.highbeam_push(cc, SimpleNamespace(enabled=enabled), cs, now)]
    cc.frame += 1
  return sent


def test_first_engagement_pushes_the_high_beam_lever_once():
  cc = make_cc()
  assert drive(cc, False, 50) == []
  sent = drive(cc, True, 100)
  assert [f for f, _ in sent] == [50, 50 + carcontroller.HIGHBEAM_PRESS_FRAMES]
  (_, (addr, press, bus)), (_, (_, release, _)) = sent
  assert addr == 0x3C1 and bus == 0
  assert press[2] == 0x34 and release[2] == 0x30           # HIGHBEAM_FORWARD (byte2 bit2) on, then released
  assert press[1] >> 4 == 9 and release[1] >> 4 == 10      # one lever change each
  for dat in (press, release):
    assert dat[0] == stalk_checksum(dat[1:])
    # panda's 0x3C1 shape check (safety_hyundai_canfd.h): byte2 within 0x34, byte3 0x00/0x40, byte4 stock forms, tail 0
    assert dat[2] & ~0x34 == 0 and dat[3] in (0x00, 0x40) and dat[4] in (0x00, 0x01, 0x10, 0x41) and dat[5:] == bytes(3)
  # the lever is a toggle: disengage and engage again in the same start sends nothing
  assert drive(cc, False, 20) == [] and drive(cc, True, 50) == []


def test_off_or_no_stalk_message_sends_nothing():
  assert drive(make_cc(on=False), True, 50) == []
  cc = make_cc()
  assert drive(cc, True, 50, stalks=None) == [] and cc.highbeam_frame is None


def test_excludes_native_spas_replay_driver_input_and_invalid_stock():
  for fault in ('length', 'crc', 'stale', 'can'):
    assert drive(make_cc(), True, 50, fault=fault) == []
  assert drive(make_cc(), True, 50, stalks={**STOCK, 'HIGHBEAM_BACKWARD': 1}) == []
  cc = make_cc()
  cc.CP.flags = HyundaiFlags.CANFD_SPAS_BLINKERS
  assert drive(cc, True, 50) == []
  cc = make_cc()
  cc.blinker_stalk = 1
  assert drive(cc, True, 50) == []
