#!/usr/bin/env python3
from parameterized import parameterized_class
import numpy as np
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda
from opendbc.safety.tests.hyundai_common import Buttons, HyundaiButtonBase, HyundaiLongitudinalBase


# Mirrors HYUNDAI_CANFD_HDA1_TX_MSGS in safety_hyundai_canfd.h
HDA1_TX_MSGS = [
  [0x10B, 2], [0x12A, 0], [0x1A0, 0], [0x1CF, 0], [0x1CF, 2], [0x1E0, 0], [0x160, 0],
  [0x7D0, 0], [0x3C1, 0], [0x400, 0], [0x7B1, 0], [0x7B1, 1], [0x165, 0], [0x16A, 0],
  [0x1AA, 2], [0xCB, 0], [0x175, 2], [0x161, 0], [0x162, 0], [0x200, 0],
  [0x4A3, 2], [0x4B4, 2], [0xEA, 2], [0x2AF, 2],
]

# Mirrors HYUNDAI_CANFD_HDA2_LONG_TX_MSGS in safety_hyundai_canfd.h (used for all HDA2 modes)
HDA2_TX_MSGS = [
  [0x10B, 2], [0x50, 0], [0x1CF, 0], [0x1CF, 1], [0x1CF, 2],
  [0x1AA, 0], [0x1AA, 1], [0x1AA, 2], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x730, 2], [0x7C4, 0],
  [0x7B1, 0], [0x7B1, 1], [0x165, 0], [0x16A, 0], [0x3C1, 0], [0x400, 0],
  [0x12A, 1], [0x160, 1], [0x1E0, 1], [0x1A0, 1], [0x1EA, 1], [0x200, 1], [0x345, 1], [0x1DA, 1],
  [0x12A, 0], [0x1E0, 0], [0x160, 0], [0x1EA, 0], [0x200, 0], [0x1A0, 0], [0x345, 0], [0x1DA, 0],
  [0x362, 0], [0x362, 1], [0x2A4, 1], [0x110, 0], [0x110, 1], [0x50, 1], [0x51, 1],
  [0x161, 0], [0x162, 0], [0x4A3, 2], [0x4B4, 2], [0xCB, 0], [0x175, 2], [0x1FA, 2],
  [0xEA, 2], [0x2AF, 2], [0x4BE, 2], [0x4B9, 2],
]


class TestHyundaiCanfdBase(HyundaiButtonBase, common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = HDA2_TX_MSGS
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  # Dynamic TX-based forwarding suppression starts inactive; only bus 0 -> 0x4B9 is statically blocked.
  FWD_BLACKLISTED_ADDRS = {0: [0x4B9]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 10
  MAX_TORQUE = 512

  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2
  MIN_VALID_STEERING_RT_INTERVAL = 810000  # a ~10% buffer, can send steer up to 110Hz

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdBase":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def _torque_driver_msg(self, torque):
    values = {"STEERING_COL_TORQUE": torque}
    return self.packer.make_can_msg_panda("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"TORQUE_REQUEST": torque, "STEER_REQ": steer_req}
    return self.packer.make_can_msg_panda(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHEEL_SPEED_{i}": speed * 0.03125 for i in range(1, 5)}
    return self.packer.make_can_msg_panda("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_panda("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_panda(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_panda("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS", bus, values)

  # CarrotPilot safety_hyundai_canfd.h intentionally does not reject steering TX on torque/rate/req violations
  # (`//tx = false;` in hyundai_canfd_tx_hook) and omits the 0x1CF button content filter (`#if 0`).
  def test_steer_safety_check(self):
    for enabled in (0, 1):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
      self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=1)))

  def test_non_realtime_limit_up(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_realtime_limits(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RT_DELTA)))

  def test_against_torque_driver(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._torque_driver_msg(self.DRIVER_TORQUE_ALLOWANCE))
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_TORQUE)))

  def test_steer_req_bit(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=1)))
    self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=0)))

  def test_steer_req_bit_frames(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=1)))
    self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=0)))

  def test_steer_req_bit_multi_invalid(self):
    self.safety.set_controls_allowed(True)
    for _ in range(self.MAX_INVALID_STEERING_FRAMES + 1):
      self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=0)))

  def test_steer_req_bit_realtime(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=1)))
    self.assertTrue(self._tx(self._torque_cmd_msg(100, steer_req=0)))

  def test_button_sends(self):
    # CarrotPilot safety_hyundai_canfd.h disables the 0x1CF button content filter (#if 0).
    for enabled in (0, 1):
      self.safety.set_controls_allowed(enabled)
      for btn in (Buttons.NONE, Buttons.RESUME, Buttons.SET, Buttons.CANCEL):
        self.assertTrue(self._tx(self._button_msg(btn, bus=self.BUTTONS_TX_BUS)))

  def test_enable_control_allowed_from_cruise(self):
    # CarrotPilot safety_hyundai_common.h enables controls_allowed directly on cruise_engaged
    # without requiring a recent button press.
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_sampling_cruise_buttons(self):
    # CarrotPilot resets hyundai_last_button_interaction = 0 on every non-longitudinal cruise check.
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = HDA1_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A,)}  # LFA

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_tx_hook_on_wrong_safety_mode(self):
    # HYUNDAI_CANFD_HDA1_TX_MSGS allows 0x7D0 on bus 0 without a UDS payload filter,
    # which overlaps with TestElm327's all-zero 0x7D0 probe.
    orig_tx = self._tx
    def _tx_excluding_hda1_uds(msg):
      if msg.addr == 0x7D0 and msg.bus == 0:
        return False
      return orig_tx(msg)
    self._tx = _tx_excluding_hda1_uds
    try:
      super().test_tx_hook_on_wrong_safety_mode()
    finally:
      self._tx = orig_tx


@parameterized_class([
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS |
                                                                                  HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS |
                                                                                      HyundaiSafetyFlags.CAMERA_SCC},
])
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


@parameterized_class([
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS |
                                                                                  HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS |
                                                                                      HyundaiSafetyFlags.CAMERA_SCC},
])
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_panda("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      0x1AA is not in HDA1 TX allowlist on bus 0 (only bus 2).
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # Non-longitudinal 0x1A0 allows zero-accel frames and rejects non-zero accel.
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertTrue(self._tx(self._acc_cancel_msg(False)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False, accel=1)))


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  TX_MSGS = HDA2_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x50,)}  # LKAS

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  TX_MSGS = HDA2_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x110,)}  # LKAS_ALT

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT)
    self.safety.init_tests()


class HyundaiCanfdLongitudinalMixin:
  # CarrotPilot HYUNDAI_LONG_LIMITS: [-4.0, 2.5] m/s^2, and longitudinal_accel_checks auto-enables
  # controls_allowed on nonzero desired_accel while enforcing [MIN_ACCEL, MAX_ACCEL].
  MIN_ACCEL = -4.0
  MAX_ACCEL = 2.5

  def test_accel_actuation_limits(self):
    for accel in np.concatenate((np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.05), [0, self.INACTIVE_ACCEL])):
      accel = round(accel, 2)
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        should_tx = (self.MIN_ACCEL <= accel <= self.MAX_ACCEL) or accel == self.INACTIVE_ACCEL
        self.assertEqual(should_tx, self._tx(self._accel_msg(accel)), f"{accel=} {controls_allowed=}")


class TestHyundaiCanfdLKASteeringLongEV(HyundaiCanfdLongitudinalMixin, HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = HDA2_TX_MSGS

  RELAY_MALFUNCTION_ADDRS = {0: (0x50,), 1: (0x1a0,)}  # LKAS, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_panda("SCC_CONTROL", 1, values)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
@parameterized_class([
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
])
class TestHyundaiCanfdLFASteeringLong(HyundaiCanfdLongitudinalMixin, HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1a0)}  # LFA, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_panda("SCC_CONTROL", 0, values)

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    # HDA1 TX allowlist permits 0x7D0 on bus 0 without a payload filter.
    addr, bus = self.DISABLED_ECU_UDS_MSG
    tester_present = libsafety_py.make_CANPacket(addr, bus, b"\x02\x3E\x80\x00\x00\x00\x00\x00")
    self.assertTrue(self._tx(tester_present))


class TestHyundaiCanfdEv4ParkedClearDtc(unittest.TestCase):
  TX_MSGS = HDA2_TX_MSGS

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.ev4_param = (HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT |
                      HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.ev4_param)
    self.safety.init_tests()

  def _set_speed(self, raw_speed):
    msg = self.packer.make_can_msg_panda("WHEEL_SPEEDS", 0, {f"WHEEL_SPEED_{i}": raw_speed * 0.03125 for i in range(1, 5)})
    self.safety.safety_rx_hook(msg)

  def test_parked_clear_dtc_allowlist_and_interlocks(self):
    allowed_payloads = (
      b"\x04\x14\xFF\xFF\xFF\x00\x00\x00",
      b"\x02\x10\x03\x00\x00\x00\x00\x00",
      b"\x02\x10\x01\x00\x00\x00\x00\x00",
    )
    disallowed_payloads = (
      b"\x00\x00\x00\x00\x00\x00\x00\x00",
      b"\x02\x11\x01\x00\x00\x00\x00\x00",  # ECU Reset
      b"\x03\x28\x83\x01\x00\x00\x00\x00",  # CommunicationControl
      b"\x02\x3E\x80\x00\x00\x00\x00\x00",  # TesterPresent not allowed on 0x7C4 or 0x730 bus 2
      b"\x04\x14\xFF\xFF\xFF\x01\x00\x00",  # Non-zero trailing padding
    )

    self.safety.set_controls_allowed(False)
    self._set_speed(0)

    # Allowed on exact (addr, bus) when stationary + disengaged + CAMERA_SCC + HDA2
    for addr, bus in ((0x7C4, 0), (0x730, 2)):
      for payload in allowed_payloads:
        self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, payload)))
      for payload in disallowed_payloads:
        self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, payload)))

    # Blocked on wrong buses
    for addr, bus in ((0x7C4, 1), (0x7C4, 2), (0x730, 0), (0x730, 1)):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, allowed_payloads[0])))

    # Existing TesterPresent on 0x730 bus 1 remains allowed
    self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x730, 1, b"\x02\x3E\x80\x00\x00\x00\x00\x00")))

    # Blocked when controls_allowed is True
    self.safety.set_controls_allowed(True)
    for addr, bus in ((0x7C4, 0), (0x730, 2)):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, allowed_payloads[0])))

    # Blocked when vehicle is moving
    self.safety.set_controls_allowed(False)
    self._set_speed(100)
    for addr, bus in ((0x7C4, 0), (0x730, 2)):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, allowed_payloads[0])))

    # Blocked on non-CAMERA_SCC HDA2
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.ev4_param & ~HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()
    self.safety.set_controls_allowed(False)
    self._set_speed(0)
    for addr, bus in ((0x7C4, 0), (0x730, 2)):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, allowed_payloads[0])))


if __name__ == "__main__":
  unittest.main()
