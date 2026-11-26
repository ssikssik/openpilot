from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.hyundaicanfd import create_acc_control, create_acc_control_scc2
from opendbc.car.hyundai.stopping import APPROACH_ACCEL, CanfdStopping
from opendbc.car.hyundai.values import CAR, SMOOTH_LEAD_STOP_CAR

APPROACH_SPEED = 3.0  # m/s, above ENTRY_SPEED so the approach phase is active


def test_smooth_lead_stop_vehicles():
  assert SMOOTH_LEAD_STOP_CAR == {CAR.KIA_K8_HEV_1ST_GEN, CAR.KIA_EV4}
  assert "KIA_EV4" in SMOOTH_LEAD_STOP_CAR  # the planner compares carFingerprint strings


@pytest.mark.parametrize("smooth", [False, True])
def test_approach_floor_only_without_smooth_profile(smooth):
  cmd = CanfdStopping().update(active=True, requested=True, speed=APPROACH_SPEED, held=False, accel=-0.2,
                               previous_value=-0.2, jerk_u=2.0, jerk_l=1.0, smooth_lead_stop=smooth)
  assert cmd.stop_req == 0
  assert cmd.raw == pytest.approx(-0.2 if smooth else APPROACH_ACCEL)


def test_smooth_profile_never_adds_throttle_or_skips_final_stop():
  controller = CanfdStopping()
  cmd = controller.update(active=True, requested=True, speed=APPROACH_SPEED, held=False, accel=0.4,
                          previous_value=0.0, jerk_u=2.0, jerk_l=1.0, smooth_lead_stop=True)
  assert cmd.raw == 0.0 and cmd.value <= 0.0
  cmd = controller.update(active=True, requested=True, speed=0.05, held=False, accel=-0.2,
                          previous_value=-0.2, jerk_u=2.0, jerk_l=1.0, smooth_lead_stop=True)
  assert (cmd.stop_req, cmd.raw, cmd.value) == (1, 0.0, 0.0)


def make_cs(fingerprint, soft_hold=0):
  return SimpleNamespace(
    CP=SimpleNamespace(carFingerprint=fingerprint),
    scc_control={"ACC_ObjRelSpd": 0.0, "InfoDisplay": 5, "ZEROS_7": 1, "AccelLimitBandLower": 1.26},
    canfdSccHoldActive=False, softHoldActive=soft_hold, paddle_button_prev=0,
    out=SimpleNamespace(aEgo=-0.2, vEgo=APPROACH_SPEED, vEgoRaw=APPROACH_SPEED,
                        wheelSpeeds=SimpleNamespace(fl=APPROACH_SPEED, fr=APPROACH_SPEED, rl=APPROACH_SPEED, rr=APPROACH_SPEED),
                        canValid=True, gearShifter="drive", brakePressed=False, gasPressed=False,
                        brakeHoldActive=False, parkingBrake=False, cruiseState=SimpleNamespace(available=True, standstill=False)),
  )


def send(camera, CS, smooth):
  packer = CANPacker("hyundai_canfd_generated")
  can = SimpleNamespace(ECAN=0)
  hud = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
  jerk = SimpleNamespace(carrot_cruise=0, jerk_u=2.0, jerk_l=1.0)
  if camera:
    msg, _ = create_acc_control_scc2(packer, can, True, -0.2, -0.2, True, False, 30.0, hud, jerk, CS, CanfdStopping(),
                                     smooth_lead_stop=smooth)
  else:
    msg, _ = create_acc_control(packer, can, True, -0.2, -0.2, True, False, 30.0, hud, 2.0, 1.0, CS, CanfdStopping(),
                                accel_value_last=-0.2, smooth_lead_stop=smooth)
  parser = CANParser("hyundai_canfd_generated", [("SCC_CONTROL", 50)], 0)
  parser.update([1_000_000_000, [msg]])
  return parser.vl["SCC_CONTROL"]


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("fingerprint, applies", [
  (CAR.KIA_K8_HEV_1ST_GEN, True), (CAR.KIA_EV4, True), (CAR.KIA_EV6, False), (CAR.HYUNDAI_IONIQ_5_PE, False),
])
def test_packet_uses_smooth_profile_only_on_supported_vehicles(camera, fingerprint, applies):
  values = send(camera, make_cs(fingerprint), smooth=True)
  assert values["StopReq"] == 0
  assert values["aReqRaw"] == pytest.approx(-0.2 if applies else APPROACH_ACCEL, abs=0.011)


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K8_HEV_1ST_GEN, CAR.KIA_EV4])
def test_flag_off_keeps_existing_approach_braking(camera, fingerprint):
  values = send(camera, make_cs(fingerprint), smooth=False)
  assert values["aReqRaw"] == pytest.approx(APPROACH_ACCEL, abs=0.011)
