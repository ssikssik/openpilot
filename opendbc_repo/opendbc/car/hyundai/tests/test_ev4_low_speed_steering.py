from types import SimpleNamespace as NS

import pytest

from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags, steering_angle_max_torque, steering_angle_smooth_tau


def setup_controller(monkeypatch, *, car=CAR.KIA_EV4, v_kph=30.0):
  settings = {"MaxAngleFrames": 89}
  params = NS(get_int=lambda key: settings.get(key, 0), get_bool=lambda key: False)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: params)
  monkeypatch.setattr(hyundaicanfd, "create_lfahda_cluster", lambda *a, **kw: [])
  monkeypatch.setattr(hyundaicanfd, "create_lfa_icon_non_camera_scc", lambda *a, **kw: [])
  monkeypatch.setattr(carcontroller.CarController, "create_button_messages", lambda *a, **kw: [])

  flags = HyundaiFlags.CANFD | HyundaiFlags.ANGLE_CONTROL | HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_HDA2
  cp = structs.CarParams(carFingerprint=car, flags=int(flags), wheelbase=2.83, steerRatio=14.5)
  controller = carcontroller.CarController({Bus.pt: "hyundai_canfd_generated"}, cp)
  cs = NS(
    out=structs.CarState(canValid=True, steeringPressed=False, steeringTorque=0, vEgoRaw=v_kph / 3.6, steeringAngleDeg=0.0),
    modelV2=NS(frameId=1, position=NS(yStd=[0.1] * 33)),
    is_metric=True,
    blinker_stalks=None,
    adrv_0x161=None,
    mdps=None,
    steer_touch_2af=None,
    lfa_alt={},
    lfa={},
  )
  cc = structs.CarControl(enabled=True, latActive=True)
  return controller, cs, cc


def step(controller, cs, cc):
  cs.modelV2.frameId = controller.frame // 5 + 1
  return controller.update(cc.as_reader(), cs, (controller.frame + 1) * 10_000_000)


@pytest.mark.parametrize("kph, expected", [
  (0.0, 155.0),
  (20.0, 155.0),
  (30.0, 155.0),
  (35.0, 165.0),
  (40.0, 175.0),
  (60.0, 215.0),
  (80.0, 250.0),
  (110.0, 250.0),
])
def test_ev4_angle_max_torque_profile(kph, expected):
  assert steering_angle_max_torque(CAR.KIA_EV4, kph / 3.6) == pytest.approx(expected)


def test_ev4_angle_max_torque_error_boost_and_override_guard():
  # Small tracking error (<= 2.5 deg) stays at low-speed base cap to avoid shudder.
  assert steering_angle_max_torque(CAR.KIA_EV4, 25.0 / 3.6, angle_error_deg=2.0) == pytest.approx(155.0)
  # Large unpressed tracking error boosts smoothly up to full 250 authority.
  mid = steering_angle_max_torque(CAR.KIA_EV4, 25.0 / 3.6, angle_error_deg=5.25)
  assert 195.0 < mid < 210.0
  assert steering_angle_max_torque(CAR.KIA_EV4, 25.0 / 3.6, angle_error_deg=8.0) == pytest.approx(250.0)
  assert steering_angle_max_torque(CAR.KIA_EV4, 25.0 / 3.6, angle_error_deg=-12.0) == pytest.approx(250.0)
  # Driver override never boosts authority.
  assert steering_angle_max_torque(CAR.KIA_EV4, 25.0 / 3.6, angle_error_deg=12.0, steering_pressed=True) == pytest.approx(155.0)


@pytest.mark.parametrize("car", [CAR.KIA_EV6, CAR.KIA_EV6_PE, CAR.HYUNDAI_IONIQ_5_PE])
@pytest.mark.parametrize("kph", [0.0, 20.0, 40.0, 80.0])
def test_other_cars_keep_default_angle_max_torque_and_no_smoothing(car, kph):
  assert steering_angle_max_torque(car, kph / 3.6) == pytest.approx(250.0)
  assert steering_angle_smooth_tau(car, kph / 3.6) == pytest.approx(0.0)


@pytest.mark.parametrize("kph, expected", [
  (0.0, 0.018),
  (40.0, 0.015),
  (50.0, 0.0),
  (90.0, 0.0),
])
def test_ev4_angle_smooth_tau_profile(kph, expected):
  assert steering_angle_smooth_tau(CAR.KIA_EV4, kph / 3.6) == pytest.approx(expected)


def test_ev4_can_max_torque_tracks_speed_error_boost_and_handover_recovery(monkeypatch):
  controller, cs, cc = setup_controller(monkeypatch, car=CAR.KIA_EV4, v_kph=25.0)
  cc.actuators.steeringAngleDeg = 10.0
  cs.out.steeringAngleDeg = 9.0  # 1 deg small error -> stays at 155 base cap
  for _ in range(120):
    output, messages = step(controller, cs, cc)
  lfa_alt = next(data for addr, data, _ in messages if addr == 0xCB)
  assert output.torqueOutputCan == pytest.approx(155.0)
  assert lfa_alt[6] == 155

  # Tight turn error (9 deg) boosts low-speed authority up to 250 smoothly.
  cs.out.steeringAngleDeg = 1.0
  for _ in range(60):
    output, messages = step(controller, cs, cc)
  lfa_alt = next(data for addr, data, _ in messages if addr == 0xCB)
  assert output.torqueOutputCan == pytest.approx(250.0)
  assert lfa_alt[6] == 250

  # Once tracking catches up, authority returns to 155 base cap.
  cs.out.steeringAngleDeg = 9.5
  output, messages = step(controller, cs, cc)
  lfa_alt = next(data for addr, data, _ in messages if addr == 0xCB)
  assert output.torqueOutputCan == pytest.approx(155.0)
  assert lfa_alt[6] == 155

  # Driver override followed by release recovers cleanly to the low-speed cap, not 250.
  cs.out.steeringPressed = True
  cs.out.steeringTorque = 300.0
  for _ in range(40):
    step(controller, cs, cc)
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0.0
  cs.out.steeringAngleDeg = 10.0
  for _ in range(150):
    output, messages = step(controller, cs, cc)
  assert controller.lkas_max_torque == pytest.approx(155.0)
  assert output.torqueOutputCan == pytest.approx(155.0)
  assert controller.steer_handover.state == "waiting"

  # Accelerating to highway speed smoothly restores full 250 authority even with zero error.
  cs.out.vEgoRaw = 85.0 / 3.6
  for _ in range(120):
    output, messages = step(controller, cs, cc)
  lfa_alt = next(data for addr, data, _ in messages if addr == 0xCB)
  assert output.torqueOutputCan == pytest.approx(250.0)
  assert lfa_alt[6] == 250


def test_ev4_low_speed_smoothing_eliminates_stale_carcontrol_zero_steps_and_double_jumps(monkeypatch):
  controller, cs, cc = setup_controller(monkeypatch, car=CAR.KIA_EV4, v_kph=30.0)
  # Simulate 30 deg/s ramp with every 4th tick repeating a stale carControl angle,
  # followed by a 2x jump on the next tick (the exact card/controlsd race seen in logs).
  cmd = 0.0
  outputs = []
  for tick in range(80):
    if tick % 4 != 3:
      cmd += 0.30 if tick % 4 != 0 else 0.60
    cc.actuators.steeringAngleDeg = cmd
    cs.out.steeringAngleDeg = max(0.0, cmd - 1.0)
    output, _ = step(controller, cs, cc)
    outputs.append(output.steeringAngleDeg)

  steps = [outputs[i] - outputs[i - 1] for i in range(15, len(outputs))]
  assert min(steps) > 0.12  # no 0.0 deg stall frames
  assert max(steps) < 0.52  # no 0.60 deg 2x step spikes


def test_ev4_smoothing_resets_on_disengage_and_bypasses_at_highway_speed(monkeypatch):
  controller, cs, cc = setup_controller(monkeypatch, car=CAR.KIA_EV4, v_kph=25.0)
  cc.actuators.steeringAngleDeg = 12.0
  for _ in range(10):
    step(controller, cs, cc)

  # Disengage resets the filter to actual steering angle so re-engagement starts cleanly.
  cc.latActive = False
  cs.out.steeringAngleDeg = -5.0
  output, _ = step(controller, cs, cc)
  assert output.steeringAngleDeg == pytest.approx(-5.0)
  assert controller.apply_angle_filtered == pytest.approx(-5.0)

  # At 50+ kph, smoothing is bypassed (zero added phase lag within physics rate limits).
  cc.latActive = True
  cs.out.vEgoRaw = 80.0 / 3.6
  cc.actuators.steeringAngleDeg = -4.8
  output, _ = step(controller, cs, cc)
  assert output.steeringAngleDeg == pytest.approx(-4.8)
