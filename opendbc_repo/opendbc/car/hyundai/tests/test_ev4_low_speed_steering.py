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


@pytest.mark.parametrize("car", [CAR.KIA_EV4, CAR.KIA_EV6, CAR.KIA_EV6_PE, CAR.HYUNDAI_IONIQ_5_PE])
@pytest.mark.parametrize("kph", [0.0, 20.0, 40.0, 80.0, 110.0])
def test_angle_max_torque_and_smoothing_use_full_authority(car, kph):
  assert steering_angle_max_torque(car, kph / 3.6) == pytest.approx(250.0)
  assert steering_angle_smooth_tau(car, kph / 3.6) == pytest.approx(0.0)


def test_ev4_can_max_torque_uses_full_250_and_recovers_after_handover(monkeypatch):
  controller, cs, cc = setup_controller(monkeypatch, car=CAR.KIA_EV4, v_kph=25.0)
  cc.actuators.steeringAngleDeg = 10.0
  cs.out.steeringAngleDeg = 9.0
  for _ in range(120):
    output, messages = step(controller, cs, cc)
  lfa_alt = next(data for addr, data, _ in messages if addr == 0xCB)
  assert output.torqueOutputCan == pytest.approx(250.0)
  assert lfa_alt[6] == 250

  # Driver override followed by release recovers cleanly to full 250 authority.
  cs.out.steeringPressed = True
  cs.out.steeringTorque = 300.0
  for _ in range(40):
    step(controller, cs, cc)
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0.0
  cs.out.steeringAngleDeg = 10.0
  for _ in range(150):
    output, messages = step(controller, cs, cc)
  assert controller.lkas_max_torque == pytest.approx(250.0)
  assert output.torqueOutputCan == pytest.approx(250.0)
  assert controller.steer_handover.state == "waiting"


def test_ev4_no_smoothing_lag_across_speeds(monkeypatch):
  controller, cs, cc = setup_controller(monkeypatch, car=CAR.KIA_EV4, v_kph=25.0)
  cc.latActive = False
  cs.out.steeringAngleDeg = -5.0
  output, _ = step(controller, cs, cc)
  assert output.steeringAngleDeg == pytest.approx(-5.0)

  cc.latActive = True
  cc.actuators.steeringAngleDeg = -4.8
  output, _ = step(controller, cs, cc)
  assert output.steeringAngleDeg == pytest.approx(-4.8)
