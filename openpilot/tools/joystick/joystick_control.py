#!/usr/bin/env python3
import os
import struct
import time
import argparse
import threading
import numpy as np

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.tools.lib.kbhit import KBHit

EXPO = 0.4
DEADZONE = 0.08
TRIGGER_DEADZONE = 0.03
MAX_GB = 0.5
MAX_STEER = 1.0

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FORMAT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)


def _clamp(val: float, lo: float = -1.0, hi: float = 1.0) -> float:
  return max(lo, min(hi, val))


def _apply_deadzone(val: float, deadzone: float) -> float:
  if abs(val) < deadzone:
    return 0.0
  sign = 1.0 if val > 0 else -1.0
  return sign * (abs(val) - deadzone) / (1.0 - deadzone)


def _normalize_stick(raw: int, deadzone: float = DEADZONE) -> float:
  norm = _apply_deadzone(_clamp(raw / 32767.0, -1.0, 1.0), deadzone)
  return EXPO * (norm ** 3) + (1.0 - EXPO) * norm


def _normalize_trigger(raw: int, deadzone: float = TRIGGER_DEADZONE) -> float:
  val = _clamp((raw + 32767.0) / 65534.0, 0.0, 1.0)
  if val < deadzone:
    return 0.0
  return (val - deadzone) / (1.0 - deadzone)


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.axis_increment = 0.05  # 5% of full actuation each key press
    self.axes_map = {'w': 'gb', 's': 'gb',
                     'a': 'steer', 'd': 'steer'}
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

  def update(self):
    key = self.kb.getch().lower()
    self.cancel = False
    if key == 'r':
      self.axes_values = dict.fromkeys(self.axes_values, 0.)
    elif key == 'c':
      self.cancel = True
    elif key in self.axes_map:
      axis = self.axes_map[key]
      incr = self.axis_increment if key in ['w', 'a'] else -self.axis_increment
      self.axes_values[axis] = float(np.clip(self.axes_values[axis] + incr, -1, 1))
    else:
      return False
    return True


class Joystick:
  """Xbox Series S|X Bluetooth controller (/dev/input/js0) for comma 3/3X.

  Verified mapping for 045E:0B13 over Bluetooth:
    - Axis 0: Left Stick X -> steer (+left, -right)
    - Axis 4: RT trigger   -> gas (+)
    - Axis 5: LT trigger   -> brake (-)
    - Btn  1: B button     -> hold for instant neutral (0, 0)
  """

  def __init__(self, device: str = "/dev/input/js0"):
    self.device = device
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

    self._raw_axes = {0: 0, 4: -32767, 5: -32767}
    self._raw_buttons = {1: 0, 6: 0}
    self._js_file = None

  def _reset(self):
    self._raw_axes = {0: 0, 4: -32767, 5: -32767}
    self._raw_buttons = {1: 0, 6: 0}
    self.axes_values['gb'] = 0.0
    self.axes_values['steer'] = 0.0
    self.cancel = False
    if self._js_file is not None:
      try:
        self._js_file.close()
      except OSError:
        pass
      self._js_file = None

  def _recompute(self):
    if self._raw_buttons.get(1, 0) == 1:
      self.axes_values['gb'] = 0.0
      self.axes_values['steer'] = 0.0
      self.cancel = True
      return

    self.cancel = False
    steer = -_normalize_stick(self._raw_axes.get(0, 0))
    gas = _normalize_trigger(self._raw_axes.get(4, -32767))
    brake = _normalize_trigger(self._raw_axes.get(5, -32767))
    gb = gas - brake

    self.axes_values['gb'] = _clamp(gb * MAX_GB, -1.0, 1.0)
    self.axes_values['steer'] = _clamp(steer * MAX_STEER, -1.0, 1.0)

  def update(self):
    if self._js_file is None:
      if not os.path.exists(self.device):
        time.sleep(0.2)
        return False
      try:
        self._js_file = open(self.device, "rb", buffering=0)
        print(f"\n[Xbox] Connected to {self.device}")
      except OSError:
        time.sleep(0.2)
        return False

    try:
      raw_ev = self._js_file.read(JS_EVENT_SIZE)
      if not raw_ev or len(raw_ev) != JS_EVENT_SIZE:
        self._reset()
        return False
    except OSError:
      print("\n[Xbox] Disconnected! Resetting outputs to 0.")
      self._reset()
      time.sleep(0.2)
      return False

    _, value, ev_type, number = struct.unpack(JS_EVENT_FORMAT, raw_ev)
    base_type = ev_type & ~JS_EVENT_INIT
    if base_type == JS_EVENT_AXIS:
      self._raw_axes[number] = value
    elif base_type == JS_EVENT_BUTTON:
      self._raw_buttons[number] = value
    else:
      return False

    self._recompute()
    return True


def send_thread(joystick):
  pm = messaging.PubMaster(['testJoystick'])

  rk = Ratekeeper(100, print_delay_threshold=None)

  while True:
    if rk.frame % 20 == 0:
      print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))

    joystick_msg = messaging.new_message('testJoystick')
    joystick_msg.valid = True
    joystick_msg.testJoystick.axes = [joystick.axes_values[ax] for ax in joystick.axes_order]

    pm.send('testJoystick', joystick_msg)

    rk.keep_time()


def joystick_control_thread(joystick):
  Params().put_bool('JoystickDebugMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()


def main():
  joystick_control_thread(Joystick())


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your Xbox controller to control your car.\n' +
                                               'openpilot must be offroad before starting joystick_control.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
  args = parser.parse_args()

  if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
    print("The car must be off before running joystick_control.")
    exit()

  print()
  if args.keyboard:
    print('Gas/brake control: `W` and `S` keys')
    print('Steering control: `A` and `D` keys')
    print('Buttons')
    print('- `R`: Resets axes')
    print('- `C`: Cancel cruise control')
  else:
    print('Using Xbox Bluetooth controller (/dev/input/js0): Left Stick=steer, RT=gas, LT=brake, B=neutral')

  joystick = Keyboard() if args.keyboard else Joystick()
  joystick_control_thread(joystick)
