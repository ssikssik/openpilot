"""Run real Device wake methods with simulated UI state and no display IO."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import pytest


@pytest.mark.parametrize("touch,size,kind,age,expected", [
  (False, 0, "", 0, False),
  (True, 0, "", 0, True),
  (False, 2, "canError/immediateDisable", 0, True),
  (False, 1, "dashcamMode/permanent", 0, False),
  (False, 0, "", 2, True),
])
def test_screen_off_touch_and_alert_wake(touch, size, kind, age, expected):
  source = Path(__file__).parents[1] / "ui_state.py"
  tree = ast.parse(source.read_text())
  device = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Device")
  device.body = [n for n in device.body if isinstance(n, ast.FunctionDef)
                 and n.name in {"_update_wakefulness", "_alert_visible"}]
  class Messages(dict):
    recv_time = {"selfdriveState": 100.0 - age}
  sm = Messages(selfdriveState=NS(alertSize=size, alertType=kind))
  env = {"time": NS(monotonic=lambda: 100.0),
         "ui_state": NS(ignition=True, started=True, screen_off=True, sm=sm),
         "gui_app": NS(mouse_events=[NS(left_down=True)] if touch else []),
         "PC": False, "SCREEN_OFF_TOUCH_WAKE_S": 5.0}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[device], type_ignores=[])), str(source), "exec"), env)
  instance = env["Device"]()
  instance._ignition = True
  instance._interaction_time = 0
  instance._prev_timed_out = True
  instance._interactive_timeout_callbacks = []
  instance._screen_off_wake_until = 0
  instance._set_awake = lambda on: setattr(instance, "awake", on)
  instance._reset_interactive_timeout = lambda: None
  instance._update_wakefulness()
  assert instance.awake is expected
