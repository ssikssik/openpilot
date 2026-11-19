import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from openpilot.cereal import log
from openpilot.selfdrive.ui.device_temperature import comma_temperature_badge


class Messages(dict):
  def __init__(self, cpu=(), gpu=(), age=0, valid=True, alive=True):
    super().__init__(deviceState=log.DeviceState.new_message(cpuTempC=list(cpu), gpuTempC=list(gpu)))
    self.recv_time = {'deviceState': 100.0 - age}
    self.valid = {'deviceState': valid}
    self.alive = {'deviceState': alive}


@pytest.mark.parametrize('cpu,gpu,text,color', [
  ([40, 48], [45], 'C 48.0°C', (0, 255, 0)),
  ([50], [], 'C 50.0°C', (0, 255, 0)),
  ([55], [65], 'C 65.0°C', (255, 255, 0)),
  ([80], [78], 'C 80.0°C', (255, 165, 0)),
  ([85], [], 'C 85.0°C', (255, 82, 0)),
  ([], [90], 'C 90.0°C', (255, 0, 0)),
  ([100], [], 'C 100.0°C', (255, 0, 0)),
])
def test_hottest_cpu_gpu_sensor_and_gradient(cpu, gpu, text, color):
  assert comma_temperature_badge(Messages(cpu, gpu), 100.0) == (text, color)


@pytest.mark.parametrize('kwargs', [
  {'cpu': []}, {'cpu': [0, -273.15, 151, float('nan'), float('inf')]},
  {'cpu': [65], 'age': 5.01}, {'cpu': [65], 'age': -1},
  {'cpu': [65], 'valid': False}, {'cpu': [65], 'alive': False},
])
def test_missing_or_stale_readings_are_hidden(kwargs):
  assert comma_temperature_badge(Messages(**kwargs), 100.0) is None


@pytest.mark.parametrize('mici', [False, True])
@pytest.mark.parametrize('jetson', [False, True])
def test_both_huds_align_temperature_row_with_and_without_external_device(mici, jetson):
  path = Path(__file__).parents[1] / ('mici/onroad/hud_renderer.py' if mici else 'onroad/hud_renderer.py')
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HudRenderer')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_draw_egpu_badge')
  ui = NS(jetlink_badge=('jetSON', 'active') if jetson else None,
          jetlink_temperature_badge=('J 60.0°C', (170, 255, 0)) if jetson else None,
          comma_temperature_badge=('C 65.0°C', (255, 255, 0)),
          usbgpu_present=False, usbgpu_active=False, usbgpu_loading=False,
          usbgpu_startup_failed=False, usbgpu_compiled=False, usbgpu_compile_pending=False)
  env = {'rl': NS(Rectangle=object), 'ui_state': ui,
         'UI_CONFIG': NS(border_size=30, button_size=60), 'usbgpu_badge_state': lambda *a: 'ready'}
  exec(compile(ast.Module(body=[method], type_ignores=[]),str(path),'exec'), env)
  calls = []
  def draw(text, color, right, top, scale=1.0):
    calls.append((text, top, scale, right))
    return NS(x=right-100*scale, y=top, height=24 * scale)
  env['_draw_egpu_badge'](NS(_draw_runtime_badge=draw), NS(x=0,y=0,width=536,height=240))
  assert [x[0] for x in calls] == (['jetSON', 'J 60.0°C', 'C 65.0°C'] if jetson else ['C 65.0°C'])
  assert all(x[2] == 0.85 for x in calls)
  if jetson:
    state, jetson_temp, comma_temp = calls
    assert jetson_temp[1] == pytest.approx(state[1] + 24 * state[2] + 8)
    assert comma_temp[1] == jetson_temp[1]
    assert comma_temp[3] == pytest.approx(jetson_temp[3] - 100 * 0.85 - 8)
    assert jetson_temp[3] == state[3]
  else:
    assert calls[0][3] == (512 if mici else 422)
