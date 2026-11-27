"""Internal policy preference, frozen before build and inference for one boot."""
from pathlib import Path
from openpilot.common import big_model_catalog as C

DEFAULT = 'cd210'  # the previous fixed internal model; Cheddar stays selectable
MODELS = {'cd210': 'CD210', 'cheddar': 'Cheddar'}
ROOT = Path(__file__).resolve().parents[1] / 'selfdrive/modeld/models'


def selected():
  key = C.read_json(C.storage() / 'internal-selection.json').get('selected')
  return key if isinstance(key, str) and key in MODELS else DEFAULT


def effective_id():
  value = C.read_json(C.storage() / 'internal-boot-selection.json')
  key = value.get('selected')
  return key if value.get('boot') == C.boot_id() and isinstance(key, str) and key in MODELS else DEFAULT


@C.synchronized
def freeze_boot():
  path = C.storage() / 'internal-boot-selection.json'
  value = C.read_json(path)
  if value.get('boot') != C.boot_id() or not isinstance(value.get('selected'), str) or value['selected'] not in MODELS:
    C.atomic_json(path, {'boot': C.boot_id(), 'selected': selected()})
  return effective_id()


@C.synchronized
def select(key):
  if not isinstance(key, str) or key not in MODELS:
    raise ValueError('unknown internal model')
  C.atomic_json(C.storage() / 'internal-selection.json', {'selected': key})


def onnx_path():
  return ROOT / ('driving_cd210.onnx' if effective_id() == 'cd210' else 'driving_supercombo.onnx')


def pkl_path():
  return ROOT / ('driving_cd210_tinygrad.pkl' if effective_id() == 'cd210' else 'driving_tinygrad.pkl')


def status():
  current, pending = effective_id(), selected()
  return {'models': [{'id': key, 'name': name} for key, name in MODELS.items()],
          'effective': current, 'selected': pending, 'reboot_required': current != pending}
