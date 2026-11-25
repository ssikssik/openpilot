import hashlib

import pytest
from openpilot.common import big_model_catalog as C, internal_model_catalog as I
from openpilot.selfdrive.modeld import model_library


@pytest.fixture
def store(tmp_path, monkeypatch):
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  monkeypatch.setattr(C, 'boot_id', lambda: 'boot-one')
  return tmp_path


def test_choice_is_independent_and_frozen_until_next_real_boot(store, monkeypatch):
  C.freeze_boot()
  assert I.freeze_boot() == 'cheddar'
  I.select('cd210')
  C.select('flip-flop')
  assert I.freeze_boot() == I.effective_id() == 'cheddar'
  assert C.effective_id() == 'n16'
  assert model_library.status()['internal_selection']['reboot_required']
  monkeypatch.setattr(C, 'boot_id', lambda: 'boot-two')
  assert I.freeze_boot() == 'cd210'
  assert C.freeze_boot() == 'flip-flop'
  assert I.pkl_path().name == 'driving_cd210_tinygrad.pkl'
  assert I.onnx_path().name == 'driving_cd210.onnx'
  assert not model_library.status()['reboot_required']
  I.select('cheddar')
  assert I.effective_id() == 'cd210'  # worker recovery must keep the boot choice


@pytest.mark.parametrize('value', ['unknown', '../cd210', '', None, [], {}])
def test_invalid_choices_do_not_replace_preference(store, value):
  I.select('cd210')
  with pytest.raises(ValueError): I.select(value)
  assert I.selected() == 'cd210'


@pytest.mark.parametrize('value', ['{bad', '{"selected": []}', '{"selected": "unknown"}'])
def test_corrupt_preference_or_boot_falls_back_to_cheddar(store, value):
  (store / 'internal-selection.json').write_text(value)
  (store / 'internal-boot-selection.json').write_text(value)
  assert I.selected() == I.effective_id() == I.freeze_boot() == 'cheddar'


def test_cd210_restores_exact_pre_cheddar_bytes():
  source = I.ROOT / 'driving_cd210.onnx'
  with source.open('rb') as stream:
    assert hashlib.file_digest(stream, 'sha256').hexdigest() == 'f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5'


def test_launcher_cli_freezes_internal_and_external_together(store):
  import os
  import subprocess
  import sys
  I.select('cd210')
  result = subprocess.run([sys.executable, '-m', 'openpilot.common.big_model_catalog'],
                          cwd=C.ROOT, env=os.environ.copy(), text=True, capture_output=True, check=True)
  assert result.stdout.strip() == 'n16'
  assert C.read_json(store / 'internal-boot-selection.json')['selected'] == 'cd210'
  assert C.read_json(store / 'boot-selection.json')['selected'] == 'n16'
