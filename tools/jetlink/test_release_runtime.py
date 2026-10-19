"""Release signing targets and update-probe lifecycle budgets."""
import io
import json
from pathlib import Path
import sys
import tarfile

import pytest
from Crypto.PublicKey import ECC

sys.path.insert(0, str(Path(__file__).parent))
import install_updates
import sign_release
import update_host


@pytest.mark.parametrize('target', ['36.4.7', '36.5.2'])
def test_signer_pins_requested_runtime_and_verifies_signature(tmp_path, monkeypatch, target):
  key = ECC.generate(curve='Ed25519')
  keyfile = tmp_path / 'key.pem'
  keyfile.write_text(key.export_key(format='PEM'))
  pin = {'sha256': 'b' * 64, 'nbytes': 3}
  files = {'SOURCE_COMMIT': b'a' * 40,
           'openpilot/selfdrive/modeld/jetlink/jetson_model.json': json.dumps(pin).encode(),
           'tools/jetlink/runtime_compatibility.json': b'{"protected_storage_format":1}'}
  bundle = tmp_path / 'bundle.tar.gz'
  with tarfile.open(bundle, 'w:gz') as archive:
    for name, body in files.items():
      member = tarfile.TarInfo(name)
      member.size = len(body)
      archive.addfile(member, io.BytesIO(body))
  output = tmp_path / 'manifest.json'
  monkeypatch.setattr(sys, 'argv', ['sign_release.py', '--bundle', str(bundle), '--key', str(keyfile),
    '--model-url', 'https://upload.shind0.synology.me/models/test/model.onnx',
    '--output', str(output), '--l4t', target])
  original = Path.read_text
  monkeypatch.setattr(Path, 'read_text', lambda p, *a, **kw:
    key.public_key().export_key(format='PEM') if p.name == 'release-signing-public.pem' else original(p, *a, **kw))
  sign_release.main()
  result = json.loads(output.read_text())
  assert result['runtime'] == {'arch': 'aarch64', 'l4t': target, 'tensorrt': '10.3.0'}
  assert result['storage_format'] == 1
  update_host.verify_signature(result)


def test_update_service_budgets_cover_probe_and_group_cleanup(tmp_path, monkeypatch):
  monkeypatch.setattr(install_updates, 'bootstrap', lambda *_: None)
  units = tmp_path / 'etc/systemd/system'
  units.mkdir(parents=True)
  install_updates.configure(tmp_path, Path(__file__).parent)
  for name in ['carrot-jetlink-update-apply.service', 'carrot-jetlink-update-stage.service']:
    fields = dict(line.split('=', 1) for line in (units / name).read_text().splitlines() if '=' in line)
    assert int(fields['TimeoutStartSec']) > update_host.PROBE_PROCESS_TIMEOUT >= 3600 + 30
