"""Reject cross-ABI releases even when the source commit is already installed."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import update_host as update
from configure_runtime_profile import configure


@pytest.mark.parametrize('profile', update.RUNTIME_PROFILES.values())
def test_installed_abi_must_match(profile, monkeypatch):
  monkeypatch.setattr(update.os, 'uname', lambda: SimpleNamespace(machine='aarch64'))
  def installed(command, **kwargs):
    return profile['l4t'] + '-20261017' if command[0] == 'dpkg-query' else profile['tensorrt']
  monkeypatch.setattr(update.subprocess, 'check_output', installed)
  update.check_runtime({'runtime': profile})
  other = next(value for value in update.RUNTIME_PROFILES.values() if value != profile)
  with pytest.raises(ValueError, match='L4T'):
    update.check_runtime({'runtime': other})
  monkeypatch.setattr(update.subprocess, 'check_output', lambda command, **kw:
                      profile['l4t'] + '-1' if command[0] == 'dpkg-query' else '0.0')
  with pytest.raises(ValueError, match='TensorRT'):
    update.check_runtime({'runtime': profile})


def test_same_commit_cannot_skip_abi_check(tmp_path, monkeypatch):
  from test_update_host import manifest
  value = manifest()
  (tmp_path / 'current').mkdir()
  (tmp_path / 'current/SOURCE_COMMIT').write_text(value['source_commit'])
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  monkeypatch.setattr(update, 'verify_signature', lambda _: None)
  monkeypatch.setattr(update, 'validate_storage_compatibility', lambda _: None)
  def reject(_):
    raise ValueError('Installed L4T mismatch')
  monkeypatch.setattr(update, 'check_runtime', reject)
  with pytest.raises(ValueError, match='L4T'):
    update.stage_manifest(value)


def test_explicit_profile_keeps_model_and_signed_release(tmp_path):
  pin = tmp_path / 'openpilot/selfdrive/modeld/jetlink'
  pin.mkdir(parents=True)
  (pin / 'jetson_model.json').write_text(json.dumps({'sha256': 'a' * 64, 'nbytes': 123}))
  (pin / 'host_release.json').write_text('signed release')
  metadata = tmp_path / 'tools/jetlink/runtime_compatibility.json'
  metadata.parent.mkdir(parents=True)
  metadata.write_text('{"protected_storage_format":1}')
  assert update.bundle_runtime(json.loads(metadata.read_text())) == update.RUNTIME_PROFILES['jetpack-6']
  configure(tmp_path, 'jetpack-7.2.1', 'nx8-jp721')
  assert update.bundle_runtime(json.loads(metadata.read_text())) == update.RUNTIME_PROFILES['jetpack-7.2.1']
  assert json.loads((pin / 'local_host_release.json').read_text())['model'] == {'sha256': 'a' * 64, 'size': 123}
  assert (pin / 'host_release.json').read_text() == 'signed release'
  with pytest.raises(ValueError):
    configure(tmp_path, 'jetpack-7.2.1', '../escape')


def test_boot_gate_rejects_matching_commit_on_wrong_os(tmp_path, monkeypatch):
  import boot_update as boot
  from test_update_host import manifest
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  monkeypatch.setattr(update, 'verify_signature', lambda _: None)
  monkeypatch.setattr(update, 'validate_storage_compatibility', lambda _: None)
  def reject(_):
    raise ValueError('Installed L4T mismatch')
  monkeypatch.setattr(update, 'check_runtime', reject)
  gate = boot.Gate()
  with pytest.raises(ValueError, match='L4T'):
    gate.select(manifest())
  assert not gate.done and gate.selected is None


def test_package_records_committed_jp721_runtime(tmp_path):
  from test_local_experiment import archive
  from prepare_local_experiment import prepare, identity
  import install_local_experiment as installer
  import io
  import tarfile
  model = tmp_path / 'model.onnx'
  model.write_bytes(b'model-fixture')
  selection = {'format': 1, 'delivery': 'local', 'profile': 'nx8-jp721', 'model': identity(model)}
  bundle = tmp_path / 'source.tar.gz'
  archive(bundle, 'a' * 40, model.read_bytes(), selection)
  with tarfile.open(bundle) as source:
    members = [(member, source.extractfile(member).read()) for member in source.getmembers()]
  with tarfile.open(bundle, 'w:gz') as output:
    for member, data in members:
      if member.name == 'tools/jetlink/runtime_compatibility.json':
        data = json.dumps({'protected_storage_format': 1, 'runtime': update.RUNTIME_PROFILES['jetpack-7.2.1']}).encode()
        member.size = len(data)
      output.addfile(member, io.BytesIO(data))
  package = tmp_path / 'package'
  prepare(bundle, model, package)
  assert installer.inspect_package(package)['runtime'] == update.RUNTIME_PROFILES['jetpack-7.2.1']


@pytest.mark.parametrize('binding', ['flags', 'two-argument', 'legacy'])
def test_cuda_graph_instantiation_across_bindings(binding, monkeypatch):
  import importlib.util
  calls = []
  def flags(graph, value):
    calls.append((graph, value))
    return (0, 'executable')
  def legacy(*args):
    calls.append(args)
    if len(args) != 4:
      raise TypeError('Expected legacy arguments')
    return (0, 'executable')
  runtime = SimpleNamespace(cudaError_t=SimpleNamespace(cudaSuccess=0),
                            cudaMemcpyKind=SimpleNamespace(cudaMemcpyHostToDevice=1, cudaMemcpyDeviceToHost=2),
                            cudaGraphInstantiate=legacy if binding == 'legacy' else flags)
  if binding == 'flags':
    runtime.cudaGraphInstantiateWithFlags = flags
    runtime.cudaGraphInstantiate = lambda *a: pytest.fail('Removed API called')
  monkeypatch.setitem(sys.modules, 'cuda', SimpleNamespace())
  monkeypatch.setitem(sys.modules, 'cuda.bindings', SimpleNamespace(runtime=runtime))
  source = Path(__file__).resolve().parents[2] / 'third_party/jetlink/jetlink/server/backends/trt/cudart.py'
  spec = importlib.util.spec_from_file_location('cudart_profile_test', source)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  assert module.graph_instantiate('graph') == 'executable'
  assert calls[-1] == (('graph', None, None, 0) if binding == 'legacy' else ('graph', 0))


def test_bundle_cannot_claim_another_runtime(tmp_path):
  from test_update_host import manifest
  value = manifest()
  value['runtime'] = update.RUNTIME_PROFILES['jetpack-7.2.1']
  # Legacy bundles without explicit metadata are JetPack 6, even if a
  # manifest or a locally installed catalog claims JetPack 7.
  with pytest.raises(ValueError, match='Bundle runtime'):
    update.check_bundle(tmp_path, value)
