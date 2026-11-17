import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from host_health import thermal_health
from openpilot.common import jetlink_status as status


def test_storage_recovery_is_not_reported_as_heat_or_legacy_protection(tmp_path):
  from host_health import storage_health
  marker, state = tmp_path / 'marker', tmp_path / 'state'
  value = {'severity': 'ok', 'reason': ''}
  storage_health(value, state, marker)
  assert value['storage_mode'] == 'legacy-writable' and value['severity'] == 'ok'
  marker.write_text('{}')
  for mode in ('protected', 'base-recovery'):
    state.write_text(json.dumps({'state': mode, 'system_read_only': True}))
    value = {'severity': 'ok', 'reason': ''}
    storage_health(value, state, marker)
    assert value['severity'] == ('ok' if mode == 'protected' else 'error')
    assert value['storage_mode'] == mode
  state.write_text('{torn')
  value = {'severity': 'error', 'reason': 'thermal fault'}
  storage_health(value, state, marker)
  assert value['storage_mode'] == 'unknown'
  assert value['reason'].startswith('thermal fault; ')


def test_thermal_uses_zone_limits_and_ignores_disconnected_sensors(tmp_path):
  zone = tmp_path / 'thermal_zone0'
  zone.mkdir()
  (zone / 'type').write_text('tj-thermal')
  (zone / 'trip_point_0_type').write_text('passive')
  (zone / 'trip_point_0_temp').write_text('92000')
  for temp, severity in ((86000, 'ok'), (87000, 'warning'), (92000, 'error'), (60000, 'ok'), (-256000, 'unknown')):
    (zone / 'temp').write_text(str(temp))
    assert thermal_health(tmp_path)['severity'] == severity


def test_expiry_and_error_override_active_model(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 20.)
  model.write_text(json.dumps({'updated': 20., 'active': True}))
  record = {'updated': 20., 'state': 'ready', 'peer': {'carrot_host': 'jetson'},
            'telemetry_updated': 19., 'telemetry': {'carrot_health': {
              'age_s': .1, 'severity': 'warning', 'temp_c': 87.,
              'addresses': [{'address': '192.168.0.199'}, {'address': '<script>'}, {'address': '127.0.0.1'}]}}}
  link.write_text(json.dumps(record))
  assert status.diagnostics()['addresses'] == ['192.168.0.199']
  assert status.onroad_badges() == (('jetSON HOT', 'loading'), ('87.0°C', (255, 38, 0)))
  model.write_text(json.dumps({'updated': 20., 'active': False, 'error': 'inference timeout'}))
  assert status.badge() == ('jetSON ERROR', 'error')
  assert status.diagnostics()['reason'] == 'inference timeout'
  model.write_text(json.dumps({'updated': 20., 'active': True, 'error': ''}))
  record['telemetry_updated'] = 10.
  link.write_text(json.dumps(record))
  assert status.diagnostics()['addresses'] == []
  assert status.diagnostics()['temp_c'] is None
  assert status.onroad_badges() == (('jetSON DATA?', 'loading'), None)
  record.update(state='retrying', error='engine failed')
  link.write_text(json.dumps(record))
  assert status.badge() == ('jetSON ERROR', 'error')


def test_hello_telemetry_never_becomes_current_health(tmp_path, monkeypatch):
  link = tmp_path / 'link'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', tmp_path / 'missing')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 10.)
  link.write_text(json.dumps({'updated': 10., 'state': 'ready', 'peer': {
    'carrot_host': 'jetson', 'telemetry': {'temp_c': 42}}}))
  assert status.diagnostics()['temp_c'] is None
  monkeypatch.setattr(status.time, 'monotonic', lambda: 15.)
  assert status.badge() == ('jetSON ERROR', 'error')


@pytest.mark.parametrize('value, rgb', [(45, (0, 255, 0)), (50, (0, 255, 0)),
                                        (60, (128, 255, 0)), (70, (255, 255, 0)),
                                        (80, (255, 128, 0)), (90, (255, 0, 0)), (100, (255, 0, 0))])
def test_temperature_badge_uses_onroad_gradient(value, rgb):
  health = {'fresh': True, 'label': 'jetSON', 'temp_c': value}
  assert status.temperature_badge(health) == (f'{value:.1f}°C', rgb)
  assert status.temperature_badge({**health, 'label': 'MAC'}) is None
  assert status.temperature_badge({**health, 'fresh': False}) is None
