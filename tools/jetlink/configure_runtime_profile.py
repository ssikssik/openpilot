"""Select a locally reviewed host experiment before committing/exporting its bundle.

This changes source metadata only. It never upgrades Jetson Linux or rewrites
an official signed release. Run prepare_local_experiment after committing and
exporting the host bundle, then register it locally on the stopped Jetson.
"""
import argparse
import json
from pathlib import Path

from local_release import validate_selection
from update_host import RUNTIME_PROFILES


def configure(root, runtime_profile, profile):
  root = Path(root)
  pin = root / 'openpilot/selfdrive/modeld/jetlink'
  spec = json.loads((pin / 'jetson_model.json').read_text())
  selection = validate_selection({'format': 1, 'delivery': 'local', 'profile': profile,
                                  'model': {'sha256': spec['sha256'], 'size': spec['nbytes']}})
  compatibility_path = root / 'tools/jetlink/runtime_compatibility.json'
  compatibility = json.loads(compatibility_path.read_text())
  if compatibility.get('protected_storage_format') != 1:
    raise ValueError('Protected storage support required')
  compatibility['runtime'] = dict(RUNTIME_PROFILES[runtime_profile])
  compatibility_path.write_text(json.dumps(compatibility, indent=2) + '\n')
  (pin / 'local_host_release.json').write_text(json.dumps(selection, indent=2) + '\n')


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--runtime-profile', choices=RUNTIME_PROFILES, required=True)
  parser.add_argument('--profile', required=True, help='Unique local catalog identifier for this committed experiment')
  args = parser.parse_args()
  configure(Path(__file__).resolve().parents[2], args.runtime_profile, args.profile)
