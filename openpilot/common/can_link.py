"""Read-only onroad status of the Jetson USB CAN receiver."""
import json
import os
import time
from pathlib import Path

PATH = Path('/dev/shm/carrot-can-link.json')
STALE_S = 7.  # longer than the UI's 5 s refresh, so a live link never blinks off between refreshes

_next_write = 0.


def receiver_problem(report, available):
  state = report.get("state")
  return bool(report.get("error") or state in ("error", "receive_only")
              or (not available and state not in (None, "missing", "stopped")))


def publish(now, available, path=PATH, *, problem=False):
  global _next_write
  if now < _next_write:
    return
  _next_write = now + .5
  try:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'stamp': now, 'available': bool(available), 'problem': bool(problem)}), encoding='utf8')
    os.replace(temporary, path)
  except OSError:
    pass  # the badge is optional; never disturb the CAN sender


def link_state(now=None, path=PATH):
  try:
    value = json.loads(path.read_text(encoding='utf8'))
  except (OSError, ValueError):
    return 'disconnected'
  now = time.monotonic() if now is None else now
  stamp = value.get('stamp') if isinstance(value, dict) else None
  if type(stamp) not in (int, float) or not 0 <= now - stamp < STALE_S:
    return 'disconnected'
  if value.get('problem') is True:
    return 'problem'
  return 'connected' if value.get('available') is True else 'disconnected'


def connected(now=None, path=PATH):
  return link_state(now, path) == 'connected'
