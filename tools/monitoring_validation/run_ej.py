"""Offline ej integration checks against current carrot-dev policies; no vehicle IPC."""
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if Path.cwd().resolve() != ROOT:
  raise SystemExit("Run from the repository root")
sys.argv = [str(ROOT / "tools/monitoring_validation/run.py"), "-c", "/dev/null", "--rootdir=.",
  "opendbc_repo/opendbc/car/hyundai/tests/test_park_assist_and_steer_test.py",
  "opendbc_repo/opendbc/car/hyundai/tests/test_steering_handover_controller.py",
  "opendbc_repo/opendbc/car/hyundai/tests/test_highbeam_test.py",
  "opendbc_repo/opendbc/car/hyundai/tests/test_stopping.py",
  "openpilot/selfdrive/controls/tests/test_lane_clamp.py",
  "openpilot/selfdrive/controls/tests/test_lateral_startup_control.py",
  "openpilot/selfdrive/controls/tests/test_long_mpc_a_change_cost.py",
  "openpilot/selfdrive/ui/tests/test_carrot_param_cache.py",
  "openpilot/selfdrive/ui/tests/test_screen_wake_policy.py",
  "openpilot/selfdrive/monitoring", *sys.argv[1:]]
runpy.run_path(sys.argv[0], run_name="__main__")
