# Exercise the actual pure cost policy without requiring a platform-specific acados binary.
import ast
from pathlib import Path
import numpy as np

_source = Path(__file__).parents[1] / "lib/longitudinal_mpc_lib/long_mpc.py"
_tree = ast.parse(_source.read_text())
_nodes = [node for node in _tree.body if
          (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "A_CHANGE_COST" for t in node.targets))
          or (isinstance(node, ast.FunctionDef) and node.name == "get_a_change_cost")]
exec(compile(ast.Module(body=_nodes, type_ignores=[]), str(_source), "exec"))



def test_active_accel_change_cost_is_fixed():
  assert get_a_change_cost(True, 0.0) == A_CHANGE_COST
  assert get_a_change_cost(True, 500.0) == A_CHANGE_COST


def test_starting_cost_is_preserved_only_without_previous_accel_constraint():
  assert get_a_change_cost(False, 10.0) == 10.0


def test_lead_response_scales_cost_inside_mpc():
  assert get_a_change_cost(True, 500.0, 0.40) == 120.0
  assert get_a_change_cost(True, 500.0, 0.18) == 54.0
  assert get_a_change_cost(True, 500.0, 0.05) == 15.0


def test_response_factor_cannot_raise_or_invert_cost():
  assert get_a_change_cost(True, 10.0, 2.0) == A_CHANGE_COST
  assert get_a_change_cost(True, 10.0, -1.0) == 0.0
