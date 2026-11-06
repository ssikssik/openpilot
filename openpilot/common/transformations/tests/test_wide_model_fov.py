import math

import numpy as np
import pytest

from openpilot.common.transformations.model import (
  SBIGMODEL_INPUT_SIZE, WIDE_MODEL_FOV_CHOICES, sbigmodel_intrinsics, wide_model_input_warp,
)


@pytest.mark.parametrize("selection,calibrated,wide", [(0, True, True), (-1, True, True), (5, True, True),
                                                       (4, False, True), (4, True, False)])
def test_unavailable_or_stock_wide_fov_preserves_projection(selection, calibrated, wide):
  stock = np.array([[1.1, 0.02, 23.], [-0.03, 0.9, 46.], [0.0002, -0.0001, 1.]])
  result = wide_model_input_warp(stock, selection, calibrated=calibrated, has_wide_camera=wide)
  np.testing.assert_array_equal(result, stock.astype(np.float32))


@pytest.mark.parametrize("selection,fov", tuple(enumerate(WIDE_MODEL_FOV_CHOICES, 1)))
def test_widening_keeps_center_and_projects_requested_edge_rays(selection, fov):
  # Use stock camera intrinsics as the input projection so expected rays can
  # be measured independently by unprojecting the output edge pixels.
  stock = sbigmodel_intrinsics.copy()
  # A stock wide warp converts wide model pixels to calibrated camera pixels.
  result = wide_model_input_warp(np.eye(3), selection, calibrated=True, has_wide_camera=True)
  cx, cy = stock[:2, 2]
  half_width = SBIGMODEL_INPUT_SIZE[0] / 2
  center = result @ np.array([cx, cy, 1.])
  np.testing.assert_allclose(center, [cx, cy, 1.], atol=1e-4)
  edge = result @ np.array([cx + half_width, cy, 1.])
  ray = np.linalg.solve(stock, edge)
  assert math.degrees(2 * math.atan2(ray[0], ray[2])) == pytest.approx(fov, abs=1e-4)
  np.testing.assert_array_equal(stock, sbigmodel_intrinsics)
