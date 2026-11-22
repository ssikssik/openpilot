import math

import pytest

from openpilot.selfdrive.carrot.carrot_man import calculate_curvature, route_curvature_at


def arc(radius, n=12, spacing=10.0, straight=0, sign=1):
  """Route points at 10 m spacing: `straight` points on a line, then a circular arc."""
  pts = [(0.0, i * spacing) for i in range(straight)]
  x0, y0 = 0.0, straight * spacing
  for k in range(n):
    a = k * spacing / radius
    pts.append((x0 + sign * radius * (1 - math.cos(a)), y0 + radius * math.sin(a)))
  return pts


def test_tight_ramp_reads_tighter_than_40m_chord():
  # 45 m ramp (EV4 2026-11-17): the 40 m chord flattens it, the 20 m chord does not.
  pts = arc(45.0)
  coarse = calculate_curvature(pts[0], pts[4], pts[8])
  fine = route_curvature_at(pts, 0)
  assert abs(fine) > abs(coarse)
  assert 1 / abs(fine) == pytest.approx(45.0, rel=0.15)


@pytest.mark.parametrize('sign', [1, -1])
def test_direction_is_kept(sign):
  assert math.copysign(1, route_curvature_at(arc(45.0, sign=sign), 0)) == math.copysign(1, calculate_curvature(*arc(45.0, sign=sign)[0:9:4]))


def test_gentle_curve_and_straight_are_unchanged():
  for pts in (arc(800.0), [(0.0, i * 10.0) for i in range(12)]):
    assert route_curvature_at(pts, 0) == pytest.approx(calculate_curvature(pts[0], pts[4], pts[8]), abs=2e-4)


def test_single_kinked_point_does_not_tighten():
  # One GPS route point off by 3 m on a straight road: the median of three 20 m
  # readings rejects it, so the result stays the 40 m reading.
  pts = [(0.0, i * 10.0) for i in range(12)]
  pts[4] = (3.0, 40.0)
  assert route_curvature_at(pts, 0) == calculate_curvature(pts[0], pts[4], pts[8])


def test_short_route_falls_back_to_coarse():
  pts = arc(45.0, n=9)
  assert route_curvature_at(pts, 0, fine=6) == calculate_curvature(pts[0], pts[4], pts[8])
