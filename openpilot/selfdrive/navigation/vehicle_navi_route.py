"""Synthesize a local road-snapped route polyline from Mapbox vector tiles and stock-navi TBT.

Stock vehicle navigation (e.g., Hyundai/Kia CAN-FD 0x2A3) broadcasts the upcoming
turn maneuver and distance, not a full route polyline. Because NOO's nav_map only
covers 256x256 m (192 m ahead of ego), we can trace the local route directly on
the Mapbox Streets v8 road graph already cached by MapRenderer and fail closed
whenever the road or upcoming turn branch cannot be matched unambiguously.
"""
from dataclasses import dataclass
import math
import time

import numpy as np

from openpilot.selfdrive.navigation.renderer import EARTH_CIRCUMFERENCE, MapRenderer, mercator, road_level
from openpilot.selfdrive.navigation.route import RouteTracker
from openpilot.selfdrive.navigation.vector_tile import Road

GRAPH_RADIUS_M = 360.0
SNAP_RADIUS_M = 2.5
T_JUNCTION_MIN_END_DIST_M = 1.0
MAX_MATCH_DIST_M = 25.0
MAX_MATCH_HEADING_DEG = 55.0
BACK_TRACE_M = 75.0
STRAIGHT_TRACE_M = 250.0
POST_TURN_TRACE_M = 200.0
TURN_LOOKAHEAD_M = 225.0
TURN_FAIL_CLOSED_M = 175.0
JUNCTION_EARLY_WINDOW_M = 50.0
JUNCTION_LATE_WINDOW_M = 60.0
BRANCH_HEADING_LOOKAHEAD_M = 22.0
LOCK_TIMEOUT_S = 12.0
LOCK_MAX_ROUTE_DIST_M = 25.0
LOCK_MAX_PAST_JUNCTION_M = 45.0
MIN_ROUTE_LENGTH_M = 25.0

ROAD_RANK = {
  'motorway': 6,
  'trunk': 5,
  'primary': 4,
  'secondary': 3,
  'tertiary': 2,
  'motorway_link': 3,
  'trunk_link': 3,
  'primary_link': 2,
  'secondary_link': 2,
  'tertiary_link': 1,
  'street': 1,
  'street_limited': 1,
  'residential': 1,
  'unclassified': 1,
  'living_street': 1,
  'service': 0,
  'driveway': 0,
  'parking_aisle': 0,
  'alley': 0,
  'track': 0,
}

MINOR_DETAILS = frozenset({'service', 'driveway', 'parking_aisle', 'alley', 'track'})


@dataclass(frozen=True)
class TurnSpec:
  min_angle: float  # degrees clockwise (+right, -left)
  max_angle: float
  ideal_angle: float
  prefer_link: bool = False
  uturn: bool = False


def turn_spec(maneuver_type: str, maneuver_modifier: str) -> TurnSpec | None:
  mtype = (maneuver_type or '').strip().lower()
  mmod = (maneuver_modifier or '').strip().lower()
  if mtype in ('', 'invalid', 'notification', 'arrive', 'depart', 'continue', 'new name'):
    return None
  if mtype == 'turn':
    if mmod == 'left':
      return TurnSpec(-140.0, -40.0, -90.0)
    if mmod == 'right':
      return TurnSpec(40.0, 140.0, 90.0)
    if mmod == 'sharp left':
      return TurnSpec(-160.0, -70.0, -120.0)
    if mmod == 'sharp right':
      return TurnSpec(70.0, 160.0, 120.0)
    if mmod == 'slight left':
      return TurnSpec(-75.0, -10.0, -30.0)
    if mmod == 'slight right':
      return TurnSpec(10.0, 75.0, 30.0)
    if mmod == 'uturn':
      return TurnSpec(135.0, 180.0, 180.0, uturn=True)
    return None
  if mtype == 'fork':
    if 'left' in mmod:
      return TurnSpec(-75.0, -8.0, -30.0)
    if 'right' in mmod:
      return TurnSpec(8.0, 75.0, 30.0)
    return None
  if mtype in ('off ramp', 'on ramp'):
    if 'left' in mmod:
      return TurnSpec(-110.0, -5.0, -30.0, prefer_link=True)
    if 'right' in mmod:
      return TurnSpec(5.0, 110.0, 30.0, prefer_link=True)
    return TurnSpec(-110.0, 110.0, 25.0, prefer_link=True)
  return None


def angle_diff_deg(a: float, b: float) -> float:
  """Signed clockwise angle from heading a to heading b in [-180, 180)."""
  return ((b - a + 180.0) % 360.0) - 180.0


def segment_heading_deg(p0: np.ndarray, p1: np.ndarray) -> float:
  dx, dy = float(p1[0] - p0[0]), float(p1[1] - p0[1])
  return math.degrees(math.atan2(dx, dy)) % 360.0


def mercator_to_enu(points: np.ndarray, origin_merc: np.ndarray, scale: float) -> np.ndarray:
  delta = points - origin_merc
  delta[:, 0] = (delta[:, 0] + 0.5) % 1.0 - 0.5
  return np.column_stack((delta[:, 0] * scale, -delta[:, 1] * scale))


def enu_to_lonlat(points_enu: np.ndarray, origin_merc: np.ndarray, scale: float) -> np.ndarray:
  mx = (origin_merc[0] + points_enu[:, 0] / scale) % 1.0
  my = origin_merc[1] - points_enu[:, 1] / scale
  lon = mx * 360.0 - 180.0
  lat = np.degrees(np.arctan(np.sinh(np.pi * (1.0 - 2.0 * my))))
  return np.column_stack((lon, lat))


def _road_rank(road: Road) -> int:
  if road.detail in ROAD_RANK:
    return ROAD_RANK[road.detail]
  if road.kind == 'highway':
    return 4
  if road.kind == 'major_road':
    return 2
  if road.kind == 'minor_road':
    return 1
  return 0


@dataclass
class _Edge:
  idx: int
  u: int
  v: int
  p0: np.ndarray
  p1: np.ndarray
  length: float
  heading: float
  level: int
  rank: int
  is_link: bool
  is_minor: bool
  reverse_idx: int = -1


class _LocalRoadGraph:
  """Directed road graph on the local tangent plane around ego."""

  def __init__(self, polylines: list[tuple[np.ndarray, Road]]):
    self.node_pos: list[np.ndarray] = []
    self.node_level: list[int] = []
    self.edges: list[_Edge] = []
    self.outgoing: list[list[int]] = []
    self.incoming: list[list[int]] = []
    self._build(polylines)

  def _build(self, polylines: list[tuple[np.ndarray, Road]]):
    # Filter to roads intersecting the local radius and deduplicate consecutive identical points.
    filtered: list[tuple[np.ndarray, int, int, bool, bool, bool]] = []
    endpoints_by_level: dict[int, list[np.ndarray]] = {}
    for pts, road in polylines:
      if len(pts) < 2:
        continue
      if (pts[:, 0].max() < -GRAPH_RADIUS_M or pts[:, 0].min() > GRAPH_RADIUS_M or
          pts[:, 1].max() < -GRAPH_RADIUS_M or pts[:, 1].min() > GRAPH_RADIUS_M):
        continue
      # Keep vertices within a slightly larger margin so segments crossing the boundary stay intact.
      diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
      keep = np.r_[True, diffs > 0.25]
      clean = pts[keep]
      if len(clean) < 2:
        continue
      lvl = road_level(road)
      rank = _road_rank(road)
      is_link = road.detail.endswith('_link')
      is_minor = road.detail in MINOR_DETAILS
      filtered.append((clean, lvl, rank, is_link, is_minor, bool(road.oneway)))
      endpoints_by_level.setdefault(lvl, []).append(clean[0])
      endpoints_by_level[lvl].append(clean[-1])

    if not filtered:
      return

    # Split polylines at T-junctions where another same-level road's endpoint touches an interior segment.
    split_polylines: list[tuple[np.ndarray, int, int, bool, bool, bool]] = []
    for pts, lvl, rank, is_link, is_minor, oneway in filtered:
      eps = endpoints_by_level.get(lvl, [])
      if len(eps) > 2:
        ep_arr = np.asarray(eps, dtype=np.float64)
        new_pts = [pts[0]]
        for i in range(len(pts) - 1):
          a, b = pts[i], pts[i + 1]
          ab = b - a
          seg_len2 = float(ab[0] * ab[0] + ab[1] * ab[1])
          if seg_len2 > 4.0:
            seg_len = math.sqrt(seg_len2)
            # Quick bbox filter for candidate T-junction endpoints.
            xmin, xmax = min(a[0], b[0]) - SNAP_RADIUS_M, max(a[0], b[0]) + SNAP_RADIUS_M
            ymin, ymax = min(a[1], b[1]) - SNAP_RADIUS_M, max(a[1], b[1]) + SNAP_RADIUS_M
            mask = (ep_arr[:, 0] >= xmin) & (ep_arr[:, 0] <= xmax) & (ep_arr[:, 1] >= ymin) & (ep_arr[:, 1] <= ymax)
            if np.any(mask):
              cands = ep_arr[mask]
              t = ((cands[:, 0] - a[0]) * ab[0] + (cands[:, 1] - a[1]) * ab[1]) / seg_len2
              proj = a + t[:, None] * ab
              perp = np.linalg.norm(cands - proj, axis=1)
              along = t * seg_len
              valid = (perp <= SNAP_RADIUS_M) & (along >= T_JUNCTION_MIN_END_DIST_M) & (along <= seg_len - T_JUNCTION_MIN_END_DIST_M)
              if np.any(valid):
                splits = np.sort(t[valid])
                last_t = 0.0
                for tv in splits:
                  if (tv - last_t) * seg_len >= 1.0:
                    new_pts.append(a + tv * ab)
                    last_t = tv
          new_pts.append(b)
        pts = np.asarray(new_pts, dtype=np.float64)
      split_polylines.append((pts, lvl, rank, is_link, is_minor, oneway))

    # Spatial grid node clustering on same level.
    grid: dict[tuple[int, int, int], list[int]] = {}
    snap2 = SNAP_RADIUS_M * SNAP_RADIUS_M

    def get_node(pt: np.ndarray, lvl: int) -> int:
      gx = int(math.floor(pt[0] / SNAP_RADIUS_M))
      gy = int(math.floor(pt[1] / SNAP_RADIUS_M))
      best_id = -1
      best_d2 = snap2
      for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
          for nid in grid.get((lvl, gx + dx, gy + dy), ()):
            np_pos = self.node_pos[nid]
            d2 = (np_pos[0] - pt[0]) ** 2 + (np_pos[1] - pt[1]) ** 2
            if d2 <= best_d2:
              best_d2 = d2
              best_id = nid
      if best_id >= 0:
        return best_id
      nid = len(self.node_pos)
      self.node_pos.append(pt.copy())
      self.node_level.append(lvl)
      self.outgoing.append([])
      self.incoming.append([])
      grid.setdefault((lvl, gx, gy), []).append(nid)
      return nid

    seen_directed: set[tuple[int, int]] = set()
    for pts, lvl, rank, is_link, is_minor, oneway in split_polylines:
      node_ids = [get_node(pt, lvl) for pt in pts]
      for i in range(len(node_ids) - 1):
        u, v = node_ids[i], node_ids[i + 1]
        if u == v:
          continue
        p0, p1 = self.node_pos[u], self.node_pos[v]
        length = float(math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
        if length < 0.25:
          continue
        fwd_idx = -1
        if (u, v) not in seen_directed:
          seen_directed.add((u, v))
          fwd_idx = len(self.edges)
          e_fwd = _Edge(fwd_idx, u, v, p0, p1, length, segment_heading_deg(p0, p1), lvl, rank, is_link, is_minor)
          self.edges.append(e_fwd)
          self.outgoing[u].append(fwd_idx)
          self.incoming[v].append(fwd_idx)
        if not oneway and (v, u) not in seen_directed:
          seen_directed.add((v, u))
          rev_idx = len(self.edges)
          e_rev = _Edge(rev_idx, v, u, p1, p0, length, segment_heading_deg(p1, p0), lvl, rank, is_link, is_minor,
                        reverse_idx=fwd_idx)
          self.edges.append(e_rev)
          self.outgoing[v].append(rev_idx)
          self.incoming[u].append(rev_idx)
          if fwd_idx >= 0:
            self.edges[fwd_idx].reverse_idx = rev_idx

  def match_ego(self, bearing_deg: float) -> tuple[_Edge, float, np.ndarray] | None:
    """Match ego at (0, 0) with bearing_deg to the closest heading-aligned directed edge."""
    best = None
    best_cost = math.inf
    for e in self.edges:
      ab = e.p1 - e.p0
      len2 = e.length * e.length
      t = float(np.clip(-(e.p0[0] * ab[0] + e.p0[1] * ab[1]) / max(len2, 1e-12), 0.0, 1.0))
      proj = e.p0 + t * ab
      dist = float(math.hypot(proj[0], proj[1]))
      if dist > MAX_MATCH_DIST_M:
        continue
      hdg_err = abs(angle_diff_deg(bearing_deg, e.heading))
      if hdg_err > MAX_MATCH_HEADING_DEG:
        continue
      cost = dist + 0.35 * hdg_err - 0.6 * e.rank + (6.0 if e.is_minor else 0.0)
      if cost < best_cost:
        best_cost = cost
        best = (e, t, proj)
    return best

  def branch_heading(self, start_edge: _Edge, max_dist: float = BRANCH_HEADING_LOOKAHEAD_M) -> float:
    """Measure the outgoing direction of a branch over ~20 m to smooth short chamfer fillets."""
    origin = start_edge.p0
    cur = start_edge
    dist = cur.length
    end_pt = cur.p1
    visited = {cur.idx}
    while dist < max_dist:
      nxt = self._straightest_next(cur, max_angle=45.0, visited=visited)
      if nxt is None:
        break
      visited.add(nxt.idx)
      dist += nxt.length
      end_pt = nxt.p1
      cur = nxt
    if math.hypot(end_pt[0] - origin[0], end_pt[1] - origin[1]) < 1.0:
      return start_edge.heading
    return segment_heading_deg(origin, end_pt)

  def _straightest_next(self, edge: _Edge, max_angle: float = 55.0, visited: set[int] | None = None) -> _Edge | None:
    best = None
    best_cost = math.inf
    for eid in self.outgoing[edge.v]:
      if eid == edge.reverse_idx or (visited is not None and eid in visited):
        continue
      cand = self.edges[eid]
      da = abs(angle_diff_deg(edge.heading, cand.heading))
      if da > max_angle:
        continue
      # Prefer continuing on the same road class and avoiding minor service exits.
      rank_penalty = abs(cand.rank - edge.rank) * 4.0
      minor_penalty = 18.0 if (cand.is_minor and not edge.is_minor) else 0.0
      link_penalty = 12.0 if (cand.is_link and not edge.is_link) else 0.0
      cost = da + rank_penalty + minor_penalty + link_penalty
      if cost < best_cost:
        best_cost = cost
        best = cand
    return best

  def _straightest_prev(self, edge: _Edge, max_angle: float = 55.0, visited: set[int] | None = None) -> _Edge | None:
    best = None
    best_cost = math.inf
    for eid in self.incoming[edge.u]:
      if eid == edge.reverse_idx or (visited is not None and eid in visited):
        continue
      cand = self.edges[eid]
      da = abs(angle_diff_deg(cand.heading, edge.heading))
      if da > max_angle:
        continue
      cost = da + abs(cand.rank - edge.rank) * 4.0 + (18.0 if (cand.is_minor and not edge.is_minor) else 0.0)
      if cost < best_cost:
        best_cost = cost
        best = cand
    return best

  def trace_backward(self, start_edge: _Edge, t_ego: float, target_dist: float = BACK_TRACE_M) -> list[np.ndarray]:
    """Trace backward from ego projection along the incoming road, returned start-to-ego."""
    pts: list[np.ndarray] = []
    traveled = t_ego * start_edge.length
    if traveled > 0.5:
      pts.append(start_edge.p0)
    cur = start_edge
    visited = {cur.idx}
    while traveled < target_dist:
      prev = self._straightest_prev(cur, visited=visited)
      if prev is None:
        break
      visited.add(prev.idx)
      traveled += prev.length
      pts.append(prev.p0)
      cur = prev
    pts.reverse()
    return pts

  def trace_straight(self, start_edge: _Edge, start_pt: np.ndarray, initial_remaining: float,
                     target_dist: float) -> list[np.ndarray]:
    pts: list[np.ndarray] = [start_pt, start_edge.p1]
    traveled = initial_remaining
    cur = start_edge
    visited = {cur.idx}
    while traveled < target_dist:
      nxt = self._straightest_next(cur, visited=visited)
      if nxt is None:
        break
      visited.add(nxt.idx)
      pts.append(nxt.p1)
      traveled += nxt.length
      cur = nxt
    return pts

  def trace_with_turn(self, start_edge: _Edge, start_pt: np.ndarray, initial_remaining: float,
                      spec: TurnSpec, maneuver_dist: float) -> tuple[list[np.ndarray], np.ndarray] | None:
    """Walk forward along the ego road, select the best matching turn junction, and trace post-turn."""
    search_min = max(-5.0, maneuver_dist - JUNCTION_EARLY_WINDOW_M)
    search_max = max(35.0, maneuver_dist + JUNCTION_LATE_WINDOW_M)
    walk_limit = min(STRAIGHT_TRACE_M + 40.0, max(search_max + 20.0, 120.0))

    prefix_pts: list[np.ndarray] = [start_pt]
    cur = start_edge
    dist_to_v = initial_remaining
    visited = {cur.idx}

    best_score = math.inf
    best_choice = None  # (prefix_copy, branch_edge, junction_pt)

    while True:
      prefix_pts.append(cur.p1)
      if search_min <= dist_to_v <= search_max:
        for eid in self.outgoing[cur.v]:
          if eid == cur.reverse_idx and not spec.uturn:
            continue
          cand = self.edges[eid]
          out_hdg = cand.heading if spec.uturn else self.branch_heading(cand)
          turn_ang = angle_diff_deg(cur.heading, out_hdg)
          if spec.uturn:
            if abs(turn_ang) < spec.min_angle:
              continue
            ang_err = abs(180.0 - abs(turn_ang))
          else:
            if not (spec.min_angle <= turn_ang <= spec.max_angle):
              continue
            ang_err = abs(turn_ang - spec.ideal_angle)
          dist_err = abs(dist_to_v - maneuver_dist)
          # Prefer links for ramps, avoid minor service alleys for regular turns.
          class_adj = 0.0
          if spec.prefer_link and cand.is_link:
            class_adj -= 20.0
          if cand.is_minor and not cur.is_minor:
            class_adj += 25.0
          if not spec.prefer_link and not cand.is_minor:
            class_adj -= min(cand.rank, 4) * 2.0
          score = dist_err + 0.55 * ang_err + class_adj
          if score < best_score:
            best_score = score
            best_choice = (list(prefix_pts), cand, cur.p1.copy())

      if dist_to_v >= walk_limit:
        break
      nxt = self._straightest_next(cur, visited=visited)
      if nxt is None:
        break
      visited.add(nxt.idx)
      dist_to_v += nxt.length
      cur = nxt

    if best_choice is None:
      return None

    chosen_prefix, branch_edge, junction_pt = best_choice
    post_pts = self.trace_straight(branch_edge, branch_edge.p0, branch_edge.length, POST_TURN_TRACE_M)
    # post_pts[0] is branch_edge.p0 == chosen_prefix[-1], so skip duplicate vertex.
    return chosen_prefix + post_pts[1:], junction_pt


def _dedup_polyline(pts: list[np.ndarray]) -> np.ndarray:
  if not pts:
    return np.empty((0, 2), dtype=np.float64)
  out = [pts[0]]
  for p in pts[1:]:
    if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= 0.5:
      out.append(p)
  return np.asarray(out, dtype=np.float64).reshape((-1, 2))


def _extract_primary_maneuver(nav_instruction) -> tuple[str, str, float] | None:
  if nav_instruction is None:
    return None
  maneuvers = getattr(nav_instruction, 'allManeuvers', None)
  if maneuvers is not None and len(maneuvers) > 0:
    m0 = maneuvers[0]
    mtype = str(getattr(m0, 'type', '') or '')
    mmod = str(getattr(m0, 'modifier', '') or '')
    mdist = float(getattr(m0, 'distance', math.nan))
    if mtype and mtype != 'invalid' and math.isfinite(mdist):
      return mtype, mmod, mdist
  mtype = str(getattr(nav_instruction, 'maneuverType', '') or '')
  mmod = str(getattr(nav_instruction, 'maneuverModifier', '') or '')
  mdist = float(getattr(nav_instruction, 'maneuverDistance', math.nan))
  if mtype and mtype != 'invalid' and math.isfinite(mdist):
    return mtype, mmod, mdist
  return None


class VehicleNaviRouteSynthesizer:
  """Stateful local route synthesizer with temporal junction lock-on for stock navi."""

  def __init__(self):
    self._locked = None

  def reset(self):
    self._locked = None

  def _try_locked(self, gps, lon: float, lat: float, bearing: float, maneuver_key: tuple[str, str],
                  mdist: float, now: float) -> tuple[np.ndarray, tuple[float, float, float]] | None:
    lock = self._locked
    if lock is None:
      return None
    if now >= lock['expires'] or lock['maneuver'] != maneuver_key:
      self._locked = None
      return None
    # Release lock if TBT jumped forward to a new distant maneuver.
    if mdist > lock['last_mdist'] + 60.0:
      self._locked = None
      return None
    tracker: RouteTracker = lock['tracker']
    dt = min(0.5, max(0.0, now - float(getattr(gps, 'time', now))))
    advance = max(0.0, float(getattr(gps, 'speed', 0.0)) * dt)
    pose = tracker.update_pose(lon, lat, advance, max_distance=LOCK_MAX_ROUTE_DIST_M + advance)
    if pose is None:
      self._locked = None
      return None
    # Keep locked while approaching or turning through the locked junction.
    if tracker.progress > lock['junction_progress'] + LOCK_MAX_PAST_JUNCTION_M:
      self._locked = None
      return None
    if abs(angle_diff_deg(bearing, pose[2])) > 75.0:
      self._locked = None
      return None
    lock['last_mdist'] = mdist
    lock['expires'] = now + LOCK_TIMEOUT_S
    return lock['route'], pose

  def synthesize(self, gps, nav_instruction, renderer: MapRenderer,
                 now: float | None = None) -> tuple[np.ndarray, tuple[float, float, float]] | None:
    if gps is None or renderer is None:
      self.reset()
      return None
    maneuver = _extract_primary_maneuver(nav_instruction)
    if maneuver is None:
      self.reset()
      return None
    mtype, mmod, mdist = maneuver
    if mdist < -40.0:
      self.reset()
      return None

    now = time.monotonic() if now is None else now
    lon, lat, bearing = float(gps.longitude), float(gps.latitude), float(gps.bearing)
    if not all(math.isfinite(v) for v in (lon, lat, bearing)):
      return None

    spec = turn_spec(mtype, mmod)
    if spec is not None:
      locked_result = self._try_locked(gps, lon, lat, bearing, (mtype, mmod), mdist, now)
      if locked_result is not None:
        return locked_result
    else:
      self._locked = None

    merc_roads, complete = renderer.view_roads(lon, lat)
    if not complete or not merc_roads:
      return None

    origin_merc = mercator(np.array([[lon, lat]], dtype=np.float64))[0]
    scale = EARTH_CIRCUMFERENCE * math.cos(math.radians(lat))
    enu_roads = [(mercator_to_enu(pts, origin_merc, scale), road) for pts, road in merc_roads]
    graph = _LocalRoadGraph(enu_roads)
    match = graph.match_ego(bearing)
    if match is None:
      return None
    edge, t_ego, ego_proj = match
    initial_remaining = (1.0 - t_ego) * edge.length

    back_pts = graph.trace_backward(edge, t_ego, BACK_TRACE_M)
    junction_pt = None
    if spec is not None and mdist <= TURN_LOOKAHEAD_M:
      turn_res = graph.trace_with_turn(edge, ego_proj, initial_remaining, spec, mdist)
      if turn_res is not None:
        fwd_pts, junction_pt = turn_res
      elif mdist <= TURN_FAIL_CLOSED_M:
        # Never draw a misleading straight route through an imminent unmatched turn.
        return None
      else:
        fwd_pts = graph.trace_straight(edge, ego_proj, initial_remaining, max(60.0, mdist - 15.0))
    else:
      fwd_pts = graph.trace_straight(edge, ego_proj, initial_remaining, STRAIGHT_TRACE_M)

    route_enu = _dedup_polyline(back_pts + fwd_pts)
    if len(route_enu) < 2:
      return None
    total_len = float(np.linalg.norm(np.diff(route_enu, axis=0), axis=1).sum())
    if total_len < MIN_ROUTE_LENGTH_M:
      return None

    route_lonlat = enu_to_lonlat(route_enu, origin_merc, scale)
    if not np.isfinite(route_lonlat).all():
      return None

    tracker = RouteTracker(route_lonlat)
    dt = min(0.5, max(0.0, now - float(getattr(gps, 'time', now))))
    advance = max(0.0, float(getattr(gps, 'speed', 0.0)) * dt)
    pose = tracker.update_pose(lon, lat, advance, max_distance=MAX_MATCH_DIST_M + advance)
    if pose is None:
      return None
    if abs(angle_diff_deg(bearing, pose[2])) > 75.0:
      return None

    if spec is not None and junction_pt is not None:
      j_lonlat = enu_to_lonlat(junction_pt.reshape(1, 2), origin_merc, scale)[0]
      j_tracker = RouteTracker(route_lonlat)
      j_progress = j_tracker.project(float(j_lonlat[0]), float(j_lonlat[1]))
      self._locked = {
        'maneuver': (mtype, mmod),
        'route': route_lonlat,
        'tracker': tracker,
        'junction_progress': j_progress,
        'last_mdist': mdist,
        'expires': now + LOCK_TIMEOUT_S,
      }

    return route_lonlat, pose
