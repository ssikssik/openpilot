"""Preserve an existing in-lane model path past a corroborated roadside track."""

from dataclasses import dataclass
import math

import numpy as np


MAX_SENSOR_AGE = 0.20
CONFIRM_TIME = 0.30
MAX_SAMPLE_GAP = 0.16
# Retain the existing conservative clearance envelope for both supported cars.
# EV4 admission does not reduce margins or shorten the swept-path check.
LANE_MARGIN = 1.20
EGO_LENGTH = 5.1
# Radar returns a point, not a measured outline. Use an inflated vehicle envelope.
OBJECT_LENGTH = 7.0
OBJECT_CLEARANCE = 2.5
MIN_CLEARANCE_GAIN = 0.25


@dataclass(frozen=True)
class LaneAvoidance:
  status: str = "inactive"
  side: int = 0  # obstacle side in model coordinates (right is positive)
  strength: float = 0.0
  start: float = 0.0


@dataclass(frozen=True)
class _Obstacle:
  ids: tuple[int, int]
  distance: float
  y: float
  speed: float


def _xy(line):
  x, y = np.asarray(line.x), np.asarray(line.y)
  if len(x) < 3 or len(y) != len(x) or not np.isfinite(x).all() or not np.isfinite(y).all() or (np.diff(x) <= 0.0).any():
    return None
  return x, y


class _Geometry:
  def __init__(self, model):
    self.valid = False
    if (len(model.laneLines) < 4 or len(model.roadEdges) < 2 or
        len(model.laneLineProbs) < 3 or len(model.laneLineStds) < 3 or len(model.roadEdgeStds) < 2):
      return
    if not all(0.85 <= model.laneLineProbs[i] <= 1.0 and 0.0 <= model.laneLineStds[i] <= 0.22 for i in (1, 2)):
      return
    if not all(0.0 <= model.roadEdgeStds[i] <= 1.5 for i in (0, 1)):
      return
    model_path = _xy(model.position)
    left, right = (_xy(model.laneLines[i]) for i in (1, 2))
    edge_left, edge_right = (_xy(model.roadEdges[i]) for i in (0, 1))
    lines = (model_path, left, right, edge_left, edge_right)
    if any(line is None for line in lines):
      return
    speeds = np.asarray(model.velocity.x)
    if len(speeds) != len(model_path[0]) or not np.isfinite(speeds).all():
      return
    self.lines, self.speeds = lines, speeds
    # A distant, less certain edge is only usable if its inward margin still
    # leaves room for the model path. A noisy edge close to the path vetoes it.
    self.edge_padding = (1.5 * model.roadEdgeStds[0], 1.5 * model.roadEdgeStds[1])
    self.first = max(float(line[0][0]) for line in lines)
    self.last = min(float(line[0][-1]) for line in lines)
    self.valid = self.first <= 0.5

  def clearance(self, object_x, object_y, object_speed, side=0):
    near, far = max(self.first, 0.0), object_x + OBJECT_LENGTH + EGO_LENGTH
    if not self.valid or far <= near or far > self.last:
      return 0
    x = np.linspace(near, far, math.ceil((far - near) * 2.0) + 1)
    path, left, right, edge_left, edge_right = (np.interp(x, *line) for line in self.lines)
    width = right - left
    center = (left + right) / 2.0
    if (width.min() < 2.9 or width.max() > 4.35 or np.ptp(width) > 0.45 or
        (np.abs(np.diff(center) / np.diff(x)) > 0.25).any() or
        (path < np.maximum(left, edge_left + self.edge_padding[0]) + LANE_MARGIN).any() or
        (path > np.minimum(right, edge_right - self.edge_padding[1]) - LANE_MARGIN).any()):
      return 0
    overlap = x >= max(near, object_x - 2.0)
    if np.min(np.interp(x[overlap], self.lines[0][0], self.speeds)) < max(2.5, object_speed + 1.5):
      return 0
    if side == 0:
      center_at_object = float(np.interp(max(near, object_x), x, center))
      if abs(object_y - center_at_object) < 0.70:
        return 0
      side = 1 if object_y > center_at_object else -1
    old_clearance = float(np.min(side * (object_y - center[overlap])))
    model_clearance = float(np.min(side * (object_y - path[overlap])))
    return side if (old_clearance < OBJECT_CLEARANCE and model_clearance >= OBJECT_CLEARANCE
                    and model_clearance - old_clearance >= MIN_CLEARANCE_GAIN) else 0

  def path_at(self, x):
    return float(np.interp(x, *self.lines[0]))


def _obstacles(points, v_ego):
  stationary = [p for p in points if (p.measured and abs(p.yv_rel) <= 0.45 and
                -0.8 <= p.v_lead <= 2.0 and p.v_rel <= -2.0 and 0.5 <= p.d_rel <= v_ego * 4.0)]
  fronts = [p for p in stationary if p.source == "frontRadar" and p.radar_track_state >= 2]
  corners = [p for p in stationary if p.source.startswith("corner")]
  return [_Obstacle((f.track_id, c.track_id), min(f.d_rel, c.d_rel), (-f.y_rel - c.y_rel) / 2.0,
                    (f.v_lead + c.v_lead) / 2.0)
          for f in fronts for c in corners if (abs(f.d_rel - c.d_rel) <= 2.0 and
                                                abs(f.y_rel - c.y_rel) <= 0.40 and
                                                abs(f.v_lead - c.v_lead) <= 1.0)]


def _blocked(geometry, obstacle, points):
  # Check other measured tracks against the proposed path, including converging traffic.
  for point in points:
    if point.track_id in obstacle.ids or point.source == "scc" or not point.measured:
      continue
    if not 0.0 <= point.d_rel <= min(geometry.last, max(0.0, obstacle.distance) + OBJECT_LENGTH + EGO_LENGTH + 5.0):
      continue
    t = min(2.0, point.d_rel / max(2.0, -point.v_rel))
    y_now, y_future = -point.y_rel, -point.y_rel - point.yv_rel * t
    path_y = geometry.path_at(point.d_rel)
    if min(y_now, y_future) - OBJECT_CLEARANCE < path_y < max(y_now, y_future) + OBJECT_CLEARANCE:
      return True
  return False


class LaneAvoidanceGuard:
  @classmethod
  def for_vehicle(cls, fingerprint):
    return cls() if fingerprint in ('KIA_K8_HEV_1ST_GEN', 'KIA_EV4') else None

  def __init__(self):
    self.reset()

  def reset(self, status="inactive"):
    self.obstacle = None
    self.radar_time = self.since = self.last_seen = self.previous_time = 0.0
    self.result = LaneAvoidance(status)
    return self.result

  def update_live(self, sm, carrot, now, front_delay):
    from openpilot.selfdrive.carrot.radar_motion import CORNER_RADAR_MEASUREMENT_DELAY_S
    from openpilot.selfdrive.carrot.radar_motion.primary import snapshot_live_radar_points

    services = ("modelV2", "liveTracks", "carState", "carControl", "controlsState")
    if not all(sm.valid[s] and 0.0 <= now - sm.logMonoTime[s] * 1e-9 <= MAX_SENSOR_AGE for s in services):
      return self.reset("stale")
    md, state, tracks = sm["modelV2"], sm["carState"], sm["liveTracks"]
    if (not sm["carControl"].latActive or state.steeringPressed or state.brakePressed or state.gasPressed
        or state.leftBlinker or state.rightBlinker or not state.canValid or carrot.atc_active
        or md.meta.hardBrakePredicted or str(md.meta.desire) != "none" or str(md.meta.laneChangeState) != "off"
        or tracks.errors.canError or tracks.errors.radarFault or tracks.errors.wrongConfig
        or tracks.errors.radarUnavailableTemporary):
      return self.reset("inhibited")
    model_time = md.timestampEof * 1e-9
    radar_time = sm.logMonoTime["liveTracks"] * 1e-9
    if not (0.0 <= now - model_time <= 0.25 and abs(model_time - radar_time) <= 0.15):
      return self.reset("stale")
    points = []
    for point in tracks.points:
      source = str(point.radarSource)
      if (point.measured and (source == "frontRadar" or source.startswith("corner"))
          and all(math.isfinite(value) for value in (point.dRel, point.yRel, point.vRel, point.yvRel))):
        delay = front_delay if source == "frontRadar" else CORNER_RADAR_MEASUREMENT_DELAY_S
        points.extend(snapshot_live_radar_points((point,), state.vEgo, model_time - radar_time + delay))
    return self.update(model_time, md, points, radar_time, state.vEgo, sm["controlsState"].curvature,
                       left_occupied=state.leftBlindspot, right_occupied=state.rightBlindspot)

  def update(self, now, model, points, radar_time, v_ego, curvature, *, allowed=True,
             left_occupied=False, right_occupied=False):
    if not allowed:
      return self.reset("inhibited")
    if not (0.0 < radar_time and abs(now - radar_time) <= MAX_SENSOR_AGE and now > self.previous_time):
      return self.reset("stale")
    if not (5.0 <= v_ego <= 18.0 and abs(curvature) <= 0.012):
      return self.reset("speedOrCurve")
    geometry = _Geometry(model)
    if not geometry.valid:
      return self.reset("geometry")
    candidates = [(obj, geometry.clearance(obj.distance, obj.y, obj.speed)) for obj in _obstacles(points, v_ego)]
    candidates = [(obj, side) for obj, side in candidates if side]
    if len(candidates) > 1:
      return self.reset("ambiguous")

    elapsed = now - self.previous_time
    if candidates:
      obstacle, side = candidates[0]
      previous = self.obstacle
      continuous = (previous is not None and previous.ids == obstacle.ids and self.result.side == side and
                    0.0 < elapsed <= MAX_SAMPLE_GAP and radar_time >= self.radar_time and
                    abs(obstacle.distance - previous.distance - (previous.speed - v_ego) * elapsed) <= 1.2 and
                    abs(obstacle.y - previous.y) <= 0.35)
      if not continuous:
        self.since = radar_time
        self.result = LaneAvoidance(side=side)
      elif radar_time - self.radar_time > MAX_SAMPLE_GAP:
        self.since = radar_time
        self.result = LaneAvoidance(side=side)
      self.last_seen = now
      status = "tracked" if radar_time - self.since >= CONFIRM_TIME else "confirming"
    elif (self.obstacle is not None and self.result.strength > 0.0 and
          0.0 < elapsed <= MAX_SAMPLE_GAP and self.obstacle.distance <= 8.0 and
          all(point.track_id not in self.obstacle.ids for point in points)):
      if now - self.last_seen > 2.5 and self.result.status != "releasing":
        return self.reset("trackTimeout")
      side = self.result.side
      obstacle = _Obstacle(self.obstacle.ids, self.obstacle.distance + (self.obstacle.speed - v_ego) * elapsed,
                           self.obstacle.y, self.obstacle.speed)
      if obstacle.distance + OBJECT_LENGTH + EGO_LENGTH > 0.0:
        if geometry.clearance(obstacle.distance, obstacle.y, obstacle.speed, side) != side:
          return self.reset("geometry")
        status = "passing"
      else:
        status = "releasing"
    else:
      return self.reset()

    if (side < 0 and right_occupied) or (side > 0 and left_occupied) or _blocked(geometry, obstacle, points):
      return self.reset("traffic")
    self.obstacle, self.previous_time, self.radar_time = obstacle, now, radar_time
    strength = self.result.strength
    if status == "tracked":
      strength = min(1.0, strength + elapsed / 0.35)
    elif status == "releasing":
      strength = max(0.0, strength - elapsed / 0.75)
    self.result = LaneAvoidance(status, side, strength, obstacle.distance - 12.0)
    return self.result
