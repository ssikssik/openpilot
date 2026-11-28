#!/usr/bin/env python3
"""Carrot's shared CPU map renderer; model, Web and USB reuse the same image."""
import os
import time

import numpy as np

from openpilot.selfdrive.navigation.helpers import ParkedPosition
from openpilot.selfdrive.navigation.jetlink_demand import jetlink_map_demand
from openpilot.selfdrive.navigation.pmtiles import PMTiles
from openpilot.selfdrive.navigation.preview import write_preview, preview_fresh, WEB_DEMAND_PATH
from openpilot.selfdrive.navigation.renderer import SIZE, MapRenderer, Position
from openpilot.selfdrive.navigation.route import RouteTracker, route_coordinates
from openpilot.selfdrive.navigation.vehicle_navi_route import VehicleNaviRouteSynthesizer


def stock_navi_map_active(sm, route_id: int, tracker, fresh_tbt: bool, enabled: bool) -> bool:
  """True when stock-navi 0x2A3 TBT should synthesize a local Mapbox-tile route."""
  if not (enabled and tracker is None and route_id != 0 and fresh_tbt):
    return False
  if sm.alive.get('carrotNavi', False) and sm.valid.get('carrotNavi', False):
    return False
  if not sm.valid.get('carrotMan', False):
    return False
  cm = sm['carrotMan']
  straight_guidance = (
    sm.valid.get('carState', False) and
    getattr(sm['carState'], 'vehicleNaviTurnInfo', -1) == 9
  )
  if not (cm.vehicleNaviAvailable and (cm.xTurnInfo > 0 or straight_guidance)):
    return False
  if cm.remote or cm.nGoPosDist > 0 or cm.nGoPosTime > 0:
    return False
  return cm.activeCarrot not in (1, 2, 3)


def render_interval(cpu_seconds):
  # At most 10 Hz / approximately 10% of one CPU. A >40 ms render can cause
  # the model's unchanged 500 ms freshness gate to reject the map.
  return max(.1, cpu_seconds * 10)


def main():
  from openpilot.selfdrive.navigation.tiles import RemoteTiles
  from openpilot.selfdrive.navigation.mapbox_tiles import MapboxTiles
  import openpilot.cereal.messaging as messaging
  from openpilot.common.params import Params
  from msgq.visionipc import VisionIpcServer, VisionStreamType
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
  from openpilot.common.swaglog import cloudlog

  # Auxiliary work shares only the little cores. Never compete at FIFO priority
  # with camera/control, radar, driving model or DM on cores 4..7.
  if hasattr(os, 'sched_setaffinity'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    allowed = os.sched_getaffinity(0) & {0, 1, 2, 3}
    if allowed:
      os.sched_setaffinity(0, allowed)
    os.nice(15)
  sm = messaging.SubMaster(['carrotMan', 'carrotNavi', 'gpsLocation', 'gpsLocationExternal', 'navRoute', 'navInstructionCarrot', 'carState'])
  position = ParkedPosition()
  server = VisionIpcServer('mapsd')
  server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_MAP, 4, SIZE, SIZE, SIZE * SIZE, SIZE, SIZE * SIZE)
  server.start_listener()
  params = Params()
  demand_checked, demand = 0., False
  vehicle_navi_atc = False
  mapbox_token = ''
  archive_created = 0.
  diagnostics = RuntimeDiagnostics('mapsd', cloudlog.event, interval=5.)
  rk = Ratekeeper(10., print_delay_threshold=None)
  route, tracker, archive, renderer = np.empty((0, 2)), None, None, None
  synthesizer = VehicleNaviRouteSynthesizer()
  retry_at = next_render = next_preview = 0.
  was_ready = False
  route_id = 0
  image = np.zeros((SIZE, SIZE), np.uint8)
  try:
    while True:
      sm.update(0)
      now = time.monotonic()
      if sm.updated['navRoute'] and sm['navRoute'].carrotRouteId != route_id:
        route_id = sm['navRoute'].carrotRouteId
        route = route_coordinates(sm['navRoute']) if sm.valid['navRoute'] else np.empty((0, 2))
        tracker = RouteTracker(route) if route_id and 2 <= len(route) <= 4096 and np.isfinite(route).all() else None
        synthesizer.reset()
        next_render = 0.
      gps = position.update(sm)
      fresh_tbt = sm.valid['navInstructionCarrot'] and 0 <= now - sm.recv_time['navInstructionCarrot'] < 2.
      # Navigation without usable position/instruction immediately publishes black.
      if now >= demand_checked:
        demand = (params.get_bool('UsbGpuActive') or jetlink_map_demand(now=now) or
                  params.get_int('ClusterHudScreenMode') == 7 or preview_fresh(WEB_DEMAND_PATH))
        vehicle_navi_atc = params.get_bool('VehicleNaviAtcControl')
        token = os.environ.get('MAPBOX_TOKEN') or params.get('MapboxPublicKey') or ''
        if isinstance(token, bytes):
          token = token.decode('utf-8')
        if token != mapbox_token or (archive is not None and now - archive_created >= 3600):
          if archive is not None:
            archive.close()
          archive = renderer = None
          synthesizer.reset()
          retry_at = next_render = 0.
          mapbox_token = token
        demand_checked = now + 1.
      stock_navi_mode = stock_navi_map_active(sm, route_id, tracker, fresh_tbt, vehicle_navi_atc)
      if not stock_navi_mode:
        synthesizer.reset()
      ready = demand and (tracker is not None or stock_navi_mode) and gps is not None and fresh_tbt
      if now >= next_render or (was_ready and not ready):
        started, cpu_started = time.monotonic(), time.process_time()
        image.fill(0)
        valid = False
        reason = ('idle' if not demand else 'route' if (tracker is None and not stock_navi_mode)
                  else 'position' if gps is None else 'instruction' if not fresh_tbt else 'tiles')
        rendered_route_points = len(route)
        try:
          if ready and now >= retry_at:
            if renderer is None:
              archive = MapboxTiles(mapbox_token) if mapbox_token else PMTiles(RemoteTiles())
              archive_created = now
              renderer = MapRenderer(archive)
            if tracker is not None:
              pose = tracker.update_pose(gps.longitude, gps.latitude, gps.speed * (now - gps.time))
              render_route = route
            else:
              synthesized = synthesizer.synthesize(gps, sm['navInstructionCarrot'], renderer, now=started)
              if synthesized is not None:
                render_route, pose = synthesized
                rendered_route_points = len(render_route)
              else:
                render_route, pose = None, None
            # Fetch only the current view; do not prefetch a whole country/route.
            if pose is not None and render_route is not None:
              lon, lat, bearing = pose
              result = renderer.render(Position(lat, lon, bearing), render_route)
            else:
              result = None
            valid = result is not None and result.valid
            reason = ('ready' if valid else 'route_match' if (tracker is None and pose is None)
                      else 'route_position' if pose is None else 'tiles')
            if valid:
              image = result.image
        except (OSError, ValueError, RuntimeError):
          cloudlog.warning('mapsd: map unavailable; retry in 30 seconds')
          if archive is not None:
            archive.close()
          archive = renderer = None
          synthesizer.reset()
          retry_at = time.monotonic() + 30.
        finished = time.monotonic()
        if finished - started >= .5:
          image.fill(0)
          valid = False
          reason = 'slow'
        cpu = time.process_time() - cpu_started
        next_render = finished + (render_interval(cpu) if valid else max(1., render_interval(cpu)))
        # Timestamp at render START prevents a slow tile request looking fresh.
        server.send(VisionStreamType.VISION_STREAM_MAP, image.tobytes(), rk.frame,
                    route_id, int(started * 1e9))
        if finished >= next_preview:
          write_preview(image, reason=reason)
          next_preview = finished + .5
        diagnostics.record(render_ms=(finished - started) * 1000, cpu_ms=cpu * 1000,
                           context={'map_valid': valid, 'route_points': rendered_route_points, 'interval': render_interval(cpu),
                                    'parked_position_hold': position.held, 'stock_navi_route': stock_navi_mode})
      was_ready = ready
      rk.keep_time()
  finally:
    if archive is not None:
      archive.close()


if __name__ == '__main__':
  main()
