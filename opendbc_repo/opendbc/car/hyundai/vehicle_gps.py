"""DBC-selected receive-only position profile, not an OEM GNSS fix flag."""
import math
import struct

MESSAGE = 'VEHICLE_POSITION_374'
CANFD_MESSAGE = 'CANFD_VEHICLE_POSITION_206'
TIMEOUT = 2.5
CANFD_COORD_SCALE = 180.0 / (1 << 31)
CANFD_HEADING_SCALE = 360.0 / 65536.0
INVALID_INT32 = (-1, 0x7FFFFFFF, -0x80000000)


def supports_vehicle_gps(cp):
  message = cp.dbc.name_to_msg.get(MESSAGE)
  if message is None or message.address != 0x374 or message.size != 8:
    return False
  for name, bit in (('LATITUDE', 0), ('LONGITUDE', 32)):
    signal = message.sigs.get(name)
    if (signal is None or signal.start_bit != bit or signal.size != 32 or
        not signal.is_little_endian or signal.is_signed or signal.factor != 1e-6 or signal.offset != 0):
      return False
  return True


def supports_canfd_vehicle_gps(cp):
  message = cp.dbc.name_to_msg.get(CANFD_MESSAGE)
  if message is None or message.address != 0x206 or message.size != 32:
    return False
  for name, bit in (('LATITUDE', 128), ('LONGITUDE', 160)):
    signal = message.sigs.get(name)
    if (signal is None or signal.start_bit != bit or signal.size != 32 or
        not signal.is_little_endian or not signal.is_signed or
        not math.isclose(signal.factor, CANFD_COORD_SCALE, rel_tol=1e-9) or signal.offset != 0):
      return False
  heading = message.sigs.get('HEADING')
  if (heading is None or heading.start_bit != 192 or heading.size != 16 or
      not heading.is_little_endian or heading.is_signed or
      not math.isclose(heading.factor, CANFD_HEADING_SCALE, rel_tol=1e-9) or heading.offset != 0):
    return False
  return True


def register_vehicle_gps(cp):
  if supports_vehicle_gps(cp):
    cp._add_message(MESSAGE, math.nan)
  if supports_canfd_vehicle_gps(cp):
    cp._add_message(CANFD_MESSAGE, math.nan)
  return cp


def displacement(a, b):
  dy = math.radians(b[0] - a[0]) * 6371000
  dx = math.radians(b[1] - a[1]) * 6371000 * math.cos(math.radians((a[0] + b[0]) / 2))
  return math.hypot(dx, dy), math.degrees(math.atan2(dx, dy)) % 360


class VehicleGPS:
  def __init__(self):
    self.stamp = self.tick = self.heading_stamp = 0
    self.point = None
    self.runs = 0
    self.travel = self.frozen_travel = 0.
    self.heading = 0.
    self.accepted = False
    self.frozen = False
    self.active_message = None

  def _reset_tracking(self):
    self.stamp = self.heading_stamp = 0
    self.point = None
    self.runs = 0
    self.travel = self.frozen_travel = 0.
    self.heading = 0.
    self.accepted = False
    self.frozen = False

  def _decode_frame(self, message, cp):
    if message == MESSAGE:
      dat = cp.dat.get(0x374, b'')
      if len(dat) != 8:
        return None, None
      # Positive northern/eastern coordinates are the only encoding validated
      # in recordings. Do not guess signed or offset encodings elsewhere.
      point = (int.from_bytes(dat[:4], 'little') / 1e6, int.from_bytes(dat[4:8], 'little') / 1e6)
      if not (0 < point[0] <= 85.05112878 and 0 < point[1] <= 180):
        return None, None
      return point, None

    dat = cp.dat.get(0x206, b'')
    if len(dat) != 32:
      return None, None
    raw_lat, raw_lon = struct.unpack('<ii', dat[16:24])
    if raw_lat in INVALID_INT32 or raw_lon in INVALID_INT32:
      return None, None
    lat = raw_lat * CANFD_COORD_SCALE
    lon = raw_lon * CANFD_COORD_SCALE
    if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 85.05112878 and abs(lon) <= 180):
      return None, None
    if abs(lat) < 1e-4 and abs(lon) < 1e-4:
      return None, None
    raw_heading = struct.unpack('<H', dat[24:26])[0]
    native_heading = None if raw_heading == 0xFFFF else (raw_heading * CANFD_HEADING_SCALE) % 360.0
    return (lat, lon), native_heading

  def update(self, cp, speed, reverse=False):
    has_legacy = supports_vehicle_gps(cp)
    has_canfd = supports_canfd_vehicle_gps(cp)
    if not (has_legacy or has_canfd):
      return {}
    now = cp._last_update_nanos
    dt = (now - self.tick) / 1e9 if self.tick else 0.
    self.tick = now
    speed_ok = math.isfinite(speed) and 0 <= speed <= 90
    if not 0 <= dt <= TIMEOUT or not speed_ok:
      self.runs = 0
      self.accepted = False
    if speed_ok and 0 <= dt <= TIMEOUT:
      self.travel += speed * dt
      self.frozen_travel += speed * dt

    legacy_stamp = cp.ts_nanos.get(MESSAGE, {}).get('LATITUDE', 0) if has_legacy else 0
    canfd_stamp = cp.ts_nanos.get(CANFD_MESSAGE, {}).get('LATITUDE', 0) if has_canfd else 0
    legacy_fresh = legacy_stamp > 0 and 0 <= (now - legacy_stamp) / 1e9 <= TIMEOUT and not cp.bus_timeout
    canfd_fresh = canfd_stamp > 0 and 0 <= (now - canfd_stamp) / 1e9 <= TIMEOUT and not cp.bus_timeout

    if legacy_fresh:
      message, stamp, fresh, min_elapsed = MESSAGE, legacy_stamp, True, 0.5
    elif canfd_fresh:
      message, stamp, fresh, min_elapsed = CANFD_MESSAGE, canfd_stamp, True, 0.05
    else:
      message = self.active_message or (MESSAGE if has_legacy else CANFD_MESSAGE)
      stamp = legacy_stamp if message == MESSAGE else canfd_stamp
      fresh = False
      min_elapsed = 0.5 if message == MESSAGE else 0.05

    if message != self.active_message:
      self._reset_tracking()
      self.active_message = message

    if stamp != self.stamp:
      elapsed = (stamp - self.stamp) / 1e9
      self.stamp = stamp
      point, native_heading = self._decode_frame(message, cp)
      layout = point is not None
      distance, course = displacement(self.point, point) if layout and self.point else (0., 0.)
      consistent = (self.point is not None and min_elapsed <= elapsed <= TIMEOUT and
                    distance <= min(90 * elapsed, 15 + 2 * self.travel))
      if layout and fresh and speed_ok:
        self.runs = min(3, self.runs + 1) if consistent else 1
        if self.point != point:
          self.frozen_travel = 0.
        # Only a newly repeated coordinate proves a freeze. Normal travel
        # between two 1 Hz samples must not invalidate highway-speed fixes.
        self.frozen = self.frozen_travel > 15
        if native_heading is not None and not reverse:
          if consistent and self.runs >= 3:
            self.heading, self.heading_stamp = native_heading, stamp
          elif not consistent:
            self.heading_stamp = 0
        elif consistent and self.runs >= 3 and distance >= 5 and speed >= 2 and not reverse:
          self.heading, self.heading_stamp = course, stamp
        elif not consistent:
          self.heading_stamp = 0
        self.point = point
        self.accepted = self.runs >= 3
      else:
        self.point = None
        self.runs = 0
        self.heading_stamp = 0
        self.accepted = False
      self.travel = 0.
    valid = fresh and self.accepted and speed_ok and not self.frozen
    if not fresh:
      self.runs = 0
      self.accepted = False
      self.heading_stamp = 0
    heading_valid = (valid and not reverse and self.heading_stamp > 0 and
                     0 <= (now - self.heading_stamp) / 1e9 <= (30 if speed < .5 else TIMEOUT))
    return dict(available=self.stamp > 0, valid=valid, latitude=self.point[0] if self.point else 0.,
                longitude=self.point[1] if self.point else 0., sampleMonoTime=self.stamp,
                headingDeg=self.heading, headingValid=heading_valid)
