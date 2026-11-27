"""Hyundai CAN FD: zero aReq with StopReq, without automatic release/retry.

Exception: motion after a stop the SCC never confirmed as held brakes at APPROACH_ACCEL until it does.

Thresholds below are experimental, not OEM acceptance conditions. This controller
cannot guarantee stopping; ECU response must be measured on the vehicle.
"""
from dataclasses import dataclass
from enum import StrEnum


DT = 0.02  # SCC_CONTROL is transmitted at 50 Hz
ENTRY_SPEED = 0.1  # m/s; retain approach braking until nearly stopped, then request stop
STOP_SPEED = 0.05
MOVING_SPEED = 0.10
STOP_CONFIRM_TIME = 0.2
APPROACH_ACCEL = -0.5
STOP_LOWER_BAND = 0.20  # fixed during active ACC, including before StopReq; never copied from stock
# SmoothFinalStop: below FINAL_TAPER_SPEED the approach request is released toward FINAL_TAPER_ACCEL at a constant
# rate (linear in time), like a driver easing off the brake, so the wheels do not stop while ~0.35 m/s^2 is still
# applied (EV4 route 0000034a--4: aEgo -0.35 -> 0 in 0.3 s at 0.4 km/h). The ramp starts from the value last sent,
# so the hand-off from the LoC stop entry never steps, and each frame picks the rate that would reach
# FINAL_TAPER_ACCEL exactly at ENTRY_SPEED if the request were followed: when the car under-delivers and slows
# later, the release slows with it instead of finishing early (a speed-indexed taper did both: EV4 0000034b--12
# released -0.53 -> -0.38 in one step, then crawled at ~0.1 m/s^3, then stepped -0.15 -> 0 at StopReq).
FINAL_TAPER_SPEED = 0.8  # m/s (~2.9 km/h)
FINAL_TAPER_ACCEL = -0.15  # request left when StopReq zeros it (EV4 still settles at ~-0.1 with it)
FINAL_TAPER_MAX_RATE = 1.0  # m/s^3: release-rate cap; a -0.96 hold entering at 0.75 m/s needs ~0.8
FINAL_TAPER_MAX_TIME = 3.0  # s: the release never takes longer; afterwards FINAL_TAPER_ACCEL holds until StopReq
# The car may deliver only part of a small request (EV4 at crawl speed: ~40-60%), which would make the ramp front-load
# its release and then crawl. Scale the expected deceleration by the measured share, filtered over ~0.5 s.
FINAL_TAPER_TRACKING_TAU = 0.5  # s
FINAL_TAPER_TRACKING_MIN = 0.3
FINAL_TAPER_TRACKING_MAX = 1.2
FINAL_TAPER_URGENT_ACCEL = -1.0  # a stronger request (urgent stop) is never tapered


class StopPhase(StrEnum):
  idle = "idle"
  approach = "approach"
  request = "request"
  held = "held"


@dataclass(frozen=True)
class StopCommand:
  stop_req: int
  raw: float
  value: float
  lower: float


class CanfdStopping:
  def __init__(self):
    # Packet history survives episode resets; it follows the final StopReq,
    # including approach and interlock overrides.
    self.last_scc_stop_req = 0
    self.last_scc_jerk_upper = 1.0
    # StopGoLongAssist >= 1 (carcontroller sets it from the setting): brake motion after a stop the SCC never confirmed as
    # held. Off = the carrot-wip stop sequence unchanged (zero aReq once StopReq is set).
    self.unheld_brake = False
    # SmoothFinalStop (carcontroller sets it from the setting). Off = approach floor unchanged.
    self.final_taper = False
    self.taper_value = None
    self.taper_tracking = 1.0
    self.taper_min_rate = 0.0
    self.reset()

  def limit_scc_jerk_upper(self, stop_req: int, jerk_upper: float) -> float:
    """Separate an Upper increase from the first StopReq frame, not its onset."""
    if stop_req == 1 and self.last_scc_stop_req != 1:
      jerk_upper = min(jerk_upper, self.last_scc_jerk_upper)
    self.last_scc_stop_req = stop_req
    self.last_scc_jerk_upper = jerk_upper
    return jerk_upper

  def reset(self):
    self.stop_object_active = False
    self.phase = StopPhase.idle
    self.reason = "inactive"
    self.stopped_time = 0.0
    self.unheld_motion = False
    self.scc_held_seen = False
    self.taper_value = None

  def enter(self, phase: StopPhase, reason: str):
    self.phase = phase
    self.reason = reason

  def update(self, *, active: bool, requested: bool, speed: float, held: bool,
             accel: float, previous_value: float, jerk_u: float, jerk_l: float,
             smooth_lead_stop: bool = False, a_ego: float | None = None) -> StopCommand | None:
    # Caller validates sensor values and applies pedal/CAN/hold interlocks.
    if not active or not requested:
      self.reset()
      return None

    if self.phase == StopPhase.idle:
      self.enter(StopPhase.approach if speed > ENTRY_SPEED else StopPhase.request, "stop_requested")
    if self.phase == StopPhase.approach and speed <= ENTRY_SPEED:
      self.enter(StopPhase.request, "entry_speed")

    self.stopped_time = self.stopped_time + DT if speed <= STOP_SPEED else 0.0
    stopped = (held and speed <= MOVING_SPEED) or self.stopped_time >= STOP_CONFIRM_TIME
    if held:
      self.unheld_motion = False
      self.scc_held_seen = True
    if stopped:
      if self.phase != StopPhase.held:
        self.enter(StopPhase.held, "stop_observed")
    elif self.phase == StopPhase.held:
      # Motion changes diagnostics only. Never release StopReq to retry, even
      # when speed rises above the initial entry threshold after a request.
      self.enter(StopPhase.request, "motion_after_hold")
      # The SCC never confirmed a hold in this stop, so zero aReq lets the car roll (Ioniq 5 PE route 0000007c:
      # 1.55 m back on a red). With unheld_brake, brake at APPROACH_ACCEL until the SCC reports the hold; StopReq
      # stays set. After a confirmed hold, motion stays diagnostic only (unchanged).
      self.unheld_motion = self.unheld_motion or not self.scc_held_seen

    if self.unheld_brake and self.phase in (StopPhase.request, StopPhase.held) and self.unheld_motion:
      value = min(0.0, max(previous_value - jerk_l * DT, APPROACH_ACCEL))
      return StopCommand(1, APPROACH_ACCEL, value, STOP_LOWER_BAND)
    if self.phase in (StopPhase.request, StopPhase.held):
      # Match the observed Ioniq stock entry: both aReq fields become zero in
      # the FIRST StopReq frame, bypassing the ordinary aReqValue jerk ramp.
      return StopCommand(1, 0.0, 0.0, STOP_LOWER_BAND)

    # Preserve the existing approach deceleration until initial low-speed entry.
    raw = min(accel, 0.0 if smooth_lead_stop else APPROACH_ACCEL)
    if self.final_taper and not smooth_lead_stop and accel >= FINAL_TAPER_URGENT_ACCEL and speed < FINAL_TAPER_SPEED:
      if self.taper_value is None:
        self.taper_value = min(0.0, previous_value)
        self.taper_tracking = 1.0
        self.taper_min_rate = max(0.0, (FINAL_TAPER_ACCEL - self.taper_value) / FINAL_TAPER_MAX_TIME)
      if a_ego is not None and self.taper_value < -0.05:
        share = min(max(a_ego / self.taper_value, FINAL_TAPER_TRACKING_MIN), FINAL_TAPER_TRACKING_MAX)
        self.taper_tracking += (share - self.taper_tracking) * min(1.0, DT / FINAL_TAPER_TRACKING_TAU)
      if self.taper_value < FINAL_TAPER_ACCEL:
        remaining_dv = max(speed - ENTRY_SPEED, 0.0)
        # A constant-rate release from a to FINAL covers dv = T * k * |a + FINAL| / 2, k = delivered share.
        mean_decel = max(-(self.taper_value + FINAL_TAPER_ACCEL) / 2.0 * self.taper_tracking, 0.03)
        time_left = remaining_dv / mean_decel
        rate = (FINAL_TAPER_ACCEL - self.taper_value) / time_left if time_left > DT else FINAL_TAPER_MAX_RATE
        rate = min(max(rate, self.taper_min_rate), FINAL_TAPER_MAX_RATE)
        self.taper_value = min(FINAL_TAPER_ACCEL, self.taper_value + rate * DT)
      return StopCommand(0, self.taper_value, self.taper_value, STOP_LOWER_BAND)
    self.taper_value = None
    value = min(0.0, max(previous_value - jerk_l * DT, min(raw, previous_value + jerk_u * DT)))
    return StopCommand(0, raw, value, STOP_LOWER_BAND)
