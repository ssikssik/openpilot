from types import SimpleNamespace

from openpilot.selfdrive.controls.lib import desire_helper
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, RECOGNISED_YELLOW_HOLD_NS

S = 1_000_000_000


def packet(now_ns, left=None, right=None):
  # lane_marking.parse output: (value, expires_ns) per head
  return {side: {'type': None, 'color': (color, now_ns + S // 2) if color else None}
          for side, color in (('left', left), ('right', right))}


def helper(on=True):
  h = DesireHelper()
  h.lane_type_on = on
  return h


def test_recognised_yellow_blocks_only_without_camera_colour_and_with_the_setting_on():
  h = helper()
  h.lane_marking = packet(0, left='yellow')
  assert h._recognised_yellow('left', 0, 0)            # HDA1: camera 'no info'
  assert not h._recognised_yellow('left', 11, 0)       # camera white (HDA2): the camera wins
  assert not h._recognised_yellow('right', 0, 0)       # nothing recognised on that side
  off = helper(on=False)
  off.lane_marking = packet(0, left='yellow')
  assert not off._recognised_yellow('left', 0, 0)      # CcncLaneTypeDetection off: unchanged


def test_yellow_holds_through_gaps_then_expires_and_white_clears_it():
  h = helper()
  h.lane_marking = packet(0, left='yellow')
  assert h._recognised_yellow('left', 0, 0)
  h.lane_marking = None                                # recogniser gap (e.g. blinker: only high-confidence heads)
  assert h._recognised_yellow('left', 0, RECOGNISED_YELLOW_HOLD_NS)
  assert not h._recognised_yellow('left', 0, RECOGNISED_YELLOW_HOLD_NS + 1)   # no unlimited hold
  h.lane_marking = packet(10 * S, left='yellow')
  assert h._recognised_yellow('left', 0, 10 * S)
  h.lane_marking = packet(11 * S, left='white')
  assert not h._recognised_yellow('left', 0, 11 * S)


def test_line_check_0_uses_it_and_1_keeps_type_only(monkeypatch):
  monkeypatch.setattr(desire_helper.time, 'monotonic_ns', lambda: 0)
  for check, expected in ((0, {'left': False, 'right': True}), (1, {'left': True, 'right': True})):
    h = helper()
    h.laneLineCheck = check
    h.lane_marking = packet(0, left='yellow', right='white')
    seen = {}
    for name in ('left', 'right'):
      side = getattr(h, name)
      side.update_lane_geometry = lambda *a, **k: None
      side.update_obstacles = lambda *a, **k: None
      side.compute_lane_change_available = (lambda n: lambda lane_line_info_lt_20, ignore_bsd: seen.__setitem__(n, lane_line_info_lt_20))(name)
    car = SimpleNamespace(vEgo=20.0, leftLaneLine=0, rightLaneLine=0, leftBlindspot=False, rightBlindspot=False,
                          leftLongDist=0.0, rightLongDist=0.0, leftRearLongDist=0.0, rightRearLongDist=0.0,
                          leftBlindspotOnnx=False, rightBlindspotOnnx=False)
    model = SimpleNamespace(laneLines=[None] * 4, laneLineProbs=[0.0] * 4, roadEdges=[None] * 2)
    radar = SimpleNamespace(leadLeft=None, leadRight=None, leadsLeft=[], leadsRight=[])
    try:
      h._process_sides(car, model, radar)
    except (AttributeError, TypeError):
      pass  # only the line check is under test; later steps need a full model
    assert seen == expected, check
    assert h.left.lane_line_info_raw == 0 and not h.left.lane_line_info_edge_detect  # the HUD raw and start trigger untouched


def test_blinker_keeps_a_live_hold_but_never_restores_an_expired_one():
  h = helper()
  h.lane_marking = packet(0, left='yellow')
  assert h._recognised_yellow('left', 0, 0)
  h.lane_marking = None                                # blinker on: the recogniser drops heads under 0.9 confidence
  assert all(h._recognised_yellow('left', 0, t * S, changing=True) for t in range(1, 11))
  h.lane_marking = packet(11 * S, left='white')
  assert not h._recognised_yellow('left', 0, 11 * S, changing=True)
  late = helper()
  late.lane_marking = packet(0, left='yellow')
  late._recognised_yellow('left', 0, 0)
  late.lane_marking = None
  assert not late._recognised_yellow('left', 0, 6 * S, changing=True)   # expired before the blinker: stays released


def _sides(h, monkeypatch, now_ns=0, blinker=False, right_raw=0):
  monkeypatch.setattr(desire_helper.time, 'monotonic_ns', lambda: now_ns)
  seen = {}
  for name in ('left', 'right'):
    side = getattr(h, name)
    side.update_lane_geometry = lambda *a, **k: None
    side.update_obstacles = lambda *a, **k: None
    side.compute_lane_change_available = (lambda n: lambda lane_line_info_lt_20, ignore_bsd: seen.__setitem__(n, lane_line_info_lt_20))(name)
  car = SimpleNamespace(vEgo=20.0, leftLaneLine=0, rightLaneLine=right_raw, leftBlindspot=False, rightBlindspot=False,
                        leftLongDist=0.0, rightLongDist=0.0, leftRearLongDist=0.0, rightRearLongDist=0.0,
                        leftBlindspotOnnx=False, rightBlindspotOnnx=False, leftBlinker=blinker, rightBlinker=False)
  model = SimpleNamespace(laneLines=[None] * 4, laneLineProbs=[0.0] * 4, roadEdges=[None] * 2)
  radar = SimpleNamespace(leadLeft=None, leadRight=None, leadsLeft=[], leadsRight=[])
  try:
    h._process_sides(car, model, radar)
  except (AttributeError, TypeError):
    pass
  return seen


def test_a_held_yellow_is_dropped_once_the_car_has_crossed(monkeypatch):
  h = helper()
  h.lane_marking = packet(0, left='yellow')
  assert _sides(h, monkeypatch, 0)['left'] is False
  h.lane_marking = None
  h.crossing.since = 0.0                               # crossed to the right: 'left' is now the line just crossed
  assert _sides(h, monkeypatch, S)['left'] is True


def test_waiting_with_the_blinker_stays_blocked_past_the_hold(monkeypatch):
  h = helper()
  h.lane_marking = packet(0, left='yellow')
  _sides(h, monkeypatch, 0, blinker=True)
  h.lane_marking = None
  for t in range(1, 12):
    assert _sides(h, monkeypatch, t * S, blinker=True)['left'] is False


def test_recognised_yellow_respects_overtake_admission_and_does_not_trigger_centerline_return(monkeypatch):
  h = helper()
  h.lane_marking = packet(0, left='yellow', right='yellow')
  h.autoOvertakeEnabled = h.centerlineReturnGuard = True
  h.right.lane_width = 3.0
  h.right.side_object_detected = False
  h.right.bsd_hold_counter = 0
  car = SimpleNamespace(vEgo=20., vCruise=100., rightBlindspot=False, leftLaneLine=0, rightLaneLine=0,
                        cruiseState=SimpleNamespace(enabled=True))
  carrot = SimpleNamespace(atcType='')
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=True))
  none = desire_helper.BLINKER_NONE
  h.auto_overtake_slow_timer = 15.0
  h.left.lane_change_available = _sides(h, monkeypatch)['left']
  h._update_auto_overtake(car, True, False, False, carrot, radar, False, none, None)
  assert not h.auto_overtake_in_progress

  h.centerline_wrong_way_timer = 1.5
  h._update_centerline_return_guard(car, True, False, False, carrot, False, none, None)
  assert not h.centerline_return_active  # inferred yellow is a veto, not a return trigger
  car.rightLaneLine = 20
  assert _sides(h, monkeypatch, right_raw=20)['right'] is False
  h.right.lane_width = 3.0
  h.centerline_wrong_way_timer = 1.5
  h._update_centerline_return_guard(car, True, False, False, carrot, False, none, None)
  assert h.centerline_return_active  # existing confirmed camera-yellow recovery

  h.carrot_lane_change_count = 0
  h.lane_marking = packet(S, left='white')
  h.left.lane_change_available = _sides(h, monkeypatch, S)['left']
  h._update_auto_overtake(car, True, False, False, carrot, radar, False, none, None)
  assert h.auto_overtake_in_progress
