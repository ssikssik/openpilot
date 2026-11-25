"""Robot face view (ej).

ej 탭의 로봇 얼굴 전환 토글(RobotFaceUI)을 켜면 차량 상태에 따라 표정이
바뀌는 로봇 얼굴을 표시한다. 주행 중에는 주행 화면(카메라·모델·HUD) 대신,
offroad(책상)에서는 홈 화면 대신 얼굴이 나온다 — 시스템 강제 onroad 없이
UI만으로 동작한다. 표정은 눈 2개의 모양·색·움직임만으로 구분한다
(입·홍조·몸통·테두리 없음 — 미리보기에서 확정한 디자인). offroad에서는
차량 신호가 없어 실시간 표정은 평소로 고정되고, 표정 테스트로는 모든
표정을 볼 수 있다.

눈은 "반짝이 눈" 스타일이다: 따뜻한 그라데이션(밝은 중심→은은한 가장자리),
윗꺼풀 그림자 띠, 주변 빛무리(글로우), 흰색 하이라이트 2개, 간헐적으로
뜨는 네 갈래 반짝이. 인게이지하면 잠깐 핸들 모양이 떴다가 민트색 집중 눈이
된다. 커브에서는 도는 방향의 반대로 얼굴이 쏠리고 현재 횡가속도를 보여준다.
인게이지 상태에서는 하단에 최소 HUD(핸들·현재 속도·목표 속도·차간거리)가
나온다. 깜빡이 때는 화살표 3개(90도 꺾쇠)가 눈 옆에서 시작해 신호 방향으로
화면 끝까지 흘러간다.

표정 우선순위 (위쪽이 이김):
  운전자 주의 > 급제동 > 끼어들기 > 후진 > 앞차 근접 > 사각지대 >
  차선 변경 ←/→ > 비상등 > 깜빡이 > 가속 페달 개입 > 급가속 > 급커브 >
  클램프 보정 > 앞차 출발 > 출발 인사 > 출발 > 주의 해제 > 디스인게이지 >
  주차 > 정체 > 신호 대기 > 인게이지 > 평소
출발 인사/출발/주의 해제/디스인게이지/앞차 출발/끼어들기는
전이 후 2~3초간만 표시되는 일시 표정이다.
앞차 근접은 레이더 앞차와의 충돌 예상 시간(TTC) 2초 미만일 때,
정체는 10km/h 미만으로 앞차를 따라갈 때, 주차는 기어가 P일 때 뜬다.
사각지대는 현대 인터페이스가 BSD 신호를 파싱하지 않아 실시간으로는
뜨지 않고 테스트로만 볼 수 있다.

RobotFaceTest 파라미터가 0이 아니면 실시간 상태와 무관하게 해당 표정을
표시한다 (표정 테스트). 토글이 꺼진 상태에서 테스트를 선택하면 10초간만
보여주고 파라미터를 0으로 되돌려 일반 주행 화면으로 복귀한다. 토글이 켜진
상태에서는 선택한 표정이 실시간을 고를 때까지 계속 유지된다.

안전 알림은 얼굴 위에 그대로 표시한다 (AlertRenderer 재사용). 급한 알림이
뜨면 얼굴보다 알림이 먼저 보인다.

화면 크기 대응: BIG_UI(2160x1080)와 mici(콤마4, 536x240) 두 화면에서
공통으로 쓴다. 모든 절대 픽셀값(폰트·선 두께·HUD 아이콘·표정 오프셋)은
REFERENCE_HEIGHT(1080) 기준값이고, _render에서 rect 높이에 비례한
스케일을 곱한다. BIG_UI에서는 스케일 1.0으로 기존과 동일하게 그려지고,
mici에서는 약 0.22배로 축소된다. 알림 렌더러는 화면별로 다르므로
생성자에서 주입받는다 (기본값은 BIG_UI AlertRenderer, mici는
mici/onroad/alert_renderer.AlertRenderer를 넘긴다).
"""

import math
import time
from dataclasses import dataclass
from enum import IntEnum

import pyray as rl

from openpilot.cereal import log
from openpilot.selfdrive.ui.carrot_param_cache import (RobotFaceParamSnapshot, TimedSnapshotCache,
                                                      read_robot_face_params, write_robot_face_test)
from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_draw import draw_text_ui_style
from openpilot.system.ui.widgets import Widget

EventName = log.OnroadEvent.EventName
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# 표정 번호 = RobotFaceTest 파라미터 값 (0은 실시간). carrot_settings.json의
# RobotFaceTest options 순서와 반드시 일치해야 한다.
class FaceState(IntEnum):
  IDLE = 1               # 평소
  ENGAGED = 2            # 인게이지
  DISENGAGED = 3         # 디스인게이지
  BLINKER_LEFT = 4       # 왼쪽 깜빡이
  BLINKER_RIGHT = 5      # 오른쪽 깜빡이
  LANE_CHANGE_LEFT = 6   # 차선 변경 ←
  LANE_CHANGE_RIGHT = 7  # 차선 변경 →
  DEPARTING = 8          # 출발
  HELLO = 9              # 출발 인사
  HARD_BRAKE = 10        # 급제동
  DM_ALERT = 11          # 운전자 주의
  DM_CLEAR = 12          # 주의 해제
  CLAMP = 13             # 클램프 보정 작동
  WAITING = 14           # 신호 대기
  REVERSE = 15           # 후진
  HARD_ACCEL = 16        # 급가속
  LEAD_CLOSE = 17        # 앞차 근접
  TRAFFIC_JAM = 18       # 정체
  PARKED = 19            # 주차
  BLIND_SPOT = 20        # 사각지대 차량
  LEAD_DEPARTED = 21     # 앞차 출발
  CUT_IN = 22            # 끼어들기
  GAS_OVERRIDE = 23      # 가속 페달 개입
  SHARP_CURVE = 24       # 급커브
  HAZARD = 25            # 비상등


FACE_NAMES = {
  FaceState.IDLE: "평소",
  FaceState.ENGAGED: "인게이지",
  FaceState.DISENGAGED: "디스인게이지",
  FaceState.BLINKER_LEFT: "왼쪽 깜빡이",
  FaceState.BLINKER_RIGHT: "오른쪽 깜빡이",
  FaceState.LANE_CHANGE_LEFT: "차선 변경 ←",
  FaceState.LANE_CHANGE_RIGHT: "차선 변경 →",
  FaceState.DEPARTING: "출발",
  FaceState.HELLO: "출발 인사",
  FaceState.HARD_BRAKE: "급제동",
  FaceState.DM_ALERT: "운전자 주의",
  FaceState.DM_CLEAR: "주의 해제",
  FaceState.CLAMP: "클램프 보정 작동",
  FaceState.WAITING: "신호 대기",
  FaceState.REVERSE: "후진",
  FaceState.HARD_ACCEL: "급가속",
  FaceState.LEAD_CLOSE: "앞차 근접",
  FaceState.TRAFFIC_JAM: "정체",
  FaceState.PARKED: "주차",
  FaceState.BLIND_SPOT: "사각지대 차량",
  FaceState.LEAD_DEPARTED: "앞차 출발",
  FaceState.CUT_IN: "끼어들기",
  FaceState.GAS_OVERRIDE: "가속 페달 개입",
  FaceState.SHARP_CURVE: "급커브",
  FaceState.HAZARD: "비상등",
}

# 운전자 주의 표정을 띄우는 이벤트 (selfdrived가 발행하는 onroadEvents 이름)
DM_EVENT_NAMES = frozenset({
  EventName.driverDistracted1, EventName.driverDistracted2, EventName.driverDistracted3,
  EventName.driverUnresponsive1, EventName.driverUnresponsive2, EventName.driverUnresponsive3,
  EventName.tooDistracted,
})

HARD_BRAKE_AEGO = -3.0        # m/s², 이보다 강한 감속이면 급제동 표정
HARD_ACCEL_AEGO = 2.0         # m/s², 이보다 강한 가속이면 급가속 표정
LEAD_CLOSE_TTC_SEC = 2.0      # 앞차와 충돌 예상 시간(TTC)이 이 미만이면 앞차 근접 표정
LEAD_CLOSE_VREL_MIN = 1.0     # m/s, 이보다 빠르게 가까워져야 앞차 근접으로 인정 (정체 크리프 제외)
LEAD_CLOSE_DREL_MAX = 50.0    # m, 앞차가 이보다 멀면 앞차 근접 계산 안 함
TRAFFIC_JAM_V_MAX = 2.78      # m/s (10km/h), 이 미만 속도로 앞차 따라가면 정체 표정
TRAFFIC_JAM_DREL_MAX = 30.0   # m, 앞차가 이 안에 있어야 정체로 인정
TRAFFIC_JAM_BLINK_PERIOD = 4.0  # 정체 상태 깜빡임 주기 (평소 2.2초보다 느리게)
SHARP_CURVE_LAT_ACCEL = 2.0   # m/s², 횡가속도가 이 이상이면 급커브 표정
CUT_IN_DREL_MAX = 25.0        # m, 앞차가 이 안에 갑자기 나타나면 끼어들기 표정
LEAD_DEPART_V_MIN = 1.5       # m/s, 정지했던 앞차가 이 속도 이상으로 움직이면 앞차 출발 표정
LEAD_DEPART_V_STOP = 0.5      # m/s, 이 미만이면 앞차가 정지한 것으로 본다
TRANSIENT_SEC = 3.0           # 출발/주의 해제/디스인게이지/출발 인사/앞차 출발 표시 시간
TRANSIENT_SHORT_SEC = 2.0     # 끼어들기/설정속도 표시 시간 (짧은 피드백)
TEST_PREVIEW_SEC = 10.0       # 토글 OFF 상태에서 표정 테스트 표시 시간
BLINK_PERIOD_SEC = 2.2        # 눈 깜빡임 주기
BLINK_DURATION_SEC = 0.12     # 눈이 감겼다 뜨는 시간
ENGAGE_FLASH_SEC = 0.8        # 인게이지 시 핸들 플래시 표시 시간
BREATHE_PERIOD_SEC = 2.5      # 호흡(미세한 커짐/작아짐) 주기
SPARKLE_COUNT = 3             # 반짝이 개수
SPARKLE_PERIOD_SEC = 3.0      # 반짝이 깜빡임 사이클
CHEVRON_SWEEP_SEC = 1.2       # 깜빡이 화살표 이동 주기 (눈 옆 → 화면 끝)
CHEVRON_COUNT = 3             # 동시에 이동하는 화살표 개수 (경로에 균등 배치)
CHEVRON_FADE = 0.1            # 이동 시작/끝에서 페이드 인/아웃하는 진행도 구간
LEAN_MAX_PX = 180.0           # 커브 쏠림 최대 이동량 (px)
LEAN_PER_LAT_ACCEL = 70.0     # 횡가속도 1 m/s²당 쏠림 (px)
MOTION_SCALE_MIN = 0.5        # 움직임 최소 스케일. 화면 높이 비례 scale을
# 그대로 쓰면 mici(0.22배)에서 쏠림·시선·떨림이 1~6px이 돼 눈에 안 보인다.
# 움직임(쏠림·오프셋·시선·반짝이·꺾쇠·속도선)은 0.5배 아래로 줄이지 않는다
# (mici: 횡가속 1m/s²당 쏠림 35px·최대 90px, BIG_UI에서는 scale 1.0이라 그대로).
LAT_ACCEL_SHOW_MIN = 0.5      # 횡가속도 표시 최소값 (m/s²)
LANE_FLOW_PER_METER = 0.18    # 인게이지 차선 흐름: 이동 1m당 점선 패턴 진행도.
# 50km/h(13.9m/s)에서 한 바퀴 0.4초, 100km/h에서 0.2초, 25km/h에서 0.8초,
# 정지(0km/h)에서는 멈춘다. 속도에 비례해 차선이 흐르는 느낌을 준다.
# (흐름 속도는 그리기 횟수에 영향을 주지 않아 메모리에 전혀 부담이 없다.)
HUD_GAP_REFRESH_SEC = 0.5     # 차간거리 파라미터 읽기 주기
HUD_SCALE_MIN = 0.6           # HUD 최소 스케일. 화면 높이에 비례한 scale을
# 그대로 쓰면 mici(0.22배)에서 속도 글자가 20px·핸들 반지름 8px로 너무
# 작아 운전 중 읽히지 않는다. HUD 요소는 0.6배 아래로 줄이지 않는다
# (mici: 속도 54px·핸들 반지름 21px·차간 막대 10.8x15.6px,
#  BIG_UI에서는 scale 1.0이라 그대로).

# 절대 픽셀값의 기준 화면 높이 (BIG_UI 1080). _render에서
# scale = rect.height / REFERENCE_HEIGHT를 계산해 모든 절대값에 곱한다.
# mici(240)에서는 약 0.22배로 축소된다. 0.1~2.0으로 클램프한다.
REFERENCE_HEIGHT = 1080.0
SCALE_MIN = 0.1
SCALE_MAX = 2.0
MS_TO_KPH = 3.6
MS_TO_MPH = 2.23694
KPH_TO_MPH = 0.621371

# 눈 색상 팔레트: (중심색, 가장자리색, 글로우색). 중심이 밝고 가장자리로
# 갈수록 은은하게 짙어지는 방사형 그라데이션을 3겹 레이어로 흉내 낸다.
EYE_PALETTES = {
  FaceState.IDLE: (rl.Color(255, 248, 235, 255), rl.Color(255, 200, 150, 255), rl.Color(255, 170, 80, 36)),
  FaceState.ENGAGED: (rl.Color(220, 255, 240, 255), rl.Color(110, 230, 170, 255), rl.Color(80, 220, 150, 36)),
  FaceState.DISENGAGED: (rl.Color(235, 235, 235, 255), rl.Color(165, 165, 165, 255), rl.Color(140, 140, 140, 36)),
  FaceState.BLINKER_LEFT: (rl.Color(255, 245, 220, 255), rl.Color(255, 180, 60, 255), rl.Color(255, 160, 40, 36)),
  FaceState.BLINKER_RIGHT: (rl.Color(255, 245, 220, 255), rl.Color(255, 180, 60, 255), rl.Color(255, 160, 40, 36)),
  FaceState.LANE_CHANGE_LEFT: (rl.Color(255, 245, 220, 255), rl.Color(255, 180, 60, 255), rl.Color(255, 160, 40, 36)),
  FaceState.LANE_CHANGE_RIGHT: (rl.Color(255, 245, 220, 255), rl.Color(255, 180, 60, 255), rl.Color(255, 160, 40, 36)),
  FaceState.DEPARTING: (rl.Color(220, 255, 240, 255), rl.Color(110, 230, 170, 255), rl.Color(80, 220, 150, 36)),
  FaceState.HELLO: (rl.Color(255, 250, 220, 255), rl.Color(255, 210, 100, 255), rl.Color(255, 190, 60, 36)),
  FaceState.HARD_BRAKE: (rl.Color(255, 255, 255, 255), rl.Color(255, 230, 230, 255), rl.Color(255, 120, 120, 40)),
  FaceState.DM_ALERT: (rl.Color(255, 220, 220, 255), rl.Color(255, 100, 100, 255), rl.Color(255, 60, 60, 40)),
  FaceState.DM_CLEAR: (rl.Color(220, 255, 240, 255), rl.Color(110, 230, 170, 255), rl.Color(80, 220, 150, 36)),
  FaceState.CLAMP: (rl.Color(220, 250, 255, 255), rl.Color(80, 210, 235, 255), rl.Color(60, 200, 230, 36)),
  FaceState.WAITING: (rl.Color(255, 248, 220, 255), rl.Color(245, 200, 80, 255), rl.Color(240, 190, 60, 36)),
  FaceState.REVERSE: (rl.Color(255, 248, 220, 255), rl.Color(245, 200, 80, 255), rl.Color(240, 190, 60, 36)),
  FaceState.HARD_ACCEL: (rl.Color(255, 240, 210, 255), rl.Color(255, 150, 60, 255), rl.Color(255, 140, 40, 36)),
  FaceState.LEAD_CLOSE: (rl.Color(255, 220, 200, 255), rl.Color(255, 110, 70, 255), rl.Color(255, 80, 50, 40)),
  FaceState.TRAFFIC_JAM: (rl.Color(210, 220, 235, 255), rl.Color(140, 160, 190, 255), rl.Color(120, 140, 170, 30)),
  FaceState.PARKED: (rl.Color(235, 225, 255, 255), rl.Color(180, 160, 230, 255), rl.Color(160, 140, 220, 30)),
  FaceState.BLIND_SPOT: (rl.Color(255, 230, 200, 255), rl.Color(255, 120, 50, 255), rl.Color(255, 100, 30, 40)),
  FaceState.LEAD_DEPARTED: (rl.Color(220, 255, 230, 255), rl.Color(100, 220, 130, 255), rl.Color(80, 200, 110, 36)),
  FaceState.CUT_IN: (rl.Color(255, 230, 230, 255), rl.Color(255, 90, 90, 255), rl.Color(255, 60, 60, 40)),
  FaceState.GAS_OVERRIDE: (rl.Color(255, 245, 220, 255), rl.Color(255, 170, 70, 255), rl.Color(255, 150, 50, 36)),
  FaceState.SHARP_CURVE: (rl.Color(220, 245, 255, 255), rl.Color(80, 180, 255, 255), rl.Color(60, 160, 255, 36)),
  FaceState.HAZARD: (rl.Color(255, 245, 220, 255), rl.Color(255, 180, 60, 255), rl.Color(255, 160, 40, 36)),
}

SHADOW_BAND_COLOR = rl.Color(120, 80, 50, 45)   # 윗꺼풀 그림자 띠
HIGHLIGHT_COLOR = rl.Color(255, 255, 255, 235)  # 큰 하이라이트
HIGHLIGHT_SMALL_COLOR = rl.Color(255, 255, 255, 130)  # 작은 하이라이트
WHEEL_FLASH_COLOR = rl.Color(110, 230, 170, 255)  # 인게이지 핸들 플래시 (밝은 민트)
LANE_LINE_COLOR = rl.Color(255, 255, 255, 90)  # 인게이지 차선 (희미한 흰색 점선)
CHEVRON_RGB = (255, 165, 0)                     # 깜빡이 꺾쇠(< >) 색 (주황).
# rl.ORANGE 같은 이름 있는 색 상수는 기기의 pyray에서 .r/.g/.b 속성이 없어
# AttributeError로 UI가 죽는다 (2026-11-18 깜빡이 켜면 콤마 로고로 바뀜).
# 잔상용 알파 색을 만들 때 이 튜플 값을 직접 쓸 것.
SPEED_LINE_COLOR = rl.Color(255, 150, 60, 180)   # 급가속 속도선 (주황, 반투명)
HUD_GREEN = rl.Color(0x16, 0x7F, 0x40, 255)      # HUD 핸들·차간 막대
HUD_TEXT = rl.Color(255, 255, 255, 230)          # HUD 글자
HUD_DIM = rl.Color(255, 255, 255, 160)           # HUD 보조 글자


@dataclass
class _Signals:
  v_ego: float = 0.0
  a_ego: float = 0.0
  standstill: bool = False
  left_blinker: bool = False
  right_blinker: bool = False
  reverse: bool = False
  parked: bool = False
  lead_close: bool = False   # 앞차 TTC 2초 미만 (레이더)
  traffic_jam: bool = False  # 10km/h 미만 + 앞차 있음 (레이더)
  engaged: bool = False
  dm_alert: bool = False
  clamp_active: bool = False
  lane_changing: bool = False
  lane_change_dir: int = 0  # -1 좌, +1 우, 0 모름
  curvature: float = 0.0    # 1/m, 양수 = 우회전 (controlsd 규약)
  lat_accel: float = 0.0    # m/s², v² × 곡률
  v_ego_display: float = 0.0  # HUD 표시용 속도 (m/s, 클러스터 우선)
  v_cruise_kph: float = 0.0   # 크루즈 목표 속도 (km/h, 0이면 미설정)
  gas_pressed: bool = False  # 운전자가 가속 페달 밟음 (carState.gasPressed)
  hazard: bool = False       # 비상등 (좌우 깜빡이 동시)
  blind_spot: bool = False   # 사각지대 차량 (carState left/rightBlindspot)
  blind_spot_dir: int = 0    # -1 좌, +1 우, 0 양쪽/모름
  sharp_curve: bool = False  # |횡가속도| 2.0 m/s² 이상
  lead_present: bool = False  # 레이더 앞차 존재 (끼어들기/앞차 출발 감지용)
  lead_d_rel: float = 0.0    # 앞차 거리 (m)
  lead_v_lead: float = 0.0   # 앞차 절대 속도 (m/s)


class RobotFaceView(Widget):
  def __init__(self, alert_renderer_cls=None):
    """alert_renderer_cls: 알림을 그릴 렌더러 클래스. None이면 BIG_UI
    AlertRenderer를 쓰고, mici 레이아웃에서는 mici AlertRenderer를 넘긴다
    (폰트·아이콘이 작은 화면용으로 따로 맞춰져 있다)."""
    super().__init__()
    self._params = TimedSnapshotCache(
      RobotFaceParamSnapshot(),
      lambda: read_robot_face_params(ui_state.params),
    )
    if alert_renderer_cls is None:
      alert_renderer_cls = AlertRenderer
    self._alert_renderer = self._child(alert_renderer_cls())
    self._face_state = FaceState.IDLE
    self._lane_change_dir = 0
    self._sig = _Signals()

    # 전이 추적. 첫 프레임에는 None으로 두고 스퓨리어스 전이를 막는다.
    self._engaged_prev: bool | None = None
    self._dm_prev: bool | None = None
    self._standstill_prev: bool | None = None
    self._depart_time = -1e9
    self._dm_clear_time = -1e9
    self._disengage_time = -1e9
    self._engage_time = -1e9
    # 새 일시 표정 전이 추적
    self._greeted = False  # 이번 주행에서 출발 인사를 했는지
    self._hello_time = -1e9
    self._lead_present_prev: bool | None = None
    self._lead_v_prev: float | None = None
    self._cut_in_time = -1e9
    self._lead_depart_time = -1e9

    # 테스트 자동 복귀 추적
    self._test_prev = 0
    self._test_since: float | None = None

    # HUD 상태
    self._gap_level = 3
    self._gap_next_refresh = 0.0
    self._v_ego_cluster_seen = False

    # 인게이지 차선 흐름 누적 (속도 감응). 0~1 사이를 순환한다.
    self._lane_flow = 0.0
    self._lane_flow_time: float | None = None

  @property
  def is_face_mode(self) -> bool:
    """얼굴 화면을 보여줄지 여부. 토글이 켜져 있거나 표정 테스트 중이면 True."""
    snap = self._params.refresh(time.monotonic())
    return snap.enabled or snap.test != 0

  def _update_state(self):
    now = time.monotonic()
    snap = self._params.refresh(now)
    self._handle_test_timeout(snap, now)
    self._refresh_gap(now)

    sig = self._read_signals()
    self._sig = sig
    self._update_transitions(sig, now)
    self._lane_change_dir = sig.lane_change_dir

    if snap.test != 0:
      try:
        self._face_state = FaceState(snap.test)
      except ValueError:
        self._face_state = FaceState.IDLE
      return
    self._face_state = self._pick_state(sig, now)

  def _refresh_gap(self, now: float) -> None:
    # 차간거리 단계: hud_renderer와 동일하게 LongitudinalPersonality + 1.
    # 0.5초마다만 읽는다 (매 프레임 파라미터 파일 읽기 금지).
    if now < self._gap_next_refresh:
      return
    self._gap_next_refresh = now + HUD_GAP_REFRESH_SEC
    try:
      personality = int(ui_state.params.get_int("LongitudinalPersonality"))
      self._gap_level = max(1, min(4, personality + 1))
    except Exception:
      pass  # 실패하면 마지막 값 유지

  def _read_signals(self) -> _Signals:
    sig = _Signals()
    sm = ui_state.sm
    try:
      cs = sm["carState"]
      sig.v_ego = float(cs.vEgo)
      sig.standstill = bool(cs.standstill)
      sig.left_blinker = bool(cs.leftBlinker)
      sig.right_blinker = bool(cs.rightBlinker)
      sig.hazard = sig.left_blinker and sig.right_blinker
      try:
        sig.gas_pressed = bool(cs.gasPressed)
      except Exception:
        pass
      # 사각지대: 현대 인터페이스가 BSD 신호를 파싱하지 않아 K8에서는
      # 항상 False다. 필드는 읽어두되 실시간으로는 안 뜨고 테스트로만 본다.
      try:
        left_bs = bool(cs.leftBlindspot)
        right_bs = bool(cs.rightBlindspot)
        sig.blind_spot = left_bs or right_bs
        if left_bs and not right_bs:
          sig.blind_spot_dir = -1
        elif right_bs and not left_bs:
          sig.blind_spot_dir = 1
      except Exception:
        pass
      # hud_renderer와 동일한 방식: cereal enum을 문자열로 바꿔 비교
      gear = str(cs.gearShifter).split(".")[-1].upper()
      sig.reverse = "REVERSE" in gear
      sig.parked = "PARK" in gear
      try:
        sig.a_ego = float(cs.aEgo)
      except Exception:
        pass
      # HUD 표시용 속도: 클러스터 속도(계기판과 동일)를 우선 사용
      try:
        v_ego_cluster = float(cs.vEgoCluster)
        self._v_ego_cluster_seen = self._v_ego_cluster_seen or v_ego_cluster != 0.0
        sig.v_ego_display = v_ego_cluster if self._v_ego_cluster_seen else sig.v_ego
      except Exception:
        sig.v_ego_display = sig.v_ego
      # 크루즈 목표 속도 (km/h): 클러스터 우선, 없으면 controlsState 구 필드
      try:
        v_cruise_cluster = float(cs.vCruiseCluster)
        if v_cruise_cluster != 0.0:
          sig.v_cruise_kph = v_cruise_cluster
      except Exception:
        pass
    except Exception:
      pass
    try:
      sig.engaged = bool(ui_state.engaged)
    except Exception:
      pass
    try:
      sig.dm_alert = any(ev.name in DM_EVENT_NAMES for ev in sm["onroadEvents"])
    except Exception:
      pass
    try:
      controls_state = sm["controlsState"]
      sig.clamp_active = bool(getattr(controls_state, "clampSteerActive", False))
      if sig.v_cruise_kph == 0.0:
        try:
          sig.v_cruise_kph = float(controls_state.deprecated.vCruise)
        except Exception:
          pass
      # 곡률: 차량 모델 경로 곡률(실제 주행 곡률) 우선
      try:
        sig.curvature = float(controls_state.curvature)
      except Exception:
        pass
    except Exception:
      pass
    if sig.curvature == 0.0:
      try:
        sig.curvature = float(sm["modelV2"].action.desiredCurvature)
      except Exception:
        pass
    # 횡가속도 = v² × 곡률. 저속에서는 v²가 작아 자연스럽게 0에 가깝다.
    try:
      if math.isfinite(sig.v_ego) and math.isfinite(sig.curvature):
        sig.lat_accel = sig.v_ego * sig.v_ego * sig.curvature
        sig.sharp_curve = abs(sig.lat_accel) >= SHARP_CURVE_LAT_ACCEL
    except Exception:
      pass
    try:
      meta = sm["modelV2"].meta
      sig.lane_changing = meta.laneChangeState != LaneChangeState.off
      direction = meta.laneChangeDirection
      if direction == LaneChangeDirection.left:
        sig.lane_change_dir = -1
      elif direction == LaneChangeDirection.right:
        sig.lane_change_dir = 1
      elif sig.left_blinker:
        sig.lane_change_dir = -1
      elif sig.right_blinker:
        sig.lane_change_dir = 1
    except Exception:
      pass
    # 앞차 (레이더 leadOne): dRel 거리, vRel 상대속도(음수 = 가까워짐),
    # vLead 앞차 절대 속도, status 앞차 존재 여부. TTC = 거리 / 접근속도.
    try:
      lead = sm["radarState"].leadOne
      d_rel = float(lead.dRel)
      v_rel = float(lead.vRel)
      present = bool(lead.status) and math.isfinite(d_rel) and 0.0 < d_rel < LEAD_CLOSE_DREL_MAX
      sig.lead_present = present
      if present:
        sig.lead_d_rel = d_rel
        try:
          sig.lead_v_lead = float(lead.vLead)
        except Exception:
          pass
      if present and v_rel < -LEAD_CLOSE_VREL_MIN:
        ttc = d_rel / -v_rel
        if math.isfinite(ttc) and ttc < LEAD_CLOSE_TTC_SEC:
          sig.lead_close = True
      # 정체: 10km/h 미만으로 앞차(30m 안)를 따라 천천히 이동 중.
      # 정지 상태는 신호 대기 표정이 맡으므로 vEgo > 0.1 조건을 둔다.
      if present and d_rel < TRAFFIC_JAM_DREL_MAX and 0.1 < sig.v_ego < TRAFFIC_JAM_V_MAX:
        sig.traffic_jam = True
    except Exception:
      pass
    return sig

  def _update_transitions(self, sig: _Signals, now: float) -> None:
    if self._engaged_prev is None:
      self._engaged_prev = sig.engaged
    elif self._engaged_prev and not sig.engaged:
      self._disengage_time = now
      self._engaged_prev = False
    elif not self._engaged_prev and sig.engaged:
      self._engage_time = now
      self._engaged_prev = True
    else:
      self._engaged_prev = sig.engaged

    if self._dm_prev is None:
      self._dm_prev = sig.dm_alert
    elif self._dm_prev and not sig.dm_alert:
      self._dm_clear_time = now
      self._dm_prev = False
    else:
      self._dm_prev = sig.dm_alert

    if self._standstill_prev is None:
      self._standstill_prev = sig.standstill
    elif self._standstill_prev and not sig.standstill:
      self._depart_time = now
      # 이번 주행 첫 출발이면 인사 표정을 먼저 보여준다
      if not self._greeted:
        self._hello_time = now
        self._greeted = True
      self._standstill_prev = False
    else:
      self._standstill_prev = sig.standstill

    # 시동이 꺼지면 다음 주행에서 다시 인사한다
    try:
      if not ui_state.started:
        self._greeted = False
    except Exception:
      pass

    # 끼어들기: 앞차가 없던 곳에 25m 안에 갑자기 나타남
    if self._lead_present_prev is None:
      self._lead_present_prev = sig.lead_present
    elif not self._lead_present_prev and sig.lead_present and sig.lead_d_rel < CUT_IN_DREL_MAX:
      self._cut_in_time = now
      self._lead_present_prev = True
    else:
      self._lead_present_prev = sig.lead_present

    # 앞차 출발: 우리가 정지한 상태에서, 정지했던 앞차가 움직이기 시작함
    if sig.lead_present:
      if (self._lead_v_prev is not None and self._lead_v_prev < LEAD_DEPART_V_STOP
          and sig.lead_v_lead >= LEAD_DEPART_V_MIN and sig.v_ego < 0.1):
        self._lead_depart_time = now
      self._lead_v_prev = sig.lead_v_lead
    else:
      self._lead_v_prev = None

  def _pick_state(self, sig: _Signals, now: float) -> FaceState:
    if not ui_state.started:
      # offroad(책상)에서는 차량 신호가 없어 실시간 표정은 평소로 고정한다.
      # (낡은 carState/modelV2 값으로 후진·신호 대기 등이 뜨는 것을 막는다.)
      # 표정 테스트는 _update_state에서 먼저 처리되므로 그대로 동작한다.
      return FaceState.IDLE
    if sig.dm_alert:
      return FaceState.DM_ALERT
    if sig.a_ego < HARD_BRAKE_AEGO:
      return FaceState.HARD_BRAKE
    if now - self._cut_in_time < TRANSIENT_SHORT_SEC:
      return FaceState.CUT_IN
    if sig.reverse:
      return FaceState.REVERSE
    if sig.lead_close:
      return FaceState.LEAD_CLOSE
    if sig.blind_spot:
      return FaceState.BLIND_SPOT
    if sig.lane_changing:
      # 방향을 알면 좌/우로 나누고, 모르면 깜빡이 방향·없으면 왼쪽으로
      if sig.lane_change_dir < 0:
        return FaceState.LANE_CHANGE_LEFT
      if sig.lane_change_dir > 0:
        return FaceState.LANE_CHANGE_RIGHT
      if sig.left_blinker:
        return FaceState.LANE_CHANGE_LEFT
      if sig.right_blinker:
        return FaceState.LANE_CHANGE_RIGHT
      return FaceState.LANE_CHANGE_LEFT
    if sig.hazard:
      return FaceState.HAZARD
    if sig.left_blinker:
      return FaceState.BLINKER_LEFT
    if sig.right_blinker:
      return FaceState.BLINKER_RIGHT
    if sig.gas_pressed and sig.engaged:
      return FaceState.GAS_OVERRIDE
    if sig.a_ego > HARD_ACCEL_AEGO:
      return FaceState.HARD_ACCEL
    if sig.sharp_curve:
      return FaceState.SHARP_CURVE
    if sig.clamp_active:
      return FaceState.CLAMP
    if now - self._lead_depart_time < TRANSIENT_SEC:
      return FaceState.LEAD_DEPARTED
    if now - self._hello_time < TRANSIENT_SEC:
      return FaceState.HELLO
    if now - self._depart_time < TRANSIENT_SEC:
      return FaceState.DEPARTING
    if now - self._dm_clear_time < TRANSIENT_SEC:
      return FaceState.DM_CLEAR
    if now - self._disengage_time < TRANSIENT_SEC:
      return FaceState.DISENGAGED
    if sig.parked:
      return FaceState.PARKED
    if sig.traffic_jam:
      return FaceState.TRAFFIC_JAM
    # 신호 대기: 인게이지 중이 아닐 때만. 크루즈로 정지(인게이지 + 정지)하면
    # WAITING으로 전환하지 않고 ENGAGED를 유지해 HUD가 계속 보인다.
    # 정지 표시는 눈 크기만 작아지는 것으로 대신한다 (_eye_transform 참조).
    if (sig.standstill or sig.v_ego < 0.1) and not sig.engaged:
      return FaceState.WAITING
    if sig.engaged:
      return FaceState.ENGAGED
    return FaceState.IDLE

  def _handle_test_timeout(self, snap: RobotFaceParamSnapshot, now: float) -> None:
    if snap.test != self._test_prev:
      self._test_prev = snap.test
      self._test_since = now if snap.test != 0 else None
    if snap.enabled or snap.test == 0:
      # 토글이 켜져 있으면 테스트가 계속 유지됨. 타이머는 돌지 않는다.
      self._test_since = None
      return
    if self._test_since is None:
      self._test_since = now
    if now - self._test_since < TEST_PREVIEW_SEC:
      return
    # 10초 경과: 파라미터를 0으로 되돌려 일반 화면으로 복귀.
    # write_robot_face_test는 미등록 키(재빌드 전)면 백킹 파일에 직접 쓴다.
    try:
      write_robot_face_test(ui_state.params, 0)
      self._params.store_pending(RobotFaceParamSnapshot(enabled=snap.enabled, test=0), now)
    except Exception:
      pass
    self._test_prev = 0
    self._test_since = None

  def _lean_offset(self, sig: _Signals, scale: float = 1.0) -> float:
    """커브 쏠림: 도는 방향의 반대로 얼굴이 미끄러지는 양 (px).
    양의 곡률 = 우회전 = 차체가 왼쪽으로 쏠림 (음의 x).
    _render에서 motion_scale(0.5 하한)을 넘겨 작은 화면에서도 쏠림이 보인다."""
    try:
      a_lat = sig.lat_accel
      if not math.isfinite(a_lat):
        return 0.0
      shift = -a_lat * LEAN_PER_LAT_ACCEL * scale
      max_px = LEAN_MAX_PX * scale
      return max(-max_px, min(max_px, shift))
    except Exception:
      return 0.0

  def _render(self, rect: rl.Rectangle):
    # started 게이트 없음: offroad(책상)에서도 얼굴을 표시한다.
    now = time.monotonic()
    state = self._face_state
    snap = self._params.value
    sig = self._sig

    # 화면 스케일: 절대 픽셀값은 1080 높이 기준이므로 rect 높이에 비례해
    # 조정한다. BIG_UI(1080)에서는 1.0, mici(240)에서는 약 0.22.
    scale = max(SCALE_MIN, min(SCALE_MAX, rect.height / REFERENCE_HEIGHT))
    # HUD 스케일: 작은 화면에서도 읽히게 0.6 아래로 줄이지 않는다.
    hud_scale = max(scale, HUD_SCALE_MIN)
    # 움직임 스케일: 작은 화면에서도 쏠림·시선·떨림이 보이게 0.5 아래로 줄이지 않는다.
    motion_scale = max(scale, MOTION_SCALE_MIN)

    # 배경: 순수 검은 화면 (테두리 없음 — 콤마 화면 자체가 프레임이다)
    rl.draw_rectangle_rec(rect, rl.BLACK)

    lean_x = self._lean_offset(sig, motion_scale)

    # 인게이지 차선: 눈 뒤에 점선 차선을 그려 도로 위 주행감을 준다.
    # 흐름은 속도에 비례한다 — 정지하면 멈추고, 빠를수록 빨리 흐른다.
    # dt가 크면(첫 프레임·일시 정지 후) 0.1초로 제한해 점선이 튀지 않게 한다.
    if self._lane_flow_time is None:
      self._lane_flow_time = now
    dt = min(max(now - self._lane_flow_time, 0.0), 0.1)
    self._lane_flow_time = now
    if state == FaceState.ENGAGED:
      self._lane_flow = (self._lane_flow + dt * max(sig.v_ego, 0.0) * LANE_FLOW_PER_METER) % 1.0
      try:
        self._draw_lane_lines(rect, lean_x, motion_scale)
      except Exception:
        pass

    # 눈 (인게이지 직후 0.8초는 핸들 플래시)
    if state == FaceState.ENGAGED and now - self._engage_time < ENGAGE_FLASH_SEC:
      self._draw_wheel_flash(rect, now, lean_x, motion_scale)
    else:
      self._draw_eyes(rect, state, now, lean_x, motion_scale)

    # 급가속 속도선
    if state == FaceState.HARD_ACCEL:
      self._draw_speed_lines(rect, now, lean_x, motion_scale)

    # 반짝이
    self._draw_sparkles(rect, state, now, lean_x, motion_scale)

    # 깜빡이 < > — 그리기 실패가 UI 전체를 죽이지 않게 가드한다
    # (2026-11-18: _draw_chevron 예외로 UI가 재시작돼 콤마 로고만 보임)
    try:
      if state == FaceState.BLINKER_LEFT:
        self._draw_chevron(rect, -1, now, lean_x, motion_scale)
      elif state == FaceState.BLINKER_RIGHT:
        self._draw_chevron(rect, 1, now, lean_x, motion_scale)
      elif state == FaceState.HAZARD:
        # 비상등: 양쪽 화살표가 반 주기 어긋나게 교대로 흐름
        self._draw_chevron(rect, -1, now, lean_x, motion_scale)
        self._draw_chevron(rect, 1, now + CHEVRON_SWEEP_SEC / 2.0, lean_x, motion_scale)
    except Exception:
      pass

    # 인게이지 HUD (핸들·현재 속도·목표 속도·차간거리)
    if state == FaceState.ENGAGED:
      try:
        self._draw_hud(rect, sig, hud_scale)
      except Exception:
        pass

    # 횡가속도 표시 (커브 중, onroad에서만). 커진 HUD와 겹치지 않게 위쪽에 표시.
    # 글자를 키우고(36→48) 위치를 조금 더 위로(0.20→0.18) 올림.
    try:
      if ui_state.started and abs(sig.lat_accel) >= LAT_ACCEL_SHOW_MIN:
        draw_text_ui_style(f"횡가속도 {abs(sig.lat_accel):.1f} m/s²",
                           rect.x + rect.width / 2.0, rect.y + rect.height * 0.18,
                           48.0 * hud_scale, HUD_DIM, align="center_bottom")
    except Exception:
      pass

    # 테스트 모드 안내 문구 (커진 HUD와 겹치지 않게 위쪽에 표시)
    if snap.test != 0:
      label = f"표정 테스트: {FACE_NAMES.get(state, '')}"
      if not snap.enabled and self._test_since is not None:
        remaining = max(0, int(math.ceil(TEST_PREVIEW_SEC - (now - self._test_since))))
        label += f" ({remaining}초 후 자동 복귀)"
      draw_text_ui_style(label, rect.x + rect.width / 2.0, rect.y + rect.height * 0.08,
                         40.0 * hud_scale, rl.WHITE, align="center_bottom")

    # 안전 알림은 얼굴 위에 그대로 표시
    self._alert_renderer.render(rect)

  def _eye_transform(self, state: FaceState, now: float, eye_w: float, eye_h: float, scale: float = 1.0):
    """(scale_x, scale_y, off_x, off_y, gaze_x)를 반환한다.
    절대 픽셀 오프셋(20/25/30/6/4/18)은 1080 기준값이므로 scale을 곱한다."""
    if state == FaceState.ENGAGED:
      # 크루즈 정지(인게이지 + 정지): WAITING으로 전환하지 않고 눈만 작게.
      # 색은 민트 그대로, HUD도 계속 보인다.
      sig = self._sig
      if sig.standstill or sig.v_ego < 0.1:
        return 1.0, 0.55, 0.0, -eye_h * 0.04, 0.0
      return 1.0, 0.72, 0.0, -eye_h * 0.04, 0.0
    if state == FaceState.DISENGAGED:
      return 1.0, 0.60, 0.0, eye_h * 0.03, 0.0
    if state == FaceState.BLINKER_LEFT:
      return 1.0, 1.0, 0.0, 0.0, -eye_w * 0.35
    if state == FaceState.BLINKER_RIGHT:
      return 1.0, 1.0, 0.0, 0.0, eye_w * 0.35
    if state == FaceState.LANE_CHANGE_LEFT:
      return 1.0, 1.0, -20.0 * scale, 0.0, -eye_w * 0.4
    if state == FaceState.LANE_CHANGE_RIGHT:
      return 1.0, 1.0, 20.0 * scale, 0.0, eye_w * 0.4
    if state == FaceState.DEPARTING:
      return 1.05, 1.05, 0.0, -30.0 * scale * abs(math.sin(2.0 * math.pi * now / 0.8)), 0.0
    if state == FaceState.HELLO:
      # 반갑게 인사: 통통 튀며 조금 커짐
      bounce = -25.0 * scale * abs(math.sin(2.0 * math.pi * now / 0.7))
      return 1.08, 1.08, 0.0, bounce, 0.0
    if state == FaceState.HARD_BRAKE:
      jx = 10.0 * scale * math.sin(2.0 * math.pi * now * 13.0)
      jy = 7.0 * scale * math.sin(2.0 * math.pi * now * 17.0 + 1.0)
      return 1.35, 1.35, jx, jy, 0.0
    if state == FaceState.DM_ALERT:
      return 1.2, 1.2, 0.0, 0.0, 25.0 * scale * math.sin(2.0 * math.pi * now / 0.7)
    if state == FaceState.DM_CLEAR:
      return 1.0, 0.70, 0.0, -eye_h * 0.03, 0.0
    if state == FaceState.CLAMP:
      nudge = 18.0 * scale * math.sin(2.0 * math.pi * now / 0.9)
      return 1.0, 1.0, nudge, 0.0, nudge * 0.5
    if state == FaceState.WAITING:
      breathe = 1.0 + 0.02 * math.sin(2.0 * math.pi * now / 2.0)
      return breathe, 0.55 * breathe, 0.0, 0.0, 0.0
    if state == FaceState.REVERSE:
      return 1.0, 0.85, 0.0, eye_h * 0.08, 0.0
    if state == FaceState.HARD_ACCEL:
      # 가로로 길게 늘어난 눈 + 살짝 아래로 처짐 (가속으로 뒤로 밀리는 느낌)
      return 1.25, 0.85, 0.0, eye_h * 0.03, 0.0
    if state == FaceState.LEAD_CLOSE:
      # 가늘게 좁힌 눈 + 미세한 떨림 (긴장)
      jx = 6.0 * scale * math.sin(2.0 * math.pi * now * 15.0)
      jy = 4.0 * scale * math.sin(2.0 * math.pi * now * 19.0 + 1.0)
      return 1.1, 0.45, jx, jy, 0.0
    if state == FaceState.TRAFFIC_JAM:
      # 반쯤 감긴 축 처진 눈 (지루함)
      return 1.0, 0.40, 0.0, eye_h * 0.05, 0.0
    if state == FaceState.PARKED:
      # 편하게 감긴 눈 + 느린 호흡 (휴식)
      breathe_y = 6.0 * scale * math.sin(2.0 * math.pi * now / 3.0)
      return 1.0, 0.10, 0.0, breathe_y, 0.0
    if state == FaceState.BLIND_SPOT:
      # 날카롭게 좁힌 눈 + 사각지대 쪽 흘끔 + 미세 떨림 (경고)
      d = self._sig.blind_spot_dir
      jx = 5.0 * scale * math.sin(2.0 * math.pi * now * 15.0)
      jy = 3.0 * scale * math.sin(2.0 * math.pi * now * 19.0 + 1.0)
      return 1.1, 0.50, jx, jy, d * eye_w * 0.45
    if state == FaceState.LEAD_DEPARTED:
      # 길이 열렸다는 반가움: 조금 커지고 위를 봄
      return 1.05, 1.05, 0.0, -eye_h * 0.05, 0.0
    if state == FaceState.CUT_IN:
      # 놀람: 확 커진 눈 + 떨림
      jx = 8.0 * scale * math.sin(2.0 * math.pi * now * 14.0)
      jy = 6.0 * scale * math.sin(2.0 * math.pi * now * 18.0 + 1.0)
      return 1.30, 1.30, jx, jy, 0.0
    if state == FaceState.GAS_OVERRIDE:
      # 페달 쪽(아래)을 흘끔 봄
      return 1.0, 0.90, 0.0, eye_h * 0.06, 0.0
    if state == FaceState.SHARP_CURVE:
      # 강하게 당겨지는 느낌: 가로로 늘어나고 세로로 눌림
      return 1.15, 0.90, 0.0, 0.0, 0.0
    if state == FaceState.HAZARD:
      return 1.0, 1.0, 0.0, 0.0, 0.0
    return 1.0, 1.0, 0.0, 0.0, 0.0  # IDLE

  def _eye_geometry(self, rect: rl.Rectangle, state: FaceState, now: float, scale: float = 1.0):
    """눈 기본 크기·위치·변환을 계산한다.
    (eye_w, eye_h, half_span, scale_x, scale_y, off_x, off_y, gaze_x)를 반환."""
    eye_h = rect.height * 0.34
    eye_w = eye_h * 0.8  # 참고 디자인 비율: 세로가 가로의 약 1.25배
    gap = eye_w * 1.0  # 두 눈 사이 간격 ≈ 눈 한 개 너비
    half_span = eye_w / 2.0 + gap / 2.0

    scale_x, scale_y, off_x, off_y, gaze_x = self._eye_transform(state, now, eye_w, eye_h, scale)

    # 호흡: 급제동·운전자 주의·앞차 근접·끼어들기·사각지대·주차를 빼고
    # 미세하게 커졌다 작아짐
    if state not in (FaceState.HARD_BRAKE, FaceState.DM_ALERT, FaceState.LEAD_CLOSE,
                     FaceState.CUT_IN, FaceState.BLIND_SPOT, FaceState.PARKED):
      breathe = 1.0 + 0.03 * math.sin(2.0 * math.pi * now / BREATHE_PERIOD_SEC)
      scale_x *= breathe
      scale_y *= breathe
      # 시선 천천히 이동 (평소·인게이지에서만)
      if state in (FaceState.IDLE, FaceState.ENGAGED):
        gaze_x += 10.0 * scale * math.sin(2.0 * math.pi * now / 7.0)

    # 깜빡임 (급제동·운전자 주의·앞차 근접·끼어들기·사각지대는 눈을 크게 뜬
    # 상태, 주차는 감은 상태라 깜빡임 없음. 정체는 4초 주기로 느리게 깜빡인다)
    if state not in (FaceState.HARD_BRAKE, FaceState.DM_ALERT, FaceState.LEAD_CLOSE,
                     FaceState.CUT_IN, FaceState.BLIND_SPOT, FaceState.PARKED):
      period = TRAFFIC_JAM_BLINK_PERIOD if state == FaceState.TRAFFIC_JAM else BLINK_PERIOD_SEC
      t = now % period
      if t > period - BLINK_DURATION_SEC:
        p = (t - (period - BLINK_DURATION_SEC)) / BLINK_DURATION_SEC
        scale_y *= 1.0 - 0.96 * math.sin(math.pi * p)

    return eye_w, eye_h, half_span, scale_x, scale_y, off_x, off_y, gaze_x

  def _draw_eyes(self, rect: rl.Rectangle, state: FaceState, now: float, lean_x: float, scale: float = 1.0) -> None:
    cx = rect.x + rect.width / 2.0 + lean_x
    cy = rect.y + rect.height / 2.0

    eye_w, eye_h, half_span, scale_x, scale_y, off_x, off_y, gaze_x = self._eye_geometry(rect, state, now, scale)

    center_color, edge_color, glow_color = EYE_PALETTES.get(state, EYE_PALETTES[FaceState.IDLE])
    # 중간색 = 중심색과 가장자리색의 평균
    mid_color = rl.Color((center_color.r + edge_color.r) // 2,
                         (center_color.g + edge_color.g) // 2,
                         (center_color.b + edge_color.b) // 2, 255)

    w = eye_w * scale_x
    h = max(eye_h * scale_y, 4.0 * scale)  # 완전히 사라지지 않게 최소 높이 유지
    base_y = cy - h / 2.0 + off_y

    for sign in (-1, 1):
      ex = cx + sign * half_span - w / 2.0 + off_x + gaze_x
      # 은은한 빛 번짐 (글로우)
      glow = rl.Rectangle(ex - w * 0.12, base_y - h * 0.08, w * 1.24, h * 1.16)
      rl.draw_rectangle_rounded(glow, 0.6, 16, glow_color)
      # 그라데이션 3겹: 가장자리 → 중간 → 중심 (밝은 중심이 은은하게 번짐)
      rl.draw_rectangle_rounded(rl.Rectangle(ex, base_y, w, h), 0.55, 16, edge_color)
      mid_w, mid_h = w * 0.82, h * 0.82
      rl.draw_rectangle_rounded(
        rl.Rectangle(ex + (w - mid_w) / 2.0, base_y + (h - mid_h) / 2.0, mid_w, mid_h),
        0.55, 16, mid_color)
      in_w, in_h = w * 0.64, h * 0.64
      rl.draw_rectangle_rounded(
        rl.Rectangle(ex + (w - in_w) / 2.0, base_y + (h - in_h) / 2.0, in_w, in_h),
        0.55, 16, center_color)
      if h < 20.0 * scale:
        continue  # 눈이 거의 감겼으면 그림자·하이라이트 생략
      # 윗꺼풀 그림자 띠 (깊이감)
      band_w, band_h = w * 0.9, h * 0.28
      rl.draw_rectangle_rounded(
        rl.Rectangle(ex + (w - band_w) / 2.0, base_y + h * 0.03, band_w, band_h),
        0.5, 12, SHADOW_BAND_COLOR)
      # 하이라이트: 큰 원 왼쪽 위 + 작은 점 오른쪽 아래 (유광 반짝임)
      rl.draw_circle(int(ex + w * 0.30), int(base_y + h * 0.28), w * 0.16, HIGHLIGHT_COLOR)
      rl.draw_circle(int(ex + w * 0.62), int(base_y + h * 0.48), w * 0.08, HIGHLIGHT_SMALL_COLOR)

  def _draw_wheel(self, cx: float, cy: float, radius: float, color: rl.Color) -> None:
    """핸들 아이콘: 두꺼운 림 + 스포크 3개 + 허브.
    draw_circle_lines는 1px 테두리라 작은 화면에서 림이 안 보여 별처럼
    보였기 때문에, 림은 바깥 원(색) + 안쪽 원(검정)으로 두꺼운 고리를
    만든다 (림 두께 = 반지름의 20%). 스포크는 반지름의 14%, 허브는 18%."""
    # 림: 바깥 원 + 안쪽 검은 원 = 두꺼운 고리
    rl.draw_circle(int(cx), int(cy), radius, color)
    rl.draw_circle(int(cx), int(cy), radius * 0.80, rl.BLACK)
    # 스포크 3개 (아래 1개 + 위 좌우 2개)
    for angle_deg in (90.0, 210.0, 330.0):
      a = math.radians(angle_deg)
      rl.draw_line_ex(
        rl.Vector2(cx, cy),
        rl.Vector2(cx + radius * 0.88 * math.cos(a), cy + radius * 0.88 * math.sin(a)),
        radius * 0.14, color)
    # 허브
    rl.draw_circle(int(cx), int(cy), radius * 0.18, color)

  def _draw_wheel_flash(self, rect: rl.Rectangle, now: float, lean_x: float, scale: float = 1.0) -> None:
    """인게이지 직후 핸들 플래시: 두 눈 자리에 큰 핸들 아이콘 + 주변 반짝이.
    0.8초 후 민트 눈 + HUD로 이어진다."""
    cx = rect.x + rect.width / 2.0 + lean_x
    cy = rect.y + rect.height / 2.0
    eye_h = rect.height * 0.34
    eye_w = eye_h * 0.8
    gap = eye_w * 1.0
    half_span = eye_w / 2.0 + gap / 2.0
    # 핸들: 눈 한 쌍이 차지하던 자리 중앙에 크게 그린다.
    self._draw_wheel(cx, cy, eye_h * 0.7, WHEEL_FLASH_COLOR)
    # 핸들 주변 작은 반짝이
    for i, (dx, dy) in enumerate(((-half_span - eye_w * 0.7, -eye_h * 0.7),
                                  (half_span + eye_w * 0.7, -eye_h * 0.6),
                                  (0.0, -eye_h * 0.9))):
      tw = 0.5 + 0.5 * math.sin(2.0 * math.pi * now * 3.0 + i * 2.1)
      self._draw_sparkle(cx + dx, cy + dy, (8.0 + 6.0 * tw) * scale,
                         rl.Color(255, 255, 255, int(200 * tw)))

  def _draw_sparkle(self, cx: float, cy: float, size: float, color: rl.Color) -> None:
    """네 갈래 별(✦): 상하좌우 4개 삼각형으로 만든 반짝이."""
    w = size * 0.22
    c = rl.Vector2(cx, cy)
    # 위·아래 뾰족점 (길게)
    rl.draw_triangle(rl.Vector2(cx, cy - size), rl.Vector2(cx - w, cy), rl.Vector2(cx + w, cy), color)
    rl.draw_triangle(rl.Vector2(cx, cy + size), rl.Vector2(cx - w, cy), rl.Vector2(cx + w, cy), color)
    # 좌·우 뾰족점 (짧게)
    rl.draw_triangle(rl.Vector2(cx - size * 0.7, cy), rl.Vector2(cx, cy - w), rl.Vector2(cx, cy + w), color)
    rl.draw_triangle(rl.Vector2(cx + size * 0.7, cy), rl.Vector2(cx, cy - w), rl.Vector2(cx, cy + w), color)
    rl.draw_circle(int(cx), int(cy), max(w * 0.8, 1.0), color)

  def _draw_speed_lines(self, rect: rl.Rectangle, now: float, lean_x: float, scale: float = 1.0) -> None:
    """급가속 속도선: 각 눈 바깥쪽에 가로 선 2개씩. 속도감을 준다."""
    cx = rect.x + rect.width / 2.0 + lean_x
    cy = rect.y + rect.height / 2.0
    eye_h = rect.height * 0.34
    eye_w = eye_h * 0.8
    gap = eye_w * 1.0
    half_span = eye_w / 2.0 + gap / 2.0
    line_len = eye_w * 0.55
    thickness = 6.0 * scale
    for sign in (-1, 1):
      eye_outer = cx + sign * (half_span + eye_w * 0.5)
      x_inner = eye_outer + sign * eye_w * 0.25
      x_outer = x_inner + sign * line_len
      for dy in (-eye_h * 0.15, eye_h * 0.15):
        rl.draw_line_ex(rl.Vector2(x_inner, cy + dy), rl.Vector2(x_outer, cy + dy),
                        thickness, SPEED_LINE_COLOR)

  def _draw_sparkles(self, rect: rl.Rectangle, state: FaceState, now: float, lean_x: float, scale: float = 1.0) -> None:
    """눈가에 드물게 떴다 사라지는 반짝이. 경고·주차 상태에서는 표시하지
    않고, 급가속(속도선이 대신함)·정체(지루한 표정)·비상등(양쪽 화살표가
    대신함)에서도 표시하지 않는다."""
    if state in (FaceState.HARD_BRAKE, FaceState.DM_ALERT,
                 FaceState.HARD_ACCEL, FaceState.LEAD_CLOSE, FaceState.TRAFFIC_JAM,
                 FaceState.PARKED, FaceState.CUT_IN, FaceState.BLIND_SPOT,
                 FaceState.HAZARD):
      return
    cx = rect.x + rect.width / 2.0 + lean_x
    cy = rect.y + rect.height / 2.0
    eye_h = rect.height * 0.34
    eye_w = eye_h * 0.8
    gap = eye_w * 1.0
    half_span = eye_w / 2.0 + gap / 2.0
    anchors = (
      (cx - half_span - eye_w * 0.55, cy - eye_h * 0.75),
      (cx + half_span + eye_w * 0.55, cy - eye_h * 0.65),
      (cx, cy - eye_h * 0.95),
    )
    for i, (ax, ay) in enumerate(anchors):
      phase = (now / SPARKLE_PERIOD_SEC + i / SPARKLE_COUNT) % 1.0
      tw = math.sin(math.pi * phase)  # 0→1→0, 사이클마다 한 번 떴다 사라짐
      if tw <= 0.05:
        continue
      size = (6.0 + 8.0 * tw) * scale
      self._draw_sparkle(ax, ay, size, rl.Color(255, 255, 255, int(220 * tw)))

  def _draw_chevron(self, rect: rl.Rectangle, direction: int, now: float, lean_x: float, scale: float = 1.0) -> None:
    """깜빡이 < >: 3개의 화살표가 눈 옆에서 시작해 신호 방향으로 화면 끝까지
    이동한다. 화살표들은 이동 경로에 균등하게 배치돼 끊기지 않고 흘러가며
    (끝에 닿으면 다시 시작점으로), 시작/끝 구간에서 페이드 인/아웃한다.
    꺾쇠 각도는 90도 — 팁 오프셋(가로)이 세로 절반과 같아 두 선이 직각으로
    만난다. 점멸 대신 이동 자체가 깜빡이 신호다."""
    cx = rect.x + rect.width / 2.0 + lean_x
    cy = rect.y + rect.height / 2.0
    eye_h = rect.height * 0.34
    eye_w = eye_h * 0.8
    gap = eye_w * 1.0
    half_span = eye_w / 2.0 + gap / 2.0
    size = eye_h * 0.35
    # 시작 위치: 눈에서 충분히 떨어진 곳 (눈 바깥 가장자리 + eye_w의 0.7배 여유)
    start_x = cx + direction * (half_span + eye_w * 1.2)
    # 끝 위치: 화면 가장자리에서 약간 안쪽 (팁이 화면을 벗어나지 않게 size/2 여유)
    margin = 24.0 * scale + size * 0.5
    end_x = rect.x + margin if direction < 0 else rect.x + rect.width - margin
    thickness = 10.0 * scale
    p = (now % CHEVRON_SWEEP_SEC) / CHEVRON_SWEEP_SEC
    for i in range(CHEVRON_COUNT):
      # 각 화살표는 주기의 1/3씩 어긋나게 배치돼 경로를 균등하게 채운다
      pp = (p + i / CHEVRON_COUNT) % 1.0
      # 시작/끝 페이드: 이동 시작과 끝에서 알파를 서서히 올리고 내림
      fade = min(pp / CHEVRON_FADE, (1.0 - pp) / CHEVRON_FADE, 1.0)
      alpha = int(255 * fade)
      if alpha <= 0:
        continue
      base_x = start_x + (end_x - start_x) * pp
      tip_x = base_x + direction * size * 0.5  # 90도: 팁 오프셋 = 세로 절반
      color = rl.Color(CHEVRON_RGB[0], CHEVRON_RGB[1], CHEVRON_RGB[2], alpha)
      rl.draw_line_ex(rl.Vector2(base_x, cy - size / 2.0), rl.Vector2(tip_x, cy), thickness, color)
      rl.draw_line_ex(rl.Vector2(tip_x, cy), rl.Vector2(base_x, cy + size / 2.0), thickness, color)

  def _draw_lane_lines(self, rect: rl.Rectangle, lean_x: float, scale: float = 1.0) -> None:
    """인게이지 차선: 눈 좌우에 원근감 있는 점선 차선 2개. 아래로 넓어지고
    점선이 아래로 흘러가 차가 앞으로 나아가는 느낌을 준다. 눈 뒤에 그린다.
    흐름 속도(_lane_flow)는 _render에서 속도에 비례해 누적한다."""
    cx = rect.x + rect.width / 2.0 + lean_x
    top_y = rect.y + rect.height * 0.15
    bottom_y = rect.y + rect.height * 0.95
    # 위쪽은 좁게(눈 옆), 아래쪽은 넓게(화면 끝 가까이) — 원근감
    top_off = rect.width * 0.28
    bottom_off = rect.width * 0.45
    thickness = 6.0 * scale
    # 점선: 8개 구간, 속도 감응 흐름(_lane_flow)에 따라 아래로 흐른다
    segments = 8
    flow = self._lane_flow
    for sign in (-1, 1):
      for i in range(segments):
        # 각 점선 조각의 시작/끝 진행도 (흐름 오프셋 적용, 끝에서 다시 위로)
        p0 = ((i / segments) + flow / segments) % 1.0
        p1 = p0 + 0.5 / segments
        if p1 > 1.0:
          continue  # 화면 아래를 벗어난 조각은 건너뜀
        x0 = cx + sign * (top_off + (bottom_off - top_off) * p0)
        x1 = cx + sign * (top_off + (bottom_off - top_off) * p1)
        y0 = top_y + (bottom_y - top_y) * p0
        y1 = top_y + (bottom_y - top_y) * p1
        rl.draw_line_ex(rl.Vector2(x0, y0), rl.Vector2(x1, y1), thickness, LANE_LINE_COLOR)

  def _draw_hud(self, rect: rl.Rectangle, sig: _Signals, scale: float = 1.0) -> None:
    """인게이지 HUD: 핸들 아이콘·현재 속도·목표 속도·차간거리 (하단).
    scale은 _render에서 계산한 hud_scale(0.5 하한)을 받는다 — 작은 화면에서도
    속도 글자·핸들·막대가 읽힐 크기로 유지된다."""
    try:
      is_metric = bool(ui_state.is_metric)
    except Exception:
      is_metric = True
    hud_y = rect.y + rect.height * 0.88
    cx = rect.x + rect.width / 2.0

    # 핸들 아이콘 (왼쪽): 두꺼운 림 + 스포크 3개 + 허브, 녹색.
    # 반지름 70으로 두 배 키움. hud_y(0.88)에 두면 하단이 잘리므로
    # 중심을 0.82로 올려 화면 안에 들어간다 (눈과는 가로로 떨어져 있음).
    wheel_x = rect.x + rect.width * 0.15
    wheel_y = rect.y + rect.height * 0.82
    self._draw_wheel(wheel_x, wheel_y, 70.0 * scale, HUD_GREEN)

    # 현재 속도 (가운데, 크게)
    speed_conv = MS_TO_KPH if is_metric else MS_TO_MPH
    speed_unit = "km/h" if is_metric else "mph"
    cur = max(0.0, sig.v_ego_display * speed_conv)
    draw_text_ui_style(f"{int(round(cur))}", cx - 60.0 * scale, hud_y + 10.0 * scale, 90.0 * scale, HUD_TEXT,
                       align="center_bottom")
    # 단위는 속도 숫자 바로 아래. 오프셋이 크면 mici 하단(240px)을 넘어 잘린다.
    draw_text_ui_style(speed_unit, cx - 60.0 * scale, hud_y + 40.0 * scale, 30.0 * scale, HUD_DIM,
                       align="center_bottom")
    # 목표 속도 (현재 속도 오른쪽, 작게). SET 라벨은 목표 숫자와 겹치지 않게
    # 위로 띄운다 (숫자가 커져서 기존 -25 오프셋이면 겹친다).
    if sig.v_cruise_kph > 0.0:
      target = sig.v_cruise_kph if is_metric else sig.v_cruise_kph * KPH_TO_MPH
      draw_text_ui_style("SET", cx + 130.0 * scale, hud_y - 45.0 * scale, 28.0 * scale, HUD_DIM,
                         align="center_bottom")
      draw_text_ui_style(f"{int(round(target))}", cx + 130.0 * scale, hud_y + 10.0 * scale, 55.0 * scale, HUD_TEXT,
                         align="center_bottom")

    # 차간거리 막대 (오른쪽): 4칸 중 채워진 칸 수 = 단계.
    # 높이만 40에서 48로 키움 (너비는 26 유지, mici 0.6배: 15.6x28.8px).
    bar_x = rect.x + rect.width * 0.84
    bar_w, bar_h, bar_gap = 26.0 * scale, 48.0 * scale, 8.0 * scale
    for i in range(4):
      x = bar_x + i * (bar_w + bar_gap)
      color = HUD_GREEN if i < self._gap_level else rl.Color(255, 255, 255, 60)
      rl.draw_rectangle_rounded(
        rl.Rectangle(x, hud_y - bar_h / 2.0, bar_w, bar_h), 0.3, 8, color)
