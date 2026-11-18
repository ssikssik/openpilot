import time
import pyray as rl
from enum import IntEnum
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.sidebar import Sidebar, SIDEBAR_WIDTH
from openpilot.selfdrive.ui.layouts.home import HomeLayout
from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout, PanelType
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.onroad.robot_face import RobotFaceView
from openpilot.selfdrive.ui.carrot_param_cache import TimedSnapshotCache, read_screen_record
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.selfdrive.ui.b1_badge import draw as draw_b1_badge
from openpilot.selfdrive.ui.widgets.carrot_web_dialog import CarrotWebDialog
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.layouts.onboarding import OnboardingWindow


class MainState(IntEnum):
  HOME = 0
  SETTINGS = 1
  ONROAD = 2


class MainLayout(Widget):
  def __init__(self):
    super().__init__()

    self._sidebar = Sidebar()
    self._current_mode = MainState.HOME
    self._prev_onroad = False

    # Initialize layouts
    self._layouts = {MainState.HOME: HomeLayout(), MainState.SETTINGS: SettingsLayout(), MainState.ONROAD: AugmentedRoadView()}
    # 로봇 페이스 UI (ej): 토글이 켜지면 얼굴 화면을 보여준다 (onroad·offroad 모두)
    self._robot_face = RobotFaceView()

    self._sidebar_rect = rl.Rectangle(0, 0, 0, 0)
    self._content_rect = rl.Rectangle(0, 0, 0, 0)

    # Set callbacks
    self._setup_callbacks()

    gui_app.push_widget(self)

    # Start onboarding if terms or training not completed, make sure to push after self
    self._onboarding_window = OnboardingWindow()
    if not self._onboarding_window.completed:
      gui_app.push_widget(self._onboarding_window)
    # carrot_man
    self._last_carrot_cmd_idx = -1
    self._screen_record_param = TimedSnapshotCache(
      gui_app.is_recording(),
      lambda: read_screen_record(ui_state.params),
    )

  def _sync_screen_record_state(self, requested: bool) -> bool:
    recording = gui_app.is_recording()
    if requested != recording:
      ui_state.params.put_bool_nonblocking("ScreenRecord", recording)
      self._screen_record_param.store_pending(recording, time.monotonic())
    return recording

  def _handle_carrot_record_cmd(self, sm) -> bool:
    screen_record = self._screen_record_param.refresh(time.monotonic())
    if screen_record and ui_state.started:
      gui_app.start_recording()
    else:
      gui_app.stop_recording()
    recording = self._sync_screen_record_state(screen_record)

    try:
      cm = sm['carrotMan']
      cmd_idx = int(cm.carrotCmdIndex)
      cmd = str(cm.carrotCmd)
      arg = str(cm.carrotArg)
    except Exception as e:
      print(f"Error reading carrotMan message: {e}")
      return recording

    if cmd_idx == self._last_carrot_cmd_idx or self._last_carrot_cmd_idx == -1:
      self._last_carrot_cmd_idx = cmd_idx
      return recording
    print(f"CarrotMan command received: {cmd} {arg} (index {cmd_idx})")
    self._last_carrot_cmd_idx = cmd_idx

    if not ui_state.started:
      return recording


    if cmd != "RECORD":
      return recording

    arg = arg.upper()
    if arg == "START":
      gui_app.start_recording()
    elif arg == "STOP":
      gui_app.stop_recording()
    elif arg == "TOGGLE":
      gui_app.toggle_recording()

    return self._sync_screen_record_state(screen_record)

  def _render(self, _):
    self._handle_onroad_transition()
    if ui_state.started:
      cluster_hud_connected = ui_state.params.get_bool("ClusterHudConnected")
      self._layouts[MainState.ONROAD].set_cluster_hud_connected(
        cluster_hud_connected,
        ui_state.show_camera_with_cluster,
      )
    self._render_main_content()
    if self._current_mode in (MainState.HOME, MainState.ONROAD):
      draw_b1_badge(self._content_rect if self._sidebar.is_visible else self._rect)
    self._handle_carrot_record_cmd(ui_state.sm)

  def _setup_callbacks(self):
    self._sidebar.set_callbacks(on_settings=self._on_settings_clicked,
                                on_carrot_web=lambda: gui_app.push_widget(CarrotWebDialog()),
                                open_settings=lambda: self.open_settings(PanelType.TOGGLES))
    self._layouts[MainState.HOME]._setup_widget.set_open_settings_callback(lambda: self.open_settings(PanelType.FIREHOSE))
    self._layouts[MainState.HOME].set_settings_callback(lambda: self.open_settings(PanelType.TOGGLES))
    self._layouts[MainState.SETTINGS].set_callbacks(on_close=self._set_mode_for_state)
    self._layouts[MainState.ONROAD].set_click_callback(self._on_onroad_clicked)
    self._robot_face.set_click_callback(self._on_onroad_clicked)
    device.add_interactive_timeout_callback(self._set_mode_for_state)

  def _update_layout_rects(self):
    self._sidebar_rect = rl.Rectangle(self._rect.x, self._rect.y, SIDEBAR_WIDTH, self._rect.height)

    x_offset = SIDEBAR_WIDTH if self._sidebar.is_visible else 0
    self._content_rect = rl.Rectangle(self._rect.y + x_offset, self._rect.y, self._rect.width - x_offset, self._rect.height)

  def _handle_onroad_transition(self):
    if ui_state.started != self._prev_onroad:
      self._prev_onroad = ui_state.started

      self._set_mode_for_state()

  def _set_mode_for_state(self):
    if ui_state.started:
      # Don't hide sidebar from interactive timeout
      if self._current_mode != MainState.ONROAD:
        self._sidebar.set_visible(False)
      self._set_current_layout(MainState.ONROAD)
    else:
      self._set_current_layout(MainState.HOME)
      self._sidebar.set_visible(True)

  def _set_current_layout(self, layout: MainState):
    if layout != self._current_mode:
      self._layouts[self._current_mode].hide_event()
      self._current_mode = layout
      self._layouts[self._current_mode].show_event()

  def open_settings(self, panel_type: PanelType):
    self._layouts[MainState.SETTINGS].set_current_panel(panel_type)
    self._set_current_layout(MainState.SETTINGS)
    self._sidebar.set_visible(False)

  def _on_settings_clicked(self):
    self.open_settings(PanelType.DEVICE)

  def _on_onroad_clicked(self):
    self._sidebar.set_visible(not self._sidebar.is_visible)

  def _render_main_content(self):
    # Render sidebar
    if self._sidebar.is_visible:
      self._sidebar.render(self._sidebar_rect)

    content_rect = self._content_rect if self._sidebar.is_visible else self._rect
    # 로봇 얼굴 (ej): 설정 화면을 제외한 모든 모드에서 토글/테스트가 켜져
    # 있으면 얼굴을 보여준다. onroad(주행 중)는 물론 offroad(책상)에서도
    # 토글만으로 얼굴을 미리볼 수 있다 — 시스템 강제 onroad 없이 UI만으로
    # 동작한다. 설정 화면에서는 얼굴을 가려 토글을 끌 수 있게 한다.
    face_mode = self._robot_face.is_face_mode and self._current_mode != MainState.SETTINGS
    if face_mode:
      self._robot_face.render(content_rect)
    else:
      self._layouts[self._current_mode].render(content_rect)
