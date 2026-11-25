import os
import tempfile
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar


CARROT_PARAM_REFRESH_INTERVAL = 0.5
PARAM_READ_RETRY_INTERVAL = 0.05
PENDING_WRITE_TIMEOUT = 1.0

T = TypeVar("T")
_NO_PENDING_WRITE = object()


class TimedSnapshotCache(Generic[T]):
  """Keep the last complete snapshot and retry failed reads with bounded backoff."""

  def __init__(self, initial: T, loader: Callable[[], T], interval: float = CARROT_PARAM_REFRESH_INTERVAL):
    self._value = initial
    self._loader = loader
    self._interval = interval
    self._next_refresh_time = 0.0
    self._read_failures = 0
    self._pending_write: object = _NO_PENDING_WRITE
    self._pending_write_deadline = 0.0

  @property
  def value(self) -> T:
    return self._value

  @property
  def next_refresh_time(self) -> float:
    return self._next_refresh_time

  def refresh(self, now: float) -> T:
    if now < self._next_refresh_time:
      return self._value

    try:
      value = self._loader()
    except Exception:
      # Retry once on the next 20 Hz UI frame, then back off to the normal TTL
      # so a persistent Params failure cannot restore per-frame I/O indefinitely.
      retry_delay = min(self._interval, PARAM_READ_RETRY_INTERVAL * (2 ** min(self._read_failures, 4)))
      self._read_failures += 1
      self._next_refresh_time = now + retry_delay
      return self._value

    self._read_failures = 0
    if self._pending_write is not _NO_PENDING_WRITE:
      if value != self._pending_write and now < self._pending_write_deadline:
        # A nonblocking write has not reached the Params file yet. Keep the
        # local source of truth and check for acknowledgement on the next TTL.
        self._next_refresh_time = now + self._interval
        return self._value
      self._pending_write = _NO_PENDING_WRITE

    self._value = value
    self._next_refresh_time = now + self._interval
    return value

  def store_pending(self, value: T, now: float) -> None:
    """Keep a nonblocking write authoritative until observed or its timeout."""
    self._pending_write = value
    self._pending_write_deadline = now + PENDING_WRITE_TIMEOUT
    self._read_failures = 0
    self._value = value
    self._next_refresh_time = now + self._interval


@dataclass(frozen=True)
class BorderParamSnapshot:
  top_left: str = ""
  top_left_op_long: str = ""
  custom_sr: float = 0.0
  bottom_left: str = ""
  bottom_right: str = ""


@dataclass(frozen=True)
class RealtimeUiParamSnapshot:
  record_audio: bool = False
  is_metric: bool = False
  always_on_dm: bool = False


@dataclass(frozen=True)
class RobotFaceParamSnapshot:
  enabled: bool = False
  test: int = 0


def read_border_params(params, params_memory) -> BorderParamSnapshot:
  car_name = params.get("CarName") or ""
  hyundai_camera_scc = params.get_int("HyundaiCameraSCC") > 0
  nnff_model_name = params.get("NNFFModelName") or ""
  custom_sr = params.get_float("CustomSR") / 10.0
  git_branch = params.get("GitBranch") or ""
  network_address = params_memory.get("NetworkAddress") or ""

  top_left = car_name + ("(CAMERA SCC)" if hyundai_camera_scc else "")
  top_left_op_long = top_left if hyundai_camera_scc else car_name + " - OP Long"
  if nnff_model_name:
    top_left += ",NNFF"
    top_left_op_long += ",NNFF"

  return BorderParamSnapshot(
    top_left=top_left,
    top_left_op_long=top_left_op_long,
    custom_sr=custom_sr,
    bottom_left=git_branch,
    bottom_right=network_address,
  )


def read_realtime_ui_params(params) -> RealtimeUiParamSnapshot:
  return RealtimeUiParamSnapshot(
    record_audio=params.get_bool("RecordAudio"),
    is_metric=params.get_bool("IsMetric"),
    always_on_dm=params.get_bool("AlwaysOnDM"),
  )


def read_screen_record(params) -> bool:
  return params.get_bool("ScreenRecord")


def _param_file_path(params, key: str) -> str:
  """미등록 키의 백킹 파일 경로. 서버 services/params.py의
  _unregistered_param_paths와 동일한 검증 (경로 이탈 방지)."""
  if not isinstance(key, str) or not key or not key.isascii() or any(not (c.isalnum() or c == "_") for c in key):
    raise ValueError(f"invalid param name: {key}")
  param_dir = os.path.abspath(params.get_param_path())
  param_path = os.path.abspath(params.get_param_path(key))
  if os.path.normcase(os.path.dirname(param_path)) != os.path.normcase(param_dir):
    raise ValueError(f"invalid param path: {key}")
  return param_path


def _read_param_file(params, key: str) -> str | None:
  """백킹 파일을 직접 읽는다. 없거나 읽을 수 없으면 None."""
  try:
    with open(_param_file_path(params, key), "rb") as f:
      return f.read().decode("utf-8", errors="replace")
  except Exception:
    return None


def read_robot_face_params(params) -> RobotFaceParamSnapshot:
  # 등록된 키는 typed getter로 읽고, 미등록 키(재빌드 전)는 백킹 파일을
  # 직접 읽는다. 웹 서버는 미등록 키도 파일에 직접 쓰기 때문에(services/
  # params.py의 _put_unregistered_setting), 파일 폴백이 없으면 토글을
  # 켜도 UI가 무반응이다. 서버의 _read_param_value와 동일한 폴백이다.
  try:
    enabled = params.get_bool("RobotFaceUI")
  except Exception:
    raw = _read_param_file(params, "RobotFaceUI")
    enabled = raw is not None and raw.strip().lower() in ("1", "true", "on", "yes")
  try:
    test = int(params.get_int("RobotFaceTest"))
  except Exception:
    raw = _read_param_file(params, "RobotFaceTest")
    try:
      test = round(float(raw)) if raw is not None else 0
    except Exception:
      test = 0
  return RobotFaceParamSnapshot(enabled=bool(enabled), test=min(max(test, 0), 25))


def write_robot_face_test(params, value: int) -> None:
  """RobotFaceTest를 쓴다 (표정 테스트 자동 복귀용). 등록된 키는
  put_int_nonblocking으로, 미등록 키(재빌드 전)는 백킹 파일에 직접 쓴다.
  서버의 _put_unregistered_setting과 동일한 원자적 쓰기 (임시 파일 +
  os.replace + fsync). 실패해도 예외를 던지지 않는다."""
  try:
    params.put_int_nonblocking("RobotFaceTest", value)
    return
  except Exception:
    pass
  try:
    param_path = _param_file_path(params, "RobotFaceTest")
    param_dir = os.path.dirname(param_path)
    # 임시 파일은 params 디렉토리 바깥에 둔다 (clear_all이 커밋 중인
    # 임시 파일을 지우지 못하게 — 서버와 동일한 이유).
    fd, temp_path = tempfile.mkstemp(prefix=".tmp_value_", dir=os.path.dirname(param_dir))
    try:
      with os.fdopen(fd, "wb") as f:
        fd = -1
        f.write(str(int(value)).encode("utf-8"))
        f.flush()
        os.fsync(f.fileno())
      os.replace(temp_path, param_path)
      temp_path = ""
      try:
        dir_fd = os.open(param_dir, os.O_RDONLY)
      except OSError:
        dir_fd = -1
      if dir_fd >= 0:
        try:
          os.fsync(dir_fd)
        finally:
          os.close(dir_fd)
    finally:
      if fd >= 0:
        os.close(fd)
      if temp_path:
        try:
          os.unlink(temp_path)
        except FileNotFoundError:
          pass
  except Exception:
    pass
