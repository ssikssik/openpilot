import importlib
import signal
import sys
import types
from dataclasses import dataclass

import pytest

from openpilot.selfdrive.pandad import panda_helpers


@dataclass
class FakePanda:
  serial: str
  hw_type: bytes
  internal: bool
  closed: bool = False

  def get_usb_serial(self) -> str:
    return self.serial

  def get_type(self) -> bytes:
    return self.hw_type

  def is_internal(self) -> bool:
    return self.internal

  def close(self) -> None:
    self.closed = True


def test_connect_all_pandas_sorts_c3_internal_first():
  pandas = {
    "red": FakePanda("red", b"\x07", False),
    "dos": FakePanda("dos", b"\x06", True),
  }
  connected = panda_helpers.connect_all_pandas(["red", "dos"], lambda serial: pandas[serial])

  assert [p.serial for p in connected] == ["dos", "red"]
  assert panda_helpers.pandas_include_internal(connected)


@pytest.mark.parametrize("hw_type", [b"\x09", b"\x0a"])
def test_connect_all_pandas_preserves_single_internal(hw_type):
  panda = FakePanda("internal", hw_type, True)
  connected = panda_helpers.connect_all_pandas(["internal"], lambda _serial: panda)

  assert connected == [panda]
  assert panda_helpers.pandas_include_internal(connected)


def test_connect_all_pandas_closes_partial_connection():
  panda = FakePanda("dos", b"\x06", True)

  def fake_flash(serial):
    if serial == "red":
      raise RuntimeError("connect failed")
    return panda

  with pytest.raises(RuntimeError, match="connect failed"):
    panda_helpers.connect_all_pandas(["dos", "red"], fake_flash)

  assert panda.closed


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_pandad_wrapper_forwards_exit_signals_to_child(monkeypatch, sig):
  if "usb1" not in sys.modules:
    try:
      importlib.import_module("usb1")
    except ModuleNotFoundError:
      fake_usb1 = types.ModuleType("usb1")
      fake_usb1.USBErrorNoDevice = type("USBErrorNoDevice", (Exception,), {})
      fake_usb1.USBErrorPipe = type("USBErrorPipe", (Exception,), {})
      monkeypatch.setitem(sys.modules, "usb1", fake_usb1)
      fake_panda_mod = types.ModuleType("panda")
      fake_panda_mod.Panda = type("Panda", (), {"list": staticmethod(lambda: [])})
      fake_panda_mod.PandaDFU = type("PandaDFU", (), {"list": staticmethod(lambda: [])})
      fake_panda_mod.PandaProtocolMismatch = type("PandaProtocolMismatch", (Exception,), {})
      fake_panda_mod.FW_PATH = ""
      monkeypatch.setitem(sys.modules, "panda", fake_panda_mod)

  from openpilot.selfdrive.pandad import pandad as pandad_wrapper

  registered = {}
  sent_signals = []

  class FakeProcess:
    def poll(self):
      return None

    def send_signal(self, s):
      sent_signals.append(s)

    def wait(self):
      # Trigger exit signal while child pandad is running to verify relay-safe SIGINT forwarding.
      registered[sig](sig, None)
      return 0

  fake_panda = FakePanda("internal", b"\x09", True)
  fake_panda.get_signature = lambda: b"sig"
  fake_panda.health = lambda: {"heartbeat_lost": False, "som_reset_triggered": False}
  fake_panda.reset = lambda reconnect=True: None

  monkeypatch.setattr(pandad_wrapper.signal, "signal", lambda s, h: registered.__setitem__(s, h))
  monkeypatch.setattr(pandad_wrapper.PandaDFU, "list", staticmethod(lambda: []))
  monkeypatch.setattr(pandad_wrapper.Panda, "list", staticmethod(lambda: ["internal"]))
  monkeypatch.setattr(pandad_wrapper, "flash_all_pandas", lambda serials, params: [fake_panda])
  monkeypatch.setattr(pandad_wrapper.HARDWARE, "has_internal_panda", lambda: True)
  monkeypatch.setattr(pandad_wrapper, "configure_boardd_firmware_check", lambda params: None)
  monkeypatch.setattr(pandad_wrapper, "Params", lambda: type("P", (), {"remove": lambda *a: None, "put": lambda *a: None, "put_bool": lambda *a: None})())
  monkeypatch.setattr(pandad_wrapper.subprocess, "Popen", lambda *a, **kw: FakeProcess())

  pandad_wrapper.main()

  assert {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}.issubset(registered.keys())
  assert sent_signals == [signal.SIGINT]
