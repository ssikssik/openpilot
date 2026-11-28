#!/usr/bin/env python3
"""EV4 FR_CMR (0x7C4) & ADAS_DRV (0x730) UDS DTC read/clear diagnostic test.

Run with openpilot/pandad stopped while parked (P or D stationary, IGN ON / READY):
  Read-only check:
    python3 selfdrive/debug/car/ev4_dtc_test.py
  Read -> ClearDTC (0x14 FF FF FF) -> Re-read check:
    python3 selfdrive/debug/car/ev4_dtc_test.py --clear
"""
import argparse
import sys
import time
from subprocess import CalledProcessError, check_output

from opendbc.car.carlog import carlog
from opendbc.car.structs import CarParams
from opendbc.car.uds import (
  DTC_GROUP_TYPE,
  DTC_REPORT_TYPE,
  DTC_STATUS_MASK_TYPE,
  MessageTimeoutError,
  NegativeResponseError,
  SESSION_TYPE,
  UdsClient,
  get_dtc_num_as_str,
  get_dtc_status_names,
)
from panda import Panda

ECUS = [
  ("FR_CMR (전방카메라)", 0x7C4, 0x7CC),
  ("ADAS_DRV (99810-EZ100)", 0x730, 0x738),
]

FAULT_162_FIELDS = (
  ("FSS", 213), ("FCA", 216), ("LSS", 219), ("SLA", 222),
  ("DAW", 225), ("HBA", 228), ("SCC", 231), ("LFA", 234),
  ("HDA", 237), ("LCA", 240), ("HDP", 243), ("DAS", 246), ("ESS", 249),
)


def decode_162_faults(dat: bytes) -> dict[str, int]:
  val = int.from_bytes(dat, "little")
  return {name: (val >> bit) & 0x7 for name, bit in FAULT_162_FIELDS if (val >> bit) & 0x7}


def sample_live_can(panda: Panda, duration_s: float = 0.6) -> tuple[dict[str, int] | None, int | None, int]:
  """Sample live 0x162 fault bits and 0x0EA MDPS byte18 on bus 0/2."""
  faults_162 = None
  mdps_b18 = None
  total_frames = 0
  t_end = time.monotonic() + duration_s
  while time.monotonic() < t_end:
    msgs = panda.can_recv()
    total_frames += len(msgs)
    for addr, dat, src in msgs:
      if addr == 0x162 and src in (0, 2) and len(dat) >= 32:
        faults_162 = decode_162_faults(dat)
      elif addr == 0x0EA and src in (0, 2) and len(dat) >= 19:
        mdps_b18 = dat[18]
    time.sleep(0.01)
  return faults_162, mdps_b18, total_frames


def parse_dtcs(data: bytes) -> list[tuple[str, str, str]]:
  out = []
  if not data:
    return out
  for i in range(1, len(data) - 3, 4):
    raw = data[i:i + 3]
    status = data[i + 3]
    dtc_str = get_dtc_num_as_str(raw)
    flags = " ".join(get_dtc_status_names(status)) or f"0x{status:02x}"
    out.append((dtc_str, raw.hex(), f"0x{status:02x} ({flags})"))
  return out


def read_ecu_dtcs(panda: Panda, name: str, tx_addr: int, rx_addr: int, bus: int, timeout: float) -> list[tuple[str, str, str]] | None:
  client = UdsClient(panda, tx_addr, rx_addr, bus=bus, timeout=timeout)
  try:
    data = client.read_dtc_information(DTC_REPORT_TYPE.DTC_BY_STATUS_MASK, DTC_STATUS_MASK_TYPE.ALL)
  except (MessageTimeoutError, NegativeResponseError) as e:
    print(f"  [{name}] 0x{tx_addr:03x} (bus {bus}) READ FAILED: {type(e).__name__}: {e}")
    return None
  dtcs = parse_dtcs(data)
  avail = f"0x{data[0]:02x}" if data else "none"
  print(f"  [{name}] 0x{tx_addr:03x}->0x{rx_addr:03x} (bus {bus}) avail={avail} count={len(dtcs)}")
  for dtc_str, raw_hex, status_str in dtcs:
    print(f"      - {dtc_str} (raw={raw_hex}) status={status_str}")
  if not dtcs:
    print("      - (DTC 없음)")
  return dtcs


def clear_ecu_dtcs(panda: Panda, name: str, tx_addr: int, rx_addr: int, bus: int, timeout: float) -> bool:
  client = UdsClient(panda, tx_addr, rx_addr, bus=bus, timeout=timeout)
  print(f"  [{name}] 0x{tx_addr:03x} (bus {bus}): trying 0x14 FF FF FF (Default session)...")
  try:
    client.clear_diagnostic_information(DTC_GROUP_TYPE.ALL)
    print(f"  [{name}] OK (0x54) in Default session!")
    return True
  except NegativeResponseError as e:
    print(f"  [{name}] Default session NRC: {e} (nrc=0x{e.error_code:02x})")
  except MessageTimeoutError as e:
    print(f"  [{name}] Default session timeout: {e}")
    return False

  print(f"  [{name}] 0x{tx_addr:03x} (bus {bus}): retrying with Extended session (0x10 03) -> 0x14 FF FF FF...")
  try:
    client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
    client.clear_diagnostic_information(DTC_GROUP_TYPE.ALL)
    print(f"  [{name}] OK (0x54) in Extended session!")
    return True
  except (NegativeResponseError, MessageTimeoutError) as e:
    print(f"  [{name}] Extended session FAILED: {type(e).__name__}: {e}")
    return False


def ensure_pandad_stopped() -> None:
  try:
    check_output(["pidof", "pandad"])
    print("ERROR: pandad가 실행 중입니다. 먼저 openpilot을 잠시 중지한 뒤 실행하세요.")
    print("  중지: tmux kill-session -t comma   (또는 systemctl stop comma)")
    print("  복구: 테스트 후 재부팅 (sudo reboot)")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def main() -> None:
  parser = argparse.ArgumentParser(description="EV4 FR_CMR (0x7C4) & ADAS_DRV (0x730) UDS DTC test")
  parser.add_argument("--clear", action="store_true", help="Send UDS ClearDiagnosticInformation (0x14 FF FF FF)")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus (default: 0; in elm327 relay bypass bus 0 & 2 are bridged)")
  parser.add_argument("--timeout", type=float, default=1.0, help="UDS response timeout in seconds")
  parser.add_argument("--debug", action="store_true", help="Enable UDS debug logging")
  args = parser.parse_args()

  if args.debug:
    carlog.setLevel("DEBUG")

  ensure_pandad_stopped()

  panda = Panda()
  try:
    # param=1 keeps bus 1 on ACAN (disables OBD-II mux) while relay remains safely bypassed.
    panda.set_safety_mode(CarParams.SafetyModel.elm327, 1)
    panda.can_clear(0xFFFF)

    print("=== [1] 현재 CAN 상태 샘플링 ===")
    f162, b18, cnt = sample_live_can(panda, 0.6)
    print(f"  rx_frames={cnt} | 0x162 faults={f162} | MDPS 0x0EA b18={hex(b18) if b18 is not None else None}")
    if cnt == 0:
      print("WARNING: CAN 프레임이 수신되지 않습니다. 차량 시동(IG-ON/READY)이 켜져 있는지 확인하세요.")

    print("\n=== [2] Clear 전 DTC 조회 (0x19 02 FF) ===")
    for name, tx, rx in ECUS:
      read_ecu_dtcs(panda, name, tx, rx, args.bus, args.timeout)

    if not args.clear:
      print("\n(읽기 전용 모드였습니다. 삭제 테스트를 하려면 --clear 옵션을 붙여 실행하세요.)")
      return

    print("\n=== [3] DTC 삭제 전송 (0x14 FF FF FF) ===")
    for name, tx, rx in ECUS:
      clear_ecu_dtcs(panda, name, tx, rx, args.bus, args.timeout)

    time.sleep(0.5)
    panda.can_clear(0xFFFF)

    print("\n=== [4] Clear 후 DTC 재조회 (0x19 02 FF) ===")
    for name, tx, rx in ECUS:
      read_ecu_dtcs(panda, name, tx, rx, args.bus, args.timeout)

    print("\n=== [5] Clear 후 CAN 0x162 상태 재확인 (2초) ===")
    f162_after, b18_after, cnt_after = sample_live_can(panda, 2.0)
    print(f"  rx_frames={cnt_after} | 0x162 faults={f162_after} | MDPS 0x0EA b18={hex(b18_after) if b18_after is not None else None}")
    print("\n완료! 이제 시동을 껐다가 다시 켠 뒤 경고등이 뜨는지 확인해 주세요.")
  finally:
    try:
      panda.set_safety_mode(CarParams.SafetyModel.noOutput, 0)
    except Exception:
      pass
    try:
      panda.close()
    except Exception:
      pass


if __name__ == "__main__":
  main()
