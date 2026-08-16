import datetime
import json
import os
import signal
import time


PROCESS_DIAGNOSTICS_PATH = os.getenv(
  "PROCESS_DIAGNOSTICS_PATH", "/data/log/process_diagnostics.jsonl"
)
ABORT_PROCESS_LOG_DIR = os.getenv("ABORT_PROCESS_LOG_DIR", "/data/log")
KST = datetime.timezone(datetime.timedelta(hours=9), name="KST")


SIGNAL_NAMES = {
  2: "SIGINT",
  4: "SIGILL",
  6: "SIGABRT",
  7: "SIGBUS",
  8: "SIGFPE",
  9: "SIGKILL",
  11: "SIGSEGV",
  13: "SIGPIPE",
  15: "SIGTERM",
}
SIGNAL_REASON_KO = {
  2: "SIGINT 종료 요청을 받았습니다.",
  4: "지원되지 않거나 손상된 CPU 명령을 실행했습니다.",
  6: "프로세스가 내부 오류를 감지해 abort를 호출했습니다.",
  7: "잘못된 메모리 정렬 또는 접근으로 버스 오류가 발생했습니다.",
  8: "0으로 나누기 등 잘못된 산술 연산이 발생했습니다.",
  9: "강제 종료되었습니다. 메모리 부족(OOM), watchdog 또는 외부 kill 가능성이 있습니다.",
  11: "잘못된 메모리 주소에 접근해 세그멘테이션 오류가 발생했습니다.",
  13: "닫힌 파이프 또는 소켓에 쓰기를 시도했습니다.",
  15: "SIGTERM 종료 요청을 받았습니다.",
}


def describe_process_exit(exit_code):
  """Return a human-readable Korean explanation for a process exit code."""
  if exit_code is None:
    return "종료 코드가 확인되지 않았습니다. 갑작스러운 프로세스 소실 또는 상태 수집 지연 가능성이 있습니다."

  if exit_code < 0:
    signum = -int(exit_code)
    try:
      signal_name = signal.Signals(signum).name
    except ValueError:
      signal_name = SIGNAL_NAMES.get(signum, "UNKNOWN_SIGNAL")
    detail = SIGNAL_REASON_KO.get(
      signum, "운영체제 시그널에 의해 프로세스가 종료되었습니다."
    )
    return f"시그널 {signum}({signal_name})로 종료됨: {detail}"

  if exit_code == 0:
    return "종료 코드 0으로 끝났지만 운행 중 필수 프로세스이므로 비정상 종료로 기록했습니다."

  return (
    f"종료 코드 {int(exit_code)}로 종료됨: 처리되지 않은 예외, 초기화 실패 또는 "
    "프로세스 내부 오류 가능성이 있습니다. 상세 traceback은 process_diagnostics.jsonl을 확인하세요."
  )


def _append_bytes(path, payload):
  os.makedirs(os.path.dirname(path), exist_ok=True)
  fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
  try:
    remaining = memoryview(payload)
    while remaining:
      written = os.write(fd, remaining)
      if written <= 0:
        raise OSError("failed to append process diagnostic")
      remaining = remaining[written:]
    os.fsync(fd)
  finally:
    os.close(fd)


def append_abort_process_log(process_name, pid, exit_code, reason=None,
                             log_dir=None, now=None):
  """Append one unexpected onroad process exit to a daily KST text log."""
  now = now or datetime.datetime.now(datetime.timezone.utc)
  if now.tzinfo is None:
    now = now.replace(tzinfo=datetime.timezone.utc)
  now_kst = now.astimezone(KST)
  path = os.path.join(
    log_dir or ABORT_PROCESS_LOG_DIR,
    f"abort_process_{now_kst:%Y%m%d}.log",
  )
  explanation = (reason or describe_process_exit(exit_code)).replace("\r", " ").replace("\n", " ")
  line = (
    f"[{now_kst:%Y-%m-%d %H:%M:%S} KST] "
    f"process={process_name} pid={int(pid)} exit_code={exit_code} "
    f"reason={explanation}\n"
  ).encode("utf-8")

  try:
    _append_bytes(path, line)
    return True
  except Exception:
    return False


def append_process_diagnostic(event_type, **fields):
  """Persist one process/communication diagnostic as a single JSON line."""
  record = {
    "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "mono_time": time.monotonic(),
    "event_type": event_type,
  }
  record.update(fields)

  path = PROCESS_DIAGNOSTICS_PATH
  try:
    line = (json.dumps(record, ensure_ascii=False, separators=(",", ":"), default=str) + "\n").encode("utf-8")
    _append_bytes(path, line)
    return True
  except Exception:
    return False
