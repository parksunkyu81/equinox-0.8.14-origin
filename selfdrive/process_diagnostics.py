import datetime
import json
import os
import time


PROCESS_DIAGNOSTICS_PATH = os.getenv(
  "PROCESS_DIAGNOSTICS_PATH", "/data/log/process_diagnostics.jsonl"
)


def append_process_diagnostic(event_type, sync=False, **fields):
  """Persist one process/communication diagnostic as a single JSON line."""
  record = {
    "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "mono_time": time.monotonic(),
    "event_type": event_type,
  }
  record.update(fields)

  path = PROCESS_DIAGNOSTICS_PATH
  try:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    line = (json.dumps(record, ensure_ascii=False, separators=(",", ":"), default=str) + "\n").encode("utf-8")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
      remaining = memoryview(line)
      while remaining:
        written = os.write(fd, remaining)
        if written <= 0:
          raise OSError("failed to append process diagnostic")
        remaining = remaining[written:]
      if sync:
        os.fsync(fd)
    finally:
      os.close(fd)
    return True
  except Exception:
    return False
