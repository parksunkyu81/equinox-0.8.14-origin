import datetime
import json
import os
import time


PROCESS_DIAGNOSTICS_PATH = os.getenv(
  "PROCESS_DIAGNOSTICS_PATH", "/data/log/process_diagnostics.jsonl"
)


class DiagnosticRateLimiter:
  """Aggregate noisy diagnostics in memory and emit bounded records."""

  def __init__(self, persistent_seconds=0.3, summary_seconds=60.0, max_signatures=16):
    self.persistent_seconds = persistent_seconds
    self.summary_seconds = summary_seconds
    self.max_signatures = max_signatures
    self.active = False
    self.active_started = 0.0
    self.active_persistent = False
    self.active_signature = ""
    self.window_started = None
    self._reset_window()

  def _reset_window(self):
    self.occurrences = 0
    self.failed_samples = 0
    self.persistent_occurrences = 0
    self.max_duration = 0.0
    self.signatures = {}

  def _count_signature(self, signature):
    signature = signature or "unspecified"
    if signature not in self.signatures and len(self.signatures) >= self.max_signatures:
      signature = "other"
    self.signatures[signature] = self.signatures.get(signature, 0) + 1

  def update(self, now, active, signature=""):
    events = []
    if self.window_started is None:
      self.window_started = now

    if active:
      if not self.active:
        self.active = True
        self.active_started = now
        self.active_persistent = False
        self.occurrences += 1
      self.active_signature = signature or "unspecified"
      self.failed_samples += 1
      self._count_signature(self.active_signature)
      duration = max(0.0, now - self.active_started)
      self.max_duration = max(self.max_duration, duration)
      if duration >= self.persistent_seconds and not self.active_persistent:
        self.active_persistent = True
        self.persistent_occurrences += 1
        events.append({
          "kind": "persistent",
          "duration": duration,
          "signature": self.active_signature,
        })
    elif self.active:
      duration = max(0.0, now - self.active_started)
      self.max_duration = max(self.max_duration, duration)
      if self.active_persistent:
        events.append({
          "kind": "recovered",
          "duration": duration,
          "signature": self.active_signature,
        })
      self.active = False
      self.active_persistent = False
      self.active_signature = ""

    if now - self.window_started >= self.summary_seconds:
      if self.occurrences:
        events.append({
          "kind": "summary",
          "window_seconds": max(0.0, now - self.window_started),
          "occurrences": self.occurrences,
          "failed_samples": self.failed_samples,
          "persistent_occurrences": self.persistent_occurrences,
          "max_duration": self.max_duration,
          "active": self.active,
          "patterns": [
            {"signature": key, "samples": value}
            for key, value in sorted(self.signatures.items(), key=lambda item: item[1], reverse=True)
          ],
        })
      self.window_started = now
      self._reset_window()
      # A fault spanning the summary boundary must remain visible in the next
      # summary without being counted as a new transition.
      if self.active:
        self.occurrences = 1
        self.persistent_occurrences = int(self.active_persistent)

    return events


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
