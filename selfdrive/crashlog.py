#!/usr/bin/env python3
"""Small, bounded crash summaries independent of loggerd/rlog."""

import contextlib
import datetime
import glob
import gzip
import hashlib
import json
import os
import re
import sys
import threading
import time
import traceback
from collections import deque
from typing import Any, Dict, Iterable, Iterator, List, Optional, Set, Tuple

CRASH_LOG_DIR = os.getenv("CRASH_LOG_DIR", "/data/log/crash_summary")
CRASH_LOG_NAME = "crash_summary.jsonl"
MAX_LOG_SIZE = 128 * 1024
MAX_LOG_FILES = 5
MAX_BACKTRACE_LINES = 32
POLL_INTERVAL = 2.0

TOMBSTONE_DIR = "/data/tombstones"
DROPBOX_DIR = "/data/system/dropbox"

_PROCESS_RE = re.compile(
  r"pid:\s*(?P<pid>\d+),\s*tid:\s*(?P<tid>\d+),\s*name:\s*(?P<name>.*?)\s+>>>\s*(?P<executable>.*?)\s*<<<"
)
_SIGNAL_RE = re.compile(r"signal\s+(?P<number>\d+)\s+\((?P<name>[^)]+)\)")
_ABORT_RE = re.compile(r"Abort message:\s*'(?P<message>.*)'")
_FRAME_RE = re.compile(r"^\s*#\d+\s+")
_thread_lock = threading.Lock()


@contextlib.contextmanager
def _log_lock(directory: str) -> Iterator[None]:
  """Serialize rotation and append across processes where flock is available."""
  os.makedirs(directory, mode=0o700, exist_ok=True)
  lock_path = os.path.join(directory, ".lock")
  with _thread_lock:
    with open(lock_path, "a", encoding="utf-8") as lock_file:
      try:
        import fcntl
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
      except (ImportError, OSError):
        pass
      try:
        yield
      finally:
        try:
          import fcntl
          fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        except (ImportError, OSError):
          pass


def _log_path(directory: str, index: int = 0) -> str:
  suffix = "" if index == 0 else f".{index}"
  return os.path.join(directory, CRASH_LOG_NAME + suffix)


def _rotate_logs(directory: str, max_files: int) -> None:
  if max_files <= 1:
    try:
      os.remove(_log_path(directory))
    except FileNotFoundError:
      pass
    return

  for index in range(max_files - 1, 0, -1):
    source = _log_path(directory, index - 1)
    destination = _log_path(directory, index)
    if os.path.exists(source):
      os.replace(source, destination)


def append_crash_record(record: Dict[str, Any], directory: str = CRASH_LOG_DIR,
                        max_size: int = MAX_LOG_SIZE, max_files: int = MAX_LOG_FILES) -> str:
  """Append one JSON record while keeping a fixed upper bound on disk use."""
  bounded = dict(record)
  bounded.setdefault("timestamp", datetime.datetime.now(datetime.timezone.utc).isoformat())
  for field in ("source", "source_file", "process", "executable", "signal"):
    if field in bounded:
      bounded[field] = str(bounded[field])[:1024]
  bounded["message"] = str(bounded.get("message", ""))[:4096]
  bounded["backtrace"] = [str(line).rstrip() for line in bounded.get("backtrace", [])[:MAX_BACKTRACE_LINES]]
  data = (json.dumps(bounded, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")

  # A malformed exception must never bypass the disk bound.
  if len(data) > max_size:
    bounded["backtrace"] = bounded["backtrace"][:4]
    bounded["message"] = str(bounded.get("message", ""))[:2048]
    data = (json.dumps(bounded, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
  if len(data) > max_size:
    bounded = {
      "timestamp": bounded["timestamp"],
      "source": bounded.get("source", "unknown"),
      "process": bounded.get("process", "unknown"),
      "signal": bounded.get("signal", "UNKNOWN"),
      "message": "crash summary exceeded record limit",
      "backtrace": [],
    }
    data = (json.dumps(bounded, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
  if len(data) > max_size:
    raise ValueError("crash log max_size is too small for a minimal record")

  with _log_lock(directory):
    path = _log_path(directory)
    current_size = os.path.getsize(path) if os.path.exists(path) else 0
    if current_size and current_size + len(data) > max_size:
      _rotate_logs(directory, max_files)

    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    try:
      written = 0
      while written < len(data):
        count = os.write(fd, data[written:])
        if count <= 0:
          raise OSError("failed to append crash record")
        written += count
      os.fsync(fd)
    finally:
      os.close(fd)
  return path


def record_python_exception(process_name: Optional[str] = None, exc_info: Any = None,
                            directory: str = CRASH_LOG_DIR) -> str:
  if isinstance(exc_info, tuple) and len(exc_info) == 3:
    formatted = traceback.format_exception(*exc_info)
  else:
    formatted = traceback.format_exception(*sys.exc_info())

  lines = [line.rstrip() for part in formatted for line in part.splitlines() if line.strip()]
  message = lines[-1] if lines else "Python exception"
  process = process_name or os.getenv("MANAGER_DAEMON") or os.path.basename(sys.argv[0]) or "python"
  return append_crash_record({
    "source": "python_exception",
    "process": process,
    "signal": "PYTHON_EXCEPTION",
    "message": message,
    "backtrace": lines,
  }, directory=directory)


def parse_android_tombstone(contents: str, source_file: str = "") -> Dict[str, Any]:
  process_match = _PROCESS_RE.search(contents)
  signal_match = _SIGNAL_RE.search(contents)
  abort_match = _ABORT_RE.search(contents)

  process = process_match.group("name").strip() if process_match else "unknown"
  executable = process_match.group("executable").strip() if process_match else ""
  pid = int(process_match.group("pid")) if process_match else None
  tid = int(process_match.group("tid")) if process_match else None

  signal_name = signal_match.group("name") if signal_match else "UNKNOWN"
  signal_number = int(signal_match.group("number")) if signal_match else None
  message = abort_match.group("message") if abort_match else ""

  backtrace: List[str] = []
  in_backtrace = False
  for line in contents.splitlines():
    if line.strip() == "backtrace:":
      in_backtrace = True
      continue
    if in_backtrace and _FRAME_RE.match(line):
      backtrace.append(line.strip())
      if len(backtrace) >= MAX_BACKTRACE_LINES:
        break
    elif in_backtrace and backtrace:
      break

  return {
    "source": "android_tombstone",
    "source_file": os.path.basename(source_file),
    "process": process,
    "executable": executable,
    "pid": pid,
    "tid": tid,
    "signal": signal_name,
    "signal_number": signal_number,
    "message": message,
    "backtrace": backtrace,
  }


def _read_tombstone(path: str) -> str:
  if path.endswith(".gz"):
    with gzip.open(path, "rt", encoding="ISO-8859-1", errors="replace") as tombstone:
      return tombstone.read()
  with open(path, encoding="ISO-8859-1", errors="replace") as tombstone:
    return tombstone.read()


def _source_files(tombstone_dir: str = TOMBSTONE_DIR, dropbox_dir: str = DROPBOX_DIR) -> Iterable[str]:
  yield from glob.glob(os.path.join(tombstone_dir, "tombstone*"))
  yield from glob.glob(os.path.join(dropbox_dir, "SYSTEM_TOMBSTONE@*.txt"))
  yield from glob.glob(os.path.join(dropbox_dir, "SYSTEM_TOMBSTONE@*.txt.gz"))


def _source_key(path: str) -> Optional[Tuple[str, int, int]]:
  try:
    stat = os.stat(path)
    return path, int(stat.st_mtime_ns), int(stat.st_size)
  except OSError:
    return None


def _record_fingerprint(record: Dict[str, Any]) -> str:
  identifying = {
    "process": record.get("process"),
    "signal": record.get("signal"),
    "message": record.get("message"),
    "backtrace": record.get("backtrace", [])[:8],
  }
  return hashlib.sha256(json.dumps(identifying, sort_keys=True).encode("utf-8")).hexdigest()


def monitor_native_crashes(directory: str = CRASH_LOG_DIR, tombstone_dir: str = TOMBSTONE_DIR,
                           dropbox_dir: str = DROPBOX_DIR, poll_interval: float = POLL_INTERVAL) -> None:
  # Existing tombstones predate this boot/process start and should not be backfilled.
  known: Set[Tuple[str, int, int]] = set()
  for path in _source_files(tombstone_dir, dropbox_dir):
    key = _source_key(path)
    if key is not None:
      known.add(key)

  recent_fingerprints: deque = deque(maxlen=256)
  recent_set: Set[str] = set()

  while True:
    current: Set[Tuple[str, int, int]] = set()
    for path in _source_files(tombstone_dir, dropbox_dir):
      key = _source_key(path)
      if key is None:
        continue
      current.add(key)
      if key in known:
        continue

      try:
        record = parse_android_tombstone(_read_tombstone(path), path)
        fingerprint = _record_fingerprint(record)
        if fingerprint not in recent_set:
          append_crash_record(record, directory=directory)
          if len(recent_fingerprints) == recent_fingerprints.maxlen:
            recent_set.discard(recent_fingerprints[0])
          recent_fingerprints.append(fingerprint)
          recent_set.add(fingerprint)
        known.add(key)
      except Exception:
        print(f"failed to summarize tombstone {path}", file=sys.stderr)
        traceback.print_exc()

    known.intersection_update(current)
    time.sleep(poll_interval)


def main() -> None:
  monitor_native_crashes()


if __name__ == "__main__":
  main()
