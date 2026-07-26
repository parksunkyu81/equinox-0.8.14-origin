import csv
import datetime
import glob
import os
import queue
import threading
import time


GM_LKAS_LOG_DIR = os.environ.get("GM_LKAS_LOG_DIR", "/data/log/gm_lkas")
GM_LKAS_LOG_MAX_BYTES = 5 * 1024 * 1024
GM_LKAS_LOG_MAX_FILES = 6
GM_LKAS_LOG_QUEUE_SIZE = 4096
GM_LKAS_LOG_FLUSH_INTERVAL_S = 0.25


def gm_lkas_checksum(active, torque, counter):
  return (0x1000 - (int(bool(active)) << 11) - (int(torque) & 0x7ff) - (int(counter) % 4)) & 0xfff


class GMSteeringDiagnosticLogger:
  """Non-blocking rotating CSV logger for GM 0x180 steering diagnostics."""

  FIELDNAMES = [
    "utc_time", "mono_time_s", "frame",
    "command_due", "command_sent", "command_block_reason",
    "command_counter", "command_torque", "command_active", "command_checksum", "command_dat_hex",
    "loopback_count", "loopback_counters", "loopback_torques",
    "loopback_actives", "loopback_checksums", "loopback_checksum_valid",
    "loopback_latest_counter", "loopback_changed", "loopback_acked",
    "send_interval_ms", "time_since_send_ms", "deadline_lag_ms",
    "gap_fault", "unacked_fault",
    "pscm_lkas_status", "pscm_torque_delivered", "pscm_driver_torque",
    "steer_fault_temporary", "steer_fault_permanent", "can_valid",
    "queue_drops",
  ]

  def __init__(self, log_dir=None, max_bytes=GM_LKAS_LOG_MAX_BYTES,
               max_files=GM_LKAS_LOG_MAX_FILES, queue_size=GM_LKAS_LOG_QUEUE_SIZE):
    self.log_dir = log_dir or GM_LKAS_LOG_DIR
    self.max_bytes = max(int(max_bytes), 1024)
    self.max_files = max(int(max_files), 1)
    self.queue = queue.Queue(maxsize=max(int(queue_size), 1))
    self.queue_drops = 0
    self.last_error = None
    self.disabled = False
    self._stop_token = object()
    self._closed = False
    self._file = None
    self._writer = None
    self._file_sequence = 0
    self._last_flush = 0.0
    self._thread = threading.Thread(target=self._run, name="gm_lkas_diag", daemon=True)
    self._thread.start()

  def log(self, **values):
    if self._closed or self.disabled:
      return
    values["queue_drops"] = self.queue_drops
    try:
      self.queue.put_nowait(values)
    except queue.Full:
      self.queue_drops += 1

  def close(self, timeout=2.0):
    if self._closed:
      return
    self._closed = True
    try:
      self.queue.put(self._stop_token, timeout=max(float(timeout), 0.0))
    except queue.Full:
      pass
    self._thread.join(timeout=max(float(timeout), 0.0))

  def _normalize(self, values):
    wall_time = float(values.get("wall_time_s", time.time()))
    utc_time = datetime.datetime.fromtimestamp(wall_time, datetime.timezone.utc)
    row = {name: values.get(name, "") for name in self.FIELDNAMES}
    row["utc_time"] = utc_time.isoformat(timespec="milliseconds").replace("+00:00", "Z")

    loopbacks = values.get("loopbacks", ())
    row["loopback_count"] = len(loopbacks)
    row["loopback_counters"] = "|".join(str(int(v[0]) % 4) for v in loopbacks)
    row["loopback_torques"] = "|".join(str(int(v[1])) for v in loopbacks)
    row["loopback_actives"] = "|".join(str(int(bool(v[2]))) for v in loopbacks)
    row["loopback_checksums"] = "|".join(str(int(v[3]) & 0xfff) for v in loopbacks)
    row["loopback_checksum_valid"] = "|".join(
      str(int((int(v[3]) & 0xfff) == gm_lkas_checksum(v[2], v[1], v[0])))
      for v in loopbacks
    )
    return row

  def _prune_logs(self):
    pattern = os.path.join(self.log_dir, "gm_lkas_*.csv")
    files = sorted((p for p in glob.glob(pattern) if os.path.isfile(p)),
                   key=lambda p: (os.path.getmtime(p), p))
    while len(files) >= self.max_files:
      os.remove(files.pop(0))

  def _open_log(self):
    os.makedirs(self.log_dir, exist_ok=True)
    self._prune_logs()
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    filename = "gm_lkas_{}_{}_{}.csv".format(now, os.getpid(), self._file_sequence)
    self._file_sequence += 1
    path = os.path.join(self.log_dir, filename)
    self._file = open(path, "w", newline="", encoding="utf-8")
    self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES, extrasaction="ignore")
    self._writer.writeheader()
    self._file.flush()
    self._last_flush = time.monotonic()

  def _close_log(self):
    if self._file is not None:
      try:
        self._file.flush()
        os.fsync(self._file.fileno())
      finally:
        self._file.close()
    self._file = None
    self._writer = None

  def _run(self):
    try:
      while True:
        try:
          values = self.queue.get(timeout=GM_LKAS_LOG_FLUSH_INTERVAL_S)
        except queue.Empty:
          if self._file is not None:
            self._file.flush()
            self._last_flush = time.monotonic()
          continue

        if values is self._stop_token:
          break

        if self._writer is None:
          self._open_log()

        self._writer.writerow(self._normalize(values))
        now = time.monotonic()
        if now - self._last_flush >= GM_LKAS_LOG_FLUSH_INTERVAL_S:
          self._file.flush()
          self._last_flush = now

        if self._file.tell() >= self.max_bytes:
          self._close_log()
    except Exception as error:
      self.last_error = repr(error)
      self.disabled = True
      print("GM LKAS diagnostic logger disabled: {}".format(self.last_error))
    finally:
      try:
        self._close_log()
      except Exception:
        pass
