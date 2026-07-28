import csv
import datetime
import glob
import os
import queue
import threading
import time
from collections import deque


GM_LKAS_LOG_DIR = os.environ.get("GM_LKAS_LOG_DIR", "/data/log/gm_lkas")
GM_LKAS_LOG_MAX_BYTES = 5 * 1024 * 1024
GM_LKAS_LOG_MAX_FILES = 6
GM_LKAS_LOG_QUEUE_SIZE = 4096
GM_LKAS_LOG_FLUSH_INTERVAL_S = 0.25
GM_LKAS_BLACKBOX_FLUSH_INTERVAL_S = 1.0
GM_LKAS_BLACKBOX_DIR = os.environ.get(
  "GM_LKAS_BLACKBOX_DIR", os.path.join(GM_LKAS_LOG_DIR, "blackbox"))
GM_LKAS_INCIDENT_DIR = os.environ.get(
  "GM_LKAS_INCIDENT_DIR", os.path.join(GM_LKAS_LOG_DIR, "incidents"))
GM_LKAS_BLACKBOX_MAX_BYTES = 5 * 1024 * 1024
GM_LKAS_BLACKBOX_MAX_FILES = 6
GM_LKAS_INCIDENT_MAX_FILES = 20
GM_LKAS_INCIDENT_PRE_SECONDS = 60.0
GM_LKAS_INCIDENT_POST_SECONDS = 30.0
GM_LKAS_INCIDENT_RETRIGGER_SECONDS = 300.0
GM_LKAS_CAN_ADDRESS = 0x180
GM_PSCM_STATUS_ADDRESS = 0x184
GM_LKAS_MAX_INTERVAL_MS = 35.0


def gm_lkas_checksum(active, torque, counter):
  return (0x1000 - (int(bool(active)) << 11) - (int(torque) & 0x7ff) - (int(counter) % 4)) & 0xfff


def decode_gm_lkas_frame(dat):
  """Decode the four-byte GM ASCMLKASteeringCmd (0x180) payload."""
  raw = bytes(dat)
  if len(raw) != 4:
    return None

  torque_unsigned = ((raw[0] & 0x07) << 8) | raw[1]
  torque = torque_unsigned - 0x800 if torque_unsigned & 0x400 else torque_unsigned
  active = bool(raw[0] & 0x08)
  counter = (raw[0] >> 4) & 0x03
  checksum = ((raw[2] & 0x0f) << 8) | raw[3]
  expected_checksum = gm_lkas_checksum(active, torque, counter)
  return {
    "counter": counter,
    "torque": torque,
    "active": active,
    "checksum": checksum,
    "checksum_valid": checksum == expected_checksum,
  }


def decode_gm_pscm_status_frame(dat):
  """Decode LKATorqueDeliveredStatus from GM PSCMStatus (0x184)."""
  raw = bytes(dat)
  if len(raw) != 8:
    return None
  return (raw[0] >> 3) & 0x07


class GMLKASBlackboxRecorder:
  """Persist raw GM 0x180 traffic and preserve pre/post-fault incident windows.

  This recorder is intentionally independent from the controls loop. Its caller
  supplies already received CAN/state events from a background subscriber.
  """

  FIELDNAMES = [
    "utc_time", "mono_time_s", "record_type", "source",
    "bus", "bus_time", "address", "dat_hex",
    "counter", "torque", "active", "checksum", "checksum_valid",
    "interval_ms", "counter_sequence_valid",
    "pscm_status", "command_active", "command_gap_ms", "loopback_acked",
    "incident_reason",
  ]

  def __init__(self, stream_dir=None, incident_dir=None,
               max_bytes=GM_LKAS_BLACKBOX_MAX_BYTES,
               max_files=GM_LKAS_BLACKBOX_MAX_FILES,
               max_incident_files=GM_LKAS_INCIDENT_MAX_FILES,
               pre_seconds=GM_LKAS_INCIDENT_PRE_SECONDS,
               post_seconds=GM_LKAS_INCIDENT_POST_SECONDS):
    self.stream_dir = stream_dir or GM_LKAS_BLACKBOX_DIR
    self.incident_dir = incident_dir or GM_LKAS_INCIDENT_DIR
    self.max_bytes = max(int(max_bytes), 1024)
    self.max_files = max(int(max_files), 1)
    self.max_incident_files = max(int(max_incident_files), 1)
    self.pre_seconds = max(float(pre_seconds), 0.0)
    self.post_seconds = max(float(post_seconds), 0.0)

    self.history = deque()
    self.previous_can = {}
    self.previous_state = None
    self.last_active_time = None
    self.last_state_sample_time = None
    self.last_trigger_time = {}
    self.last_loopback_time = None
    self.last_loopback_interval_ms = 0.0

    self._stream_file = None
    self._stream_writer = None
    self._incident_file = None
    self._incident_writer = None
    self._incident_until = None
    self._last_flush = 0.0
    self._stream_sequence = 0
    self._incident_sequence = 0

  @staticmethod
  def _utc_time(wall_time_s=None):
    wall_time = time.time() if wall_time_s is None else float(wall_time_s)
    value = datetime.datetime.fromtimestamp(wall_time, datetime.timezone.utc)
    return value.isoformat(timespec="milliseconds").replace("+00:00", "Z")

  @staticmethod
  def _prune_files(directory, pattern, max_files):
    os.makedirs(directory, exist_ok=True)
    files = sorted((path for path in glob.glob(os.path.join(directory, pattern))
                    if os.path.isfile(path)),
                   key=lambda path: (os.path.getmtime(path), path))
    while len(files) >= max_files:
      os.remove(files.pop(0))

  def _open_stream(self):
    self._prune_files(self.stream_dir, "gm_lkas_can_*.csv", self.max_files)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    filename = "gm_lkas_can_{}_{}_{}.csv".format(
      stamp, os.getpid(), self._stream_sequence)
    self._stream_sequence += 1
    self._stream_file = open(os.path.join(self.stream_dir, filename),
                             "w", newline="", encoding="utf-8")
    self._stream_writer = csv.DictWriter(
      self._stream_file, fieldnames=self.FIELDNAMES, extrasaction="ignore")
    self._stream_writer.writeheader()
    self._stream_file.flush()
    self._last_flush = time.monotonic()

  def _close_stream(self):
    if self._stream_file is not None:
      try:
        self._stream_file.flush()
        os.fsync(self._stream_file.fileno())
      finally:
        self._stream_file.close()
    self._stream_file = None
    self._stream_writer = None

  def _open_incident(self, mono_time_s, reasons):
    self._prune_files(
      self.incident_dir, "gm_lkas_incident_*.csv", self.max_incident_files)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    reason_label = "-".join(reasons)[:80].replace(os.sep, "_")
    filename = "gm_lkas_incident_{}_{}_{}_{}.csv".format(
      stamp, reason_label, os.getpid(), self._incident_sequence)
    self._incident_sequence += 1
    self._incident_file = open(os.path.join(self.incident_dir, filename),
                               "w", newline="", encoding="utf-8")
    self._incident_writer = csv.DictWriter(
      self._incident_file, fieldnames=self.FIELDNAMES, extrasaction="ignore")
    self._incident_writer.writeheader()
    for row in self.history:
      self._incident_writer.writerow(row)
    self._incident_until = float(mono_time_s) + self.post_seconds
    self._incident_file.flush()

  def _close_incident(self):
    if self._incident_file is not None:
      try:
        self._incident_file.flush()
        os.fsync(self._incident_file.fileno())
      finally:
        self._incident_file.close()
    self._incident_file = None
    self._incident_writer = None
    self._incident_until = None

  def _record(self, row, reasons=()):
    mono_time_s = float(row["mono_time_s"])
    reasons = tuple(sorted(set(reason for reason in reasons if reason)))
    trigger_reasons = tuple(
      reason for reason in reasons
      if reason not in self.last_trigger_time or
      mono_time_s - self.last_trigger_time[reason] >= GM_LKAS_INCIDENT_RETRIGGER_SECONDS
    )
    row = {name: row.get(name, "") for name in self.FIELDNAMES}
    row["incident_reason"] = "|".join(reasons)

    while self.history and mono_time_s - float(self.history[0]["mono_time_s"]) > self.pre_seconds:
      self.history.popleft()

    if self._incident_until is not None and mono_time_s > self._incident_until and not trigger_reasons:
      self._close_incident()

    if trigger_reasons:
      for reason in trigger_reasons:
        self.last_trigger_time[reason] = mono_time_s
      if self._incident_file is None:
        self._open_incident(mono_time_s, trigger_reasons)
      else:
        self._incident_until = max(self._incident_until, mono_time_s + self.post_seconds)

    if self._stream_writer is None:
      self._open_stream()
    self._stream_writer.writerow(row)
    if self._incident_writer is not None:
      self._incident_writer.writerow(row)
    self.history.append(row)

    now = time.monotonic()
    if now - self._last_flush >= GM_LKAS_BLACKBOX_FLUSH_INTERVAL_S:
      self._stream_file.flush()
      if self._incident_file is not None:
        self._incident_file.flush()
      self._last_flush = now

    if self._stream_file.tell() >= self.max_bytes:
      self._close_stream()

  def record_can(self, source, bus, bus_time, dat, mono_time_s, wall_time_s=None):
    decoded = decode_gm_lkas_frame(dat)
    if decoded is None:
      return []

    source = str(source)
    bus = int(bus)
    mono_time_s = float(mono_time_s)
    channel = (source, bus)
    previous = self.previous_can.get(channel)
    interval_ms = ""
    counter_sequence_valid = ""
    reasons = []

    if previous is not None:
      interval_ms = max((mono_time_s - previous["time"]) * 1000.0, 0.0)
      if interval_ms <= 1000.0:
        counter_sequence_valid = decoded["counter"] == (previous["counter"] + 1) % 4
        if not counter_sequence_valid:
          reasons.append("counter_sequence_{}_bus{}".format(source, bus))

        recently_active = self.last_active_time is not None and \
                          mono_time_s - self.last_active_time <= 1.0
        if recently_active and interval_ms > GM_LKAS_MAX_INTERVAL_MS:
          reasons.append("interval_{}_bus{}".format(source, bus))

    if not decoded["checksum_valid"]:
      reasons.append("checksum_{}_bus{}".format(source, bus))

    # OP transmissions return on bus 128. Any received 0x180 on vehicle bus 0
    # therefore came from outside the OP TX loopback path.
    if source == "can" and bus == 0:
      reasons.append("vehicle_bus_0x180")

    self.previous_can[channel] = {
      "time": mono_time_s,
      "counter": decoded["counter"],
    }
    if source == "can" and bus == 128:
      if previous is not None:
        self.last_loopback_interval_ms = interval_ms
      self.last_loopback_time = mono_time_s
      if decoded["active"]:
        self.last_active_time = mono_time_s

    row = {
      "utc_time": self._utc_time(wall_time_s),
      "mono_time_s": "{:.9f}".format(mono_time_s),
      "record_type": "can",
      "source": source,
      "bus": bus,
      "bus_time": int(bus_time),
      "address": GM_LKAS_CAN_ADDRESS,
      "dat_hex": bytes(dat).hex(),
      "counter": decoded["counter"],
      "torque": decoded["torque"],
      "active": int(decoded["active"]),
      "checksum": decoded["checksum"],
      "checksum_valid": int(decoded["checksum_valid"]),
      "interval_ms": "" if interval_ms == "" else "{:.3f}".format(interval_ms),
      "counter_sequence_valid": "" if counter_sequence_valid == "" else int(counter_sequence_valid),
    }
    self._record(row, reasons)
    return reasons

  def record_state(self, mono_time_s, pscm_status, command_active,
                   command_gap_ms, loopback_acked, wall_time_s=None,
                   source="controlsState"):
    mono_time_s = float(mono_time_s)
    pscm_status = int(pscm_status)
    command_active = bool(command_active)
    command_gap_ms = float(command_gap_ms)
    loopback_acked = bool(loopback_acked)
    if command_active:
      self.last_active_time = mono_time_s

    state = (pscm_status, command_active, command_gap_ms > GM_LKAS_MAX_INTERVAL_MS)
    state_changed = state != self.previous_state
    sample_due = self.last_state_sample_time is None or \
                 mono_time_s - self.last_state_sample_time >= 0.1
    reasons = []

    previous_status = None if self.previous_state is None else self.previous_state[0]
    recently_active = self.last_active_time is not None and \
                      mono_time_s - self.last_active_time <= 1.0
    if pscm_status == 3 and previous_status != 3:
      reasons.append("pscm_status_3")
    if pscm_status == 2 and previous_status != 2 and recently_active:
      reasons.append("pscm_status_2_active")
    if command_gap_ms > GM_LKAS_MAX_INTERVAL_MS and \
       (self.previous_state is None or not self.previous_state[2]):
      reasons.append("command_gap")

    if state_changed or sample_due or reasons:
      row = {
        "utc_time": self._utc_time(wall_time_s),
        "mono_time_s": "{:.9f}".format(mono_time_s),
        "record_type": "state",
        "source": source,
        "pscm_status": pscm_status,
        "command_active": int(command_active),
        "command_gap_ms": "{:.3f}".format(command_gap_ms),
        "loopback_acked": int(loopback_acked),
      }
      self._record(row, reasons)
      self.last_state_sample_time = mono_time_s

    self.previous_state = state
    return reasons

  def record_pscm_status(self, bus, dat, mono_time_s, wall_time_s=None):
    status = decode_gm_pscm_status_frame(dat)
    if status is None or int(bus) != 0:
      return []

    mono_time_s = float(mono_time_s)
    command_active = self.last_active_time is not None and \
                     mono_time_s - self.last_active_time <= 0.25
    loopback_acked = self.last_loopback_time is not None and \
                     mono_time_s - self.last_loopback_time <= 0.045
    return self.record_state(
      mono_time_s,
      status,
      command_active,
      self.last_loopback_interval_ms,
      loopback_acked,
      wall_time_s=wall_time_s,
      source="can:bus0:0x184",
    )

  def close(self):
    self._close_incident()
    self._close_stream()


def run_gm_lkas_can_blackbox(recorder=None):
  """Run the raw CAN recorder in its own process, isolated from controlsd."""
  import cereal.messaging as messaging

  recorder = recorder or GMLKASBlackboxRecorder()
  poller = messaging.Poller()
  messaging.sub_sock("can", poller=poller)

  try:
    while True:
      for sock in poller.poll(100):
        for event in messaging.drain_sock(sock):
          mono_time_s = event.logMonoTime * 1e-9
          for frame in event.can:
            if frame.address == GM_LKAS_CAN_ADDRESS:
              recorder.record_can(
                "can", frame.src, frame.busTime, frame.dat, mono_time_s)
            elif frame.address == GM_PSCM_STATUS_ADDRESS:
              recorder.record_pscm_status(
                frame.src, frame.dat, mono_time_s)
  finally:
    recorder.close()


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
