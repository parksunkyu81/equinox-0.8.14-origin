#!/usr/bin/env python3
import atexit
import csv
import datetime
import glob
import os
import queue
import threading
import time


class PedalTuningLogger:
  """Write bounded 10 Hz comma-pedal diagnostics without blocking controlsd."""

  FIELDNAMES = (
    "utc_time",
    "mono_time_s",
    "v_ego_mps",
    "v_ego_kph",
    "pid_accel_request_mps2",
    "accel_limit_max_mps2",
    "launch_accel_request_mps2",
    "launch_accel_floor_mps2",
    "applied_accel_mps2",
    "pedal_command",
    "vehicle_accel_mps2",
    "lead_valid",
    "lead_d_rel_m",
    "lead_v_rel_mps",
    "lead_v_mps",
    "lead_accel_mps2",
    "radar_age_s",
    "launch_state",
    "launch_active",
    "brake_pressed",
    "gas_pressed",
    "controls_active",
    "adaptive_cruise",
  )

  _STOP = object()

  def __init__(self, directory, sample_hz=10.0, max_file_bytes=5 * 1024 * 1024,
               max_files=10, queue_size=1000, flush_interval=1.0):
    self.directory = directory
    self.sample_period = 1.0 / sample_hz
    self.max_file_bytes = max_file_bytes
    self.max_files = max_files
    self.flush_interval = flush_interval
    self.dropped_samples = 0
    self.last_error = None

    self._last_sample_time = float("-inf")
    self._closed = False
    self._queue = queue.Queue(maxsize=queue_size)
    self._thread = threading.Thread(target=self._writer_loop, name="pedal_tuning_writer", daemon=True)
    self._thread.start()
    atexit.register(self.close)

  def log_sample(self, mono_time_s, v_ego_mps, pid_accel_request_mps2,
                 accel_limit_max_mps2, applied_accel_mps2, pedal_command,
                 vehicle_accel_mps2, brake_pressed, gas_pressed,
                 controls_active, adaptive_cruise,
                 launch_accel_request_mps2=0.0, launch_accel_floor_mps2=0.0,
                 lead_valid=False, lead_d_rel_m=0.0, lead_v_rel_mps=0.0,
                 lead_v_mps=0.0, lead_accel_mps2=0.0, radar_age_s=0.0,
                 launch_state=0, launch_active=False):
    if self._closed or self.last_error is not None:
      return False

    # Accept the first sample immediately and recover cleanly if a replay or
    # clock reset moves monotonic time backwards.
    if mono_time_s >= self._last_sample_time and \
       mono_time_s - self._last_sample_time < self.sample_period:
      return False
    self._last_sample_time = mono_time_s

    sample = (
      time.time(),
      float(mono_time_s),
      float(v_ego_mps),
      float(v_ego_mps) * 3.6,
      float(pid_accel_request_mps2),
      float(accel_limit_max_mps2),
      float(launch_accel_request_mps2),
      float(launch_accel_floor_mps2),
      float(applied_accel_mps2),
      float(pedal_command),
      float(vehicle_accel_mps2),
      int(bool(lead_valid)),
      float(lead_d_rel_m),
      float(lead_v_rel_mps),
      float(lead_v_mps),
      float(lead_accel_mps2),
      float(radar_age_s),
      int(launch_state),
      int(bool(launch_active)),
      int(bool(brake_pressed)),
      int(bool(gas_pressed)),
      int(bool(controls_active)),
      int(bool(adaptive_cruise)),
    )

    try:
      self._queue.put_nowait(sample)
      return True
    except queue.Full:
      # Never make the 100 Hz control loop wait for disk I/O.
      self.dropped_samples += 1
      return False

  def close(self, timeout=2.0):
    if self._closed:
      return
    self._closed = True
    try:
      self._queue.put(self._STOP, timeout=timeout)
    except queue.Full:
      return
    self._thread.join(timeout=timeout)

  def _existing_files(self):
    pattern = os.path.join(self.directory, "pedal_tuning_*.csv")
    return sorted(glob.glob(pattern), key=lambda path: (os.path.getmtime(path), path))

  def _remove_old_files(self):
    files = self._existing_files()
    while len(files) >= self.max_files:
      oldest = files.pop(0)
      try:
        os.remove(oldest)
      except OSError:
        break

  def _open_file(self, sequence):
    os.makedirs(self.directory, exist_ok=True)
    self._remove_old_files()
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())

    while True:
      path = os.path.join(self.directory, "pedal_tuning_%s_%03d.csv" % (timestamp, sequence))
      try:
        stream = open(path, "x", newline="", encoding="utf-8", buffering=64 * 1024)
        break
      except FileExistsError:
        sequence += 1

    writer = csv.writer(stream)
    writer.writerow(self.FIELDNAMES)
    return stream, writer, sequence + 1

  @staticmethod
  def _format_sample(sample):
    wall_time = datetime.datetime.fromtimestamp(
      sample[0], tz=datetime.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    return (wall_time,) + sample[1:]

  def _writer_loop(self):
    stream = None
    writer = None
    sequence = 0
    pending_rows = 0
    last_flush = time.monotonic()

    try:
      while True:
        sample = self._queue.get()
        if sample is self._STOP:
          break

        if stream is None or stream.tell() >= self.max_file_bytes:
          if stream is not None:
            stream.flush()
            stream.close()
          stream, writer, sequence = self._open_file(sequence)
          pending_rows = 0
          last_flush = time.monotonic()

        writer.writerow(self._format_sample(sample))
        pending_rows += 1

        now = time.monotonic()
        if pending_rows >= 10 or now - last_flush >= self.flush_interval:
          stream.flush()
          pending_rows = 0
          last_flush = now
    except Exception as error:
      self.last_error = repr(error)
    finally:
      if stream is not None:
        try:
          stream.flush()
          stream.close()
        except OSError:
          pass
