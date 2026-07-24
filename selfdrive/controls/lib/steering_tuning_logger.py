#!/usr/bin/env python3
import atexit
import csv
import datetime
import glob
import os
import queue
import threading
import time


class SteeringTuningLogger:
  """Write bounded 10 Hz torque-steering diagnostics without blocking controlsd."""

  FIELDNAMES = (
    "utc_time",
    "mono_time_s",
    "v_ego_mps",
    "v_ego_kph",
    "controls_active",
    "desired_curvature",
    "desired_lateral_accel_mps2",
    "actual_lateral_accel_mps2",
    "lateral_accel_error_mps2",
    "steering_angle_deg",
    "angle_offset_deg",
    "angle_offset_average_deg",
    "roll_rad",
    "lat_accel_factor",
    "friction",
    "center_offset_target_mps2",
    "center_offset_applied_mps2",
    "center_offset_rate_limited",
    "pid_error",
    "pid_p",
    "pid_i",
    "pid_f",
    "pid_output",
    "requested_steer",
    "applied_steer",
    "steer_limited",
    "steering_pressed",
    "steering_torque",
    "center_i_unwind_active",
    "center_i_before_unwind",
    "center_i_after_unwind",
    "directional_enabled",
    "directional_assist_left",
    "directional_assist_right",
    "directional_side",
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
    self._thread = threading.Thread(target=self._writer_loop, name="steering_tuning_writer", daemon=True)
    self._thread.start()
    atexit.register(self.close)

  def log_sample(self, mono_time_s, v_ego_mps, controls_active,
                 desired_curvature, desired_lateral_accel_mps2,
                 actual_lateral_accel_mps2, steering_angle_deg,
                 angle_offset_deg, angle_offset_average_deg, roll_rad,
                 lat_accel_factor, friction, center_offset_target_mps2,
                 center_offset_applied_mps2, center_offset_rate_limited,
                 pid_error, pid_p, pid_i, pid_f, pid_output,
                 requested_steer, applied_steer, steer_limited,
                 steering_pressed, steering_torque,
                 center_i_unwind_active, center_i_before_unwind,
                 center_i_after_unwind, directional_enabled,
                 directional_assist_left, directional_assist_right,
                 directional_side):
    if self._closed or self.last_error is not None:
      return False

    if mono_time_s >= self._last_sample_time and \
       mono_time_s - self._last_sample_time < self.sample_period:
      return False
    self._last_sample_time = mono_time_s

    desired_lat = float(desired_lateral_accel_mps2)
    actual_lat = float(actual_lateral_accel_mps2)
    sample = (
      time.time(),
      float(mono_time_s),
      float(v_ego_mps),
      float(v_ego_mps) * 3.6,
      int(bool(controls_active)),
      float(desired_curvature),
      desired_lat,
      actual_lat,
      desired_lat - actual_lat,
      float(steering_angle_deg),
      float(angle_offset_deg),
      float(angle_offset_average_deg),
      float(roll_rad),
      float(lat_accel_factor),
      float(friction),
      float(center_offset_target_mps2),
      float(center_offset_applied_mps2),
      int(bool(center_offset_rate_limited)),
      float(pid_error),
      float(pid_p),
      float(pid_i),
      float(pid_f),
      float(pid_output),
      float(requested_steer),
      float(applied_steer),
      int(bool(steer_limited)),
      int(bool(steering_pressed)),
      float(steering_torque),
      int(bool(center_i_unwind_active)),
      float(center_i_before_unwind),
      float(center_i_after_unwind),
      int(bool(directional_enabled)),
      float(directional_assist_left),
      float(directional_assist_right),
      int(directional_side),
    )

    try:
      self._queue.put_nowait(sample)
      return True
    except queue.Full:
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
    pattern = os.path.join(self.directory, "steering_tuning_*.csv")
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
      path = os.path.join(self.directory, "steering_tuning_%s_%03d.csv" % (timestamp, sequence))
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
