#!/usr/bin/env python3
"""Low-overhead lane-centering diagnostics for real on-road drives."""

import csv
import math
import os
import time
from datetime import datetime, timezone

import cereal.messaging as messaging

from selfdrive.swaglog import cloudlog


LOG_DIR = os.getenv("LATERAL_CENTER_LOG_ROOT", "/data/log/lateral_center")
SAMPLE_HZ = 5.0
SAMPLE_PERIOD_NS = int(1e9 / SAMPLE_HZ)
MIN_SPEED_MPS = 3.0
FLUSH_INTERVAL_S = 5.0
ROWS_PER_FLUSH = int(SAMPLE_HZ * FLUSH_INTERVAL_S)
MAX_FILE_BYTES = 5 * 1024 * 1024
MAX_LOG_FILES = 8
FILE_BUFFER_BYTES = 64 * 1024
MODEL_IDXS = (0, 5, 10)

SERVICES = (
  "modelV2",
  "lateralPlan",
  "liveParameters",
  "liveTorqueParameters",
  "controlsState",
  "carState",
  "carControl",
)


def _safe_float(value):
  try:
    value = float(value)
    return value if math.isfinite(value) else math.nan
  except (TypeError, ValueError):
    return math.nan


def _safe_index(values, index):
  try:
    return _safe_float(values[index])
  except (IndexError, TypeError):
    return math.nan


def _message_age_ms(sm, service, now_ns):
  try:
    mono_time = int(sm.logMonoTime[service])
    return max(0.0, (now_ns - mono_time) / 1e6) if mono_time > 0 else math.nan
  except (KeyError, TypeError, ValueError):
    return math.nan


def _torque_state(controls_state):
  try:
    lateral_state = controls_state.lateralControlState
    return lateral_state.torqueState if lateral_state.which() == "torqueState" else None
  except Exception:
    return None


def should_record(sm):
  """Only log engaged lateral control at meaningful road speed."""
  try:
    # This 0.8.14 fork does not consistently publish carControl.latActive even
    # while torque control is engaged, so controlsState.active is authoritative.
    return bool(sm["controlsState"].active) and float(sm["carState"].vEgo) >= MIN_SPEED_MPS
  except (KeyError, TypeError, ValueError):
    return False


FIELDNAMES = (
  "wall_time_unix_s", "mono_time_s",
  "model_valid", "lateral_plan_valid", "live_params_valid", "live_torque_valid",
  "model_age_ms", "lateral_plan_age_ms", "live_params_age_ms", "live_torque_age_ms",
  "controls_active", "lat_active", "v_ego_mps", "v_ego_kph",
  "steering_angle_deg", "steering_rate_deg",
  "driver_steering_torque", "eps_steering_torque", "steering_pressed",
  "requested_steer", "applied_steer", "steer_limited",
  "controls_curvature", "controls_angle_steers_deg",
  "torque_active", "torque_error", "torque_error_rate", "torque_p", "torque_i",
  "torque_d", "torque_f", "torque_output", "torque_saturated",
  "actual_lat_accel_mps2", "desired_lat_accel_mps2",
  "angle_offset_deg", "angle_offset_average_deg", "angle_offset_fast_std",
  "angle_offset_average_std", "steer_ratio", "stiffness_factor", "road_roll_rad",
  "live_torque_ok", "lat_accel_factor_raw", "lat_accel_offset_raw", "friction_raw",
  "lat_accel_factor_filtered", "lat_accel_offset_filtered", "friction_filtered",
  "torque_bucket_points",
  "use_lane_lines", "lane_width_m", "left_lane_prob", "right_lane_prob",
  "path_lane_prob", "mpc_solution_valid", "lane_change_state", "lane_change_direction",
  "total_camera_offset_m",
) + tuple(
  name
  for index in MODEL_IDXS
  for name in (
    f"model_x_i{index}_m", f"model_path_y_i{index}_m", f"dpath_y_i{index}_m",
    f"left_lane_y_i{index}_m", f"right_lane_y_i{index}_m",
    f"raw_lane_center_y_i{index}_m", f"corrected_lane_center_y_i{index}_m",
    f"lane_width_i{index}_m",
  )
)


def build_row(sm, now_ns=None, wall_time=None):
  now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
  wall_time = time.time() if wall_time is None else float(wall_time)

  model = sm["modelV2"]
  plan = sm["lateralPlan"]
  live_params = sm["liveParameters"]
  live_torque = sm["liveTorqueParameters"]
  controls = sm["controlsState"]
  car_state = sm["carState"]
  car_control = sm["carControl"]
  torque = _torque_state(controls)

  def valid(service):
    try:
      return int(bool(sm.valid[service]) and bool(sm.alive[service]))
    except (KeyError, TypeError):
      return 0

  row = [
    wall_time, now_ns / 1e9,
    valid("modelV2"), valid("lateralPlan"), valid("liveParameters"), valid("liveTorqueParameters"),
    _message_age_ms(sm, "modelV2", now_ns),
    _message_age_ms(sm, "lateralPlan", now_ns),
    _message_age_ms(sm, "liveParameters", now_ns),
    _message_age_ms(sm, "liveTorqueParameters", now_ns),
    int(bool(controls.active)), int(bool(car_control.latActive)),
    _safe_float(car_state.vEgo), _safe_float(car_state.vEgo * 3.6),
    _safe_float(car_state.steeringAngleDeg), _safe_float(car_state.steeringRateDeg),
    _safe_float(car_state.steeringTorque), _safe_float(car_state.steeringTorqueEps),
    int(bool(car_state.steeringPressed)),
    _safe_float(car_control.actuators.steer), _safe_float(car_control.actuatorsOutput.steer),
    int(bool(car_control.actuators.steer != car_control.actuatorsOutput.steer)),
    _safe_float(controls.curvature), _safe_float(controls.angleSteers),
    int(bool(torque.active)) if torque is not None else 0,
    _safe_float(torque.error) if torque is not None else math.nan,
    _safe_float(torque.errorRate) if torque is not None else math.nan,
    _safe_float(torque.p) if torque is not None else math.nan,
    _safe_float(torque.i) if torque is not None else math.nan,
    _safe_float(torque.d) if torque is not None else math.nan,
    _safe_float(torque.f) if torque is not None else math.nan,
    _safe_float(torque.output) if torque is not None else math.nan,
    int(bool(torque.saturated)) if torque is not None else 0,
    _safe_float(torque.actualLateralAccel) if torque is not None else math.nan,
    _safe_float(torque.desiredLateralAccel) if torque is not None else math.nan,
    _safe_float(live_params.angleOffsetDeg), _safe_float(live_params.angleOffsetAverageDeg),
    _safe_float(live_params.angleOffsetFastStd), _safe_float(live_params.angleOffsetAverageStd),
    _safe_float(live_params.steerRatio), _safe_float(live_params.stiffnessFactor),
    _safe_float(live_params.roll),
    int(bool(live_torque.liveValid)),
    _safe_float(live_torque.latAccelFactorRaw), _safe_float(live_torque.latAccelOffsetRaw),
    _safe_float(live_torque.frictionCoefficientRaw),
    _safe_float(live_torque.latAccelFactorFiltered), _safe_float(live_torque.latAccelOffsetFiltered),
    _safe_float(live_torque.frictionCoefficientFiltered), _safe_float(live_torque.totalBucketPoints),
    int(bool(plan.useLaneLines)), _safe_float(plan.laneWidth),
    _safe_float(plan.lProb), _safe_float(plan.rProb), _safe_float(plan.dProb),
    int(bool(plan.mpcSolutionValid)), str(plan.laneChangeState), str(plan.laneChangeDirection),
    _safe_float(plan.totalCameraOffset),
  ]

  left_y = model.laneLines[1].y if len(model.laneLines) > 1 else ()
  right_y = model.laneLines[2].y if len(model.laneLines) > 2 else ()
  for index in MODEL_IDXS:
    left = _safe_index(left_y, index)
    right = _safe_index(right_y, index)
    raw_center = _safe_float((left + right) / 2.0)
    row.extend((
      _safe_index(model.position.x, index), _safe_index(model.position.y, index),
      _safe_index(plan.dPathPoints, index), left, right, raw_center,
      _safe_float(raw_center + plan.totalCameraOffset), _safe_float(abs(right - left)),
    ))

  if len(row) != len(FIELDNAMES):
    raise RuntimeError("lateral center logger field count mismatch")
  return row


class CsvSink:
  def __init__(self, directory=LOG_DIR, max_file_bytes=MAX_FILE_BYTES, max_files=MAX_LOG_FILES):
    self.directory = directory
    self.max_file_bytes = max_file_bytes
    self.max_files = max_files
    self.stream = None
    self.writer = None
    self.rows_since_flush = 0
    self.last_flush = time.monotonic()
    self.sequence = 0

  def _files(self):
    if not os.path.isdir(self.directory):
      return []
    files = [os.path.join(self.directory, name) for name in os.listdir(self.directory)
             if name.startswith("lateral_center_") and name.endswith(".csv")]
    return sorted(files, key=lambda path: (os.path.getmtime(path), path))

  def _remove_old_files(self):
    files = self._files()
    while len(files) >= self.max_files:
      try:
        os.remove(files.pop(0))
      except OSError:
        break

  def _open(self):
    os.makedirs(self.directory, exist_ok=True)
    self._remove_old_files()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    while True:
      path = os.path.join(self.directory, f"lateral_center_{stamp}_{self.sequence:03d}.csv")
      self.sequence += 1
      try:
        self.stream = open(path, "x", newline="", encoding="utf-8", buffering=FILE_BUFFER_BYTES)
        break
      except FileExistsError:
        continue
    self.writer = csv.writer(self.stream)
    self.writer.writerow(FIELDNAMES)
    self.rows_since_flush = 0
    self.last_flush = time.monotonic()
    cloudlog.warning(f"lateral center logger recording to {path}")

  def write(self, row):
    if self.stream is None:
      self._open()
    self.writer.writerow(row)
    self.rows_since_flush += 1
    now = time.monotonic()
    if self.rows_since_flush >= ROWS_PER_FLUSH or now - self.last_flush >= FLUSH_INTERVAL_S:
      self.flush()
      if self.stream.tell() >= self.max_file_bytes:
        self.close(sync=False)

  def flush(self):
    if self.stream is None or self.rows_since_flush == 0:
      return
    self.stream.flush()
    self.rows_since_flush = 0
    self.last_flush = time.monotonic()

  def close(self, sync=True):
    if self.stream is None:
      return
    try:
      self.stream.flush()
      if sync:
        os.fsync(self.stream.fileno())
    finally:
      self.stream.close()
      self.stream = None
      self.writer = None


def main():
  sm = messaging.SubMaster(list(SERVICES), poll=["modelV2"], ignore_avg_freq=list(SERVICES))
  sink = CsvSink()
  next_update_ns = 0
  failures = 0

  try:
    while True:
      # Poll the conflated sockets at the same rate we write. This avoids
      # waking Python for every 20 Hz model frame just to discard most frames.
      now_ns = time.monotonic_ns()
      if next_update_ns > now_ns:
        time.sleep((next_update_ns - now_ns) / 1e9)
      sm.update(1000)
      now_ns = time.monotonic_ns()
      next_update_ns = now_ns + SAMPLE_PERIOD_NS
      if not sm.updated["modelV2"]:
        continue

      if not should_record(sm):
        # Persist a short engagement as soon as lateral control disengages.
        sink.flush()
        continue

      try:
        sink.write(build_row(sm, now_ns=now_ns))
        failures = 0
      except Exception:
        failures += 1
        if failures == 1 or failures % 50 == 0:
          cloudlog.exception("lateral center logger sample failed")
        if failures >= 50:
          time.sleep(1.0)
  finally:
    sink.close()


if __name__ == "__main__":
  main()
