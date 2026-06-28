#!/usr/bin/env python3
import csv
import math
import os
import time
from datetime import datetime

import cereal.messaging as messaging

from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.loggerd.config import ROOT
from selfdrive.swaglog import cloudlog


LOG_DIR = os.getenv("LATERAL_BIAS_LOG_ROOT", os.path.join(ROOT, "lateral_bias_csv"))
MODEL_IDXS = (0, 5, 10, 20, 32)
PLAN_IDXS = (0, 1, 2, 5, 10, 16)
ROWS_PER_FILE = 20 * 60 * 30  # 30 minutes at the model's 20 Hz update rate
MAX_LOG_FILES = 8  # retain up to four hours of 30-minute diagnostic files
FLUSH_EVERY_ROWS = 20

SERVICES = (
  "modelV2",
  "lateralPlan",
  "liveParameters",
  "controlsState",
  "carState",
  "carControl",
)


def _safe_float(value):
  try:
    value = float(value)
    return round(value, 8) if math.isfinite(value) else math.nan
  except (TypeError, ValueError):
    return math.nan


def _safe_index(values, index):
  try:
    return _safe_float(values[index])
  except (IndexError, TypeError):
    return math.nan


def _age_ms(sm, service):
  model_time = int(sm.logMonoTime["modelV2"])
  service_time = int(sm.logMonoTime[service])
  if model_time == 0 or service_time == 0:
    return math.nan
  return _safe_float((model_time - service_time) / 1e6)


def _fieldnames():
  fields = [
    "wall_time",
    "model_mono_time_ns",
    "model_frame_id",
    "model_frame_drop_percent",
  ]

  for service in SERVICES:
    fields.extend((f"{service}_alive", f"{service}_valid", f"{service}_age_ms"))

  fields.extend([
    "controls_active",
    "v_ego_mps",
    "v_ego_kph",
    "steering_angle_deg",
    "steering_rate_deg",
    "driver_steering_torque",
    "eps_steering_torque",
    "steering_pressed",
    "requested_steer",
    "applied_steer",
    "controls_curvature",
    "controls_angle_steers_deg",
    "controls_total_camera_offset_m",
    "controls_lat_accel_factor",
    "controls_lat_accel_offset",
    "controls_friction",
    "torque_state_active",
    "torque_state_error",
    "torque_state_output",
    "torque_state_actual_lat_accel",
    "torque_state_desired_lat_accel",
    "torque_state_saturated",
    "live_params_valid",
    "angle_offset_deg",
    "angle_offset_average_deg",
    "angle_offset_fast_std",
    "angle_offset_average_std",
    "steer_ratio",
    "stiffness_factor",
    "road_roll_rad",
    "use_lane_lines",
    "lane_width_m",
    "left_lane_prob",
    "right_lane_prob",
    "path_lane_prob",
    "mpc_solution_valid",
    "planner_total_camera_offset_m",
    "model_left_lane_prob",
    "model_right_lane_prob",
    "model_left_lane_std",
    "model_right_lane_std",
    "camera_offset_m",
  ])

  for index in MODEL_IDXS:
    fields.extend([
      f"model_x_i{index}_m",
      f"model_path_y_i{index}_m",
      f"model_path_y_std_i{index}_m",
      f"left_lane_y_i{index}_m",
      f"right_lane_y_i{index}_m",
      f"raw_lane_center_y_i{index}_m",
      f"corrected_lane_center_y_i{index}_m",
      f"lane_width_i{index}_m",
    ])

  for index in PLAN_IDXS:
    fields.extend([
      f"dpath_y_i{index}_m",
      f"psi_i{index}_rad",
      f"curvature_i{index}_1pm",
      f"curvature_rate_i{index}_1pmps",
    ])

  return fields


FIELDNAMES = _fieldnames()


def _torque_state(controls_state):
  try:
    lateral_state = controls_state.lateralControlState
    if lateral_state.which() == "torqueState":
      return lateral_state.torqueState
  except Exception:
    pass
  return None


def _build_row(sm):
  md = sm["modelV2"]
  lateral_plan = sm["lateralPlan"]
  live_params = sm["liveParameters"]
  controls_state = sm["controlsState"]
  car_state = sm["carState"]
  car_control = sm["carControl"]
  torque_state = _torque_state(controls_state)

  row = {
    "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
    "model_mono_time_ns": int(sm.logMonoTime["modelV2"]),
    "model_frame_id": int(md.frameId),
    "model_frame_drop_percent": _safe_float(md.frameDropPerc),
  }

  for service in SERVICES:
    row[f"{service}_alive"] = int(bool(sm.alive[service]))
    row[f"{service}_valid"] = int(bool(sm.valid[service]))
    row[f"{service}_age_ms"] = _age_ms(sm, service)

  row.update({
    "controls_active": int(bool(controls_state.active)),
    "v_ego_mps": _safe_float(car_state.vEgo),
    "v_ego_kph": _safe_float(_safe_float(car_state.vEgo) * 3.6),
    "steering_angle_deg": _safe_float(car_state.steeringAngleDeg),
    "steering_rate_deg": _safe_float(car_state.steeringRateDeg),
    "driver_steering_torque": _safe_float(car_state.steeringTorque),
    "eps_steering_torque": _safe_float(car_state.steeringTorqueEps),
    "steering_pressed": int(bool(car_state.steeringPressed)),
    "requested_steer": _safe_float(car_control.actuators.steer),
    "applied_steer": _safe_float(car_control.actuatorsOutput.steer),
    "controls_curvature": _safe_float(controls_state.curvature),
    "controls_angle_steers_deg": _safe_float(controls_state.angleSteers),
    "controls_total_camera_offset_m": _safe_float(controls_state.totalCameraOffset),
    "controls_lat_accel_factor": _safe_float(controls_state.latAccelFactor),
    "controls_lat_accel_offset": _safe_float(controls_state.latAccelOffset),
    "controls_friction": _safe_float(controls_state.friction),
    "torque_state_active": int(bool(torque_state.active)) if torque_state is not None else 0,
    "torque_state_error": _safe_float(torque_state.error) if torque_state is not None else math.nan,
    "torque_state_output": _safe_float(torque_state.output) if torque_state is not None else math.nan,
    "torque_state_actual_lat_accel":
      _safe_float(torque_state.actualLateralAccel) if torque_state is not None else math.nan,
    "torque_state_desired_lat_accel":
      _safe_float(torque_state.desiredLateralAccel) if torque_state is not None else math.nan,
    "torque_state_saturated": int(bool(torque_state.saturated)) if torque_state is not None else 0,
    "live_params_valid": int(bool(live_params.valid)),
    "angle_offset_deg": _safe_float(live_params.angleOffsetDeg),
    "angle_offset_average_deg": _safe_float(live_params.angleOffsetAverageDeg),
    "angle_offset_fast_std": _safe_float(live_params.angleOffsetFastStd),
    "angle_offset_average_std": _safe_float(live_params.angleOffsetAverageStd),
    "steer_ratio": _safe_float(live_params.steerRatio),
    "stiffness_factor": _safe_float(live_params.stiffnessFactor),
    "road_roll_rad": _safe_float(live_params.roll),
    "use_lane_lines": int(bool(lateral_plan.useLaneLines)),
    "lane_width_m": _safe_float(lateral_plan.laneWidth),
    "left_lane_prob": _safe_float(lateral_plan.lProb),
    "right_lane_prob": _safe_float(lateral_plan.rProb),
    "path_lane_prob": _safe_float(lateral_plan.dProb),
    "mpc_solution_valid": int(bool(lateral_plan.mpcSolutionValid)),
    "planner_total_camera_offset_m": _safe_float(lateral_plan.totalCameraOffset),
    "model_left_lane_prob": _safe_index(md.laneLineProbs, 1),
    "model_right_lane_prob": _safe_index(md.laneLineProbs, 2),
    "model_left_lane_std": _safe_index(md.laneLineStds, 1),
    "model_right_lane_std": _safe_index(md.laneLineStds, 2),
    "camera_offset_m": CAMERA_OFFSET,
  })

  left_lane_y = md.laneLines[1].y if len(md.laneLines) > 1 else ()
  right_lane_y = md.laneLines[2].y if len(md.laneLines) > 2 else ()
  for index in MODEL_IDXS:
    left_y = _safe_index(left_lane_y, index)
    right_y = _safe_index(right_lane_y, index)
    raw_center_y = _safe_float((left_y + right_y) / 2.0)
    row.update({
      f"model_x_i{index}_m": _safe_index(md.position.x, index),
      f"model_path_y_i{index}_m": _safe_index(md.position.y, index),
      f"model_path_y_std_i{index}_m": _safe_index(md.position.yStd, index),
      f"left_lane_y_i{index}_m": left_y,
      f"right_lane_y_i{index}_m": right_y,
      f"raw_lane_center_y_i{index}_m": raw_center_y,
      f"corrected_lane_center_y_i{index}_m": _safe_float(raw_center_y + CAMERA_OFFSET),
      f"lane_width_i{index}_m": _safe_float(right_y - left_y),
    })

  for index in PLAN_IDXS:
    row.update({
      f"dpath_y_i{index}_m": _safe_index(lateral_plan.dPathPoints, index),
      f"psi_i{index}_rad": _safe_index(lateral_plan.psis, index),
      f"curvature_i{index}_1pm": _safe_index(lateral_plan.curvatures, index),
      f"curvature_rate_i{index}_1pmps": _safe_index(lateral_plan.curvatureRates, index),
    })

  return row


def _cleanup_old_logs():
  if not os.path.isdir(LOG_DIR):
    return

  try:
    files = [
      os.path.join(LOG_DIR, name)
      for name in os.listdir(LOG_DIR)
      if name.startswith("lateral_bias_") and name.endswith(".csv")
    ]
    files.sort(key=os.path.getmtime, reverse=True)
    # Keep one slot free for the file that will be opened immediately after cleanup.
    for path in files[MAX_LOG_FILES - 1:]:
      os.remove(path)
  except OSError:
    cloudlog.exception("lateral_bias_logger failed to clean old logs")


def _open_log(part):
  os.makedirs(LOG_DIR, exist_ok=True)
  timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
  path = os.path.join(LOG_DIR, f"lateral_bias_{timestamp}_p{part:02d}.csv")
  file_handle = open(path, "w", newline="")
  writer = csv.DictWriter(file_handle, fieldnames=FIELDNAMES, extrasaction="ignore")
  writer.writeheader()
  file_handle.flush()
  cloudlog.warning(f"lateral_bias_logger recording to {path}")
  return path, file_handle, writer


def main():
  sm = messaging.SubMaster(
    list(SERVICES),
    poll=["modelV2"],
    ignore_avg_freq=list(SERVICES),
  )

  part = 0
  row_count = 0
  file_handle = None
  writer = None

  try:
    while True:
      sm.update()
      if not sm.updated["modelV2"]:
        continue

      if file_handle is None or row_count >= ROWS_PER_FILE:
        if file_handle is not None:
          file_handle.flush()
          os.fsync(file_handle.fileno())
          file_handle.close()
          part += 1
        _cleanup_old_logs()
        _, file_handle, writer = _open_log(part)
        row_count = 0

      try:
        row = _build_row(sm)
      except Exception:
        cloudlog.exception("lateral_bias_logger row collection failed")
        continue

      try:
        writer.writerow(row)
        row_count += 1
        if row_count % FLUSH_EVERY_ROWS == 0:
          file_handle.flush()
      except (OSError, ValueError):
        cloudlog.exception("lateral_bias_logger write failed")
        try:
          file_handle.close()
        except (OSError, ValueError):
          pass
        file_handle = None
        writer = None
        part += 1
        time.sleep(1.0)
  finally:
    if file_handle is not None:
      try:
        file_handle.flush()
        os.fsync(file_handle.fileno())
        file_handle.close()
      except OSError:
        pass


if __name__ == "__main__":
  main()
