#!/usr/bin/env python3
"""Compact onroad perception/control diagnostics without recording images."""

from collections import deque
from datetime import datetime, timedelta, timezone
import glob
import json
import math
import os


SAMPLE_HZ = 5.0
SAMPLE_INTERVAL_S = 1.0 / SAMPLE_HZ
DEFAULT_LOG_DIR = "/data/media/0/perception_diagnostics"
FALLBACK_LOG_DIR = "/tmp/perception_diagnostics"
MAX_FILE_BYTES = 16 * 1024 * 1024
MAX_FILES = 8
KST = timezone(timedelta(hours=9))
PATH_DISTANCES_M = (5.0, 10.0, 20.0, 30.0, 50.0)
PLAN_POINT_INDICES = (0, 4, 8, 12, 16)
WOBBLE_WINDOW_S = 2.0
WOBBLE_DELTA_DEADBAND_M = 0.02


def finite_float(value, default=0.0):
  try:
    value = float(value)
    return value if math.isfinite(value) else float(default)
  except Exception:
    return float(default)


def rounded(value, digits=5):
  return round(finite_float(value), digits)


def rounded_list(values, digits=5, limit=None):
  try:
    values = list(values)
  except Exception:
    return []
  if limit is not None:
    values = values[:int(limit)]
  return [rounded(value, digits) for value in values]


def sample_path_at_distances(xs, ys, distances=PATH_DISTANCES_M):
  """Linearly sample a model polyline without requiring numpy."""
  try:
    points = [(finite_float(x), finite_float(y)) for x, y in zip(xs, ys)]
  except Exception:
    points = []
  points = sorted(points, key=lambda point: point[0])
  if not points:
    return [0.0 for _ in distances]

  result = []
  for distance in distances:
    target = float(distance)
    if target <= points[0][0]:
      result.append(points[0][1])
      continue
    if target >= points[-1][0]:
      result.append(points[-1][1])
      continue
    for index in range(1, len(points)):
      x0, y0 = points[index - 1]
      x1, y1 = points[index]
      if target <= x1:
        weight = (target - x0) / max(x1 - x0, 1e-6)
        result.append(y0 + weight * (y1 - y0))
        break
  return result


def sample_plan_points(points, indices=PLAN_POINT_INDICES):
  values = [finite_float(value) for value in points]
  if not values:
    return [0.0 for _ in indices]
  return [values[min(max(0, int(index)), len(values) - 1)] for index in indices]


class FrameDeltaTracker:
  def __init__(self):
    self.previous_frame_id = None

  def update(self, frame_id):
    frame_id = int(frame_id)
    delta = 0
    if self.previous_frame_id is not None and frame_id > self.previous_frame_id:
      delta = frame_id - self.previous_frame_id
    self.previous_frame_id = frame_id
    return delta


class PathWobbleTracker:
  def __init__(self, window_s=WOBBLE_WINDOW_S):
    self.window_s = float(window_s)
    self.samples = deque()
    self.previous_path = None

  def update(self, mono_time, path_y):
    now = float(mono_time)
    path = [finite_float(value) for value in path_y]
    delta = ([0.0] * len(path) if self.previous_path is None else
             [cur - prev for cur, prev in zip(path, self.previous_path)])
    self.previous_path = path
    self.samples.append((now, path, delta))
    while self.samples and now - self.samples[0][0] > self.window_s:
      self.samples.popleft()

    ranges = []
    for index in range(len(path)):
      values = [sample[1][index] for sample in self.samples]
      ranges.append(max(values) - min(values))

    direction_flips = 0
    previous_sign = 0
    # The 20 m path point is far enough to expose model wobble without being
    # dominated by immediate vehicle pose noise.
    path_index = min(2, max(0, len(path) - 1))
    for _, _, sample_delta in self.samples:
      value = sample_delta[path_index]
      sign = 1 if value > WOBBLE_DELTA_DEADBAND_M else (-1 if value < -WOBBLE_DELTA_DEADBAND_M else 0)
      if sign and previous_sign and sign != previous_sign:
        direction_flips += 1
      if sign:
        previous_sign = sign

    return {
      "delta": [rounded(value) for value in delta],
      "range2s": [rounded(value) for value in ranges],
      "directionFlips2s": direction_flips,
    }


class RotatingJsonlWriter:
  def __init__(self, log_dir, max_file_bytes=MAX_FILE_BYTES, max_files=MAX_FILES):
    self.log_dir = log_dir
    self.max_file_bytes = int(max_file_bytes)
    self.max_files = int(max_files)
    self.file = None
    self.path = None
    self.file_date = None
    self.bytes_written = 0
    self.records_since_flush = 0
    self.sequence = 0
    os.makedirs(self.log_dir, exist_ok=True)

  def _prune(self):
    paths = sorted(glob.glob(os.path.join(self.log_dir, "perception_*.jsonl")),
                   key=lambda path: (os.path.getmtime(path), path))
    while len(paths) > self.max_files:
      os.remove(paths.pop(0))

  def _open(self):
    self.close()
    now = datetime.now(KST)
    self.file_date = now.date()
    stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
    self.sequence += 1
    self.path = os.path.join(self.log_dir, f"perception_{stamp}_{self.sequence:04d}.jsonl")
    self.file = open(self.path, "w", encoding="utf-8")
    self.bytes_written = 0
    self.records_since_flush = 0
    metadata = {
      "type": "metadata",
      "formatVersion": 1,
      "sampleHz": SAMPLE_HZ,
      "containsVideo": False,
      "pathDistancesM": list(PATH_DISTANCES_M),
      "planPointIndices": list(PLAN_POINT_INDICES),
      "maxFileBytes": self.max_file_bytes,
      "maxFiles": self.max_files,
    }
    self._write_line(metadata)
    self._prune()

  def _write_line(self, record):
    line = json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
    self.file.write(line)
    self.bytes_written += len(line.encode("utf-8"))
    self.records_since_flush += 1

  def write(self, record):
    now = datetime.now(KST)
    estimated_size = len(json.dumps(record, ensure_ascii=False, separators=(",", ":")).encode("utf-8")) + 1
    if (self.file is None or self.file_date != now.date() or
        self.bytes_written + estimated_size > self.max_file_bytes):
      self._open()
    self._write_line(record)
    if self.records_since_flush >= int(SAMPLE_HZ):
      self.file.flush()
      self.records_since_flush = 0

  def close(self):
    if self.file is not None:
      self.file.flush()
      self.file.close()
      self.file = None


def _lateral_torque_snapshot(controls):
  result = {"type": "unavailable"}
  try:
    lateral = controls.lateralControlState
    which = lateral.which()
    result["type"] = str(which)
    if which == "torqueState":
      torque = lateral.torqueState
      result.update({
        "active": bool(torque.active),
        "error": rounded(torque.error),
        "output": rounded(torque.output),
        "saturated": bool(torque.saturated),
        "actualLateralAccel": rounded(torque.actualLateralAccel),
        "desiredLateralAccel": rounded(torque.desiredLateralAccel),
        "latAccelFactor": rounded(torque.latAccelFactor),
        "latAccelOffset": rounded(torque.latAccelOffset),
        "friction": rounded(torque.friction),
      })
  except Exception:
    pass
  return result


def _lead_snapshots(model):
  result = []
  try:
    for lead in list(model.leadsV3)[:2]:
      result.append({
        "prob": rounded(lead.prob),
        "x": rounded(lead.x[0] if len(lead.x) else 0.0),
        "y": rounded(lead.y[0] if len(lead.y) else 0.0),
        "v": rounded(lead.v[0] if len(lead.v) else 0.0),
        "a": rounded(lead.a[0] if len(lead.a) else 0.0),
      })
  except Exception:
    pass
  return result


class PerceptionDiagnosticsRecorder:
  SERVICES = ["modelV2", "roadCameraState", "lateralPlan", "controlsState", "carControl", "carState"]

  def __init__(self, log_dir=None):
    from cereal import messaging

    self.sm = messaging.SubMaster(self.SERVICES, poll=["modelV2"],
                                  ignore_avg_freq=["modelV2", "roadCameraState"])
    requested_dir = log_dir or os.getenv("PERCEPTION_DIAGNOSTICS_LOG_DIR", DEFAULT_LOG_DIR)
    try:
      self.writer = RotatingJsonlWriter(requested_dir)
    except OSError:
      self.writer = RotatingJsonlWriter(FALLBACK_LOG_DIR)
    self.camera_frames = FrameDeltaTracker()
    self.model_frames = FrameDeltaTracker()
    self.model_wobble = PathWobbleTracker()
    self.plan_wobble = PathWobbleTracker()
    self.last_sample_time = -SAMPLE_INTERVAL_S
    from common.realtime import sec_since_boot
    self.now_fn = sec_since_boot

  def _service_health(self, now):
    health = {}
    for service in self.SERVICES:
      mono = self.sm.logMonoTime[service] / 1e9 if self.sm.logMonoTime[service] else 0.0
      health[service] = {
        "alive": bool(self.sm.alive[service]),
        "valid": bool(self.sm.valid[service]),
        "ageMs": rounded(max(0.0, now - mono) * 1000.0, 2) if mono else -1.0,
      }
    return health

  def _snapshot(self, now):
    model = self.sm["modelV2"]
    camera = self.sm["roadCameraState"]
    plan = self.sm["lateralPlan"]
    controls = self.sm["controlsState"]
    car_control = self.sm["carControl"]
    car_state = self.sm["carState"]

    model_path = sample_path_at_distances(model.position.x, model.position.y)
    plan_points = list(plan.dPathPoints)
    plan_path = sample_plan_points(plan_points)
    model_wobble = self.model_wobble.update(now, model_path)
    plan_wobble = self.plan_wobble.update(now, plan_path)

    camera_frame_id = int(camera.frameId)
    model_frame_id = int(model.frameId)
    camera_frame_delta = self.camera_frames.update(camera_frame_id)
    model_frame_delta = self.model_frames.update(model_frame_id)
    lane_lines_y20 = []
    for lane in list(model.laneLines)[:4]:
      lane_lines_y20.append(sample_path_at_distances(lane.x, lane.y, (20.0,))[0])
    road_edges_y20 = []
    for edge in list(model.roadEdges)[:2]:
      road_edges_y20.append(sample_path_at_distances(edge.x, edge.y, (20.0,))[0])

    actuators = car_control.actuators
    return {
      "type": "sample",
      "wallTimeKST": datetime.now(KST).isoformat(timespec="milliseconds"),
      "monoTime": rounded(now, 6),
      "health": self._service_health(now),
      "camera": {
        "frameId": camera_frame_id,
        "frameIdDelta": camera_frame_delta,
        "timestampEof": int(camera.timestampEof),
        "processingTimeMs": rounded(float(camera.processingTime) * 1000.0, 3),
        "gain": rounded(camera.gain),
        "measuredGreyFraction": rounded(camera.measuredGreyFraction),
        "targetGreyFraction": rounded(camera.targetGreyFraction),
        "lensPos": int(camera.lensPos),
        "lensTruePos": rounded(camera.lensTruePos),
        "lensErr": rounded(camera.lensErr),
        "recoverState": int(camera.recoverState),
      },
      "model": {
        "frameId": model_frame_id,
        "frameAge": int(model.frameAge),
        "frameIdDelta": model_frame_delta,
        "frameDropPerc": rounded(model.frameDropPerc),
        "cameraFrameLag": camera_frame_id - model_frame_id,
        "cameraModelFrameDeltaDifference": camera_frame_delta - model_frame_delta,
        "executionTimeMs": rounded(float(model.modelExecutionTime) * 1000.0, 3),
        "gpuExecutionTimeMs": rounded(float(model.gpuExecutionTime) * 1000.0, 3),
        "pathY": rounded_list(model_path),
        "pathYDelta": model_wobble["delta"],
        "pathYRange2s": model_wobble["range2s"],
        "pathDirectionFlips2s": model_wobble["directionFlips2s"],
        "laneLineProbs": rounded_list(model.laneLineProbs, limit=4),
        "laneLineStds": rounded_list(model.laneLineStds, limit=4),
        "laneLinesY20": rounded_list(lane_lines_y20),
        "roadEdgeStds": rounded_list(model.roadEdgeStds, limit=2),
        "roadEdgesY20": rounded_list(road_edges_y20),
        "leads": _lead_snapshots(model),
      },
      "plan": {
        "modelMonoTime": int(plan.modelMonoTime),
        "laneWidth": rounded(plan.laneWidth),
        "leftProb": rounded(plan.lProb),
        "rightProb": rounded(plan.rProb),
        "pathProb": rounded(plan.dProb),
        "useLaneLines": bool(plan.useLaneLines),
        "mpcSolutionValid": bool(plan.mpcSolutionValid),
        "laneChangeState": str(plan.laneChangeState),
        "laneChangeDirection": str(plan.laneChangeDirection),
        "pathY": rounded_list(plan_path),
        "pathYDelta": plan_wobble["delta"],
        "pathYRange2s": plan_wobble["range2s"],
        "pathDirectionFlips2s": plan_wobble["directionFlips2s"],
        "dPathPoints": rounded_list(plan_points, limit=17),
        "curvatures": rounded_list(plan.curvatures, limit=17),
        "curvatureRates": rounded_list(plan.curvatureRates, limit=17),
        "solverExecutionTimeMs": rounded(float(plan.solverExecutionTime) * 1000.0, 3),
      },
      "control": {
        "enabled": bool(controls.enabled),
        "active": bool(controls.active),
        "curvature": rounded(controls.curvature),
        "requestedSteer": rounded(actuators.steer),
        "requestedSteeringAngleDeg": rounded(actuators.steeringAngleDeg),
        "actualSteeringAngleDeg": rounded(car_state.steeringAngleDeg),
        "actualSteeringRateDeg": rounded(car_state.steeringRateDeg),
        "driverTorque": rounded(car_state.steeringTorque),
        "epsTorque": rounded(car_state.steeringTorqueEps),
        "steeringPressed": bool(car_state.steeringPressed),
        "vEgo": rounded(car_state.vEgo),
        "torque": _lateral_torque_snapshot(controls),
        "dynamicTorque": {
          "active": bool(controls.dynamicTorqueActive),
          "latAccelFactor": rounded(controls.dynamicTorqueLatAccelFactor),
          "friction": rounded(controls.dynamicTorqueFriction),
          "blend": rounded(controls.dynamicTorqueBlend),
          "authorityCeiling": rounded(controls.dynamicTorqueAuthorityCeiling),
          "cornerStrength": rounded(controls.dynamicTorqueCornerStrength),
          "directionDamping": bool(controls.dynamicTorqueDirectionDamping),
        },
      },
    }

  def run(self):
    while True:
      self.sm.update(1000)
      if not self.sm.updated["modelV2"]:
        continue
      now = self.now_fn()
      if now - self.last_sample_time < SAMPLE_INTERVAL_S:
        continue
      self.last_sample_time = now
      self.writer.write(self._snapshot(now))

  def close(self):
    self.writer.close()


def main():
  from selfdrive.swaglog import cloudlog

  recorder = None
  try:
    recorder = PerceptionDiagnosticsRecorder()
    cloudlog.info(f"perception diagnostics started at {recorder.writer.log_dir}")
    recorder.run()
  except Exception:
    cloudlog.exception("perception diagnostics failed")
    raise
  finally:
    if recorder is not None:
      recorder.close()


if __name__ == "__main__":
  main()
