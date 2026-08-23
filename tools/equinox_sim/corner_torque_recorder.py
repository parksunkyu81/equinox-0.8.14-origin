#!/usr/bin/env python3
"""Low-rate, event-only recorder for torque-control corner diagnosis."""

from collections import deque
from datetime import datetime
import json
import os

from cereal import messaging
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog


SAMPLE_HZ = 10
CONTROL_HZ = 100
SAMPLE_DIVISOR = CONTROL_HZ // SAMPLE_HZ
PRE_EVENT_SECONDS = 2.0
POST_EVENT_SECONDS = 2.0
CORNER_START_LAT_ACCEL = 0.50
CORNER_STOP_LAT_ACCEL = 0.25
DEFAULT_LOG_DIR = "/data/media/0/corner_torque_logs"
FALLBACK_LOG_DIR = "/tmp/corner_torque_logs"


class CornerTorqueRecorder:
  """Persist only torque-controller samples around an actual corner.

  Normal driving keeps a small in-memory pre-event buffer. Disk I/O happens
  only after a corner has ended, so this process does not add file writes to
  the control loop or record camera/model data.
  """

  def __init__(self):
    self.sm = messaging.SubMaster(["carState", "controlsState"],
                                  poll="controlsState")
    requested_dir = os.getenv("CORNER_TORQUE_LOG_DIR", DEFAULT_LOG_DIR)
    parent_dir = os.path.dirname(requested_dir)
    self.log_dir = requested_dir if os.path.isdir(parent_dir) else FALLBACK_LOG_DIR
    self.pre_event = deque(maxlen=int(PRE_EVENT_SECONDS * SAMPLE_HZ))
    self.event_samples = []
    self.recording = False
    self.post_event_deadline = 0.0
    self.event_sequence = 0

  def _snapshot(self, now):
    controls = self.sm["controlsState"]
    car_state = self.sm["carState"]
    lateral_state = controls.lateralControlState
    if lateral_state.which() != "torqueState":
      return None

    torque = lateral_state.torqueState
    return {
      "monoTime": round(now, 6),
      "vEgo": round(float(car_state.vEgo), 4),
      "steeringAngleDeg": round(float(car_state.steeringAngleDeg), 4),
      "steeringTorque": round(float(car_state.steeringTorque), 4),
      "steeringPressed": bool(car_state.steeringPressed),
      "controlsActive": bool(controls.active),
      "torqueActive": bool(torque.active),
      "desiredCurvature": round(float(torque.desiredCurvature), 7),
      "actualCurvature": round(float(torque.actualCurvature), 7),
      "curvatureError": round(float(torque.curvatureError), 7),
      "desiredLateralAccel": round(float(torque.desiredLateralAccel), 4),
      "actualLateralAccel": round(float(torque.actualLateralAccel), 4),
      "requestedSteer": round(float(torque.requestedSteer), 5),
      "appliedSteer": round(float(torque.appliedSteer), 5),
      "appliedRequestGap": round(
        float(torque.requestedSteer - torque.appliedSteer), 5),
      "steerLimited": bool(torque.steerLimited),
      "saturated": bool(torque.saturated),
      "pidOutput": round(float(torque.output), 5),
      "pidP": round(float(torque.p), 5),
      "pidI": round(float(torque.i), 5),
      "pidF": round(float(torque.f), 5),
    }

  @staticmethod
  def _is_corner(sample, threshold):
    return bool(sample and sample["controlsActive"] and sample["torqueActive"] and
                abs(sample["desiredLateralAccel"]) >= threshold)

  def _start_event(self, sample, now):
    self.recording = True
    self.event_samples = list(self.pre_event)
    self.event_samples.append(sample)
    self.post_event_deadline = 0.0
    self.event_sequence += 1

  def _write_event(self):
    if not self.event_samples:
      return
    os.makedirs(self.log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_path = os.path.join(
      self.log_dir, f"corner_torque_{stamp}_{self.event_sequence:04d}.jsonl")
    temp_path = output_path + ".tmp"
    metadata = {
      "type": "metadata",
      "formatVersion": 1,
      "sampleFrequencyHz": SAMPLE_HZ,
      "preEventSeconds": PRE_EVENT_SECONDS,
      "postEventSeconds": POST_EVENT_SECONDS,
      "startLateralAccel": CORNER_START_LAT_ACCEL,
      "stopLateralAccel": CORNER_STOP_LAT_ACCEL,
      "sampleCount": len(self.event_samples),
    }
    with open(temp_path, "w", encoding="utf8") as output_file:
      output_file.write(json.dumps(metadata, ensure_ascii=False) + "\n")
      for sample in self.event_samples:
        output_file.write(json.dumps(sample, ensure_ascii=False,
                                     separators=(",", ":")) + "\n")
    os.replace(temp_path, output_path)
    cloudlog.warning(f"Corner torque log saved: {output_path}")
    self.event_samples = []
    self.recording = False
    self.post_event_deadline = 0.0

  def run(self):
    try:
      os.nice(10)
    except OSError:
      pass

    while True:
      self.sm.update(1000)
      if not self.sm.updated["controlsState"] or self.sm.frame % SAMPLE_DIVISOR:
        continue

      now = sec_since_boot()
      sample = self._snapshot(now)
      if sample is None:
        continue

      start_corner = self._is_corner(sample, CORNER_START_LAT_ACCEL)
      keep_corner = self._is_corner(sample, CORNER_STOP_LAT_ACCEL)
      if not self.recording:
        if start_corner:
          self._start_event(sample, now)
        self.pre_event.append(sample)
        continue

      self.event_samples.append(sample)
      if keep_corner:
        self.post_event_deadline = 0.0
      elif self.post_event_deadline == 0.0:
        self.post_event_deadline = now + POST_EVENT_SECONDS

      if self.post_event_deadline and now >= self.post_event_deadline:
        self._write_event()
      self.pre_event.append(sample)


def main():
  CornerTorqueRecorder().run()


if __name__ == "__main__":
  main()
