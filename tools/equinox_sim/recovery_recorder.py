#!/usr/bin/env python3
from collections import deque
from datetime import datetime
import json
import os

from cereal import car, messaging
from common.conversions import Conversions as CV
from common.params import Params
from common.realtime import sec_since_boot
from opendbc.can.parser import CANParser
from selfdrive.car import crc8_pedal
from selfdrive.car.gm.values import CAR, DBC, CanBus
from selfdrive.controls.lib.pedal_force_recovery import (
  PEDAL_FORCE_RECOVERY_SPEED_ERROR,
  recovery_log_trigger,
)
from selfdrive.controls.lib.drive_helpers import CONTROL_N, V_CRUISE_ENABLE_MIN
from selfdrive.swaglog import cloudlog


PRE_EVENT_SECONDS = 5.0
POST_EVENT_SECONDS = 10.0
MIN_EVENT_INTERVAL_SECONDS = 60.0
EXPECTED_CONTROL_HZ = 50
PRE_EVENT_SAMPLES = int(PRE_EVENT_SECONDS * EXPECTED_CONTROL_HZ) + 50
DEFAULT_LOG_DIR = "/data/media/0/pedal_recovery_logs"
FALLBACK_LOG_DIR = "/tmp/pedal_recovery_logs"


class PedalRecoveryRecorder:
  """Read-only event recorder for pedal-force recovery diagnosis.

  Samples stay in memory during normal driving. A candidate accel=0 event or a
  recovery activation flushes the preceding five seconds plus the following
  ten seconds to JSONL, keeping disk I/O out of the timing-critical control path.
  """

  def __init__(self, log_dir=None):
    self.sm = messaging.SubMaster(
      ["carState", "controlsState", "carControl", "longitudinalPlan",
       "driverMonitoringState"],
      poll=["controlsState"],
    )
    self.sendcan_sock = messaging.sub_sock("sendcan")
    self.can_sock = messaging.sub_sock("can")
    dbc_name = DBC[CAR.EQUINOX_NR]["pt"]
    self.sendcan_parser = CANParser(
      dbc_name,
      [
        ("GAS_COMMAND", "GAS_COMMAND"),
        ("GAS_COMMAND2", "GAS_COMMAND"),
        ("ENABLE", "GAS_COMMAND"),
        ("COUNTER_PEDAL", "GAS_COMMAND"),
        ("CHECKSUM_PEDAL", "GAS_COMMAND"),
      ],
      [("GAS_COMMAND", 0)],
      CanBus.POWERTRAIN,
    )
    # Panda marks a successfully transmitted frame as bus + 0x80. Frames
    # rejected by the safety hook use bus + 0xC0 and are tracked separately.
    self.panda_can_parser = CANParser(
      dbc_name,
      [
        ("GAS_COMMAND", "GAS_COMMAND"),
        ("GAS_COMMAND2", "GAS_COMMAND"),
        ("ENABLE", "GAS_COMMAND"),
        ("COUNTER_PEDAL", "GAS_COMMAND"),
        ("CHECKSUM_PEDAL", "GAS_COMMAND"),
      ],
      [("GAS_COMMAND", 0)],
      CanBus.POWERTRAIN + 0x80,
    )
    car_params_bytes = Params().get("CarParams", block=True)
    self.enable_gas_interceptor = bool(
      car.CarParams.from_bytes(car_params_bytes).enableGasInterceptor
    )

    requested_dir = log_dir or os.getenv("PEDAL_RECOVERY_LOG_DIR", DEFAULT_LOG_DIR)
    self.log_dir = requested_dir if os.path.isdir(os.path.dirname(requested_dir)) else FALLBACK_LOG_DIR
    self.pre_event = deque(maxlen=PRE_EVENT_SAMPLES)
    self.event_samples = []
    self.recording = False
    self.post_event_deadline = 0.0
    self.last_trigger_signal = False
    self.event_sequence = 0
    self.last_event_started_at = -MIN_EVENT_INTERVAL_SECONDS
    self.latest_sendcan = {
      "gasCommand": 0.0,
      "gasCommand2": 0.0,
      "enable": False,
      "counter": -1,
      "checksum": -1,
      "checksumValid": False,
      "monoTime": 0.0,
    }
    self.latest_panda_can = {
      "gasCommand": 0.0,
      "gasCommand2": 0.0,
      "enable": False,
      "counter": -1,
      "checksum": -1,
      "checksumValid": False,
      "accepted": False,
      "rejected": False,
      "monoTime": 0.0,
    }

  def _update_sendcan(self):
    for raw in messaging.drain_sock_raw(self.sendcan_sock):
      updated = self.sendcan_parser.update_string(raw, sendcan=True)
      if 512 in updated:
        values = self.sendcan_parser.vl["GAS_COMMAND"]
        checksum_valid = False
        event = messaging.log_from_bytes(raw)
        for can_message in event.sendcan:
          if can_message.address == 512:
            data = bytes(can_message.dat)
            checksum_valid = len(data) == 6 and crc8_pedal(data[:-1]) == data[-1]
            break
        self.latest_sendcan = {
          "gasCommand": max(0.0, float(values["GAS_COMMAND"]) / 255.0),
          "gasCommand2": max(0.0, float(values["GAS_COMMAND2"]) / 255.0),
          "enable": bool(values["ENABLE"]),
          "counter": int(values["COUNTER_PEDAL"]),
          "checksum": int(values["CHECKSUM_PEDAL"]),
          "checksumValid": checksum_valid,
          "monoTime": sec_since_boot(),
        }

  def _update_panda_can(self):
    for raw in messaging.drain_sock_raw(self.can_sock):
      event = messaging.log_from_bytes(raw)
      rejected = any(
        can_message.address == 512 and 0xC0 <= can_message.src < 0x100
        for can_message in event.can
      )
      updated = self.panda_can_parser.update_string(raw)
      if 512 in updated:
        values = self.panda_can_parser.vl["GAS_COMMAND"]
        checksum_valid = False
        for can_message in event.can:
          if can_message.address == 512 and 0x80 <= can_message.src < 0xC0:
            data = bytes(can_message.dat)
            checksum_valid = len(data) == 6 and crc8_pedal(data[:-1]) == data[-1]
            break
        self.latest_panda_can = {
          "gasCommand": max(0.0, float(values["GAS_COMMAND"]) / 255.0),
          "gasCommand2": max(0.0, float(values["GAS_COMMAND2"]) / 255.0),
          "enable": bool(values["ENABLE"]),
          "counter": int(values["COUNTER_PEDAL"]),
          "checksum": int(values["CHECKSUM_PEDAL"]),
          "checksumValid": checksum_valid,
          "accepted": True,
          "rejected": False,
          "monoTime": sec_since_boot(),
        }
      elif rejected:
        self.latest_panda_can["rejected"] = True
        self.latest_panda_can["accepted"] = False
        self.latest_panda_can["monoTime"] = sec_since_boot()

  def _snapshot(self, now):
    car_state = self.sm["carState"]
    controls = self.sm["controlsState"]
    car_control = self.sm["carControl"]
    plan = self.sm["longitudinalPlan"]
    driver_monitoring = self.sm["driverMonitoringState"]
    speeds = plan.speeds
    future_speed = float(speeds[-1]) if len(speeds) else float(car_state.vEgo)
    speed_error = float(controls.vPid - car_state.vEgo)
    future_speed_error = float(future_speed - car_state.vEgo)
    plan_age = now - self.sm.logMonoTime["longitudinalPlan"] / 1e9 \
      if self.sm.logMonoTime["longitudinalPlan"] else -1.0
    sendcan_age = now - self.latest_sendcan["monoTime"] \
      if self.latest_sendcan["monoTime"] else -1.0
    panda_can_age = now - self.latest_panda_can["monoTime"] \
      if self.latest_panda_can["monoTime"] else -1.0

    return {
      "type": "sample",
      "monoTime": round(now, 6),
      "car": {
        "vEgo": round(float(car_state.vEgo), 5),
        "aEgo": round(float(car_state.aEgo), 5),
        "gas": round(float(car_state.gas), 5),
        "gasPressed": bool(car_state.gasPressed),
        "brake": round(float(car_state.brake), 5),
        "brakePressed": bool(car_state.brakePressed),
        "standstill": bool(car_state.standstill),
        "adaptiveCruise": bool(car_state.adaptiveCruise),
        "cruiseEnabled": bool(car_state.cruiseState.enabled),
      },
      "controls": {
        "enabled": bool(controls.enabled),
        "active": bool(controls.active),
        "longControlState": str(controls.longControlState),
        "vPid": round(float(controls.vPid), 5),
        "speedError": round(speed_error, 5),
        "futureSpeedError": round(future_speed_error, 5),
        "recoveryRawAccel": round(float(controls.pedalForceRecoveryRawAccel), 5),
        "recoveryForcedAccel": round(float(controls.pedalForceRecoveryAccel), 5),
        "recoveryActive": bool(controls.pedalForceRecoveryActive),
        "recoveryDuration": round(float(controls.pedalForceRecoveryDuration), 5),
        "recoveryCount": int(controls.pedalForceRecoveryCount),
        "forceDecel": bool(controls.forceDecel),
        "curvDriving": bool(controls.curvDriving),
      },
      "plan": {
        "valid": bool(self.sm.valid["longitudinalPlan"]),
        "speedCount": len(speeds),
        "ageMs": round(plan_age * 1000.0, 3) if plan_age >= 0.0 else -1.0,
        "source": str(plan.longitudinalPlanSource),
        "futureSpeed": round(future_speed, 5),
      },
      "eligibility": {
        "gasInterceptor": self.enable_gas_interceptor,
        "controlsActive": bool(controls.active),
        "adaptiveCruise": bool(car_state.adaptiveCruise),
        "pidState": str(controls.longControlState) == "pid",
        "noBrake": not bool(car_state.brakePressed),
        "noDriverGas": not bool(car_state.gasPressed),
        "notStandstill": not bool(car_state.standstill),
        "aboveMinSpeed": float(car_state.vEgo) > V_CRUISE_ENABLE_MIN * CV.KPH_TO_MS,
        "driverAware": float(driver_monitoring.awarenessStatus) >= 0.0,
        "notForceDecel": not bool(controls.forceDecel),
        "notCurve": not bool(controls.curvDriving),
        "clearRoadPlan": str(plan.longitudinalPlanSource) == "cruise",
        "fullPlan": len(speeds) == CONTROL_N,
        "planFresh": 0.0 <= plan_age <= 0.25,
        "speedDemand": speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and
                       future_speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR,
      },
      "carControl": {
        "enabled": bool(car_control.enabled),
        "longActive": bool(car_control.longActive),
        "requestedAccel": round(float(car_control.actuators.accel), 5),
        "outputAccel": round(float(car_control.actuatorsOutput.accel), 5),
        "outputGas": round(float(car_control.actuatorsOutput.gas), 5),
      },
      "sendcan": {
        "gasCommand": round(float(self.latest_sendcan["gasCommand"]), 5),
        "gasCommand2": round(float(self.latest_sendcan["gasCommand2"]), 5),
        "enable": bool(self.latest_sendcan["enable"]),
        "counter": int(self.latest_sendcan["counter"]),
        "checksum": int(self.latest_sendcan["checksum"]),
        "checksumValid": bool(self.latest_sendcan["checksumValid"]),
        "ageMs": round(sendcan_age * 1000.0, 3) if sendcan_age >= 0.0 else -1.0,
      },
      "pandaCan": {
        "gasCommand": round(float(self.latest_panda_can["gasCommand"]), 5),
        "gasCommand2": round(float(self.latest_panda_can["gasCommand2"]), 5),
        "enable": bool(self.latest_panda_can["enable"]),
        "counter": int(self.latest_panda_can["counter"]),
        "checksum": int(self.latest_panda_can["checksum"]),
        "checksumValid": bool(self.latest_panda_can["checksumValid"]),
        "accepted": bool(self.latest_panda_can["accepted"]),
        "rejected": bool(self.latest_panda_can["rejected"]),
        "ageMs": round(panda_can_age * 1000.0, 3) if panda_can_age >= 0.0 else -1.0,
      },
    }

  @staticmethod
  def _is_trigger(snapshot):
    controls = snapshot["controls"]
    car = snapshot["car"]
    plan = snapshot["plan"]
    return recovery_log_trigger(
      controls["recoveryActive"], controls["active"], car["adaptiveCruise"],
      car["brakePressed"], car["gasPressed"], car["standstill"],
      plan["valid"], plan["ageMs"], controls["speedError"],
      controls["futureSpeedError"], controls["recoveryRawAccel"],
    )

  def _start_event(self, snapshot, now):
    self.recording = True
    self.event_samples = list(self.pre_event)
    self.event_samples.append(snapshot)
    self.post_event_deadline = now + POST_EVENT_SECONDS
    self.event_sequence += 1
    self.last_event_started_at = now
    cloudlog.warning(
      f"Pedal recovery event {self.event_sequence} triggered; "
      f"capturing {PRE_EVENT_SECONDS:.0f}s before and {POST_EVENT_SECONDS:.0f}s after"
    )

  def _write_event(self):
    os.makedirs(self.log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"pedal_recovery_{stamp}_{self.event_sequence:04d}.jsonl"
    output_path = os.path.join(self.log_dir, filename)
    temp_path = output_path + ".tmp"
    metadata = {
      "type": "metadata",
      "formatVersion": 1,
      "preEventSeconds": PRE_EVENT_SECONDS,
      "postEventSeconds": POST_EVENT_SECONDS,
      "minimumEventIntervalSeconds": MIN_EVENT_INTERVAL_SECONDS,
      "sampleCount": len(self.event_samples),
    }
    with open(temp_path, "w", encoding="utf8") as output_file:
      output_file.write(json.dumps(metadata, ensure_ascii=False) + "\n")
      for sample in self.event_samples:
        output_file.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")
    os.replace(temp_path, output_path)
    cloudlog.warning(f"Pedal recovery event saved: {output_path}")
    print(f"Pedal recovery event saved: {output_path}")
    self.event_samples = []
    self.recording = False

  def run(self):
    try:
      os.nice(10)
    except OSError:
      pass
    cloudlog.warning("Starting read-only pedal recovery event recorder")
    try:
      while True:
        self.sm.update(1000)
        if not self.sm.updated["controlsState"]:
          continue
        # Keep 20 ms diagnostic resolution (enough for the 40 ms acceptance
        # limit) while halving Python/JSON work on resource-constrained EON.
        if self.sm.frame % 2:
          continue
        self._update_sendcan()
        self._update_panda_can()
        now = sec_since_boot()
        snapshot = self._snapshot(now)
        trigger_signal = self._is_trigger(snapshot)
        trigger_edge = trigger_signal and not self.last_trigger_signal

        cooldown_elapsed = now - self.last_event_started_at >= MIN_EVENT_INTERVAL_SECONDS
        if trigger_edge and not self.recording and cooldown_elapsed:
          self._start_event(snapshot, now)
        elif self.recording:
          self.event_samples.append(snapshot)

        self.pre_event.append(snapshot)
        self.last_trigger_signal = trigger_signal

        if self.recording and now >= self.post_event_deadline:
          self._write_event()
    except KeyboardInterrupt:
      if self.recording and self.event_samples:
        self._write_event()
      raise


def main():
  PedalRecoveryRecorder().run()


if __name__ == "__main__":
  main()
