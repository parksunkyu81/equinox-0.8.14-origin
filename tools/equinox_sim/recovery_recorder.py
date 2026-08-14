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
# Traffic-light launches are normally farther apart than this, while the
# cooldown still prevents repeated files from one continuous activation.
MIN_EVENT_INTERVAL_SECONDS = 20.0
EXPECTED_CONTROL_HZ = 50
PRE_EVENT_SAMPLES = int(PRE_EVENT_SECONDS * EXPECTED_CONTROL_HZ) + 50
POST_ARBITRATION_ZERO_SECONDS = 0.16
# The recorder samples every other controlsState frame (25 Hz).
POST_ARBITRATION_ZERO_SAMPLES = max(2, int(round(
  POST_ARBITRATION_ZERO_SECONDS * EXPECTED_CONTROL_HZ / 2.0)))
DEFAULT_LOG_DIR = "/data/media/0/pedal_recovery_logs"
FALLBACK_LOG_DIR = "/tmp/pedal_recovery_logs"


class PedalRecoveryRecorder:
  """Read-only event recorder for pedal recovery and stop-boost diagnosis.

  Samples stay in memory during normal driving. A recovery candidate or a
  confirmed stopped-lead launch flushes the preceding five seconds plus the
  following ten seconds to JSONL, keeping disk I/O out of the control path.
  """

  def __init__(self, log_dir=None):
    self.sm = messaging.SubMaster(
      ["carState", "controlsState", "carControl", "longitudinalPlan",
       "driverMonitoringState", "dynamicFollowData", "radarState"],
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
    self.post_arbitration_zero_samples = 0
    self.current_trigger_reason = ""
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
    dynamic_follow = self.sm["dynamicFollowData"]
    lead = self.sm["radarState"].leadOne
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
        "recoveryMode": int(controls.pedalForceRecoveryMode),
        "leadAssistActive": bool(controls.pedalLeadAssistActive),
        "leadAssistCandidateDuration": round(float(controls.pedalLeadAssistCandidateDuration), 5),
        "leadAssistFilteredVRel": round(float(controls.pedalLeadAssistFilteredVRel), 5),
        "leadAssistActualTR": round(float(controls.pedalLeadAssistActualTR), 5),
        "leadAssistDesiredTR": round(float(controls.pedalLeadAssistDesiredTR), 5),
        "leadAssistTrMargin": round(float(controls.pedalLeadAssistTrMargin), 5),
        "leadAssistCancelReason": int(controls.pedalLeadAssistCancelReason),
        "leadAssistCount": int(controls.pedalLeadAssistCount),
        "leadAssistPedalTarget": round(float(controls.pedalLeadAssistPedalTarget), 5),
        "movingGapActive": bool(controls.movingGapCatchupActive),
        "movingGapCandidateDuration": round(
          float(controls.movingGapCatchupCandidateDuration), 5),
        "movingGapLeadStableDuration": round(
          float(controls.movingGapCatchupLeadStableDuration), 5),
        "movingGapFilteredVRel": round(float(controls.movingGapCatchupFilteredVRel), 5),
        "movingGapDesiredGap": round(float(controls.movingGapCatchupDesiredGap), 5),
        "movingGapDistanceMargin": round(
          float(controls.movingGapCatchupDistanceMargin), 5),
        "movingGapEnterMargin": round(float(controls.movingGapCatchupEnterMargin), 5),
        "movingGapExitMargin": round(float(controls.movingGapCatchupExitMargin), 5),
        "movingGapTargetAccel": round(float(controls.movingGapCatchupTargetAccel), 5),
        "movingGapFinalAccel": round(float(controls.movingGapCatchupFinalAccel), 5),
        "movingGapPedalTarget": round(float(controls.movingGapCatchupPedalTarget), 5),
        "movingGapCancelReason": int(controls.movingGapCatchupCancelReason),
        "movingGapCount": int(controls.movingGapCatchupCount),
        "movingGapLeadJump": bool(controls.movingGapCatchupLeadJump),
        "forceDecel": bool(controls.forceDecel),
        "curvDriving": bool(controls.curvDriving),
        "stopAccelBoostActive": bool(controls.stopAccelBoostActive),
        "stopAccelBoostApplied": bool(controls.stopAccelBoostApplied),
        "stopAccelBoostRawAccel": round(float(controls.stopAccelBoostRawAccel), 5),
        "stopAccelBoostFinalAccel": round(float(controls.stopAccelBoostFinalAccel), 5),
        "stopAccelBoostFactor": round(float(controls.stopAccelBoostFactor), 3),
        "driverLaunchHandoffActive": bool(controls.driverLaunchHandoffActive),
        "driverLaunchHandoffShadowAccel": round(
          float(controls.driverLaunchHandoffShadowAccel), 5),
        "stopAccelBoostFloorAccel": round(float(controls.stopAccelBoostFloorAccel), 5),
        "stopAccelBoostHillExtraAccel": round(
          float(controls.stopAccelBoostHillExtraAccel), 5),
        "drivingStyleGain": round(float(controls.drivingStyleAIGain), 5),
        "drivingStyleTrOffset": round(float(controls.drivingStyleAITrOffset), 5),
        "drivingStyleBrakeEvents": int(controls.drivingStyleAIBrakeEvents),
      },
      "plan": {
        "valid": bool(self.sm.valid["longitudinalPlan"]),
        "speedCount": len(speeds),
        "ageMs": round(plan_age * 1000.0, 3) if plan_age >= 0.0 else -1.0,
        "source": str(plan.longitudinalPlanSource),
        "futureSpeed": round(future_speed, 5),
      },
      "stopAccelBoost": {
        "enabled": bool(dynamic_follow.stopAccelBoostEnabled),
        "launchState": int(dynamic_follow.leadLaunchState),
        "requestActive": bool(dynamic_follow.leadCatchupActive),
        "catchupFactor": round(float(dynamic_follow.catchupFactor), 5),
        "leadStatus": bool(lead.status),
        "leadSpeed": round(float(dynamic_follow.leadSpeed), 5),
        "leadRelativeSpeed": round(float(dynamic_follow.leadRelativeSpeed), 5),
        "leadDistance": round(float(dynamic_follow.leadDistance), 5),
        "egoSpeed": round(float(dynamic_follow.egoSpeed), 5),
        "followingDistanceProfile": str(dynamic_follow.followingDistanceProfile),
        "followingDistanceOffset": round(float(dynamic_follow.followingDistanceOffset), 5),
        "rawTR": round(float(dynamic_follow.rawTR), 5),
        "learnedTROffset": round(float(dynamic_follow.learnedTROffset), 5),
        "profileChanging": bool(dynamic_follow.followingDistanceProfileChanging),
        "baseTR": round(float(dynamic_follow.baseTR), 5),
        "finalTR": round(float(dynamic_follow.mpcTR), 5),
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
        "noFcw": not bool(plan.fcw),
        "canValid": bool(car_state.canValid),
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

  def _is_trigger(self, snapshot):
    controls = snapshot["controls"]
    car = snapshot["car"]
    plan = snapshot["plan"]
    recovery_trigger = recovery_log_trigger(
      controls["recoveryActive"], controls["active"], car["adaptiveCruise"],
      car["brakePressed"], car["gasPressed"], car["standstill"],
      plan["valid"], plan["ageMs"], controls["speedError"],
      controls["futureSpeedError"], controls["recoveryRawAccel"],
    )
    # Catch the second class of stalls: planner/controlsd requests positive
    # acceleration, but final pedal arbitration (for example predictive
    # coasting) outputs zero. Require 160 ms so a single asynchronous CAN or
    # state-transition sample cannot create an event.
    post_arbitration_zero = bool(
      controls["active"] and car["adaptiveCruise"] and
      not car["brakePressed"] and not car["gasPressed"] and
      not car["standstill"] and car["vEgo"] > V_CRUISE_ENABLE_MIN * CV.KPH_TO_MS and
      snapshot["carControl"]["requestedAccel"] >= 0.10 and
      snapshot["carControl"]["outputGas"] <= 0.001 and
      snapshot["eligibility"]["canValid"] and plan["valid"] and
      0.0 <= plan["ageMs"] <= 250.0)
    self.post_arbitration_zero_samples = (
      self.post_arbitration_zero_samples + 1 if post_arbitration_zero else 0)
    post_arbitration_trigger = (
      self.post_arbitration_zero_samples >= POST_ARBITRATION_ZERO_SAMPLES)
    return (recovery_trigger or controls["stopAccelBoostActive"] or
            controls["driverLaunchHandoffActive"] or post_arbitration_trigger)

  def _start_event(self, snapshot, now):
    self.recording = True
    self.event_samples = list(self.pre_event)
    self.event_samples.append(snapshot)
    self.post_event_deadline = now + POST_EVENT_SECONDS
    self.event_sequence += 1
    self.last_event_started_at = now
    if snapshot["controls"]["driverLaunchHandoffActive"]:
      self.current_trigger_reason = "driver_launch_handoff"
    elif snapshot["controls"]["stopAccelBoostActive"]:
      self.current_trigger_reason = "stop_accel_boost"
    elif self.post_arbitration_zero_samples >= POST_ARBITRATION_ZERO_SAMPLES:
      self.current_trigger_reason = "post_arbitration_zero"
    else:
      self.current_trigger_reason = "pedal_force_recovery"
    cloudlog.warning(
      f"Pedal event {self.event_sequence} ({self.current_trigger_reason}) triggered; "
      f"capturing {PRE_EVENT_SECONDS:.0f}s before and {POST_EVENT_SECONDS:.0f}s after"
    )

  def _write_event(self):
    os.makedirs(self.log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{self.current_trigger_reason}_{stamp}_{self.event_sequence:04d}.jsonl"
    output_path = os.path.join(self.log_dir, filename)
    temp_path = output_path + ".tmp"
    metadata = {
      "type": "metadata",
      "formatVersion": 1,
      "preEventSeconds": PRE_EVENT_SECONDS,
      "postEventSeconds": POST_EVENT_SECONDS,
      "minimumEventIntervalSeconds": MIN_EVENT_INTERVAL_SECONDS,
      "sampleCount": len(self.event_samples),
      "triggerReason": self.current_trigger_reason,
    }
    with open(temp_path, "w", encoding="utf8") as output_file:
      output_file.write(json.dumps(metadata, ensure_ascii=False) + "\n")
      for sample in self.event_samples:
        output_file.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")
    os.replace(temp_path, output_path)
    cloudlog.warning(f"Pedal event saved: {output_path}")
    print(f"Pedal event saved: {output_path}")
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
