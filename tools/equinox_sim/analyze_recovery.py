#!/usr/bin/env python3
import argparse
import json

from selfdrive.controls.lib.pedal_force_recovery import (
  PEDAL_FORCE_RECOVERY_ACCEL,
  PEDAL_FORCE_RECOVERY_PEDAL_FLOOR,
  PEDAL_FORCE_RECOVERY_SPEED_ERROR,
)


RECOVERY_REQUIRED_GATES = (
  "gasInterceptor", "controlsActive", "adaptiveCruise", "pidState",
  "noBrake", "noDriverGas", "notStandstill", "aboveMinSpeed",
  "driverAware", "notForceDecel", "notCurve", "noFcw", "canValid",
  "fullPlan", "planFresh", "speedDemand",
)


def _first_sample(samples, predicate, start_time=None, end_time=None):
  for sample in samples:
    mono_time = float(sample["monoTime"])
    if start_time is not None and mono_time < start_time:
      continue
    if end_time is not None and mono_time > end_time:
      break
    if predicate(sample):
      return sample
  return None


def _latency_ms(start, end):
  if start is None or end is None:
    return None
  return round((float(end["monoTime"]) - float(start["monoTime"])) * 1000.0, 2)


def analyze_samples(samples):
  samples = sorted((s for s in samples if s.get("type") == "sample"),
                   key=lambda sample: sample["monoTime"])
  if not samples:
    raise ValueError("no recovery samples found")

  zero_demand = _first_sample(samples, lambda s:
    s["controls"]["speedError"] >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and
    s["controls"]["futureSpeedError"] >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and
    -0.001 <= s["controls"]["recoveryRawAccel"] <= 0.001)
  eligibility_at_zero = zero_demand["eligibility"] if zero_demand else {}
  failed_eligibility = sorted(
    name for name in RECOVERY_REQUIRED_GATES if not bool(eligibility_at_zero.get(name, False))
  )
  candidate = zero_demand if zero_demand is not None and not failed_eligibility else None
  recovery = _first_sample(samples, lambda s: s["controls"]["recoveryActive"])
  recovery_time = float(recovery["monoTime"]) if recovery else None
  recovery_window_end = recovery_time + 2.0 if recovery_time is not None else None

  forced = _first_sample(samples, lambda s:
    s["controls"]["recoveryForcedAccel"] >= PEDAL_FORCE_RECOVERY_ACCEL,
    recovery_time, recovery_window_end)
  controller_gas = _first_sample(samples, lambda s:
    s["carControl"]["outputGas"] >= PEDAL_FORCE_RECOVERY_PEDAL_FLOOR,
    recovery_time, recovery_window_end)
  sendcan_gas = _first_sample(samples, lambda s:
    s["sendcan"]["enable"] and s["sendcan"]["gasCommand"] >= PEDAL_FORCE_RECOVERY_PEDAL_FLOOR and
    0.0 <= s["sendcan"].get("ageMs", -1.0) <= 50.0,
    recovery_time, recovery_window_end)
  panda_can_gas = _first_sample(samples, lambda s:
    s["pandaCan"]["accepted"] and not s["pandaCan"]["rejected"] and
    s["pandaCan"]["enable"] and
    s["pandaCan"]["gasCommand"] >= PEDAL_FORCE_RECOVERY_PEDAL_FLOOR and
    0.0 <= s["pandaCan"].get("ageMs", -1.0) <= 50.0,
    recovery_time, recovery_window_end)
  positive_accel = _first_sample(samples, lambda s: s["car"]["aEgo"] > 0.05,
                                 recovery_time, recovery_window_end)
  brake = _first_sample(samples, lambda s: s["car"]["brakePressed"], recovery_time)
  brake_time = float(brake["monoTime"]) if brake else None
  gas_zero_after_brake = _first_sample(samples, lambda s:
    not s["sendcan"]["enable"] or s["sendcan"]["gasCommand"] <= 0.001,
    brake_time, brake_time + 0.2 if brake_time is not None else None)

  recovery_counts = [int(s["controls"]["recoveryCount"]) for s in samples]
  continuous_recovery = []
  if recovery is not None:
    started = False
    for sample in samples:
      if sample is recovery:
        started = True
      if started and not sample["controls"]["recoveryActive"]:
        break
      if started:
        continuous_recovery.append(int(sample["controls"]["recoveryCount"]))
  continuous_count_delta = max(continuous_recovery) - min(continuous_recovery) \
    if continuous_recovery else None
  checksum_ok = bool(sendcan_gas and sendcan_gas["sendcan"]["checksumValid"])
  panda_checksum_ok = bool(panda_can_gas and panda_can_gas["pandaCan"]["checksumValid"])
  sendcan_latency_ms = _latency_ms(recovery, sendcan_gas)
  panda_can_latency_ms = _latency_ms(recovery, panda_can_gas)
  report = {
    "zeroAccelWithSpeedDemandDetected": zero_demand is not None,
    "candidateDetected": candidate is not None,
    "eligibilityAtFirstZeroDemand": eligibility_at_zero or None,
    "failedEligibilityGates": failed_eligibility,
    "recoveryTriggered": recovery is not None,
    "forcedAccelOk": forced is not None,
    "controllerGasOk": controller_gas is not None,
    "sendcanGasOk": sendcan_gas is not None,
    "sendcanChecksumOk": checksum_ok,
    "sendcanLatencyWithin40Ms": sendcan_latency_ms is not None and sendcan_latency_ms <= 40.0,
    "pandaCanGasOk": panda_can_gas is not None,
    "pandaCanChecksumOk": panda_checksum_ok,
    "pandaCanLatencyWithin40Ms": panda_can_latency_ms is not None and panda_can_latency_ms <= 40.0,
    "vehicleAccelerationObserved": positive_accel is not None,
    "brakeObserved": brake is not None,
    "brakeCancelObserved": gas_zero_after_brake is not None if brake else None,
    "candidateToRecoveryMs": _latency_ms(candidate, recovery),
    "recoveryToControllerGasMs": _latency_ms(recovery, controller_gas),
    "recoveryToSendcanMs": sendcan_latency_ms,
    "recoveryToPandaCanMs": panda_can_latency_ms,
    "recoveryToPositiveAccelMs": _latency_ms(recovery, positive_accel),
    "brakeToGasZeroMs": _latency_ms(brake, gas_zero_after_brake),
    "recoveryCountStart": recovery_counts[0],
    "recoveryCountEnd": recovery_counts[-1],
    "recoveryCountDelta": recovery_counts[-1] - recovery_counts[0],
    "continuousRecoveryCountDelta": continuous_count_delta,
    "continuousRecoverySingleActivation": continuous_count_delta == 0
      if continuous_count_delta is not None else None,
  }
  report["softwarePathPassed"] = bool(
    report["candidateDetected"] and report["recoveryTriggered"] and
    report["forcedAccelOk"] and report["controllerGasOk"]
  )
  report["requestedCanPathPassed"] = bool(
    report["softwarePathPassed"] and report["sendcanGasOk"] and
    report["sendcanChecksumOk"] and report["sendcanLatencyWithin40Ms"]
  )
  report["pandaCanPathPassed"] = bool(
    report["requestedCanPathPassed"] and report["pandaCanGasOk"] and
    report["pandaCanChecksumOk"] and report["pandaCanLatencyWithin40Ms"]
  )
  report["vehicleResponsePassed"] = bool(report["vehicleAccelerationObserved"])
  report["endToEndPassed"] = bool(
    report["pandaCanPathPassed"] and report["vehicleResponsePassed"]
  )
  return report


def load_samples(path):
  with open(path, encoding="utf8") as input_file:
    return [json.loads(line) for line in input_file if line.strip()]


def main():
  parser = argparse.ArgumentParser(description="Analyze an Equinox pedal recovery JSONL event")
  parser.add_argument("event_file")
  args = parser.parse_args()
  report = analyze_samples(load_samples(args.event_file))
  print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
  main()
