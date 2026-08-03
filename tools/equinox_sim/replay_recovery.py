#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path

# Replay must exercise production gates, never the bench-only injection path.
os.environ["REPLAY"] = "1"
os.environ.pop("EQUINOX_SIMULATOR", None)
os.environ.pop("NOBOARD", None)

from opendbc.can.parser import CANParser  # noqa: E402
from selfdrive.car import crc8_pedal  # noqa: E402
from selfdrive.car.gm.values import CAR, DBC, CanBus  # noqa: E402
from selfdrive.hardware import PC  # noqa: E402
from selfdrive.test.process_replay.compare_logs import save_log  # noqa: E402
from selfdrive.test.process_replay.process_replay import CONFIGS, replay_process  # noqa: E402
from tools.equinox_sim.analyze_recovery import analyze_samples  # noqa: E402
from tools.equinox_sim.recovery_recorder import PedalRecoveryRecorder  # noqa: E402
from tools.lib.logreader import LogReader  # noqa: E402


TRACKED_SERVICES = {
  "carState", "controlsState", "carControl", "longitudinalPlan",
  "driverMonitoringState",
}


class OfflineSubMaster:
  def __init__(self):
    self.data = {}
    self.valid = {}
    self.logMonoTime = {}

  def __getitem__(self, service):
    return self.data[service]

  def update(self, message):
    service = message.which()
    self.data[service] = getattr(message, service)
    self.valid[service] = bool(message.valid)
    self.logMonoTime[service] = int(message.logMonoTime)


def extract_recovery_samples(messages):
  offline_sm = OfflineSubMaster()
  recorder = PedalRecoveryRecorder.__new__(PedalRecoveryRecorder)
  recorder.sm = offline_sm
  recorder.enable_gas_interceptor = False
  recorder.latest_sendcan = {
    "gasCommand": 0.0,
    "gasCommand2": 0.0,
    "enable": False,
    "counter": -1,
    "checksum": -1,
    "checksumValid": False,
    "monoTime": 0.0,
  }
  recorder.latest_panda_can = {
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

  dbc_name = DBC[CAR.EQUINOX_NR]["pt"]
  sendcan_parser = CANParser(
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
  panda_can_parser = CANParser(
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

  samples = []
  for message in sorted(messages, key=lambda msg: msg.logMonoTime):
    service = message.which()
    if service == "carParams":
      recorder.enable_gas_interceptor = bool(message.carParams.enableGasInterceptor)
    elif service == "sendcan":
      updated = sendcan_parser.update_string(message.as_builder().to_bytes(), sendcan=True)
      if 512 in updated:
        values = sendcan_parser.vl["GAS_COMMAND"]
        checksum_valid = False
        for can_message in message.sendcan:
          if can_message.address == 512:
            data = bytes(can_message.dat)
            checksum_valid = len(data) == 6 and crc8_pedal(data[:-1]) == data[-1]
            break
        recorder.latest_sendcan = {
          "gasCommand": max(0.0, float(values["GAS_COMMAND"]) / 255.0),
          "gasCommand2": max(0.0, float(values["GAS_COMMAND2"]) / 255.0),
          "enable": bool(values["ENABLE"]),
          "counter": int(values["COUNTER_PEDAL"]),
          "checksum": int(values["CHECKSUM_PEDAL"]),
          "checksumValid": checksum_valid,
          "monoTime": message.logMonoTime / 1e9,
        }
    elif service == "can":
      serialized = message.as_builder().to_bytes()
      updated = panda_can_parser.update_string(serialized)
      rejected = any(
        can_message.address == 512 and 0xC0 <= can_message.src < 0x100
        for can_message in message.can
      )
      if 512 in updated:
        values = panda_can_parser.vl["GAS_COMMAND"]
        checksum_valid = False
        for can_message in message.can:
          if can_message.address == 512 and 0x80 <= can_message.src < 0xC0:
            data = bytes(can_message.dat)
            checksum_valid = len(data) == 6 and crc8_pedal(data[:-1]) == data[-1]
            break
        recorder.latest_panda_can = {
          "gasCommand": max(0.0, float(values["GAS_COMMAND"]) / 255.0),
          "gasCommand2": max(0.0, float(values["GAS_COMMAND2"]) / 255.0),
          "enable": bool(values["ENABLE"]),
          "counter": int(values["COUNTER_PEDAL"]),
          "checksum": int(values["CHECKSUM_PEDAL"]),
          "checksumValid": checksum_valid,
          "accepted": True,
          "rejected": False,
          "monoTime": message.logMonoTime / 1e9,
        }
      elif rejected:
        recorder.latest_panda_can["rejected"] = True
        recorder.latest_panda_can["accepted"] = False
        recorder.latest_panda_can["monoTime"] = message.logMonoTime / 1e9
    elif service in TRACKED_SERVICES:
      offline_sm.update(message)
      if service == "controlsState" and TRACKED_SERVICES.issubset(offline_sm.data):
        samples.append(recorder._snapshot(message.logMonoTime / 1e9))
  return samples


def write_jsonl(path, samples):
  metadata = {"type": "metadata", "formatVersion": 1, "source": "controlsdReplay",
              "sampleCount": len(samples)}
  with open(path, "w", encoding="utf8") as output_file:
    output_file.write(json.dumps(metadata, ensure_ascii=False) + "\n")
    for sample in samples:
      output_file.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")


def main():
  parser = argparse.ArgumentParser(
    description="Replay an actual rlog through the current controlsd and analyze pedal recovery"
  )
  parser.add_argument("rlog", help="local rlog or rlog.bz2 containing the real accel=0 event")
  parser.add_argument("--fingerprint", default=None,
                      help="optional exact car fingerprint if the log cannot fingerprint itself")
  parser.add_argument("--output-prefix", default=None)
  args = parser.parse_args()
  if not PC:
    raise SystemExit(
      "Refusing to run process replay on a comma device. Copy the rlog to a separate PC clone first."
    )

  input_path = Path(args.rlog)
  prefix = Path(args.output_prefix) if args.output_prefix else input_path.with_suffix("")
  replay_path = str(prefix) + ".controlsd_replay.bz2"
  event_path = str(prefix) + ".recovery.jsonl"

  inputs = list(LogReader(str(input_path)))
  controlsd_config = next(config for config in CONFIGS if config.proc_name == "controlsd")
  outputs = replay_process(controlsd_config, inputs, fingerprint=args.fingerprint)
  produced_services = {message.which() for message in outputs}
  merged = [message for message in inputs if message.which() not in produced_services]
  merged = sorted(merged + outputs, key=lambda message: message.logMonoTime)
  save_log(replay_path, merged)

  samples = extract_recovery_samples(merged)
  write_jsonl(event_path, samples)
  report = analyze_samples(samples) if samples else {
    "error": "replay produced no aligned recovery samples; check required services in the rlog"
  }
  print(json.dumps({
    "replayLog": replay_path,
    "recoverySamples": event_path,
    "report": report,
  }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
  main()
