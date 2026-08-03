#!/usr/bin/env python3
import argparse
import json
import os
import time

from common.params import Params
from tools.equinox_sim.command_state import load_command_state, save_command_state


STATUS_FILE = "/tmp/equinox_sim/status.json"
STATUS_STALE_SECONDS = 3.0
BOOL_PARAMS = {
  "fault": "EquinoxSimAccelZero",
  "brake": "EquinoxSimBrakePressed",
  "gas": "EquinoxSimGasPressed",
  "ignition": "EquinoxSimIgnition",
}


def parse_on_off(value):
  normalized = value.lower()
  if normalized not in ("on", "off"):
    raise argparse.ArgumentTypeError("use 'on' or 'off'")
  return normalized == "on"


def print_status():
  if not os.path.exists(STATUS_FILE):
    raise SystemExit("No simulator status. Start tools/equinox_sim/launch.sh first.")
  status_age = time.time() - os.path.getmtime(STATUS_FILE)
  if status_age > STATUS_STALE_SECONDS:
    raise SystemExit(
      "Simulator status is stale "
      f"({status_age:.1f}s old). equinoxcan is not running; start the simulator first."
    )
  with open(STATUS_FILE, encoding="utf8") as status_file:
    print(json.dumps(json.load(status_file), ensure_ascii=False, indent=2))


def main():
  parser = argparse.ArgumentParser(description="Control the Equinox virtual Panda simulator")
  subparsers = parser.add_subparsers(dest="command", required=True)
  subparsers.add_parser("status", help="show live simulator state")

  target_parser = subparsers.add_parser("target", help="set target cruise speed")
  target_parser.add_argument("kph", type=float)

  for name in BOOL_PARAMS:
    bool_parser = subparsers.add_parser(name, help=f"turn simulated {name} on or off")
    bool_parser.add_argument("state", type=parse_on_off)

  recovery_parser = subparsers.add_parser(
    "recovery", help="run one force-recovery cycle for the injected accel-zero fault"
  )
  recovery_parser.add_argument("state", type=parse_on_off)

  production_parser = subparsers.add_parser(
    "production", help="hold accel-zero and use the unmodified production recovery gates"
  )
  production_parser.add_argument("state", type=parse_on_off)

  subparsers.add_parser("reset", help="reset speed, distance and controls")
  subparsers.add_parser("engage", help="request a virtual SET button pulse")
  args = parser.parse_args()

  if args.command == "status":
    print_status()
    return

  params = Params()
  command_state = load_command_state()
  if args.command == "target":
    target = min(145.0, max(20.0, args.kph))
    params.put("EquinoxSimTargetSpeedKph", f"{target:.1f}")
    command_state["targetSpeedKph"] = target
    save_command_state(command_state)
    print(f"target speed: {target:.1f} km/h")
  elif args.command in BOOL_PARAMS:
    params.put_bool(BOOL_PARAMS[args.command], args.state)
    state_keys = {
      "fault": "faultMode",
      "brake": "brakePressed",
      "gas": "gasPressed",
      "ignition": "ignition",
    }
    command_state[state_keys[args.command]] = int(args.state) if args.command == "fault" else args.state
    save_command_state(command_state)
    print(f"{args.command}: {'on' if args.state else 'off'}")
  elif args.command == "recovery":
    # EquinoxSimAccelZero is intentionally a four-state test control:
    #   0: no injected fault (normal production recovery behavior)
    #   1: accel=0 fault held, recovery blocked
    #   2: one-shot recovery requested; controlsd clears the fault after success
    #   3: persistent fault with unmodified production recovery gates
    raw_mode = params.get("EquinoxSimAccelZero", encoding="utf8")
    try:
      current_mode = int(raw_mode) if raw_mode is not None else 0
    except ValueError:
      current_mode = 0
    next_mode = 2 if args.state else (1 if current_mode in (1, 2) else 0)
    params.put("EquinoxSimAccelZero", str(next_mode))
    command_state["faultMode"] = next_mode
    save_command_state(command_state)
    print(
      "fault: on, recovery: " + ("on" if args.state else "off")
      if next_mode else "fault: off, recovery: normal"
    )
  elif args.command == "production":
    next_mode = 3 if args.state else 0
    params.put("EquinoxSimAccelZero", str(next_mode))
    command_state["faultMode"] = next_mode
    save_command_state(command_state)
    print("production-fidelity fault: " + ("on" if args.state else "off"))
  elif args.command == "reset":
    command_state["resetToken"] = int(command_state.get("resetToken", 0)) + 1
    save_command_state(command_state)
    print("simulator reset requested")
  elif args.command == "engage":
    command_state["engageToken"] = int(command_state.get("engageToken", 0)) + 1
    save_command_state(command_state)
    print("virtual cruise engagement requested")


if __name__ == "__main__":
  main()
