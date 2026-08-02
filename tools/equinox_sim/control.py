#!/usr/bin/env python3
import argparse
import json
import os

from common.params import Params


STATUS_FILE = "/tmp/equinox_sim/status.json"
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

  subparsers.add_parser("reset", help="reset speed, distance and controls")
  subparsers.add_parser("engage", help="request a virtual SET button pulse")
  args = parser.parse_args()

  if args.command == "status":
    print_status()
    return

  params = Params()
  if args.command == "target":
    target = min(145.0, max(20.0, args.kph))
    params.put("EquinoxSimTargetSpeedKph", f"{target:.1f}")
    print(f"target speed: {target:.1f} km/h")
  elif args.command in BOOL_PARAMS:
    params.put_bool(BOOL_PARAMS[args.command], args.state)
    print(f"{args.command}: {'on' if args.state else 'off'}")
  elif args.command == "reset":
    params.put_bool("EquinoxSimReset", True)
    print("simulator reset requested")
  elif args.command == "engage":
    params.put_bool("EquinoxSimEngage", True)
    print("virtual cruise engagement requested")


if __name__ == "__main__":
  main()

