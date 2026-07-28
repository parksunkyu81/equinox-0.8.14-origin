#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import sys


DEFAULT_BLACKBOX_PATH = "/data/log/gm_lkas"


def parse_int(value, default=0):
  try:
    return int(value)
  except (TypeError, ValueError):
    return default


def parse_float(value, default=0.0):
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def parse_bool(value):
  return str(value).strip().lower() in ("1", "true", "yes", "on")


def find_blackbox_files(path):
  if os.path.isfile(path):
    return [path]
  if not os.path.isdir(path):
    return []

  patterns = [
    os.path.join(path, "gm_lkas_can_*.csv"),
    os.path.join(path, "gm_lkas_incident_*.csv"),
    os.path.join(path, "blackbox", "gm_lkas_can_*.csv"),
    os.path.join(path, "incidents", "gm_lkas_incident_*.csv"),
  ]
  files = []
  for pattern in patterns:
    files.extend(glob.glob(pattern))
  return sorted(set(files))


def load_blackbox_rows(path):
  files = find_blackbox_files(path)
  rows = []
  for filename in files:
    with open(filename, newline="", encoding="utf-8") as log_file:
      for row_index, row in enumerate(csv.DictReader(log_file), start=2):
        row["_filename"] = filename
        row["_row_index"] = row_index
        rows.append(row)
  return files, rows


def analyze_blackbox_rows(rows):
  errors = []
  warnings = []
  channel_counts = {}
  incident_counts = {}
  pscm_status_counts = {}

  for row in rows:
    location = "{}:{}".format(
      os.path.basename(row.get("_filename", "<memory>")),
      row.get("_row_index", "?"))
    record_type = row.get("record_type", "")

    for reason in filter(None, row.get("incident_reason", "").split("|")):
      incident_counts[reason] = incident_counts.get(reason, 0) + 1
      if reason in ("pscm_status_2_active", "pscm_status_3", "command_gap"):
        errors.append("{} incident {}".format(location, reason))

    if record_type == "can":
      source = row.get("source", "")
      bus = parse_int(row.get("bus"), -1)
      channel = "{}:bus{}".format(source, bus)
      channel_counts[channel] = channel_counts.get(channel, 0) + 1

      if not parse_bool(row.get("checksum_valid")):
        errors.append("{} invalid 0x180 checksum on {}".format(location, channel))
      sequence_value = row.get("counter_sequence_valid", "")
      if sequence_value != "" and not parse_bool(sequence_value):
        errors.append("{} invalid 0x180 counter sequence on {}".format(location, channel))
      if source == "can" and bus == 0:
        errors.append("{} external 0x180 appeared on vehicle bus 0".format(location))

    elif record_type == "state":
      status = parse_int(row.get("pscm_status"))
      pscm_status_counts[status] = pscm_status_counts.get(status, 0) + 1
      command_active = parse_bool(row.get("command_active"))
      if status == 3:
        errors.append("{} PSCM status 3".format(location))
      elif status == 2 and command_active:
        errors.append("{} PSCM status 2 while command active".format(location))
      if parse_float(row.get("command_gap_ms")) > 35.0:
        errors.append("{} command gap {}ms".format(
          location, row.get("command_gap_ms")))

  if not channel_counts:
    errors.append("no raw GM 0x180 CAN records found")
  if not any(channel.startswith("can:bus1") or channel.startswith("can:bus2")
             for channel in channel_counts):
    warnings.append("no stock camera 0x180 traffic observed on bus 1 or 2")
  if not channel_counts.get("can:bus128"):
    warnings.append("no Panda TX loopback 0x180 traffic observed on bus 128")
  return {
    "errors": errors,
    "warnings": warnings,
    "channel_counts": channel_counts,
    "incident_counts": incident_counts,
    "pscm_status_counts": pscm_status_counts,
  }


def main():
  parser = argparse.ArgumentParser(
    description="Validate GM LKAS camera/vehicle/loopback blackbox logs")
  parser.add_argument("path", nargs="?", default=DEFAULT_BLACKBOX_PATH,
                      help="blackbox file or directory")
  args = parser.parse_args()

  files, rows = load_blackbox_rows(args.path)
  if not files or not rows:
    print("NO DATA: no GM LKAS blackbox rows found at {}".format(args.path))
    return 2

  result = analyze_blackbox_rows(rows)
  print("files={} rows={} channels={}".format(
    len(files), len(rows), result["channel_counts"]))
  print("PSCM status counts={}".format(result["pscm_status_counts"]))
  print("incident reasons={}".format(result["incident_counts"]))
  for warning in result["warnings"]:
    print("WARNING: {}".format(warning))
  for error in result["errors"][:50]:
    print("ERROR: {}".format(error))
  if len(result["errors"]) > 50:
    print("ERROR: {} additional errors omitted".format(
      len(result["errors"]) - 50))

  if result["errors"]:
    print("RESULT: FAIL")
    return 1
  print("RESULT: PASS")
  return 0


if __name__ == "__main__":
  sys.exit(main())
