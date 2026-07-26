#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import statistics
import sys

try:
  from .steer_diagnostics import gm_lkas_checksum
except ImportError:
  from steer_diagnostics import gm_lkas_checksum


DEFAULT_LOG_PATH = "/data/log/gm_lkas"
MIN_COMMAND_INTERVAL_MS = 18.0
MAX_COMMAND_INTERVAL_MS = 35.0
MIN_OBSERVED_LOOPBACK_INTERVAL_MS = 8.0
IDEAL_LOOPBACK_INTERVAL_MS = 18.0
MAX_OBSERVED_LOOPBACK_INTERVAL_MS = 35.0
MIN_VALIDATION_SAMPLES = 20
SESSION_BREAK_S = 1.0


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
  if isinstance(value, bool):
    return value
  if isinstance(value, (int, float)):
    return bool(value)
  normalized = str(value).strip().lower()
  if normalized in ("1", "true", "yes", "on"):
    return True
  if normalized in ("0", "false", "no", "off", ""):
    return False
  return False


def parse_pipe_ints(value):
  if not value:
    return []
  return [parse_int(item) for item in value.split("|")]


def percentile(values, fraction):
  if not values:
    return 0.0
  ordered = sorted(values)
  index = max(0, min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction))))
  return ordered[index]


def find_log_files(path):
  if os.path.isdir(path):
    return sorted(glob.glob(os.path.join(path, "gm_lkas_*.csv")))
  if os.path.isfile(path):
    return [path]
  return []


def load_rows(path):
  rows = []
  files = find_log_files(path)
  for file_index, filename in enumerate(files):
    with open(filename, newline="", encoding="utf-8") as log_file:
      for row_index, row in enumerate(csv.DictReader(log_file), start=2):
        row["_file_index"] = file_index
        row["_filename"] = filename
        row["_row_index"] = row_index
        rows.append(row)
  return files, rows


def same_session(previous_time, current_time):
  delta = current_time - previous_time
  return 0.0 <= delta <= SESSION_BREAK_S


def analyze_rows(rows, min_samples=MIN_VALIDATION_SAMPLES):
  errors = []
  warnings = []
  command_intervals = []
  loopback_intervals = []
  loopback_ack_latencies = []
  command_count = 0
  loopback_count = 0
  paired_loopback_count = 0
  unacked_blocks = 0
  gap_rows = 0
  pscm_temporary_rows = 0
  pscm_permanent_rows = 0
  batched_loopback_rows = 0
  quantized_loopback_intervals = 0
  unpaired_prefix_loopbacks = 0
  invalid_can_rows = 0
  invalid_can_streak = 0
  max_invalid_can_streak = 0
  max_queue_drops = 0
  command_counters_seen = set()
  loopback_counters_seen = set()
  pscm_status_counts = {}
  pending_commands = []

  previous_command_counter = None
  previous_command_time = None
  previous_loopback_counter = None
  previous_loopback_time = None

  for row in rows:
    mono_time = parse_float(row.get("mono_time_s"))
    location = "{}:{}".format(os.path.basename(row.get("_filename", "<memory>")),
                              row.get("_row_index", "?"))

    if row.get("command_block_reason") == "unacked":
      unacked_blocks += 1
    if parse_bool(row.get("gap_fault")):
      gap_rows += 1

    pscm_status = parse_int(row.get("pscm_lkas_status"))
    pscm_status_counts[pscm_status] = pscm_status_counts.get(pscm_status, 0) + 1
    if pscm_status == 2:
      pscm_temporary_rows += 1
    elif pscm_status == 3:
      pscm_permanent_rows += 1

    if parse_bool(row.get("can_valid")):
      invalid_can_streak = 0
    else:
      invalid_can_rows += 1
      invalid_can_streak += 1
      max_invalid_can_streak = max(max_invalid_can_streak, invalid_can_streak)

    max_queue_drops = max(max_queue_drops, parse_int(row.get("queue_drops")))

    counters = parse_pipe_ints(row.get("loopback_counters"))
    torques = parse_pipe_ints(row.get("loopback_torques"))
    actives = parse_pipe_ints(row.get("loopback_actives"))
    checksums = parse_pipe_ints(row.get("loopback_checksums"))
    sample_count = min(len(counters), len(torques), len(actives), len(checksums))
    declared_sample_count = parse_int(row.get("loopback_count"))
    if declared_sample_count != sample_count or len({len(counters), len(torques), len(actives), len(checksums)}) != 1:
      errors.append("{} incomplete loopback fields: declared={} parsed={}/{}/{}/{}".format(
        location, declared_sample_count, len(counters), len(torques), len(actives), len(checksums)))
    if sample_count > 1:
      batched_loopback_rows += 1

    for sample_index in range(sample_count):
      loopback_count += 1
      counter = counters[sample_index] % 4
      loopback_counters_seen.add(counter)
      torque = torques[sample_index]
      active = bool(actives[sample_index])
      checksum = checksums[sample_index] & 0xfff
      expected_checksum = gm_lkas_checksum(active, torque, counter)
      if checksum != expected_checksum:
        errors.append("{} loopback[{}] checksum {} != {}".format(
          location, sample_index, checksum, expected_checksum))

      if previous_loopback_time is not None and same_session(previous_loopback_time, mono_time):
        expected_counter = (previous_loopback_counter + 1) % 4
        if counter != expected_counter:
          errors.append("{} loopback counter {} after {}, expected {}".format(
            location, counter, previous_loopback_counter, expected_counter))

        # Multiple samples in one controls cycle are batched, so their exact
        # bus interval is unavailable. Preserve counter/checksum validation but
        # do not claim a zero millisecond CAN interval.
        if sample_index == 0 and mono_time > previous_loopback_time:
          interval_ms = (mono_time - previous_loopback_time) * 1000.0
          loopback_intervals.append(interval_ms)
          if interval_ms < MIN_OBSERVED_LOOPBACK_INTERVAL_MS:
            errors.append("{} observed loopback interval {:.3f}ms is too short".format(location, interval_ms))
          elif interval_ms < IDEAL_LOOPBACK_INTERVAL_MS:
            quantized_loopback_intervals += 1
          elif interval_ms > MAX_OBSERVED_LOOPBACK_INTERVAL_MS:
            errors.append("{} observed loopback interval {:.3f}ms exceeds gap limit".format(location, interval_ms))

      while pending_commands and not same_session(pending_commands[0]["time"], mono_time):
        expired = pending_commands.pop(0)
        errors.append("{} command counter {} had no loopback before a session break".format(
          expired["location"], expired["counter"]))

      if pending_commands:
        expected = pending_commands.pop(0)
        paired_loopback_count += 1
        loopback_ack_latencies.append(max((mono_time - expected["time"]) * 1000.0, 0.0))
        actual = (counter, torque, active, checksum)
        wanted = (expected["counter"], expected["torque"], expected["active"], expected["checksum"])
        if actual != wanted:
          errors.append("{} loopback payload {} does not match command {} from {}".format(
            location, actual, wanted, expected["location"]))
      elif command_count:
        errors.append("{} loopback counter {} has no pending command".format(location, counter))
      else:
        unpaired_prefix_loopbacks += 1

      previous_loopback_counter = counter
      previous_loopback_time = mono_time

    command_sent = parse_bool(row.get("command_sent"))
    command_due = parse_bool(row.get("command_due"))
    loopback_changed = parse_bool(row.get("loopback_changed"))
    block_reason = row.get("command_block_reason", "")

    if command_sent:
      command_count += 1
      counter = parse_int(row.get("command_counter")) % 4
      command_counters_seen.add(counter)
      torque = parse_int(row.get("command_torque"))
      active = parse_bool(row.get("command_active"))
      checksum = parse_int(row.get("command_checksum")) & 0xfff
      expected_checksum = gm_lkas_checksum(active, torque, counter)

      if not command_due:
        errors.append("{} command was sent outside the fixed 50Hz frame gate".format(location))
      if loopback_changed:
        errors.append("{} command was sent in the same cycle as a changed loopback".format(location))
      if block_reason != "sent":
        errors.append("{} sent command has block reason {!r}".format(location, block_reason))
      if checksum != expected_checksum:
        errors.append("{} command checksum {} != {}".format(location, checksum, expected_checksum))
      if pending_commands:
        errors.append("{} command counter {} was sent before counter {} received loopback".format(
          location, counter, pending_commands[0]["counter"]))

      if previous_command_time is not None and same_session(previous_command_time, mono_time):
        interval_ms = (mono_time - previous_command_time) * 1000.0
        command_intervals.append(interval_ms)
        expected_counter = (previous_command_counter + 1) % 4
        if counter != expected_counter:
          errors.append("{} command counter {} after {}, expected {}".format(
            location, counter, previous_command_counter, expected_counter))
        if interval_ms < MIN_COMMAND_INTERVAL_MS:
          errors.append("{} command interval {:.3f}ms is too short".format(location, interval_ms))
        elif interval_ms > MAX_COMMAND_INTERVAL_MS:
          errors.append("{} command interval {:.3f}ms exceeds gap limit".format(location, interval_ms))

      pending_commands.append({
        "counter": counter,
        "torque": torque,
        "active": active,
        "checksum": checksum,
        "time": mono_time,
        "location": location,
      })
      previous_command_counter = counter
      previous_command_time = mono_time
    else:
      allowed_due_blocks = ("initial_sync", "loopback_changed", "unacked", "min_interval")
      if command_due and block_reason not in allowed_due_blocks:
        errors.append("{} due command was not sent and has unexpected reason {!r}".format(location, block_reason))

  if pending_commands:
    warnings.append("{} final command(s) have no loopback inside the selected log boundary".format(
      len(pending_commands)))
  if unpaired_prefix_loopbacks:
    warnings.append("{} initial loopback(s) preceded the first command in the selected log".format(
      unpaired_prefix_loopbacks))
  if unacked_blocks:
    warnings.append("{} due commands were safely blocked waiting for Panda loopback".format(unacked_blocks))
  if gap_rows:
    errors.append("{} rows reported a steering command gap over {:.0f}ms".format(
      gap_rows, MAX_COMMAND_INTERVAL_MS))
  if batched_loopback_rows:
    warnings.append("{} rows contained multiple loopbacks; exact bus interval was not observable".format(
      batched_loopback_rows))
  if quantized_loopback_intervals:
    warnings.append("{} observed loopback intervals were below {:.0f}ms; 10ms control-cycle "
                    "timestamp quantization may apply".format(
                      quantized_loopback_intervals, IDEAL_LOOPBACK_INTERVAL_MS))
  if invalid_can_rows:
    warnings.append("{} rows reported canValid=false (maximum consecutive rows: {})".format(
      invalid_can_rows, max_invalid_can_streak))
  if max_invalid_can_streak >= 5:
    errors.append("canValid was false for {} consecutive control rows".format(max_invalid_can_streak))
  if max_queue_drops:
    errors.append("diagnostic logger dropped at least {} rows".format(max_queue_drops))
  if pscm_temporary_rows:
    errors.append("PSCM temporary LKAS status appeared in {} rows".format(pscm_temporary_rows))
  if pscm_permanent_rows:
    errors.append("PSCM permanent LKAS fault appeared in {} rows".format(pscm_permanent_rows))
  if command_count < min_samples:
    errors.append("only {} commands found; at least {} are required".format(command_count, min_samples))
  if loopback_count < min_samples:
    errors.append("only {} loopbacks found; at least {} are required".format(loopback_count, min_samples))
  if command_count >= 4 and command_counters_seen != {0, 1, 2, 3}:
    errors.append("command counters did not cover 0->1->2->3: {}".format(
      sorted(command_counters_seen)))
  if loopback_count >= 4 and loopback_counters_seen != {0, 1, 2, 3}:
    errors.append("loopback counters did not cover 0->1->2->3: {}".format(
      sorted(loopback_counters_seen)))

  return {
    "errors": errors,
    "warnings": warnings,
    "command_count": command_count,
    "loopback_count": loopback_count,
    "paired_loopback_count": paired_loopback_count,
    "command_intervals": command_intervals,
    "loopback_intervals": loopback_intervals,
    "loopback_ack_latencies": loopback_ack_latencies,
    "pscm_status_counts": pscm_status_counts,
  }


def print_interval_summary(label, values):
  if not values:
    print("{}: no intervals".format(label))
    return
  print("{}: min={:.3f}ms median={:.3f}ms p95={:.3f}ms max={:.3f}ms".format(
    label, min(values), statistics.median(values), percentile(values, 0.95), max(values)))


def main():
  parser = argparse.ArgumentParser(description="Validate GM 0x180 steering diagnostic logs")
  parser.add_argument("path", nargs="?", default=DEFAULT_LOG_PATH,
                      help="log file or directory (default: {})".format(DEFAULT_LOG_PATH))
  parser.add_argument("--min-samples", type=int, default=MIN_VALIDATION_SAMPLES,
                      help="minimum command and loopback samples required (default: {})".format(
                        MIN_VALIDATION_SAMPLES))
  args = parser.parse_args()

  files, rows = load_rows(args.path)
  if not files or not rows:
    print("NO DATA: no gm_lkas CSV rows found at {}".format(args.path))
    return 2

  result = analyze_rows(rows, min_samples=max(args.min_samples, 1))
  print("files={} rows={} commands={} loopbacks={} paired={}".format(
    len(files), len(rows), result["command_count"], result["loopback_count"],
    result["paired_loopback_count"]))
  print("PSCM LKATorqueDeliveredStatus counts={}".format(result["pscm_status_counts"]))
  print_interval_summary("command intervals", result["command_intervals"])
  print_interval_summary("observed loopback intervals", result["loopback_intervals"])
  print_interval_summary("command-to-loopback latency", result["loopback_ack_latencies"])

  for warning in result["warnings"]:
    print("WARNING: {}".format(warning))
  for error in result["errors"][:50]:
    print("ERROR: {}".format(error))
  if len(result["errors"]) > 50:
    print("ERROR: {} additional errors omitted".format(len(result["errors"]) - 50))

  if result["errors"]:
    print("RESULT: FAIL")
    return 1
  print("RESULT: PASS")
  return 0


if __name__ == "__main__":
  sys.exit(main())
