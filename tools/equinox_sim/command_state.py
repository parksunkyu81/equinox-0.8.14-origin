import json
import os


COMMAND_DIR = "/tmp/equinox_sim"
COMMAND_FILE = os.path.join(COMMAND_DIR, "commands.json")

DEFAULT_COMMAND_STATE = {
  "targetSpeedKph": 100.0,
  "ignition": True,
  "faultMode": 0,
  "brakePressed": False,
  "gasPressed": False,
  "resetToken": 0,
  "engageToken": 1,
}


def normalize_command_state(state):
  normalized = dict(DEFAULT_COMMAND_STATE)
  if isinstance(state, dict):
    normalized.update(state)
  normalized["targetSpeedKph"] = min(145.0, max(20.0, float(normalized["targetSpeedKph"])))
  normalized["ignition"] = bool(normalized["ignition"])
  normalized["faultMode"] = min(3, max(0, int(normalized["faultMode"])))
  normalized["brakePressed"] = bool(normalized["brakePressed"])
  normalized["gasPressed"] = bool(normalized["gasPressed"])
  normalized["resetToken"] = max(0, int(normalized["resetToken"]))
  normalized["engageToken"] = max(0, int(normalized["engageToken"]))
  return normalized


def load_command_state():
  try:
    with open(COMMAND_FILE, encoding="utf8") as command_file:
      return normalize_command_state(json.load(command_file))
  except (OSError, ValueError, TypeError):
    return dict(DEFAULT_COMMAND_STATE)


def save_command_state(state):
  normalized = normalize_command_state(state)
  os.makedirs(COMMAND_DIR, exist_ok=True)
  temp_file = COMMAND_FILE + ".tmp"
  with open(temp_file, "w", encoding="utf8") as command_file:
    json.dump(normalized, command_file, separators=(",", ":"))
  os.replace(temp_file, COMMAND_FILE)
  return normalized


class CommandStateReader:
  def __init__(self):
    self.state = load_command_state()
    self.mtime_ns = None

  def read(self):
    try:
      mtime_ns = os.stat(COMMAND_FILE).st_mtime_ns
    except OSError:
      return self.state
    if mtime_ns != self.mtime_ns:
      self.state = load_command_state()
      self.mtime_ns = mtime_ns
    return self.state
