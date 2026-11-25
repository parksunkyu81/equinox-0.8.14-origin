"""Presses of the steering-wheel follow-distance button (ASCMSteeringButton bit 22).

The DBC has DistanceButton in 0x1E1, the same message the cruise SET/RES
buttons come from on this car, and the parser already subscribed to it, but
nothing read it. Whether this Equinox actually sends it is not known, so each
press goes to /data/log/distance_button.jsonl -- when it went down, how long
it was held, and the speed -- before anything is built on it.

Per-frame cost: one dict read and a compare; a line only on release.
"""
import json
import os
import threading

LOG_PATH = os.getenv("DISTANCE_BUTTON_LOG_PATH", "/data/log/distance_button.jsonl")
LOG_MAX_BYTES = 512 * 1024


def _write(line):
  def write():
    try:
      if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
        os.replace(LOG_PATH, LOG_PATH + ".1")
      with open(LOG_PATH, 'a') as f:
        f.write(line + "\n")
    except Exception:
      pass
  threading.Thread(target=write, daemon=True).start()


class DistanceButtonLog:
  def __init__(self):
    self.pressed = False
    self.down_t = 0.0
    self.down_kph = 0.0
    self.count = 0

  def update(self, pressed, now, v_ego):
    pressed = bool(pressed)
    if pressed and not self.pressed:
      self.down_t = now
      self.down_kph = v_ego * 3.6
    elif not pressed and self.pressed:
      self.count += 1
      _write(json.dumps({
        't': round(self.down_t, 2), 'event': 'press', 'n': self.count,
        'held_s': round(now - self.down_t, 2), 'kph': round(self.down_kph, 1),
      }, separators=(',', ':')))
    self.pressed = pressed
    return pressed
