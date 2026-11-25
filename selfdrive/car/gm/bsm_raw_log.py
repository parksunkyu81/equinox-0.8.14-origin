"""Raw bytes of GM message 0x142 (BCMBlindSpotMonitor) while driving.

The Equinox's fingerprint carries 0x142, the message newer openpilot reads GM
Side Blind Zone Alert from, and the car has the mirror warning lights. Which
bits mean "car in the left / right blind zone" on this car is not known yet,
so this records the seven bytes as they change, with what the car was doing:
one JSON line per change to /data/log/bsm_raw.jsonl.

A byte that keeps changing (a counter or checksum) would flood the file, so a
byte that has changed more than COUNTER_CHANGES times stops being watched and
is listed under "ignored" in a summary line written every SUMMARY_S. Lines are
capped at MAX_LINES_PER_S. Written from a thread; the per-frame cost is seven
dict reads and a tuple compare.
"""
import json
import os
import threading
import time

LOG_PATH = os.getenv("BSM_RAW_LOG_PATH", "/data/log/bsm_raw.jsonl")
# The same change log for the steering-wheel button message 0x1E1: the
# follow-distance button never set DistanceButton (bit 22) on the 2026-11-25
# drive, so its seven raw bytes are recorded to find where it really is.
BUTTON_RAW_LOG_PATH = os.getenv("BUTTON_RAW_LOG_PATH", "/data/log/steer_button_raw.jsonl")
LOG_MAX_BYTES = 2 * 1024 * 1024
N_BYTES = 7
COUNTER_CHANGES = 500
SUMMARY_S = 60.0
MAX_LINES_PER_S = 10


def _write(lines, path=None):
  path = LOG_PATH if path is None else path

  def write():
    try:
      if os.path.exists(path) and os.path.getsize(path) > LOG_MAX_BYTES:
        os.replace(path, path + ".1")
      with open(path, 'a') as f:
        f.write("".join(line + "\n" for line in lines))
    except Exception:
      pass
  threading.Thread(target=write, daemon=True).start()


class BsmRawLog:
  def __init__(self, log_path=None):
    self.log_path = log_path
    self.last = None
    self.changes = [0] * N_BYTES
    self.ignored = set()
    self.seen = False
    self.summary_t = 0.0
    self.sec_t = 0.0
    self.sec_lines = 0

  def update(self, vals, now, v_ego, left_blinker, right_blinker, steering_angle):
    """vals: the seven bytes as read this frame (all 0 until the message is seen)."""
    cur = tuple(int(v) for v in vals)
    if self.last is None:
      self.last = cur
      self.summary_t = now
      return
    if cur != self.last:
      self.seen = True
      changed = [i for i in range(N_BYTES) if cur[i] != self.last[i]]
      for i in changed:
        self.changes[i] += 1
        if self.changes[i] > COUNTER_CHANGES:
          self.ignored.add(i)
      watched = [i for i in changed if i not in self.ignored]
      prev = self.last
      self.last = cur
      if watched:
        if now - self.sec_t >= 1.0:
          self.sec_t = now
          self.sec_lines = 0
        if self.sec_lines < MAX_LINES_PER_S:
          self.sec_lines += 1
          _write([json.dumps({
            't': round(now, 2), 'event': 'change',
            'bytes': [f"{b:02x}" for b in cur], 'prev': [f"{b:02x}" for b in prev],
            'changed': watched, 'kph': round(v_ego * 3.6, 1),
            'blinker': 'L' if left_blinker else ('R' if right_blinker else ''),
            'steer_deg': round(steering_angle, 1),
          }, separators=(',', ':'))], self.log_path)
    if now - self.summary_t >= SUMMARY_S:
      self.summary_t = now
      _write([json.dumps({
        't': round(now, 2), 'event': 'summary', 'seen': self.seen,
        'bytes': [f"{b:02x}" for b in self.last], 'changes': self.changes,
        'ignored': sorted(self.ignored), 'unix_s': int(time.time()),
      }, separators=(',', ':'))], self.log_path)
