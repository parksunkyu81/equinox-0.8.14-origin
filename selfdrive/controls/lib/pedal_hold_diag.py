"""Why the comma pedal sat at zero while openpilot asked for acceleration.

On the 11-16/17 drives the driver pressed the gas 60 times right after openpilot
had asked for +0.4 m/s^2 and the car was slowing at -0.2 m/s^2. In every one of
them the pedal actually sent was 0.00, mostly within 0.3-1.6 s of the driver
letting go of the brake. The rlog cannot say which of the zero ceilings held it
(predictive coasting's brake latch, a lead / curve / speed-limit pressure, the
corner lift, the post-brake rise hold, the ACC gate), so this records it.

An episode starts when, with openpilot active and the driver off both pedals,
the planner asks for at least ACCEL_MIN while the final pedal stays below
PEDAL_ZERO for HOLD_S. It ends when the pedal comes up, the request goes away,
the driver presses a pedal or openpilot disengages. One JSON line per episode
goes to /data/log/pedal_hold.jsonl: what the pedal path looked like at the
start, which causes were seen while it lasted, and what ended it.

Cost: a handful of comparisons a frame; the snapshot is taken every 10th frame
and only inside an episode; the file is written from a thread.
"""
import json
import math
import os
import threading
import time

LOG_PATH = os.getenv("PEDAL_HOLD_LOG_PATH", "/data/log/pedal_hold.jsonl")
LOG_MAX_BYTES = 2 * 1024 * 1024

ACCEL_MIN = 0.2       # m/s^2 asked by the planner
PEDAL_ZERO = 0.01     # final pedal at or below this counts as zero
MIN_SPEED_MS = 1.0
HOLD_S = 0.5          # zero pedal this long before it counts
SAMPLE_EVERY = 10     # frames between cause samples inside an episode
MAX_CAUSES = 8


def _f(x, nd=2):
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return round(x, nd) if math.isfinite(x) else None


def _write_async(line):
  def write():
    try:
      if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
        os.replace(LOG_PATH, LOG_PATH + ".1")
      with open(LOG_PATH, 'a') as f:
        f.write(line + "\n")
    except Exception:
      pass
  threading.Thread(target=write, daemon=True).start()


def classify(s):
  """The zero ceiling that explains the snapshot, first match wins."""
  if not s.get('acc_on', True):
    return 'acc_off'
  if s.get('stop_boost'):
    return 'stop_boost'
  if (s.get('raw') or 0.0) < 0.005:
    return 'raw_zero'
  scale = s.get('coast_scale')
  if scale is not None and scale < 0.05:
    if s.get('corner_lift') and (s.get('corner_scale') or 0.0) < 0.05:
      return 'corner_lift'
    phase = s.get('coast_phase') or ''
    if phase == 'brake_needed':
      return 'coast_urgent'
    if (s.get('coast_pressure') or 0.0) > 0.0:
      return 'coast_' + (s.get('coast_source') or 'pressure')
    if s.get('brake_latched'):
      return 'coast_brake_latch'
    return 'coast_' + (phase or 'zero')
  if s.get('rise_holding'):
    return 'post_brake_hold'
  return 'other'


class PedalHoldDiag:
  def __init__(self, dt=0.01, write_fn=None):
    self.dt = dt
    self.write_fn = write_fn or _write_async
    self.zero_s = 0.0
    self.episode = None
    self.frame = 0
    self.last_driver = None      # 'brake' / 'gas'
    self.driver_off_s = math.inf

  def update(self, active, v_ego, accel, final_pedal, gas_pressed, brake_pressed, snapshot_fn):
    """snapshot_fn() returns the dict of pedal-path state; called only inside an episode."""
    self.frame += 1
    if brake_pressed:
      self.last_driver, self.driver_off_s = 'brake', 0.0
    elif gas_pressed:
      self.last_driver, self.driver_off_s = 'gas', 0.0
    else:
      self.driver_off_s += self.dt

    asking = bool(active and not gas_pressed and not brake_pressed and
                  v_ego >= MIN_SPEED_MS and accel >= ACCEL_MIN)
    zero = final_pedal <= PEDAL_ZERO

    if self.episode is not None:
      ep = self.episode
      ep['dur'] += self.dt
      ep['accel_max'] = max(ep['accel_max'], accel)
      end = None
      if not active:
        end = 'disengaged'
      elif brake_pressed:
        end = 'brake'
      elif gas_pressed:
        end = 'gas'
      elif not zero:
        end = 'pedal'
      elif accel < ACCEL_MIN:
        end = 'accel_drop'
      elif v_ego < MIN_SPEED_MS:
        end = 'stopped'
      if end is None:
        if self.frame % SAMPLE_EVERY == 0:
          self._sample(snapshot_fn)
        return
      self._finish(end, v_ego)

    if asking and zero:
      self.zero_s += self.dt
      if self.zero_s >= HOLD_S:
        self._start(v_ego, accel, snapshot_fn)
    else:
      self.zero_s = 0.0

  def _sample(self, snapshot_fn):
    try:
      cause = classify(snapshot_fn())
    except Exception:
      cause = 'error'
    causes = self.episode['causes']
    if (not causes or causes[-1] != cause) and len(causes) < MAX_CAUSES:
      causes.append(cause)

  def _start(self, v_ego, accel, snapshot_fn):
    try:
      snap = snapshot_fn()
      cause = classify(snap)
    except Exception as e:
      snap, cause = {'error': str(e)}, 'error'
    self.episode = {
      'wall': round(time.time(), 1),
      'v_kph': _f(v_ego * 3.6, 1),
      'accel': _f(accel),
      'accel_max': accel,
      'dur': self.zero_s,
      'after': self.last_driver,
      'driver_off_s': _f(self.driver_off_s - self.zero_s, 2) if math.isfinite(self.driver_off_s) else None,
      'cause': cause,
      'causes': [cause],
      'snap': snap,
    }

  def _finish(self, end, v_ego):
    ep = self.episode
    self.episode = None
    self.zero_s = 0.0
    ep['dur'] = _f(ep['dur'], 2)
    ep['accel_max'] = _f(ep['accel_max'])
    ep['end'] = end
    ep['v_end_kph'] = _f(v_ego * 3.6, 1)
    try:
      self.write_fn(json.dumps(ep, separators=(',', ':')))
    except Exception:
      pass
