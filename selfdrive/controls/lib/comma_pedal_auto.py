"""AUTO comma-pedal profile: learn LOW / MID / HIGH from how the driver drives.

The learner only ever picks one of the three existing profiles, so AUTO can
never make the pedal stronger or weaker than a manual choice could.

Two kinds of evidence:

* Style, from the driver's own acceleration. While the driver is on the gas
  (or openpilot's longitudinal control is off) every rise of aEgo above
  EPISODE_START_A is one episode. Its peak acceleration and rise jerk are
  scored against per-speed-bin thresholds, because 1.5 m/s^2 is ordinary at
  20 km/h and very brisk at 100 km/h. Episodes taken behind a close lead, in a
  corner or during a lane change are dropped: there the road chose the
  acceleration, not the driver.

* Feedback, relative to the profile that was active at the time. A gas press
  that out-accelerates openpilot while openpilot was already accelerating says
  "too weak"; a brake press while openpilot was pushing hard on an open road
  says "too strong". This car cannot brake under openpilot, so the driver
  supplies every deceleration -- an earlier learner that counted ordinary
  brake presses could not tell "openpilot accelerated too hard" from "I needed
  to slow" and drove the gain down with no way back up. Brake feedback here
  therefore needs openpilot to have been accelerating hard for a full second
  with no lead close, and a hard brake by itself is not counted as style.

Old drives fade out: every new drive (controlsd start) multiplies the stored
weights by DRIVE_DECAY, halving a drive's influence after about three drives.
"""

import json
import math
import os
import threading

PROFILES = ('low', 'mid', 'high')
LEVEL = {'low': -1.0, 'mid': 0.0, 'high': 1.0}

# Speed bins: below 40 km/h, 40-80 km/h, above 80 km/h.
SPEED_BIN_EDGES_MS = (40.0 / 3.6, 80.0 / 3.6)
# Peak acceleration that scores -1 (calm) and +1 (brisk), per speed bin.
PEAK_ACCEL_LOW = (1.0, 0.7, 0.45)
PEAK_ACCEL_HIGH = (2.0, 1.4, 0.9)
# Mean jerk of the rise to the peak that scores -1 and +1. An ordinary
# pull-away -- 0.3 to 1.5 m/s^2 in about 1.2 s -- is 1.0 m/s^3 and should land
# just under the middle, not at the calm end.
RISE_JERK_LOW = 0.5
RISE_JERK_HIGH = 2.0
PEAK_WEIGHT = 0.7

EPISODE_START_A = 0.3
EPISODE_END_A = 0.15
EPISODE_END_HOLD_S = 0.5
EPISODE_MIN_S = 1.0
EPISODE_MIN_PEAK_V = 2.0      # m/s; creeping in a queue says nothing
LEAD_MIN_HEADWAY_S = 1.0
LAT_ACCEL_MAX = 1.5

# Feedback.
FEEDBACK_OP_ACCEL_MIN = 0.3   # openpilot must be accelerating for a gas press to count
FEEDBACK_GAS_MARGIN = 0.3     # and the driver must then out-accelerate it by this much
FEEDBACK_GAS_WINDOW_S = 2.0
FEEDBACK_BRAKE_OP_ACCEL = 0.5
FEEDBACK_BRAKE_A_EGO = 0.3
FEEDBACK_BRAKE_HOLD_S = 1.0
FEEDBACK_BRAKE_LEAD_HEADWAY_S = 2.5
FEEDBACK_COOLDOWN_S = 5.0
FEEDBACK_MIN_EVENTS = 3

MIN_EPISODES = 15
MAX_EPISODES = 60
MAX_FEEDBACK = 30
DRIVE_DECAY = 0.79
MIN_WEIGHT = 0.05

STYLE_WEIGHT = 0.6
ENTER_THRESHOLD = 0.30
LEAVE_THRESHOLD = 0.20
MIN_CHANGE_INTERVAL_S = 180.0
DECIDE_INTERVAL_S = 1.0
SAVE_INTERVAL_S = 60.0
LOG_INTERVAL_S = 300.0

LOG_PATH = os.getenv("COMMA_PEDAL_AUTO_LOG_PATH", "/data/log/comma_pedal_auto.jsonl")
LOG_MAX_BYTES = 2 * 1024 * 1024


def _clip(x, lo, hi):
  return lo if x < lo else hi if x > hi else x


def _ramp(x, lo, hi):
  """-1 at lo, +1 at hi, linear between, clipped."""
  if hi <= lo:
    return 0.0
  return _clip(2.0 * (x - lo) / (hi - lo) - 1.0, -1.0, 1.0)


def _speed_bin(v):
  if v < SPEED_BIN_EDGES_MS[0]:
    return 0
  if v < SPEED_BIN_EDGES_MS[1]:
    return 1
  return 2


def _weighted_quantile(pairs, q):
  """pairs = [(value, weight)]; q in [0, 1]."""
  pairs = sorted((v, w) for v, w in pairs if w > 0)
  total = sum(w for _, w in pairs)
  if total <= 0:
    return 0.0
  target = q * total
  acc = 0.0
  for v, w in pairs:
    acc += w
    if acc >= target:
      return v
  return pairs[-1][0]


def score_episode(peak_a, v_at_peak, rise_s):
  b = _speed_bin(v_at_peak)
  peak_score = _ramp(peak_a, PEAK_ACCEL_LOW[b], PEAK_ACCEL_HIGH[b])
  jerk = (peak_a - EPISODE_START_A) / max(rise_s, 0.2)
  jerk_score = _ramp(jerk, RISE_JERK_LOW, RISE_JERK_HIGH)
  return PEAK_WEIGHT * peak_score + (1.0 - PEAK_WEIGHT) * jerk_score


def _write_async(fn):
  t = threading.Thread(target=fn, daemon=True)
  t.start()


class CommaPedalAutoLearner:
  def __init__(self, stats=None, put_fn=None):
    """stats: the saved JSON (str/bytes) or None. put_fn(key, value) stores a param."""
    self.put_fn = put_fn
    self.episodes = []      # [score, weight]
    self.feedback = []      # [target_level, weight]
    self.result = None      # 'low' | 'mid' | 'high' once decided
    self.score = 0.0
    self.style = 0.0
    self.fb_target = None
    self._load(stats)
    # A new drive: fade what earlier drives taught.
    self.episodes = [[s, w * DRIVE_DECAY] for s, w in self.episodes if w * DRIVE_DECAY >= MIN_WEIGHT]
    self.feedback = [[t, w * DRIVE_DECAY] for t, w in self.feedback if w * DRIVE_DECAY >= MIN_WEIGHT]

    self._ep = None
    self._end_hold = 0.0
    self._pending_gas = None
    self._op_push_s = 0.0
    self._last_feedback_t = -1e9
    self._prev_gas = False
    self._prev_brake = False
    self._last_decide_t = -1e9
    self._last_change_t = -1e9
    self._last_save_t = None
    self._last_log_t = None
    self._dirty = False
    self._status = None
    self.last_episode = None
    self._decide(force=True)

  # ---- persistence ----
  def _load(self, stats):
    if not stats:
      return
    try:
      if isinstance(stats, bytes):
        stats = stats.decode('utf8')
      d = json.loads(stats)
      self.episodes = [[float(s), float(w)] for s, w in d.get('episodes', [])
                       if math.isfinite(float(s)) and float(w) > 0][-MAX_EPISODES:]
      self.feedback = [[float(t), float(w)] for t, w in d.get('feedback', [])
                       if math.isfinite(float(t)) and float(w) > 0][-MAX_FEEDBACK:]
      r = d.get('result')
      self.result = r if r in PROFILES else None
    except Exception:
      self.episodes, self.feedback, self.result = [], [], None

  def to_json(self):
    return json.dumps({
      'v': 1,
      'result': self.result,
      'score': round(self.score, 3),
      'episodes': [[round(s, 3), round(w, 3)] for s, w in self.episodes],
      'feedback': [[round(t, 3), round(w, 3)] for t, w in self.feedback],
    }, separators=(',', ':'))

  @property
  def profile(self):
    """The profile to drive with now. MID while still learning."""
    return self.result or 'mid'

  @property
  def status(self):
    """'learn N/15' while learning, else the learned profile."""
    if self.result is None:
      return "learn %d/%d" % (min(len(self.episodes), MIN_EPISODES), MIN_EPISODES)
    return self.result

  # ---- per-step update (call at ~20 Hz) ----
  def update(self, now, dt, v_ego, a_ego, gas_pressed, brake_pressed, long_active,
             op_accel, lead_headway_s, lead_closing, lat_accel, lane_change):
    gas_pressed = bool(gas_pressed)
    brake_pressed = bool(brake_pressed)
    constrained = (lead_headway_s is not None and lead_headway_s < LEAD_MIN_HEADWAY_S) or \
                  abs(lat_accel) > LAT_ACCEL_MAX or bool(lane_change)

    self._update_episode(dt, v_ego, a_ego, gas_pressed or not long_active, constrained)
    self._update_feedback(now, dt, a_ego, gas_pressed, brake_pressed, long_active,
                          op_accel, lead_headway_s, lead_closing, constrained)
    self._prev_gas = gas_pressed
    self._prev_brake = brake_pressed

    if now - self._last_decide_t >= DECIDE_INTERVAL_S:
      self._last_decide_t = now
      self._decide(now=now)
    self._maybe_persist(now)

  def _update_episode(self, dt, v_ego, a_ego, driver_in_control, constrained):
    ep = self._ep
    if ep is None:
      if driver_in_control and a_ego > EPISODE_START_A:
        self._ep = {'t': 0.0, 'peak': a_ego, 'v': v_ego, 'rise': 0.0, 'bad': constrained}
        self._end_hold = 0.0
      return

    if not driver_in_control:
      # openpilot took the pedal back mid-episode: the rest is not the driver's.
      self._ep = None
      return

    ep['t'] += dt
    ep['bad'] = ep['bad'] or constrained
    if a_ego > ep['peak']:
      ep['peak'] = a_ego
      ep['v'] = v_ego
      ep['rise'] = ep['t']
    if a_ego < EPISODE_END_A:
      self._end_hold += dt
      if self._end_hold >= EPISODE_END_HOLD_S:
        self._finish_episode(ep)
        self._ep = None
    else:
      self._end_hold = 0.0

  def _finish_episode(self, ep):
    duration = ep['t'] - self._end_hold
    if ep['bad'] or duration < EPISODE_MIN_S or ep['v'] < EPISODE_MIN_PEAK_V:
      return
    s = score_episode(ep['peak'], ep['v'], ep['rise'])
    self.episodes.append([s, 1.0])
    self.episodes = self.episodes[-MAX_EPISODES:]
    self.last_episode = {'peak': round(ep['peak'], 2), 'kph': round(ep['v'] * 3.6),
                         'rise': round(ep['rise'], 2), 'score': round(s, 2)}
    self._dirty = True

  def _add_feedback(self, now, delta):
    if now - self._last_feedback_t < FEEDBACK_COOLDOWN_S:
      return
    self._last_feedback_t = now
    target = _clip(LEVEL[self.profile] + delta, -1.0, 1.0)
    self.feedback.append([target, 1.0])
    self.feedback = self.feedback[-MAX_FEEDBACK:]
    self._dirty = True

  def _update_feedback(self, now, dt, a_ego, gas_pressed, brake_pressed, long_active,
                       op_accel, lead_headway_s, lead_closing, constrained):
    # How long openpilot has been pushing hard on its own.
    if long_active and not gas_pressed and not brake_pressed and \
        op_accel >= FEEDBACK_BRAKE_OP_ACCEL and a_ego >= FEEDBACK_BRAKE_A_EGO:
      self._op_push_s += dt
    elif not brake_pressed:
      self._op_push_s = 0.0

    # Too weak: a gas press while openpilot was already accelerating, which
    # the driver then out-accelerates.
    if long_active and gas_pressed and not self._prev_gas and \
        op_accel >= FEEDBACK_OP_ACCEL_MIN and not constrained:
      self._pending_gas = {'t': now, 'need': op_accel + FEEDBACK_GAS_MARGIN}
    pg = self._pending_gas
    if pg is not None:
      if a_ego >= pg['need']:
        self._add_feedback(now, +1.0)
        self._pending_gas = None
      elif now - pg['t'] > FEEDBACK_GAS_WINDOW_S or not gas_pressed:
        self._pending_gas = None

    # Too strong: a brake press while openpilot had been pushing hard for a
    # full second with nothing close ahead.
    if brake_pressed and not self._prev_brake and self._op_push_s >= FEEDBACK_BRAKE_HOLD_S:
      lead_clear = lead_headway_s is None or \
        (lead_headway_s >= FEEDBACK_BRAKE_LEAD_HEADWAY_S and not lead_closing)
      if lead_clear and not constrained:
        self._add_feedback(now, -1.0)
      self._op_push_s = 0.0

  # ---- decision ----
  def _decide(self, now=0.0, force=False):
    if self.episodes:
      self.style = 0.5 * _weighted_quantile(self.episodes, 0.5) + \
                   0.5 * _weighted_quantile(self.episodes, 0.9)
    else:
      self.style = 0.0
    if len(self.feedback) >= FEEDBACK_MIN_EVENTS:
      wsum = sum(w for _, w in self.feedback)
      self.fb_target = sum(t * w for t, w in self.feedback) / wsum if wsum > 0 else None
    else:
      self.fb_target = None
    self.score = self.style if self.fb_target is None else \
      STYLE_WEIGHT * self.style + (1.0 - STYLE_WEIGHT) * self.fb_target

    if len(self.episodes) < MIN_EPISODES and self.result is None:
      return

    cur = self.result or 'mid'
    new = cur
    if cur == 'high':
      if self.score < LEAVE_THRESHOLD:
        new = 'low' if self.score <= -ENTER_THRESHOLD else 'mid'
    elif cur == 'low':
      if self.score > -LEAVE_THRESHOLD:
        new = 'high' if self.score >= ENTER_THRESHOLD else 'mid'
    else:
      if self.score >= ENTER_THRESHOLD:
        new = 'high'
      elif self.score <= -ENTER_THRESHOLD:
        new = 'low'

    if self.result is None:
      self.result = new
      self._last_change_t = now
      self._dirty = True
      self._log(now, 'decided')
    elif new != cur and (force or now - self._last_change_t >= MIN_CHANGE_INTERVAL_S):
      self.result = new
      self._last_change_t = now
      self._dirty = True
      self._log(now, 'changed')

  # ---- output ----
  def _maybe_persist(self, now):
    if self.put_fn is None:
      return
    status = self.status
    if status != self._status:
      self._status = status
      self.put_fn("CommaPedalAutoStatus", status)
    if self._last_save_t is None:
      self._last_save_t = now
      self._last_log_t = now
    if self._dirty and now - self._last_save_t >= SAVE_INTERVAL_S:
      self._last_save_t = now
      self._dirty = False
      self.put_fn("CommaPedalAutoStats", self.to_json())
    if now - self._last_log_t >= LOG_INTERVAL_S:
      self._log(now, 'periodic')

  def save_now(self):
    if self.put_fn is not None:
      self._dirty = False
      self.put_fn("CommaPedalAutoStats", self.to_json())

  def _log(self, now, reason):
    self._last_log_t = now
    if self.put_fn is None:
      return
    line = json.dumps({
      'reason': reason, 't': round(now, 1), 'result': self.result, 'status': self.status,
      'score': round(self.score, 3), 'style': round(self.style, 3),
      'feedback_target': None if self.fb_target is None else round(self.fb_target, 3),
      'episodes': len(self.episodes), 'feedback': len(self.feedback),
      'last_episode': self.last_episode,
    }, separators=(',', ':'))

    def write():
      try:
        if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
          os.replace(LOG_PATH, LOG_PATH + ".1")
        with open(LOG_PATH, 'a') as f:
          f.write(line + "\n")
      except Exception:
        pass
    _write_async(write)
