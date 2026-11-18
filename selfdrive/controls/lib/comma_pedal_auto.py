"""AUTO comma-pedal profile: step LOW / MID / HIGH by how the driver answers openpilot's acceleration.

The learner only ever picks one of the three existing profiles, so AUTO can
never make the pedal stronger or weaker than a manual choice could.

Why votes. The previous learner mixed a style score from the driver's own
accelerations with "too weak" / "too strong" feedback. On the 11-17/18 drives
it sat on LOW and could not leave: the style score read calm (-0.15), twelve
brake taps said "too strong", and "too weak" never fired once -- it needed the
driver to out-accelerate openpilot by 0.3 m/s^2 while openpilot was already
asking +0.3, which almost never happens. Down had evidence, up had none, and
old drives faded only by half every three drives.

What a profile changes is how hard openpilot accelerates (the planner's
ceiling and the pedal rise rate). So the question is asked only while
openpilot is accelerating -- accel command above PUSH_ACCEL for PUSH_MIN_S, no
lead it is closing on, not in a corner or lane change, above a crawl:

* the driver presses the gas  -> one vote "more"
* the driver taps the brake (released within BRAKE_TAP_MAX_S, the car still
  above BRAKE_TAP_KEEP_SPEED of its speed, from BRAKE_MIN_V) -> one vote "less"
* a brake press that slows or stops the car is ignored: this car cannot brake
  under openpilot, so the driver brakes for every light, crossing and bump,
  which says nothing about pedal strength.

Presses within CLUSTER_S of the previous pedal input are one press. Votes
halve every HALF_LIFE_S of engaged driving, so the choice follows the drive
as it goes. When one side has at least STEP_MIN_VOTES and STEP_SHARE of the
votes, the profile steps one level that way and the votes reset -- they were
cast against the old level. A step is held at least MIN_CHANGE_INTERVAL_S.
The level is kept across drives; the votes are halved at each start.

On the 11-18 drives this counted 25 "more" and 20 "less" (51 slowing / stopping
brakes ignored), and the "more" share of 10-minute windows ranged 0.22-0.73.
"""

import json
import math
import os
import threading

PROFILES = ('low', 'mid', 'high')

PUSH_ACCEL = 0.2              # m/s^2 accel command that counts as accelerating
PUSH_MIN_S = 1.0              # and for this long before a pedal press counts
PUSH_MIN_V = 3.0 / 3.6
LAT_ACCEL_MAX = 1.5
CLUSTER_S = 3.0
BRAKE_TAP_MAX_S = 1.5
BRAKE_TAP_KEEP_SPEED = 0.80
BRAKE_MIN_V = 20.0 / 3.6

HALF_LIFE_S = 600.0           # engaged driving
STEP_MIN_VOTES = 2.5          # three recent votes (the decay shaves a little off each)
STEP_SHARE = 0.65
MIN_CHANGE_INTERVAL_S = 60.0
DRIVE_START_DECAY = 0.5

STATS_VERSION = 3
DECIDE_INTERVAL_S = 1.0
SAVE_INTERVAL_S = 60.0
LOG_INTERVAL_S = 300.0

LOG_PATH = os.getenv("COMMA_PEDAL_AUTO_LOG_PATH", "/data/log/comma_pedal_auto.jsonl")
LOG_MAX_BYTES = 2 * 1024 * 1024


def _write_async(fn):
  threading.Thread(target=fn, daemon=True).start()


class CommaPedalAutoLearner:
  def __init__(self, stats=None, put_fn=None):
    """stats: the saved JSON (str/bytes) or None. put_fn(key, value) stores a param."""
    self.put_fn = put_fn
    self.level = 'mid'
    self.more = 0.0
    self.less = 0.0
    self._load(stats)
    # A new drive: earlier votes count half.
    self.more *= DRIVE_START_DECAY
    self.less *= DRIVE_START_DECAY

    self._push_s = 0.0
    self._push_at_press = 0.0
    self._last_pedal_t = -1e9
    self._prev_gas = False
    self._prev_brake = False
    self._pending_brake = None
    self._last_decide_t = -1e9
    self._last_change_t = -1e9
    self._last_save_t = None
    self._last_log_t = None
    self._dirty = False
    self._status = None
    self.counts = {'more': 0, 'less': 0, 'brake_ignored': 0, 'steps': 0}

  # ---- persistence ----
  def _load(self, stats):
    if not stats:
      return
    try:
      if isinstance(stats, bytes):
        stats = stats.decode('utf8')
      d = json.loads(stats)
      if int(d.get('v', 1)) < STATS_VERSION:
        # The old learner's result was pinned by one-sided feedback; start
        # again from the middle.
        return
      lvl = d.get('level')
      self.level = lvl if lvl in PROFILES else 'mid'
      self.more = max(0.0, float(d.get('more', 0.0)))
      self.less = max(0.0, float(d.get('less', 0.0)))
      if not (math.isfinite(self.more) and math.isfinite(self.less)):
        self.more = self.less = 0.0
    except Exception:
      self.level, self.more, self.less = 'mid', 0.0, 0.0

  def to_json(self):
    return json.dumps({'v': STATS_VERSION, 'level': self.level,
                       'more': round(self.more, 3), 'less': round(self.less, 3)},
                      separators=(',', ':'))

  @property
  def profile(self):
    return self.level

  @property
  def status(self):
    return self.level

  # ---- per-step update (call at ~20 Hz) ----
  def update(self, now, dt, v_ego, a_ego, gas_pressed, brake_pressed, long_active,
             op_accel, lead_headway_s, lead_closing, lat_accel, lane_change):
    gas = bool(gas_pressed)
    brake = bool(brake_pressed)
    v_ego = float(v_ego)

    if long_active:
      k = 0.5 ** (dt / HALF_LIFE_S)
      self.more *= k
      self.less *= k

    # How long openpilot has been accelerating on its own, with nothing that
    # would make a pedal press about something other than its strength.
    pushing = bool(long_active and not gas and not brake and
                   float(op_accel) > PUSH_ACCEL and v_ego > PUSH_MIN_V and
                   not lead_closing and abs(float(lat_accel)) <= LAT_ACCEL_MAX and
                   not lane_change)
    if pushing:
      self._push_s += dt
    elif not (gas or brake):
      self._push_s = 0.0

    gas_edge = gas and not self._prev_gas
    brake_edge = brake and not self._prev_brake
    fresh = now - self._last_pedal_t >= CLUSTER_S
    if gas_edge and fresh and self._push_s >= PUSH_MIN_S:
      self._vote(now, 'more', v_ego)
    elif brake_edge and fresh and self._push_s >= PUSH_MIN_S and v_ego >= BRAKE_MIN_V:
      self._pending_brake = {'t': now, 'v': v_ego}
    if gas or brake:
      self._last_pedal_t = now
      if gas_edge or brake_edge:
        self._push_s = 0.0

    pb = self._pending_brake
    if pb is not None:
      held = now - pb['t']
      if held > BRAKE_TAP_MAX_S:
        self._pending_brake = None
        self._ignored(now, pb, held, v_ego)
      elif not brake:
        self._pending_brake = None
        if v_ego >= BRAKE_TAP_KEEP_SPEED * pb['v']:
          self._vote(now, 'less', pb['v'])
        else:
          self._ignored(now, pb, held, v_ego)

    self._prev_gas = gas
    self._prev_brake = brake

    if now - self._last_decide_t >= DECIDE_INTERVAL_S:
      self._last_decide_t = now
      self._decide(now)
    self._maybe_persist(now)

  def _vote(self, now, side, v):
    if side == 'more':
      self.more += 1.0
    else:
      self.less += 1.0
    self.counts[side] += 1
    self._dirty = True
    self._log(now, 'vote_' + side, kph=round(v * 3.6))

  def _ignored(self, now, pb, held, v_now):
    self.counts['brake_ignored'] += 1
    self._log(now, 'brake_ignored', kph=round(pb['v'] * 3.6), held=round(held, 2),
              kph_after=round(v_now * 3.6))

  # ---- decision ----
  def _decide(self, now):
    total = self.more + self.less
    if total <= 0.0 or now - self._last_change_t < MIN_CHANGE_INTERVAL_S:
      return
    i = PROFILES.index(self.level)
    step = 0
    if self.more >= STEP_MIN_VOTES and self.more / total >= STEP_SHARE and i < len(PROFILES) - 1:
      step = 1
    elif self.less >= STEP_MIN_VOTES and self.less / total >= STEP_SHARE and i > 0:
      step = -1
    if step:
      old = self.level
      votes = (round(self.more, 2), round(self.less, 2))
      self.level = PROFILES[i + step]
      self.more = self.less = 0.0
      self._last_change_t = now
      self.counts['steps'] += 1
      self._dirty = True
      self._log(now, 'step', frm=old, to=self.level, more=votes[0], less=votes[1])

  # ---- output ----
  def _maybe_persist(self, now):
    if self.put_fn is None:
      return
    if self.level != self._status:
      self._status = self.level
      self.put_fn("CommaPedalAutoStatus", self.level)
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

  def _log(self, now, reason, **extra):
    self._last_log_t = now
    if self.put_fn is None:
      return
    d = {'reason': reason, 't': round(now, 1), 'level': self.level,
         'more': round(self.more, 2), 'less': round(self.less, 2), 'counts': dict(self.counts)}
    d.update(extra)
    line = json.dumps(d, separators=(',', ':'))

    def write():
      try:
        if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
          os.replace(LOG_PATH, LOG_PATH + ".1")
        with open(LOG_PATH, 'a') as f:
          f.write(line + "\n")
      except Exception:
        pass
    _write_async(write)
