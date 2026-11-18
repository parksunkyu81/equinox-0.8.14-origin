"""What the brake-minimizing follow control learns while driving.

1. Coast deceleration per speed bin: how hard this car slows with the comma
   pedal at zero and no brake. The lift timing in predictive_coasting.py
   compares the deceleration a closing lead requires against it.

2. Two adjustments from the driver's corrections while following a lead:
   * lift_margin (k): multiplies the required deceleration before it is
     compared with coast capability. A brake press while closing on a moving
     lead means the pedal came off too late -> k up (lift earlier).
   * gap_scale (s): widens or narrows the gap band in follow_gap.py. The same
     brake press widens it a little, but only when the car really was running
     close (within 10% of the floor gap) -- a wider gap did not cut brake
     presses and invited cut-ins; a gas press while following a lead that
     is not closing, with the gap already past the floor, means it was too
     far or too slow -> s and k down.
   Brake presses are only counted against a lead that is still moving
   (> 15 km/h) above 30 km/h, that has been tracked for 2 s (not a cut-in), out
   of corners and lane changes. Stopping for a light or a queue needs the brake
   whatever the gap -- counting those is what sank the earlier style learner.

3. A score to judge the change: brake presses per 10 km of following, also
   split into flowing traffic and jams.

In a traffic jam (traffic_state.JamDetector) the gas/brake corrections are
not learned: the driver presses the gas there to keep cut-ins out and brakes
because the queue stops. Coast deceleration is physics and is still learned.

State is saved to the CommaFollowLearn param once a minute and read back
by plannerd's dynamic follow for gap_scale. The first state seeds gap_scale
from the old Dynamic Follow multiplier (globalDfMod) so the gap does not jump.
"""
import json
import math
import os
import threading

from common.numpy_fast import clip, interp
from selfdrive.controls.lib.dynamic_follow.follow_gap import (
  FOLLOW_LEARN_PARAM, SCALE_MAX, SCALE_MIN, floor_tr, MPC_STOP_DISTANCE)

COAST_BINS_KPH = (10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130)
DEFAULT_COAST_BP_KPH = (0.0, 20.0, 60.0, 100.0, 130.0)
DEFAULT_COAST_MS2 = (0.30, 0.35, 0.50, 0.70, 0.80)
COAST_MIN_KPH = 10.0
COAST_SETTLE_S = 1.0          # pedal fall and engine braking take a moment
COAST_PEDAL_ZERO = 0.01
COAST_LAT_MAX = 1.0
COAST_ALPHA_MIN = 0.005       # long-run average over ~10 s of samples at 20 Hz
COAST_TRUST_SAMPLES = 40
COAST_CLIP = (0.05, 1.5)

FOLLOW_MAX_DIST_M = 80.0
LEAD_MIN_AGE_S = 2.0
CLOSING_VREL_MS = -0.3
CLOSING_MEMORY_S = 2.0
BRAKE_MIN_KPH = 30.0
BRAKE_LEAD_MIN_KPH = 15.0
GAS_MIN_KPH = 10.0
GAS_OP_ACCEL_MAX = 0.3
LAT_MAX = 1.5
FEEDBACK_COOLDOWN_S = 10.0

K_MIN, K_MAX = 0.8, 1.5
K_BRAKE_STEP = 0.05
S_BRAKE_STEP = 0.02
K_GAS_STEP = 0.03
BRAKE_WIDEN_GAP_FACTOR = 1.1   # a brake widens the gap only within 10% of the floor gap
S_GAS_STEP = 0.03

SAVE_INTERVAL_S = 60.0
LOG_INTERVAL_S = 300.0
LOG_PATH = os.getenv("FOLLOW_LEARN_LOG_PATH", "/data/log/follow_learn.jsonl")
LOG_MAX_BYTES = 2 * 1024 * 1024
PARAM_KEY = FOLLOW_LEARN_PARAM


def _write_async(fn):
  threading.Thread(target=fn, daemon=True).start()


class FollowLearner:
  def __init__(self, stats=None, put_fn=None, seed_scale=None):
    self.put_fn = put_fn
    self.coast = [[0.0, 0] for _ in COAST_BINS_KPH]   # [mean decel, samples]
    self._coast_curve = None
    self.gap_scale = 1.0
    self.lift_margin = 1.0
    if not self._load(stats) and seed_scale is not None:
      try:
        self.gap_scale = float(clip(float(seed_scale), SCALE_MIN, SCALE_MAX))
      except (TypeError, ValueError):
        pass

    self._coast_hold = 0.0
    self._lead_age = 0.0
    self._last_closing_t = -1e9
    self._prev_gas = False
    self._prev_brake = False
    self._last_feedback_t = -1e9
    self.follow_m = 0.0
    self.follow_brakes = 0
    self.brakes_counted = 0
    self.gas_counted = 0
    self.follow_m_jam = 0.0
    self.follow_brakes_jam = 0
    self.jam_skipped = 0
    self._dirty = False
    self._last_save_t = None
    self._last_log_t = None

  # ---- persistence ----
  def _load(self, stats):
    if not stats:
      return False
    try:
      if isinstance(stats, bytes):
        stats = stats.decode('utf8')
      d = json.loads(stats)
      self.gap_scale = float(clip(float(d.get('s', 1.0)), SCALE_MIN, SCALE_MAX))
      self.lift_margin = float(clip(float(d.get('k', 1.0)), K_MIN, K_MAX))
      coast = d.get('coast', [])
      if len(coast) == len(COAST_BINS_KPH):
        self.coast = [[float(clip(float(m), *COAST_CLIP)), int(n)] for m, n in coast]
      return True
    except Exception:
      self.coast = [[0.0, 0] for _ in COAST_BINS_KPH]
      self.gap_scale, self.lift_margin = 1.0, 1.0
      return False

  def to_json(self):
    return json.dumps({
      'v': 1, 's': round(self.gap_scale, 3), 'k': round(self.lift_margin, 3),
      'coast': [[round(m, 3), n] for m, n in self.coast],
    }, separators=(',', ':'))

  # ---- coast table ----
  def coast_decel(self, v_ego):
    """Coast deceleration (positive m/s^2) at this speed: learned bins with
    enough samples, the default curve elsewhere."""
    # Called at 100 Hz by predictive coasting: the curve is rebuilt only when
    # a bin changes (at most 20 Hz, and only while coasting).
    if self._coast_curve is None:
      self._coast_curve = [
        m if n >= COAST_TRUST_SAMPLES else float(interp(b, DEFAULT_COAST_BP_KPH, DEFAULT_COAST_MS2))
        for b, (m, n) in zip(COAST_BINS_KPH, self.coast)]
    return float(interp(max(0.0, float(v_ego)) * 3.6, COAST_BINS_KPH, self._coast_curve))

  def _update_coast(self, dt, v_ego, a_ego, pedal_cmd, gas, brake, long_active, lat_accel):
    coasting = (long_active and not gas and not brake and pedal_cmd <= COAST_PEDAL_ZERO and
                v_ego * 3.6 >= COAST_MIN_KPH and abs(lat_accel) < COAST_LAT_MAX)
    self._coast_hold = self._coast_hold + dt if coasting else 0.0
    if self._coast_hold < COAST_SETTLE_S:
      return
    kph = v_ego * 3.6
    i = min(range(len(COAST_BINS_KPH)), key=lambda j: abs(COAST_BINS_KPH[j] - kph))
    m, n = self.coast[i]
    sample = float(clip(-a_ego, *COAST_CLIP))
    n += 1
    alpha = max(1.0 / n, COAST_ALPHA_MIN)
    self.coast[i] = [m + alpha * (sample - m), n]
    self._coast_curve = None
    self._dirty = True

  # ---- per-step update (~20 Hz) ----
  def update(self, now, dt, v_ego, a_ego, pedal_cmd, gas_pressed, brake_pressed, long_active,
             op_accel, lat_accel, lane_change, lead_status, lead_dist, lead_vrel, jam=False):
    v_ego = max(0.0, float(v_ego))
    gas = bool(gas_pressed)
    brake = bool(brake_pressed)
    jam = bool(jam)
    self._update_coast(dt, v_ego, float(a_ego), float(pedal_cmd), gas, brake, bool(long_active),
                       float(lat_accel))

    following = bool(long_active and lead_status and 0.0 < lead_dist < FOLLOW_MAX_DIST_M)
    self._lead_age = self._lead_age + dt if (lead_status and lead_dist > 0.0) else 0.0
    if following:
      self.follow_m += v_ego * dt
      if jam:
        self.follow_m_jam += v_ego * dt
    if following and lead_vrel < CLOSING_VREL_MS:
      self._last_closing_t = now
    calm = abs(lat_accel) < LAT_MAX and not lane_change
    seasoned = self._lead_age >= LEAD_MIN_AGE_S

    if jam and following and ((brake and not self._prev_brake) or (gas and not self._prev_gas)):
      self.jam_skipped += 1
    if brake and not self._prev_brake and following:
      self.follow_brakes += 1
      if jam:
        self.follow_brakes_jam += 1
      lead_speed_kph = (v_ego + lead_vrel) * 3.6
      if (not jam and seasoned and calm and v_ego * 3.6 >= BRAKE_MIN_KPH and
          lead_speed_kph >= BRAKE_LEAD_MIN_KPH and
          now - self._last_closing_t <= CLOSING_MEMORY_S and
          now - self._last_feedback_t >= FEEDBACK_COOLDOWN_S):
        self._last_feedback_t = now
        self.lift_margin = min(K_MAX, self.lift_margin + K_BRAKE_STEP)
        # Widen the gap only for a brake that came from really running close.
        # Otherwise the answer is lifting earlier (lift_margin, above): in
        # flowing traffic on 11-16..18 brake presses ran at 33-52 per 10 min
        # whatever the time gap, while cut-ins went from 0-4 per 10 min below
        # 2.2 s to 11 at 2.2-3.5 s and 44 beyond.
        if lead_dist <= (MPC_STOP_DISTANCE + v_ego * floor_tr(v_ego)) * BRAKE_WIDEN_GAP_FACTOR:
          self.gap_scale = min(SCALE_MAX, self.gap_scale + S_BRAKE_STEP)
        self.brakes_counted += 1
        self._dirty = True

    if gas and not self._prev_gas and following:
      floor_gap = MPC_STOP_DISTANCE + v_ego * floor_tr(v_ego)
      if (not jam and seasoned and calm and v_ego * 3.6 >= GAS_MIN_KPH and lead_vrel > CLOSING_VREL_MS and
          lead_dist > floor_gap * 1.1 and op_accel < GAS_OP_ACCEL_MAX and
          now - self._last_feedback_t >= FEEDBACK_COOLDOWN_S):
        self._last_feedback_t = now
        self.gap_scale = max(SCALE_MIN, self.gap_scale - S_GAS_STEP)
        self.lift_margin = max(K_MIN, self.lift_margin - K_GAS_STEP)
        self.gas_counted += 1
        self._dirty = True

    self._prev_gas = gas
    self._prev_brake = brake
    self._maybe_persist(now)

  # ---- output ----
  def brakes_per_10km(self):
    km = self.follow_m / 1000.0
    return round(self.follow_brakes / km * 10.0, 2) if km >= 0.5 else None

  @staticmethod
  def _per_10km(brakes, m):
    return round(brakes / (m / 1000.0) * 10.0, 2) if m >= 500.0 else None

  def _maybe_persist(self, now):
    if self.put_fn is None:
      return
    if self._last_save_t is None:
      self._last_save_t = now
      self._last_log_t = now
      return
    if self._dirty and now - self._last_save_t >= SAVE_INTERVAL_S:
      self._last_save_t = now
      self._dirty = False
      self.put_fn(PARAM_KEY, self.to_json())
    if now - self._last_log_t >= LOG_INTERVAL_S:
      self._last_log_t = now
      self._log(now)

  def _log(self, now):
    line = json.dumps({
      't': round(now, 1),
      'follow_km': round(self.follow_m / 1000.0, 2),
      'follow_brakes': self.follow_brakes,
      'brakes_per_10km': self.brakes_per_10km(),
      'flow_km': round((self.follow_m - self.follow_m_jam) / 1000.0, 2),
      'flow_brakes_per_10km': self._per_10km(self.follow_brakes - self.follow_brakes_jam,
                                             self.follow_m - self.follow_m_jam),
      'jam_km': round(self.follow_m_jam / 1000.0, 2),
      'jam_brakes_per_10km': self._per_10km(self.follow_brakes_jam, self.follow_m_jam),
      'jam_skipped': self.jam_skipped,
      'brakes_counted': self.brakes_counted, 'gas_counted': self.gas_counted,
      'gap_scale': round(self.gap_scale, 3), 'lift_margin': round(self.lift_margin, 3),
      'coast_kph_decel_n': [[b, round(m, 2), n] for b, (m, n) in zip(COAST_BINS_KPH, self.coast) if n],
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
