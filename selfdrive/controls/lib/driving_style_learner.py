"""Bounded online driver-style adaptation for the GM gas-interceptor setup.

Acceleration style is measured directly instead of scored from rare events.
The driver's own acceleration and openpilot's acceleration are accumulated
into the same speed bins, and the ratio between them is the learned pedal
gain. The driver reference is collected whenever the driver owns the pedal --
an engaged override or fully manual driving -- so evidence no longer depends
on a clean-context event surviving a long condition chain.

Braking contributes on two independent paths. The always-available one is the
brake veto rate: the fraction of openpilot acceleration episodes the driver
overruled with the brake pedal. It needs no radar and no vision lead, and it
is the only negative acceleration evidence this car can produce reliably.
The lead-dependent path still feeds time-gap (TR) learning as before.
"""

import json
import math
import time
from dataclasses import dataclass


STATE_VERSION = 3

GAIN_MIN = 0.85
GAIN_MAX = 1.12
# How far evidence is allowed to move the gain from neutral, scaled by
# confidence. Down is wider than up: braking is a direct complaint, while a
# request for more acceleration is inferred.
GAIN_DOWN_AUTHORITY = 0.15
GAIN_UP_AUTHORITY = 0.12
TR_OFFSET_MIN = -0.20
TR_OFFSET_MAX = 0.40

# Make a learned preference perceptible without widening the learner's raw
# safety bounds. Low/mid-speed acceleration gets the clearest response, while
# highway gain and time-gap shortening stay deliberately conservative.
STYLE_RESPONSE_BP_KPH = (0.0, 30.0, 60.0, 100.0, 130.0)
GAIN_RESPONSE_V = (2.0, 2.0, 1.7, 1.4, 1.2)
GAIN_APPLIED_MAX_V = (1.08, 1.08, 1.065, 1.045, 1.04)
GAIN_APPLIED_MIN = 0.85
TR_RESPONSE_V = (4.0, 3.8, 3.2, 2.5, 2.0)
TR_APPLIED_MIN_V = (-0.12, -0.12, -0.10, -0.08, -0.06)
TR_APPLIED_MAX_V = (0.25, 0.25, 0.28, 0.30, 0.30)

SPEED_BIN_CENTERS_KPH = (5.0, 20.0, 45.0, 80.0, 115.0)
SPEED_BIN_EDGES_KPH = (10.0, 30.0, 60.0, 100.0)

PARAM_REFRESH_S = 1.0
SAVE_INTERVAL_S = 60.0
LEARNING_UPDATE_COOLDOWN_S = 30.0
MIN_TR_EVENTS_PER_UPDATE = 3
MIN_STABLE_FOLLOW_S = 60.0

# --- Paired acceleration measurement -----------------------------------------
# Sampling is decimated from the 100 Hz control loop. Only real acceleration
# episodes are sampled: a coasting or cruising frame says nothing about how
# hard the driver likes to accelerate, and including those frames would drag
# both sides toward zero and make the ratio meaningless.
ACCEL_SAMPLE_DT_S = 0.10
ACCEL_SAMPLE_MIN_MS2 = 0.15
ACCEL_SAMPLE_MAX_MS2 = 3.00
# EMA time constant expressed in sampled seconds, not wall-clock seconds.
ACCEL_EMA_TAU_S = 90.0
ACCEL_EMA_MIN_ALPHA = ACCEL_SAMPLE_DT_S / ACCEL_EMA_TAU_S
ACCEL_SAMPLE_COUNT_MAX = 100000
# A bin contributes only once both sides have seen this many samples, i.e.
# ~12 s of qualifying acceleration from the driver and from openpilot.
ACCEL_MIN_PAIRED_SAMPLES = 120
# Global evidence required before the gain is allowed to move at all.
ACCEL_MIN_TOTAL_SAMPLES = 300
# Only the upper bound remains. The paired measurement may raise the gain but
# never lower it -- see _maybe_apply_gain.
ACCEL_RATIO_MAX = 1.35
# Maximum gain movement per update. With the 30 s cooldown this bounds the
# learner to roughly +0.02/minute, so a bad measurement can never step the
# pedal response.
GAIN_STEP_MAX = 0.01
BIN_OFFSET_MAX = 0.03

# --- Brake veto --------------------------------------------------------------
# A brake press that lands while openpilot was commanding positive pedal is an
# unambiguous "that was too much" from the driver, and unlike the time-gap
# evidence it needs no lead at all. The rate is a plain fraction: 0.0 means the
# driver never overrules openpilot's acceleration, 1.0 means always.
#
# How long openpilot acceleration must stay stopped before the episode counts
# as accepted by the driver.
BRAKE_VETO_EPISODE_END_S = 0.7
BRAKE_VETO_MIN_ALPHA = 1.0 / 60.0
BRAKE_VETO_COUNT_MAX = 100000
# Counted in openpilot acceleration episodes, not control frames: the numerator
# is one brake press, so the denominator has to be an episode too.
BRAKE_VETO_MIN_SAMPLES = 12
# Braking out of openpilot's acceleration is the primary downward signal, so it
# maps straight to a target gain instead of shading someone else's number.
BRAKE_VETO_BP = (0.00, 0.15, 0.30, 0.50)
BRAKE_VETO_TARGET_GAIN = (1.00, 0.95, 0.90, 0.85)
# A brake landing in the first moments of an episode is usually a reaction to
# something outside the car, not a verdict on how hard it just accelerated.
BRAKE_VETO_MIN_EPISODE_S = 0.5

# Low-speed manual-brake learning does not change the always-on following TR.
# It learns a small, closing-only predictive-coasting headway so the comma
# pedal is released slightly earlier without inviting cut-ins on a steady or
# receding lead.
LOW_SPEED_COAST_MIN_KPH = 3.0
LOW_SPEED_COAST_MAX_KPH = 35.0
LOW_SPEED_COAST_MAX_OFFSET_S = 0.10
LOW_SPEED_COAST_FIRST_OFFSET_S = 0.05
LOW_SPEED_COAST_UPDATE_STEP_S = 0.01
LOW_SPEED_COAST_EVENTS_PER_UPDATE = 3
LOW_SPEED_COAST_MIN_PEDAL = 0.01
LOW_SPEED_COAST_MAX_VREL_MS = -0.20
LOW_SPEED_COAST_ZERO_PEDAL_MAX_VREL_MS = -0.50
LOW_SPEED_COAST_BASE_DISTANCE_MARGIN_M = 3.0
LOW_SPEED_COAST_MAX_DISTANCE_MARGIN_M = 5.0
LOW_SPEED_COAST_DISTANCE_MARGIN_SPEED_S = 0.20
LOW_SPEED_COAST_ZERO_PEDAL_MAX_DISTANCE_MARGIN_M = 1.5
LOW_SPEED_COAST_STANDSTILL_GAP_M = 4.5
LOW_SPEED_COAST_SNAPSHOT_MAX_AGE_S = 0.30
LOW_SPEED_COAST_MAX_EVENT_DURATION_S = 6.0

MIN_EVENT_DURATION_S = 0.12
MAX_EVENT_DURATION_S = 4.0
MIN_LEARN_SPEED_MS = 1.0 / 3.6
MIN_FOLLOW_LEARN_SPEED_MS = 18.0 / 3.6
MIN_LEAD_DISTANCE_M = 3.0
MAX_LEAD_DISTANCE_M = 160.0
LEAD_STABLE_REQUIRED_S = 1.0
LEAD_DISTANCE_JUMP_M = 6.0

_NUM_BINS = len(SPEED_BIN_CENTERS_KPH)


def _clip(value, lower, upper):
  return max(lower, min(upper, value))


def _finite(value, default=0.0):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return float(default)
  return value if math.isfinite(value) else float(default)


def _interp(value, xp, fp):
  if value <= xp[0]:
    return fp[0]
  if value >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if value <= xp[i]:
      ratio = (value - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + ratio * (fp[i] - fp[i - 1])
  return fp[-1]


def applied_driving_style_gain(raw_gain, v_ego):
  speed_kph = max(0.0, _finite(v_ego)) * 3.6
  response = _interp(speed_kph, STYLE_RESPONSE_BP_KPH, GAIN_RESPONSE_V)
  upper = _interp(speed_kph, STYLE_RESPONSE_BP_KPH, GAIN_APPLIED_MAX_V)
  gain = 1.0 + (_finite(raw_gain, 1.0) - 1.0) * response
  return _clip(gain, GAIN_APPLIED_MIN, upper)


def applied_driving_style_tr_offset(raw_offset, v_ego):
  speed_kph = max(0.0, _finite(v_ego)) * 3.6
  response = _interp(speed_kph, STYLE_RESPONSE_BP_KPH, TR_RESPONSE_V)
  lower = _interp(speed_kph, STYLE_RESPONSE_BP_KPH, TR_APPLIED_MIN_V)
  upper = _interp(speed_kph, STYLE_RESPONSE_BP_KPH, TR_APPLIED_MAX_V)
  return _clip(_finite(raw_offset) * response, lower, upper)


@dataclass(frozen=True)
class DrivingStyleStatus:
  enabled: bool
  gain: float
  tr_offset: float
  confidence: float
  gas_events: int
  brake_events: int
  stable_follow_s: float
  low_speed_coast_offset_s: float
  low_speed_brake_events: int
  low_speed_coast_updates: int


class DrivingStyleLearner:
  """Bounded, measurement-based online adaptation for the gas interceptor.

  The class does not determine gasPressed itself; it consumes the existing
  CarState.gasPressed value so the GM `ret.gas > 15` behavior stays unchanged.
  """

  def __init__(self, params=None, writer=None, clock=None):
    if params is None:
      from common.params import Params
      params = Params()

    self.params = params
    self._writer = writer
    self._clock = clock or time.monotonic

    self.enabled = False
    self.global_gain = 1.0
    self.bin_offsets = [0.0] * _NUM_BINS
    self.tr_offset = 0.0
    self.low_speed_coast_offset_s = 0.0

    # Paired acceleration statistics. driver_* is what the driver produces,
    # op_* is what openpilot produces, both in the same speed bin.
    self.driver_accel = [0.0] * _NUM_BINS
    self.driver_count = [0] * _NUM_BINS
    self.op_accel = [0.0] * _NUM_BINS
    self.op_count = [0] * _NUM_BINS

    # Fraction of openpilot acceleration episodes the driver braked out of.
    self.brake_veto_rate = 0.0
    self.brake_veto_count = 0

    self.gas_events = 0
    self.brake_events = 0
    self.gain_evidence_events = 0
    self.tr_evidence_events = 0
    self.stable_follow_s = 0.0
    self.gain_updates = 0
    self.tr_updates = 0
    self.low_speed_brake_events = 0
    self.low_speed_coast_updates = 0
    self.low_speed_brake_batch_events = 0

    self.tr_batch_score = 0.0
    self.tr_batch_events = 0

    self._paired_ratio = None
    self._paired_weight = 0.0
    self._bin_gains = [1.0] * _NUM_BINS

    self._gas_event = None
    self._brake_event = None
    self._low_speed_brake_event = None
    self._lead_stable_s = 0.0
    self._last_lead_distance = None
    self._last_control_active = False
    self._last_param_refresh = -1e9
    self._last_save = self._clock()
    self._last_gain_update = -1e9
    self._last_tr_update = -1e9
    self._sample_accum = 0.0
    self._last_op_pedal_time = -1e9
    self._prev_brake_pressed = False
    self._op_episode_active = False
    self._op_episode_idle_s = 0.0
    self._op_episode_elapsed_s = 0.0
    self._dirty = False

    # Per-frame snapshots are held as flat attributes. Building a dict on every
    # control frame allocated ~200 dicts/s for data that is read only when an
    # event actually starts.
    self._clear_snapshot()
    self._clear_low_speed_snapshot()

    self._load_state()
    self._refresh_pair_stats()
    self._refresh_bin_gains()
    self._refresh_enabled(self._clock(), force=True)

  # --- snapshots -------------------------------------------------------------

  def _clear_snapshot(self):
    self._snap_valid = False
    self._snap_time = -1e9
    self._snap_v_ego = 0.0
    self._snap_bin = 0
    self._snap_requested_accel = 0.0
    self._snap_lead_valid = False
    self._snap_lead_distance = 0.0
    self._snap_lead_rel_speed = 0.0
    self._snap_base_tr = 1.3
    self._snap_lead_stable_s = 0.0

  def _clear_low_speed_snapshot(self):
    self._ls_valid = False
    self._ls_time = -1e9
    self._ls_v_ego = 0.0
    self._ls_requested_accel = 0.0
    self._ls_lead_valid = False
    self._ls_lead_distance = 0.0
    self._ls_lead_rel_speed = 0.0
    self._ls_base_tr = 1.3
    self._ls_pedal_output = 0.0

  @staticmethod
  def _speed_bin(v_ego):
    speed_kph = max(0.0, _finite(v_ego)) * 3.6
    for index, edge in enumerate(SPEED_BIN_EDGES_KPH):
      if speed_kph < edge:
        return index
    return _NUM_BINS - 1

  # --- params ----------------------------------------------------------------

  def _param_text(self, key, default=""):
    try:
      value = self.params.get(key, encoding="utf8")
    except TypeError:
      value = self.params.get(key)
      if isinstance(value, bytes):
        value = value.decode("utf8", errors="ignore")
    except Exception:
      return default
    return default if value is None else str(value)

  def _param_bool(self, key):
    try:
      return bool(self.params.get_bool(key))
    except Exception:
      return self._param_text(key, "0") == "1"

  def _queue_put(self, key, value):
    value = str(value)
    if self._writer is not None:
      self._writer(key, value)
      return
    from common.params import put_nonblocking
    put_nonblocking(key, value)

  def _load_state(self):
    raw = self._param_text("DrivingStyleAIState")
    if not raw:
      return
    try:
      state = json.loads(raw)
      version = int(state.get("version", 0))
      if version not in (1, 2, STATE_VERSION):
        return

      # Time-gap and low-speed coast learning have never changed meaning and
      # are expensive to re-earn, so they carry over from every version.
      self._load_v1_shared(state)

      if version >= 2:
        # The paired acceleration measurement is unchanged; only what the gain
        # rule does with it changed.
        for name in ("driver_accel", "op_accel"):
          values = state.get(name, [])
          if isinstance(values, list) and len(values) == _NUM_BINS:
            setattr(self, name, [_clip(_finite(v), 0.0, ACCEL_SAMPLE_MAX_MS2) for v in values])
        for name in ("driver_count", "op_count"):
          values = state.get(name, [])
          if isinstance(values, list) and len(values) == _NUM_BINS:
            setattr(self, name, [int(_clip(int(v), 0, ACCEL_SAMPLE_COUNT_MAX)) for v in values])
        self.gain_evidence_events = max(0, int(state.get("gain_evidence_events", 0)))

      if version >= STATE_VERSION:
        # v2 counted a brake-veto episode as ending only once the pedal had been
        # released, which openpilot rarely does while holding speed. Every
        # episode therefore closed on a brake press and the stored rate ran to
        # ~1.0, so v2 values are discarded rather than migrated.
        self.brake_veto_rate = _clip(_finite(state.get("brake_veto_rate")), 0.0, 1.0)
        self.brake_veto_count = max(0, min(BRAKE_VETO_COUNT_MAX,
                                           int(state.get("brake_veto_count", 0))))
        # The gain rule itself changed in v3, so an older gain is not carried:
        # it is re-derived from the retained evidence within a few updates.
        self.global_gain = _clip(_finite(state.get("global_gain"), 1.0), GAIN_MIN, GAIN_MAX)
        offsets = state.get("bin_offsets", [])
        if isinstance(offsets, list) and len(offsets) == _NUM_BINS:
          self.bin_offsets = [_clip(_finite(v), -BIN_OFFSET_MAX, BIN_OFFSET_MAX) for v in offsets]
        self.gain_updates = max(0, int(state.get("gain_updates", 0)))

      self._dirty = True
    except (TypeError, ValueError, json.JSONDecodeError):
      # Invalid or partially-written state is ignored; bounded defaults remain.
      return

  def _load_v1_shared(self, state):
    """Carry over the v1 fields whose meaning is identical in v2."""
    self.tr_offset = _clip(_finite(state.get("tr_offset")), TR_OFFSET_MIN, TR_OFFSET_MAX)
    self.low_speed_coast_offset_s = _clip(
      _finite(state.get("low_speed_coast_offset_s")), 0.0,
      LOW_SPEED_COAST_MAX_OFFSET_S)
    self.stable_follow_s = max(0.0, _finite(state.get("stable_follow_s")))
    self.tr_batch_score = _finite(state.get("tr_batch_score"))
    self.tr_batch_events = max(0, int(state.get("tr_batch_events", 0)))
    for name in ("gas_events", "brake_events", "tr_evidence_events", "tr_updates",
                 "low_speed_brake_events", "low_speed_coast_updates",
                 "low_speed_brake_batch_events"):
      setattr(self, name, max(0, int(state.get(name, 0))))

  def _state_dict(self):
    return {
      "version": STATE_VERSION,
      "global_gain": round(self.global_gain, 6),
      "bin_offsets": [round(value, 6) for value in self.bin_offsets],
      "tr_offset": round(self.tr_offset, 6),
      "low_speed_coast_offset_s": round(self.low_speed_coast_offset_s, 6),
      "driver_accel": [round(value, 6) for value in self.driver_accel],
      "driver_count": list(self.driver_count),
      "op_accel": [round(value, 6) for value in self.op_accel],
      "op_count": list(self.op_count),
      "brake_veto_rate": round(self.brake_veto_rate, 6),
      "brake_veto_count": self.brake_veto_count,
      "gas_events": self.gas_events,
      "brake_events": self.brake_events,
      "gain_evidence_events": self.gain_evidence_events,
      "tr_evidence_events": self.tr_evidence_events,
      "stable_follow_s": round(self.stable_follow_s, 3),
      "gain_updates": self.gain_updates,
      "tr_updates": self.tr_updates,
      "low_speed_brake_events": self.low_speed_brake_events,
      "low_speed_coast_updates": self.low_speed_coast_updates,
      "low_speed_brake_batch_events": self.low_speed_brake_batch_events,
      "tr_batch_score": round(self.tr_batch_score, 6),
      "tr_batch_events": self.tr_batch_events,
    }

  def _save(self, now, force=False):
    if not self._dirty or (not force and now - self._last_save < SAVE_INTERVAL_S):
      return
    status = self.status(0.0)
    self._queue_put("DrivingStyleAIState", json.dumps(self._state_dict(), separators=(",", ":")))
    self._queue_put("DrivingStyleAIGain", "{:.6f}".format(self.global_gain))
    self._queue_put("DrivingStyleAITrOffset", "{:.6f}".format(self.tr_offset))
    self._queue_put("DrivingStyleAIConfidence", "{:.6f}".format(status.confidence))
    self._last_save = now
    self._dirty = False

  def _refresh_enabled(self, now, force=False):
    if not force and now - self._last_param_refresh < PARAM_REFRESH_S:
      return
    was_enabled = self.enabled
    self.enabled = self._param_bool("DrivingStyleAI")
    self._last_param_refresh = now
    if was_enabled != self.enabled:
      self._gas_event = None
      self._brake_event = None
      self._low_speed_brake_event = None
      self._lead_stable_s = 0.0
      self._last_lead_distance = None
      self._sample_accum = 0.0
      self._last_op_pedal_time = -1e9
      self._op_episode_active = False
      self._op_episode_idle_s = 0.0
      self._op_episode_elapsed_s = 0.0
      self._clear_snapshot()
      self._clear_low_speed_snapshot()
      if was_enabled and not self.enabled:
        self._save(now, force=True)

  # --- gain ------------------------------------------------------------------

  def _refresh_bin_gains(self):
    """Precompute the per-bin gain so the per-frame path is one interpolation."""
    self._bin_gains = [_clip(self.global_gain + offset, GAIN_MIN, GAIN_MAX)
                       for offset in self.bin_offsets]

  def gain_for_speed(self, v_ego):
    if not self.enabled:
      return 1.0
    speed_kph = max(0.0, _finite(v_ego)) * 3.6
    raw_gain = _clip(_interp(speed_kph, SPEED_BIN_CENTERS_KPH, self._bin_gains),
                     GAIN_MIN, GAIN_MAX)
    return applied_driving_style_gain(raw_gain, v_ego)

  def _refresh_pair_stats(self):
    """Weighted ratio of driver acceleration to openpilot acceleration.

    Each bin is weighted by its weaker side, so a bin where only one side has
    data contributes nothing and cannot bias the result.
    """
    numerator = 0.0
    weight = 0.0
    for index in range(_NUM_BINS):
      paired = min(self.driver_count[index], self.op_count[index])
      if paired < ACCEL_MIN_PAIRED_SAMPLES or self.op_accel[index] <= 1e-3:
        continue
      numerator += paired * (self.driver_accel[index] / self.op_accel[index])
      weight += paired
    if weight <= 0.0:
      self._paired_ratio = None
      self._paired_weight = 0.0
    else:
      self._paired_ratio = numerator / weight
      self._paired_weight = weight

  def _push_accel_sample(self, is_driver, speed_bin, a_ego):
    if is_driver:
      means, counts = self.driver_accel, self.driver_count
    else:
      means, counts = self.op_accel, self.op_count

    count = counts[speed_bin]
    if count < ACCEL_SAMPLE_COUNT_MAX:
      count += 1
      counts[speed_bin] = count
    # Converge quickly while the bin is empty, then settle to the slow EMA.
    alpha = max(ACCEL_EMA_MIN_ALPHA, 1.0 / count)
    means[speed_bin] += alpha * (a_ego - means[speed_bin])

    self.gain_evidence_events = min(ACCEL_SAMPLE_COUNT_MAX, self.gain_evidence_events + 1)
    self._refresh_pair_stats()
    self._dirty = True

  def _push_brake_veto(self, vetoed):
    count = self.brake_veto_count
    if count < BRAKE_VETO_COUNT_MAX:
      count += 1
      self.brake_veto_count = count
    alpha = max(BRAKE_VETO_MIN_ALPHA, 1.0 / count)
    self.brake_veto_rate += alpha * ((1.0 if vetoed else 0.0) - self.brake_veto_rate)
    self.brake_veto_rate = _clip(self.brake_veto_rate, 0.0, 1.0)
    self._dirty = True

  def brake_veto_target(self):
    """Target gain implied by how often the driver brakes out of acceleration.

    Returns (target, confidence), or (1.0, 0.0) when there is not yet enough
    evidence. This is the primary downward signal: a driver who lets the comma
    pedal do the accelerating produces almost no gas samples, so braking is the
    only preference this setup reliably observes.
    """
    if self.brake_veto_count < BRAKE_VETO_MIN_SAMPLES:
      return 1.0, 0.0
    target = _interp(self.brake_veto_rate, BRAKE_VETO_BP, BRAKE_VETO_TARGET_GAIN)
    confidence = min(1.0, self.brake_veto_count / (4.0 * BRAKE_VETO_MIN_SAMPLES))
    return target, confidence

  def _maybe_apply_gain(self, now):
    if now - self._last_gain_update < LEARNING_UPDATE_COOLDOWN_S:
      return False

    brake_target, brake_confidence = self.brake_veto_target()
    accel_ready = (self._paired_ratio is not None and
                   self._paired_weight >= ACCEL_MIN_TOTAL_SAMPLES)
    if brake_confidence <= 0.0 and not accel_ready:
      return False

    self._last_gain_update = now

    # The paired acceleration measurement may only RAISE the gain. Reading a
    # handful of gentle manual-driving samples as "this driver wants half the
    # acceleration" is what pinned the gain to its floor for a driver who lets
    # the comma pedal do the accelerating.
    accel_target, accel_confidence = 1.0, 0.0
    if accel_ready and self._paired_ratio > 1.0:
      accel_target = min(ACCEL_RATIO_MAX, self._paired_ratio)
      accel_confidence = min(1.0, self._paired_weight / (4.0 * ACCEL_MIN_TOTAL_SAMPLES))

    # Braking wins outright when it asks for less: it is a direct complaint
    # about acceleration that already happened.
    if brake_target < 1.0:
      target, confidence = brake_target, brake_confidence
    else:
      target, confidence = accel_target, accel_confidence

    # Authority grows with evidence, so a thin measurement can only nudge.
    target = _clip(target,
                   max(GAIN_MIN, 1.0 - GAIN_DOWN_AUTHORITY * confidence),
                   min(GAIN_MAX, 1.0 + GAIN_UP_AUTHORITY * confidence))
    step = _clip(target - self.global_gain, -GAIN_STEP_MAX, GAIN_STEP_MAX)
    new_gain = _clip(self.global_gain + step, GAIN_MIN, GAIN_MAX)
    changed = abs(new_gain - self.global_gain) > 1e-6
    self.global_gain = new_gain

    # Per-speed residuals come from the paired measurement, so they are only
    # meaningful while that measurement is the one steering the gain.
    if accel_confidence > 0.0 and target == accel_target:
      for index in range(_NUM_BINS):
        paired = min(self.driver_count[index], self.op_count[index])
        if paired < ACCEL_MIN_PAIRED_SAMPLES or self.op_accel[index] <= 1e-3:
          continue
        bin_ratio = self.driver_accel[index] / self.op_accel[index]
        bin_confidence = min(1.0, paired / (4.0 * ACCEL_MIN_PAIRED_SAMPLES))
        bound = BIN_OFFSET_MAX * bin_confidence
        self.bin_offsets[index] = _clip(0.5 * (bin_ratio - self._paired_ratio),
                                        -bound, bound)

    if changed:
      self.gain_updates += 1
    self._refresh_bin_gains()
    self._dirty = True
    return changed

  def confidence(self):
    gain_confidence = max(
      min(1.0, self._paired_weight / (4.0 * ACCEL_MIN_TOTAL_SAMPLES)),
      min(1.0, self.brake_veto_count / (4.0 * BRAKE_VETO_MIN_SAMPLES)))
    tr_confidence = 0.5 * min(1.0, self.tr_evidence_events / 10.0) + \
                    0.5 * min(1.0, self.stable_follow_s / 300.0)
    return _clip(max(gain_confidence, tr_confidence), 0.0, 1.0)

  def status(self, v_ego):
    return DrivingStyleStatus(
      enabled=self.enabled,
      gain=self.gain_for_speed(v_ego),
      tr_offset=applied_driving_style_tr_offset(self.tr_offset, v_ego) if self.enabled else 0.0,
      confidence=self.confidence(),
      gas_events=self.gas_events,
      brake_events=self.brake_events,
      stable_follow_s=self.stable_follow_s,
      low_speed_coast_offset_s=(self.low_speed_coast_offset_s
                                if self.enabled else 0.0),
      low_speed_brake_events=self.low_speed_brake_events,
      low_speed_coast_updates=self.low_speed_coast_updates,
    )

  # --- lead / events ---------------------------------------------------------

  def _update_lead_stability(self, context_ok, lead_valid, lead_distance, lead_rel_speed, dt):
    lead_distance = _finite(lead_distance)
    lead_rel_speed = _finite(lead_rel_speed)
    plausible = context_ok and lead_valid and \
        MIN_LEAD_DISTANCE_M <= lead_distance <= MAX_LEAD_DISTANCE_M
    if plausible and self._last_lead_distance is not None:
      expected_delta = lead_rel_speed * dt
      if abs((lead_distance - self._last_lead_distance) - expected_delta) > LEAD_DISTANCE_JUMP_M:
        plausible = False

    if plausible:
      self._lead_stable_s += dt
      self._last_lead_distance = lead_distance
    else:
      self._lead_stable_s = 0.0
      self._last_lead_distance = lead_distance if lead_valid else None

  def _start_event(self, now, signal_value):
    if not self._snap_valid or now - self._snap_time > 0.5:
      return None
    if not self._last_control_active or self._snap_v_ego < MIN_LEARN_SPEED_MS:
      return None
    return {
      "v_ego": self._snap_v_ego,
      "speed_bin": self._snap_bin,
      "requested_accel": self._snap_requested_accel,
      "lead_valid": self._snap_lead_valid,
      "lead_distance": self._snap_lead_distance,
      "lead_rel_speed": self._snap_lead_rel_speed,
      "base_tr": self._snap_base_tr,
      "lead_stable_s": self._snap_lead_stable_s,
      "start": now,
      "peak": max(0.0, _finite(signal_value)),
      "valid": True,
    }

  def _low_speed_brake_candidate(self, now):
    if not self._ls_valid or now - self._ls_time > LOW_SPEED_COAST_SNAPSHOT_MAX_AGE_S:
      return None
    if not self._last_control_active:
      return None
    speed_kph = self._ls_v_ego * 3.6
    desired_gap = LOW_SPEED_COAST_STANDSTILL_GAP_M + self._ls_v_ego * self._ls_base_tr
    distance_margin = self._ls_lead_distance - desired_gap
    distance_margin_limit = min(
      LOW_SPEED_COAST_MAX_DISTANCE_MARGIN_M,
      LOW_SPEED_COAST_BASE_DISTANCE_MARGIN_M +
      self._ls_v_ego * LOW_SPEED_COAST_DISTANCE_MARGIN_SPEED_S)
    pedal_applied = self._ls_pedal_output >= LOW_SPEED_COAST_MIN_PEDAL
    # A manual brake after the comma pedal has already reached zero is still
    # useful evidence when the lead is both close and closing decisively. This
    # is the common real-world case when coasting started, but started too late.
    zero_pedal_closing_evidence = bool(
      not pedal_applied and
      self._ls_lead_rel_speed <= LOW_SPEED_COAST_ZERO_PEDAL_MAX_VREL_MS and
      distance_margin <= LOW_SPEED_COAST_ZERO_PEDAL_MAX_DISTANCE_MARGIN_M)
    candidate = bool(
      LOW_SPEED_COAST_MIN_KPH <= speed_kph <= LOW_SPEED_COAST_MAX_KPH and
      self._ls_lead_valid and
      self._ls_lead_rel_speed <= LOW_SPEED_COAST_MAX_VREL_MS and
      distance_margin <= distance_margin_limit and
      (pedal_applied or zero_pedal_closing_evidence))
    if not candidate:
      return None
    return {"start": now, "valid": True}

  def _finish_low_speed_brake_event(self, now):
    event = self._low_speed_brake_event
    self._low_speed_brake_event = None
    duration = 0.0 if event is None else now - event["start"]
    if event is None or not event["valid"] or \
       not MIN_EVENT_DURATION_S <= duration <= LOW_SPEED_COAST_MAX_EVENT_DURATION_S:
      return False
    self.low_speed_brake_events += 1
    self.low_speed_brake_batch_events += 1
    changed = False
    if self.low_speed_brake_batch_events >= LOW_SPEED_COAST_EVENTS_PER_UPDATE:
      target = (LOW_SPEED_COAST_FIRST_OFFSET_S if self.low_speed_coast_updates == 0 else
                self.low_speed_coast_offset_s + LOW_SPEED_COAST_UPDATE_STEP_S)
      new_offset = _clip(target, 0.0, LOW_SPEED_COAST_MAX_OFFSET_S)
      changed = new_offset > self.low_speed_coast_offset_s + 1e-9
      self.low_speed_coast_offset_s = new_offset
      self.low_speed_coast_updates += int(changed)
      self.low_speed_brake_batch_events = 0
    self._dirty = True
    return changed

  def _add_tr_evidence(self, score):
    score = _clip(_finite(score), -1.0, 1.0)
    self.tr_batch_score += score
    self.tr_batch_events += 1
    self.tr_evidence_events += 1
    self._dirty = True

  @staticmethod
  def _event_headway(event):
    return event["lead_distance"] / max(event["v_ego"], 1.0)

  def _finish_gas_event(self, now):
    event = self._gas_event
    self._gas_event = None
    if event is None:
      return
    duration = now - event["start"]
    if not event["valid"] or not MIN_EVENT_DURATION_S <= duration <= MAX_EVENT_DURATION_S:
      return

    self.gas_events += 1
    duration_score = _clip(duration / 1.5, 0.0, 1.0)
    # Peak intensity is deliberately a weak modifier because the interceptor's
    # raw engineering scale varies by DBC. Event direction remains dominant.
    raw_intensity = _clip((event["peak"] - 15.0) / 60.0, 0.0, 1.0)
    event_strength = 0.50 + 0.30 * duration_score + 0.20 * raw_intensity

    # Acceleration gain is no longer scored here. It is measured continuously
    # from the paired driver/openpilot statistics, which do not depend on this
    # event surviving a clean-context chain.
    headway = self._event_headway(event) if event["lead_valid"] else 0.0

    # A driver repeatedly closing an unnecessarily large, stable gap is weak
    # evidence for a shorter preferred time gap. Shortening is more conservative
    # than lengthening and remains globally limited to -0.20 s.
    if event["lead_valid"] and event["lead_stable_s"] >= LEAD_STABLE_REQUIRED_S and \
       event["v_ego"] >= MIN_FOLLOW_LEARN_SPEED_MS and \
       headway > event["base_tr"] + 0.25 and event["lead_rel_speed"] > -0.5:
      self._add_tr_evidence(-event_strength)

    self._dirty = True

  def _finish_brake_event(self, now):
    event = self._brake_event
    self._brake_event = None
    if event is None:
      return
    duration = now - event["start"]
    if not event["valid"] or not MIN_EVENT_DURATION_S <= duration <= MAX_EVENT_DURATION_S:
      return

    self.brake_events += 1
    intensity = _clip(event["peak"], 0.0, 1.0)
    event_strength = 0.60 + 0.20 * _clip(duration / 1.5, 0.0, 1.0) + 0.20 * intensity

    # Braking near a stable lead is evidence about following distance only.
    # The acceleration-gain consequence of braking is carried by the brake veto
    # rate, which does not need a lead at all.
    if event["lead_valid"] and event["lead_stable_s"] >= LEAD_STABLE_REQUIRED_S and \
       event["v_ego"] >= MIN_FOLLOW_LEARN_SPEED_MS:
      headway = self._event_headway(event)
      if headway <= event["base_tr"] + 0.35 and event["lead_rel_speed"] < 0.5:
        self._add_tr_evidence(event_strength)

    self._dirty = True

  def _maybe_apply_tr(self, now):
    if self.tr_batch_events < MIN_TR_EVENTS_PER_UPDATE or \
       self.stable_follow_s < MIN_STABLE_FOLLOW_S or \
       now - self._last_tr_update < LEARNING_UPDATE_COOLDOWN_S:
      return False

    average = self.tr_batch_score / max(1, self.tr_batch_events)
    changed = False
    if abs(average) >= 0.35:
      if average > 0.0:
        delta = 0.01 + 0.02 * _clip((average - 0.35) / 0.65, 0.0, 1.0)
      else:
        delta = -(0.005 + 0.015 * _clip((-average - 0.35) / 0.65, 0.0, 1.0))

      confidence = 0.5 * min(1.0, self.tr_evidence_events / 10.0) + \
                   0.5 * min(1.0, self.stable_follow_s / 300.0)
      lower = max(TR_OFFSET_MIN, TR_OFFSET_MIN * confidence)
      upper = min(TR_OFFSET_MAX, TR_OFFSET_MAX * confidence)
      new_offset = _clip(self.tr_offset + delta, lower, upper)
      changed = abs(new_offset - self.tr_offset) > 1e-6
      self.tr_offset = new_offset
      self.tr_updates += 1
      self._last_tr_update = now

    self.tr_batch_score = 0.0
    self.tr_batch_events = 0
    self._dirty = True
    return changed

  # --- main entry point ------------------------------------------------------

  def update(self, *, v_ego, a_ego, gas, gas_pressed, brake, brake_pressed,
             cruise_enabled, control_active, requested_accel, lead_valid,
             lead_distance, lead_rel_speed, base_tr, pedal_output=0.0,
             unsafe_context=False, low_speed_brake_context_ok=True,
             can_valid=True, dt=0.01, now=None):
    now = self._clock() if now is None else _finite(now, self._clock())
    dt = _clip(_finite(dt, 0.01), 0.001, 0.2)
    self._refresh_enabled(now)

    gas_pressed = bool(gas_pressed)
    brake_pressed = bool(brake_pressed)
    v_ego_f = _finite(v_ego)
    pedal_output_f = max(0.0, _finite(pedal_output))
    context_ok = bool(can_valid and cruise_enabled and not unsafe_context)
    low_speed_context_ok = bool(
      self.enabled and can_valid and cruise_enabled and control_active and
      low_speed_brake_context_ok)
    self._update_lead_stability(context_ok, bool(lead_valid), lead_distance,
                                lead_rel_speed, dt)

    if self.enabled and context_ok and lead_valid and self._lead_stable_s >= 2.0 and \
       v_ego_f >= MIN_FOLLOW_LEARN_SPEED_MS and abs(_finite(lead_rel_speed)) < 1.0 and \
       abs(_finite(a_ego)) < 1.2:
      self.stable_follow_s += dt
      self._dirty = True

    if not self.enabled:
      self._gas_event = None
      self._brake_event = None
      self._low_speed_brake_event = None
      self._sample_accum = 0.0
      self._last_op_pedal_time = -1e9
      self._op_episode_active = False
      self._op_episode_idle_s = 0.0
      self._op_episode_elapsed_s = 0.0
      self._prev_brake_pressed = brake_pressed
      self._clear_snapshot()
      self._clear_low_speed_snapshot()
      self._last_control_active = bool(control_active and cruise_enabled)
      return self.status(v_ego)

    style_context_ok = bool(can_valid and not unsafe_context and
                            v_ego_f >= MIN_LEARN_SPEED_MS)
    a_ego_f = _finite(a_ego)
    op_pedal_live = bool(control_active and pedal_output_f > 0.01)
    if op_pedal_live:
      self._last_op_pedal_time = now

    # --- brake veto ---------------------------------------------------------
    # One openpilot acceleration episode is one sample: it starts when openpilot
    # is producing acceleration and ends once that acceleration has stopped for
    # BRAKE_VETO_EPISODE_END_S. It resolves as vetoed only if the driver brakes
    # out of it. The end condition must track the acceleration itself, not the
    # pedal: openpilot holds pedal continuously to maintain speed, so waiting
    # for a released pedal left every episode open until a brake press and drove
    # the measured rate toward 1.0.
    op_accelerating = bool(op_pedal_live and not gas_pressed and
                           a_ego_f >= ACCEL_SAMPLE_MIN_MS2)
    brake_edge = brake_pressed and not self._prev_brake_pressed
    if not self._op_episode_active:
      if op_accelerating and style_context_ok:
        self._op_episode_active = True
        self._op_episode_idle_s = 0.0
        self._op_episode_elapsed_s = 0.0
    else:
      self._op_episode_elapsed_s += dt
      if brake_edge:
        # A brake in the first moments is a reaction to something outside the
        # car, not a verdict on the acceleration. Drop the episode rather than
        # scoring it either way.
        if self._op_episode_elapsed_s >= BRAKE_VETO_MIN_EPISODE_S:
          self._push_brake_veto(True)
        self._op_episode_active = False
      elif op_accelerating:
        self._op_episode_idle_s = 0.0
      else:
        self._op_episode_idle_s += dt
        if self._op_episode_idle_s >= BRAKE_VETO_EPISODE_END_S:
          self._push_brake_veto(False)
          self._op_episode_active = False
    self._prev_brake_pressed = brake_pressed

    # --- paired acceleration sampling ---------------------------------------
    # The driver reference deliberately does not require cruise_enabled or an
    # engaged controller: manual driving is the purest statement of preferred
    # acceleration, and excluding it was why evidence never accumulated.
    self._sample_accum += dt
    if self._sample_accum >= ACCEL_SAMPLE_DT_S:
      self._sample_accum = 0.0
      if style_context_ok and not brake_pressed and \
         ACCEL_SAMPLE_MIN_MS2 <= a_ego_f <= ACCEL_SAMPLE_MAX_MS2:
        speed_bin = self._speed_bin(v_ego_f)
        if gas_pressed:
          self._push_accel_sample(True, speed_bin, a_ego_f)
        elif op_pedal_live:
          self._push_accel_sample(False, speed_bin, a_ego_f)

    # --- time-gap events ----------------------------------------------------
    if gas_pressed and self._gas_event is None and not brake_pressed:
      self._gas_event = None if unsafe_context or not can_valid else self._start_event(now, gas)
    elif gas_pressed and self._gas_event is not None:
      self._gas_event["peak"] = max(self._gas_event["peak"], max(0.0, _finite(gas)))
      if unsafe_context or not can_valid or now - self._gas_event["start"] > MAX_EVENT_DURATION_S:
        self._gas_event["valid"] = False
    elif not gas_pressed and self._gas_event is not None:
      self._finish_gas_event(now)

    if brake_pressed and self._brake_event is None and not gas_pressed:
      self._brake_event = None if unsafe_context or not can_valid else self._start_event(now, brake)
    elif brake_pressed and self._brake_event is not None:
      self._brake_event["peak"] = max(self._brake_event["peak"], max(0.0, _finite(brake)))
      if unsafe_context or not can_valid or now - self._brake_event["start"] > MAX_EVENT_DURATION_S:
        self._brake_event["valid"] = False
    elif not brake_pressed and self._brake_event is not None:
      self._finish_brake_event(now)

    low_speed_changed = False
    if brake_pressed and self._low_speed_brake_event is None and not gas_pressed:
      self._low_speed_brake_event = (None if not low_speed_context_ok else
                                     self._low_speed_brake_candidate(now))
    elif brake_pressed and self._low_speed_brake_event is not None:
      if not low_speed_context_ok or \
         now - self._low_speed_brake_event["start"] > LOW_SPEED_COAST_MAX_EVENT_DURATION_S:
        self._low_speed_brake_event["valid"] = False
    elif not brake_pressed and self._low_speed_brake_event is not None:
      low_speed_changed = self._finish_low_speed_brake_event(now)

    # --- snapshots (flat, no per-frame allocation) --------------------------
    pedals_free = not gas_pressed and not brake_pressed
    moving = v_ego_f >= MIN_LEARN_SPEED_MS
    if context_ok and control_active and pedals_free and moving:
      self._snap_valid = True
      self._snap_time = now
      self._snap_v_ego = max(0.0, v_ego_f)
      self._snap_bin = self._speed_bin(v_ego_f)
      self._snap_requested_accel = _finite(requested_accel)
      self._snap_lead_valid = bool(lead_valid)
      self._snap_lead_distance = max(0.0, _finite(lead_distance))
      self._snap_lead_rel_speed = _finite(lead_rel_speed)
      self._snap_base_tr = _clip(_finite(base_tr, 1.3), 0.8, 2.7)
      self._snap_lead_stable_s = self._lead_stable_s

    if low_speed_context_ok and pedals_free and moving:
      self._ls_valid = True
      self._ls_time = now
      self._ls_v_ego = max(0.0, v_ego_f)
      self._ls_requested_accel = _finite(requested_accel)
      self._ls_lead_valid = bool(lead_valid)
      self._ls_lead_distance = max(0.0, _finite(lead_distance))
      self._ls_lead_rel_speed = _finite(lead_rel_speed)
      self._ls_base_tr = _clip(_finite(base_tr, 1.3), 0.8, 2.7)
      self._ls_pedal_output = pedal_output_f
    elif gas_pressed or not low_speed_context_ok:
      self._clear_low_speed_snapshot()

    gain_changed = self._maybe_apply_gain(now)
    tr_changed = self._maybe_apply_tr(now)
    self._save(now, force=gain_changed or tr_changed or low_speed_changed)
    self._last_control_active = bool(control_active and cruise_enabled)
    return self.status(v_ego)
