import json
import math
import time
from dataclasses import dataclass


STATE_VERSION = 1

GAIN_MIN = 0.90
GAIN_MAX = 1.12
TR_OFFSET_MIN = -0.20
TR_OFFSET_MAX = 0.40

# Make a learned preference perceptible without widening the learner's raw
# safety bounds. Low/mid-speed acceleration gets the clearest response, while
# highway gain and time-gap shortening stay deliberately conservative.
STYLE_RESPONSE_BP_KPH = (0.0, 30.0, 60.0, 100.0, 130.0)
GAIN_RESPONSE_V = (2.0, 2.0, 1.7, 1.4, 1.2)
GAIN_APPLIED_MAX_V = (1.08, 1.08, 1.065, 1.045, 1.04)
GAIN_APPLIED_MIN = 0.94
TR_RESPONSE_V = (4.0, 3.8, 3.2, 2.5, 2.0)
TR_APPLIED_MIN_V = (-0.12, -0.12, -0.10, -0.08, -0.06)
TR_APPLIED_MAX_V = (0.25, 0.25, 0.28, 0.30, 0.30)

SPEED_BIN_CENTERS_KPH = (5.0, 20.0, 45.0, 80.0, 115.0)
SPEED_BIN_EDGES_KPH = (10.0, 30.0, 60.0, 100.0)

PARAM_REFRESH_S = 1.0
SAVE_INTERVAL_S = 60.0
LEARNING_UPDATE_COOLDOWN_S = 30.0
MIN_GAS_EVENTS_FIRST_UPDATE = 8
MIN_GAS_EVENTS_NEXT_UPDATE = 4
MIN_TR_EVENTS_PER_UPDATE = 3
MIN_STABLE_FOLLOW_S = 60.0

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
LOW_SPEED_COAST_MAX_DISTANCE_MARGIN_M = 3.0
LOW_SPEED_COAST_STANDSTILL_GAP_M = 4.5
LOW_SPEED_COAST_SNAPSHOT_MAX_AGE_S = 0.30

MIN_EVENT_DURATION_S = 0.12
MAX_EVENT_DURATION_S = 4.0
MIN_LEARN_SPEED_MS = 1.0 / 3.6
MIN_FOLLOW_LEARN_SPEED_MS = 18.0 / 3.6
MIN_LEAD_DISTANCE_M = 3.0
MAX_LEAD_DISTANCE_M = 160.0
LEAD_STABLE_REQUIRED_S = 1.0
LEAD_DISTANCE_JUMP_M = 6.0


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
  """Bounded, event-based online adaptation for the GM gas-interceptor setup.

  Driver inputs are counted as complete press/release events, never as 100 Hz
  frames. The class does not determine gasPressed itself; it consumes the
  existing CarState.gasPressed value so the GM `ret.gas > 15` behavior remains
  unchanged.
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
    self.bin_offsets = [0.0] * len(SPEED_BIN_CENTERS_KPH)
    self.tr_offset = 0.0
    self.low_speed_coast_offset_s = 0.0

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

    self.gain_batch_score = 0.0
    self.gain_batch_events = 0
    self.bin_batch_score = [0.0] * len(SPEED_BIN_CENTERS_KPH)
    self.bin_batch_events = [0] * len(SPEED_BIN_CENTERS_KPH)
    self.bin_total_events = [0] * len(SPEED_BIN_CENTERS_KPH)
    self.tr_batch_score = 0.0
    self.tr_batch_events = 0

    self._gas_event = None
    self._brake_event = None
    self._low_speed_brake_event = None
    self._low_speed_snapshot = None
    self._lead_stable_s = 0.0
    self._last_lead_distance = None
    self._last_snapshot = None
    self._last_control_active = False
    self._last_param_refresh = -1e9
    self._last_save = self._clock()
    self._last_gain_update = -1e9
    self._last_tr_update = -1e9
    self._dirty = False

    self._load_state()
    self._refresh_enabled(self._clock(), force=True)

  @staticmethod
  def _speed_bin(v_ego):
    speed_kph = max(0.0, _finite(v_ego)) * 3.6
    for index, edge in enumerate(SPEED_BIN_EDGES_KPH):
      if speed_kph < edge:
        return index
    return len(SPEED_BIN_CENTERS_KPH) - 1

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
      if int(state.get("version", 0)) != STATE_VERSION:
        return

      self.global_gain = _clip(_finite(state.get("global_gain"), 1.0), GAIN_MIN, GAIN_MAX)
      offsets = state.get("bin_offsets", [])
      if isinstance(offsets, list) and len(offsets) == len(self.bin_offsets):
        self.bin_offsets = [_clip(_finite(value), -0.03, 0.03) for value in offsets]
      self.tr_offset = _clip(_finite(state.get("tr_offset")), TR_OFFSET_MIN, TR_OFFSET_MAX)
      self.low_speed_coast_offset_s = _clip(
        _finite(state.get("low_speed_coast_offset_s")), 0.0,
        LOW_SPEED_COAST_MAX_OFFSET_S)

      for name in ("gas_events", "brake_events", "gain_evidence_events", "tr_evidence_events",
                   "gain_updates", "tr_updates", "low_speed_brake_events",
                   "low_speed_coast_updates", "low_speed_brake_batch_events"):
        setattr(self, name, max(0, int(state.get(name, 0))))
      self.stable_follow_s = max(0.0, _finite(state.get("stable_follow_s")))

      self.gain_batch_score = _finite(state.get("gain_batch_score"))
      self.gain_batch_events = max(0, int(state.get("gain_batch_events", 0)))
      self.tr_batch_score = _finite(state.get("tr_batch_score"))
      self.tr_batch_events = max(0, int(state.get("tr_batch_events", 0)))

      for name, default, cast in (("bin_batch_score", self.bin_batch_score, float),
                                  ("bin_batch_events", self.bin_batch_events, int),
                                  ("bin_total_events", self.bin_total_events, int)):
        values = state.get(name, [])
        if isinstance(values, list) and len(values) == len(default):
          if cast is float:
            setattr(self, name, [_finite(value) for value in values])
          else:
            setattr(self, name, [max(0, int(value)) for value in values])
    except (TypeError, ValueError, json.JSONDecodeError):
      # Invalid or partially-written state is ignored; bounded defaults remain.
      return

  def _state_dict(self):
    return {
      "version": STATE_VERSION,
      "global_gain": round(self.global_gain, 6),
      "bin_offsets": [round(value, 6) for value in self.bin_offsets],
      "tr_offset": round(self.tr_offset, 6),
      "low_speed_coast_offset_s": round(self.low_speed_coast_offset_s, 6),
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
      "gain_batch_score": round(self.gain_batch_score, 6),
      "gain_batch_events": self.gain_batch_events,
      "bin_batch_score": [round(value, 6) for value in self.bin_batch_score],
      "bin_batch_events": self.bin_batch_events,
      "bin_total_events": self.bin_total_events,
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
      self._low_speed_snapshot = None
      self._lead_stable_s = 0.0
      self._last_lead_distance = None
      self._last_snapshot = None
      if was_enabled and not self.enabled:
        self._save(now, force=True)

  def gain_for_speed(self, v_ego):
    if not self.enabled:
      return 1.0
    speed_kph = max(0.0, _finite(v_ego)) * 3.6
    gains = [_clip(self.global_gain + offset, GAIN_MIN, GAIN_MAX) for offset in self.bin_offsets]
    raw_gain = _clip(_interp(speed_kph, SPEED_BIN_CENTERS_KPH, gains), GAIN_MIN, GAIN_MAX)
    return applied_driving_style_gain(raw_gain, v_ego)

  def confidence(self):
    gain_confidence = min(1.0, self.gain_evidence_events / 20.0)
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

  def _update_lead_stability(self, context_ok, lead_valid, lead_distance, lead_rel_speed, dt):
    lead_distance = _finite(lead_distance)
    lead_rel_speed = _finite(lead_rel_speed)
    plausible = context_ok and lead_valid and MIN_LEAD_DISTANCE_M <= lead_distance <= MAX_LEAD_DISTANCE_M
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

  def _make_snapshot(self, now, v_ego, requested_accel, lead_valid, lead_distance,
                     lead_rel_speed, base_tr):
    return {
      "time": now,
      "v_ego": max(0.0, _finite(v_ego)),
      "speed_bin": self._speed_bin(v_ego),
      "requested_accel": _finite(requested_accel),
      "lead_valid": bool(lead_valid),
      "lead_distance": max(0.0, _finite(lead_distance)),
      "lead_rel_speed": _finite(lead_rel_speed),
      "base_tr": _clip(_finite(base_tr, 1.3), 0.8, 2.7),
      "lead_stable_s": self._lead_stable_s,
    }

  def _start_event(self, now, signal_value):
    snapshot = self._last_snapshot
    if snapshot is None or now - snapshot["time"] > 0.5:
      return None
    if not self._last_control_active or snapshot["v_ego"] < MIN_LEARN_SPEED_MS:
      return None
    event = dict(snapshot)
    event.update({"start": now, "peak": max(0.0, _finite(signal_value)), "valid": True})
    return event

  def _low_speed_brake_candidate(self, now):
    snapshot = self._low_speed_snapshot
    if snapshot is None or now - snapshot["time"] > LOW_SPEED_COAST_SNAPSHOT_MAX_AGE_S:
      return None
    if not self._last_control_active:
      return None
    speed_kph = snapshot["v_ego"] * 3.6
    desired_gap = (LOW_SPEED_COAST_STANDSTILL_GAP_M +
                   snapshot["v_ego"] * snapshot["base_tr"])
    distance_margin = snapshot["lead_distance"] - desired_gap
    candidate = bool(
      LOW_SPEED_COAST_MIN_KPH <= speed_kph <= LOW_SPEED_COAST_MAX_KPH and
      snapshot["lead_valid"] and
      snapshot["pedal_output"] >= LOW_SPEED_COAST_MIN_PEDAL and
      snapshot["lead_rel_speed"] <= LOW_SPEED_COAST_MAX_VREL_MS and
      distance_margin <= LOW_SPEED_COAST_MAX_DISTANCE_MARGIN_M)
    if not candidate:
      return None
    event = dict(snapshot)
    event.update({"start": now, "valid": True})
    return event

  def _finish_low_speed_brake_event(self, now):
    event = self._low_speed_brake_event
    self._low_speed_brake_event = None
    duration = 0.0 if event is None else now - event["start"]
    if event is None or not event["valid"] or \
       not MIN_EVENT_DURATION_S <= duration <= MAX_EVENT_DURATION_S:
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

  def _add_gain_evidence(self, score, speed_bin):
    score = _clip(_finite(score), -1.0, 1.0)
    self.gain_batch_score += score
    self.gain_batch_events += 1
    self.gain_evidence_events += 1
    self.bin_batch_score[speed_bin] += score
    self.bin_batch_events[speed_bin] += 1
    self.bin_total_events[speed_bin] += 1
    self._dirty = True

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

    headway = self._event_headway(event) if event["lead_valid"] else 0.0
    lead_allows_catchup = (not event["lead_valid"] or event["lead_rel_speed"] > 0.3 or
                           headway > event["base_tr"] + 0.25)
    if event["requested_accel"] > 0.05 and lead_allows_catchup:
      self._add_gain_evidence(event_strength, event["speed_bin"])

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

    if event["lead_valid"] and event["lead_stable_s"] >= LEAD_STABLE_REQUIRED_S and \
       event["v_ego"] >= MIN_FOLLOW_LEARN_SPEED_MS:
      headway = self._event_headway(event)
      if headway <= event["base_tr"] + 0.35 and event["lead_rel_speed"] < 0.5:
        self._add_tr_evidence(event_strength)

      # If a stable lead gap was already generous but automatic acceleration
      # was still positive immediately before braking, reduce pedal gain rather
      # than incorrectly increasing following distance.
      if event["requested_accel"] > 0.15 and headway > event["base_tr"] + 0.45 and \
         event["lead_rel_speed"] > -0.5:
        self._add_gain_evidence(-event_strength, event["speed_bin"])

    self._dirty = True

  def _maybe_apply_gain(self, now):
    required = MIN_GAS_EVENTS_FIRST_UPDATE if self.gain_updates == 0 else MIN_GAS_EVENTS_NEXT_UPDATE
    if self.gain_batch_events < required or now - self._last_gain_update < LEARNING_UPDATE_COOLDOWN_S:
      return False

    average = self.gain_batch_score / max(1, self.gain_batch_events)
    changed = False
    if abs(average) >= 0.25:
      confidence = min(1.0, self.gain_evidence_events / 20.0)
      delta = _clip(average * 0.015, -0.015, 0.020)
      lower = 1.0 - 0.10 * confidence
      upper = 1.0 + 0.12 * confidence
      new_gain = _clip(self.global_gain + delta, max(GAIN_MIN, lower), min(GAIN_MAX, upper))
      changed = abs(new_gain - self.global_gain) > 1e-6
      self.global_gain = new_gain

      # Speed bins learn only their residual from the shared global direction.
      # This shares sparse evidence across all speeds without creating steps at
      # 10/30/60/100 km/h (gain_for_speed interpolates between bin centers).
      for index, count in enumerate(self.bin_batch_events):
        if count >= 3:
          bin_average = self.bin_batch_score[index] / count
          bin_confidence = min(1.0, self.bin_total_events[index] / 12.0)
          residual_delta = _clip((bin_average - average) * 0.008, -0.006, 0.006)
          bound = 0.03 * bin_confidence
          self.bin_offsets[index] = _clip(self.bin_offsets[index] + residual_delta, -bound, bound)

      self.gain_updates += 1
      self._last_gain_update = now

    self.gain_batch_score = 0.0
    self.gain_batch_events = 0
    self.bin_batch_score = [0.0] * len(self.bin_batch_score)
    self.bin_batch_events = [0] * len(self.bin_batch_events)
    self._dirty = True
    return changed

  def _maybe_apply_tr(self, now):
    if self.tr_batch_events < MIN_TR_EVENTS_PER_UPDATE or self.stable_follow_s < MIN_STABLE_FOLLOW_S or \
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
    context_ok = bool(can_valid and cruise_enabled and not unsafe_context)
    low_speed_context_ok = bool(
      self.enabled and can_valid and cruise_enabled and control_active and
      low_speed_brake_context_ok)
    self._update_lead_stability(context_ok, bool(lead_valid), lead_distance, lead_rel_speed, dt)

    if self.enabled and context_ok and lead_valid and self._lead_stable_s >= 2.0 and \
       _finite(v_ego) >= MIN_FOLLOW_LEARN_SPEED_MS and abs(_finite(lead_rel_speed)) < 1.0 and \
       abs(_finite(a_ego)) < 1.2:
      self.stable_follow_s += dt
      self._dirty = True

    if not self.enabled:
      self._gas_event = None
      self._brake_event = None
      self._low_speed_brake_event = None
      self._low_speed_snapshot = None
      self._last_snapshot = None
      self._last_control_active = bool(control_active and cruise_enabled)
      return self.status(v_ego)

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
      if not low_speed_context_ok or now - self._low_speed_brake_event["start"] > MAX_EVENT_DURATION_S:
        self._low_speed_brake_event["valid"] = False
    elif not brake_pressed and self._low_speed_brake_event is not None:
      low_speed_changed = self._finish_low_speed_brake_event(now)

    if context_ok and control_active and not gas_pressed and not brake_pressed and \
       _finite(v_ego) >= MIN_LEARN_SPEED_MS:
      self._last_snapshot = self._make_snapshot(now, v_ego, requested_accel, lead_valid,
                                                lead_distance, lead_rel_speed, base_tr)
    if low_speed_context_ok and not gas_pressed and not brake_pressed and \
       _finite(v_ego) >= MIN_LEARN_SPEED_MS:
      self._low_speed_snapshot = self._make_snapshot(
        now, v_ego, requested_accel, lead_valid, lead_distance,
        lead_rel_speed, base_tr)
      self._low_speed_snapshot["pedal_output"] = max(0.0, _finite(pedal_output))
    elif gas_pressed or not low_speed_context_ok:
      self._low_speed_snapshot = None

    gain_changed = self._maybe_apply_gain(now)
    tr_changed = self._maybe_apply_tr(now)
    self._save(now, force=gain_changed or tr_changed or low_speed_changed)
    self._last_control_active = bool(control_active and cruise_enabled)
    return self.status(v_ego)
