import math

from common.numpy_fast import clip


PREDICTIVE_COAST_MIN_SPEED_KPH = 5.0
PREDICTIVE_COAST_MAX_LEAD_DISTANCE_M = 160.0
PREDICTIVE_COAST_STANDSTILL_GAP_M = 4.5
PREDICTIVE_COAST_MIN_TR_S = 0.85
PREDICTIVE_COAST_MAX_TR_S = 2.0
PREDICTIVE_COAST_ENTER_TTG_S = 4.5
PREDICTIVE_COAST_FULL_LIFT_TTG_S = 2.0
PREDICTIVE_COAST_IMMEDIATE_TTG_S = 1.25
PREDICTIVE_COAST_CONFIRM_S = 0.15
PREDICTIVE_COAST_EXIT_TTG_S = 5.5
PREDICTIVE_COAST_EXIT_MARGIN_M = 2.0
PREDICTIVE_COAST_CLEAR_S = 0.50
PREDICTIVE_COAST_SCALE_FALL_S = 0.80
PREDICTIVE_COAST_SCALE_RISE_S = 1.00
PREDICTIVE_COAST_FAST_RISE_S = 0.55
PREDICTIVE_COAST_ACCEL_FILTER_TAU_S = 0.35
CURVE_APPROACH_START_S = 2.50
CURVE_FULL_LIFT_S = 0.80
SOURCE_SPEED_GAP_FULL_LIFT_KPH = 10.0
SPEED_LIMIT_APPROACH_START_S = 6.0
SPEED_LIMIT_FULL_LIFT_S = 2.0
BRAKE_SHADOW_MIN_CONFIDENCE = 0.65
BRAKE_SHADOW_DECEL_MARGIN_MS2 = 0.20
BRAKE_BOOTSTRAP_MIN_PRESSURE = 0.55
BRAKE_BOOTSTRAP_DECEL_MARGIN_MS2 = 0.35
BRAKE_SHADOW_CONFIRM_S = 0.30
BRAKE_SHADOW_CLEAR_S = 0.80


def _finite(value, default=0.0):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return float(default)
  return value if math.isfinite(value) else float(default)


def _time_to_gap(distance_margin, v_rel, a_rel):
  """Return the first future time when the lead reaches the desired gap."""
  margin = _finite(distance_margin)
  rel_speed = _finite(v_rel)
  rel_accel = _finite(a_rel)
  if margin <= 0.0:
    return 0.0 if rel_speed < -0.05 or rel_accel < -0.05 else math.inf

  if abs(rel_accel) < 0.05:
    return margin / -rel_speed if rel_speed < -0.05 else math.inf

  # margin + v_rel*t + 0.5*a_rel*t^2 = 0
  discriminant = rel_speed * rel_speed - 2.0 * rel_accel * margin
  if discriminant < 0.0:
    return math.inf
  root = math.sqrt(discriminant)
  roots = [(-rel_speed - root) / rel_accel,
           (-rel_speed + root) / rel_accel]
  positive_roots = [value for value in roots if value > 0.0 and math.isfinite(value)]
  return min(positive_roots) if positive_roots else math.inf


def _positive_or_inf(value):
  value = _finite(value, math.inf)
  return value if value > 0.0 else math.inf


def _required_accel_for_speed(v_ego, target_speed, distance=math.inf,
                              time_s=math.inf):
  v_ego = max(0.0, _finite(v_ego))
  target = max(0.0, _finite(target_speed, v_ego))
  if target >= v_ego:
    return 0.0
  distance = _positive_or_inf(distance)
  time_s = _positive_or_inf(time_s)
  if math.isfinite(distance):
    required = (target * target - v_ego * v_ego) / (2.0 * max(distance, 1.0))
  elif math.isfinite(time_s):
    required = (target - v_ego) / max(time_s, 0.5)
  else:
    return 0.0
  return float(clip(required, -4.0, 0.0))


class PredictiveCoastingCoordinator:
  """Final positive-pedal limiter for learned GM gas-interceptor response.

  The coordinator never adds pedal and never commands braking. It predicts when
  a tracked lead will consume the selected following gap, then supplies a
  0..1 ceiling applied after the driving-style gain in CarController.
  """

  def __init__(self, dt=0.01):
    self.dt = max(1e-3, float(dt))
    self.brake_shadow_events = 0
    self.brake_shadow_brake_responses = 0
    self.brake_shadow_no_brake_resolutions = 0
    self.reset()

  def reset(self):
    self.phase = "idle"
    self.pedal_scale = 1.0
    self.intervening = False
    self.brake_latched = False
    self.risk_elapsed = 0.0
    self.clear_elapsed = PREDICTIVE_COAST_CLEAR_S
    self.filtered_rel_accel = 0.0
    self.desired_gap_m = 0.0
    self.distance_margin_m = 0.0
    self.time_to_gap_s = math.inf
    self.ttc_s = math.inf
    self.lead_distance_m = 0.0
    self.lead_rel_speed_ms = 0.0
    self.lead_accel_ms2 = 0.0
    self.lead_model_prob = 0.0
    self.last_pressure = 0.0
    self.source_pressures = {"lead": 0.0, "curve": 0.0, "speed_limit": 0.0}
    self.dominant_source = "none"
    self.required_decel_ms2 = 0.0
    self.natural_decel_ms2 = 0.0
    self.natural_decel_confidence = 0.0
    self.brake_needed_shadow = False
    self.brake_advisory = False
    self.brake_bootstrap = True
    self.brake_min_pressure = BRAKE_BOOTSTRAP_MIN_PRESSURE
    self.brake_decel_margin_ms2 = BRAKE_BOOTSTRAP_DECEL_MARGIN_MS2
    self.brake_shadow_elapsed = 0.0
    self.brake_shadow_clear_elapsed = BRAKE_SHADOW_CLEAR_S
    self.driver_brake_pressed = False

  @property
  def learning_blocked(self):
    return bool(self.intervening or self.brake_latched or self.phase != "idle")

  def _rate_scale(self, target, rise_time=PREDICTIVE_COAST_SCALE_RISE_S):
    target = float(clip(target, 0.0, 1.0))
    if target < self.pedal_scale:
      step = self.dt / PREDICTIVE_COAST_SCALE_FALL_S
      self.pedal_scale = max(target, self.pedal_scale - step)
    else:
      step = self.dt / max(rise_time, self.dt)
      self.pedal_scale = min(target, self.pedal_scale + step)
    self.pedal_scale = float(clip(self.pedal_scale, 0.0, 1.0))

  def _recover(self, fast=False):
    self.phase = "recover"
    self.risk_elapsed = 0.0
    self.last_pressure = 0.0
    self._rate_scale(1.0, PREDICTIVE_COAST_FAST_RISE_S if fast else
                     PREDICTIVE_COAST_SCALE_RISE_S)
    if self.pedal_scale >= 1.0 - 1e-6:
      self.phase = "idle"
      self.intervening = False
      self.brake_latched = False
      self.pedal_scale = 1.0
    else:
      self.intervening = True
    return self.pedal_scale

  def _update_brake_shadow(self, candidate, alert_enabled):
    was_active = self.brake_needed_shadow
    if candidate:
      self.brake_shadow_elapsed = min(BRAKE_SHADOW_CONFIRM_S,
                                      self.brake_shadow_elapsed + self.dt)
      self.brake_shadow_clear_elapsed = 0.0
      if self.brake_shadow_elapsed >= BRAKE_SHADOW_CONFIRM_S - 1e-9:
        self.brake_needed_shadow = True
    else:
      self.brake_shadow_elapsed = 0.0
      self.brake_shadow_clear_elapsed = min(
        BRAKE_SHADOW_CLEAR_S, self.brake_shadow_clear_elapsed + self.dt)
      if self.brake_shadow_clear_elapsed >= BRAKE_SHADOW_CLEAR_S - 1e-9:
        self.brake_needed_shadow = False
    if self.brake_needed_shadow and not was_active:
      self.brake_shadow_events += 1
    elif was_active and not self.brake_needed_shadow:
      self.brake_shadow_no_brake_resolutions += 1
    self.brake_advisory = bool(alert_enabled and self.brake_needed_shadow)

  def update(self, *, enabled, control_active, requested_accel, v_ego, a_ego,
             brake_pressed, gas_pressed, lead_valid, lead_distance,
             lead_rel_speed, lead_accel, lead_model_prob, effective_tr,
             fcw=False, radar_valid=True, can_valid=True,
             curve_active=False, curve_target_speed=0.0,
             curve_time_s=math.inf, curve_distance_m=math.inf,
             speed_limit_active=False, speed_limit_target=0.0,
             speed_limit_distance_m=math.inf, natural_decel_ms2=0.0,
             natural_decel_confidence=0.0, brake_alert_enabled=False):
    if not enabled or not control_active or not can_valid:
      self.reset()
      return self.pedal_scale

    if gas_pressed:
      # Explicit accelerator input always wins and clears an automatic hold.
      self.reset()
      return self.pedal_scale

    v_ego = max(0.0, _finite(v_ego))
    self.driver_brake_pressed = bool(brake_pressed)
    if v_ego * 3.6 < PREDICTIVE_COAST_MIN_SPEED_KPH:
      self.reset()
      return self.pedal_scale

    if brake_pressed:
      # CarController already sends physical pedal zero while brake is pressed.
      # Retain a zero ceiling so brake release cannot cause a one-frame surge.
      self.phase = "brake_hold"
      self.pedal_scale = 0.0
      self.intervening = True
      self.brake_latched = True
      self.risk_elapsed = 0.0
      if self.brake_needed_shadow:
        self.brake_shadow_brake_responses += 1
      self.brake_needed_shadow = False
      self.brake_advisory = False
      self.brake_shadow_elapsed = 0.0
      self.brake_shadow_clear_elapsed = BRAKE_SHADOW_CLEAR_S
      return self.pedal_scale

    plausible_lead = bool(
      radar_valid and lead_valid and
      0.0 < _finite(lead_distance) <= PREDICTIVE_COAST_MAX_LEAD_DISTANCE_M)
    lead_pressure = 0.0
    lead_required_accel = 0.0
    v_rel = 0.0
    if plausible_lead:
      d_rel = max(0.0, _finite(lead_distance))
      v_rel = _finite(lead_rel_speed)
      a_lead = float(clip(_finite(lead_accel), -3.5, 1.5))
      a_ego = float(clip(_finite(a_ego), -3.5, 2.5))
      tr = float(clip(_finite(effective_tr, 1.3),
                      PREDICTIVE_COAST_MIN_TR_S, PREDICTIVE_COAST_MAX_TR_S))
      self.desired_gap_m = PREDICTIVE_COAST_STANDSTILL_GAP_M + v_ego * tr
      self.distance_margin_m = d_rel - self.desired_gap_m

      raw_rel_accel = float(clip(a_lead - a_ego, -3.5, 1.5))
      alpha = self.dt / (PREDICTIVE_COAST_ACCEL_FILTER_TAU_S + self.dt)
      self.filtered_rel_accel += alpha * (raw_rel_accel - self.filtered_rel_accel)
      self.time_to_gap_s = _time_to_gap(
        self.distance_margin_m, v_rel, self.filtered_rel_accel)
      self.ttc_s = d_rel / max(-v_rel, 0.1) if v_rel < -0.1 else math.inf
      self.lead_distance_m = d_rel
      self.lead_rel_speed_ms = v_rel
      self.lead_accel_ms2 = a_lead
      self.lead_model_prob = float(clip(_finite(lead_model_prob), 0.0, 1.0))
      if math.isfinite(self.time_to_gap_s):
        lead_pressure = float(clip(
          (PREDICTIVE_COAST_ENTER_TTG_S - self.time_to_gap_s) /
          (PREDICTIVE_COAST_ENTER_TTG_S - PREDICTIVE_COAST_FULL_LIFT_TTG_S),
          0.0, 1.0))
      if lead_pressure > 0.0:
        horizon = float(clip(self.time_to_gap_s, 1.0,
                             PREDICTIVE_COAST_ENTER_TTG_S))
        lead_required_accel = float(clip(
          a_lead + 2.0 * (self.distance_margin_m + v_rel * horizon) /
          (horizon * horizon), -4.0, 0.0))
    else:
      self.desired_gap_m = 0.0
      self.distance_margin_m = 0.0
      self.time_to_gap_s = math.inf
      self.ttc_s = math.inf
      self.lead_distance_m = 0.0
      self.lead_rel_speed_ms = 0.0
      self.lead_accel_ms2 = 0.0
      self.lead_model_prob = 0.0
      accel_alpha = self.dt / (PREDICTIVE_COAST_ACCEL_FILTER_TAU_S + self.dt)
      self.filtered_rel_accel += accel_alpha * (0.0 - self.filtered_rel_accel)

    curve_target = max(0.0, _finite(curve_target_speed, v_ego))
    curve_time = _positive_or_inf(curve_time_s)
    curve_distance = _positive_or_inf(curve_distance_m)
    curve_pressure = 0.0
    curve_required_accel = 0.0
    if curve_active and curve_target < v_ego:
      speed_ratio = float(clip(
        (v_ego - curve_target) * 3.6 / SOURCE_SPEED_GAP_FULL_LIFT_KPH,
        0.0, 1.0))
      time_ratio = (1.0 if not math.isfinite(curve_time) else float(clip(
        (CURVE_APPROACH_START_S - curve_time) /
        (CURVE_APPROACH_START_S - CURVE_FULL_LIFT_S), 0.0, 1.0)))
      curve_pressure = speed_ratio * time_ratio
      curve_required_accel = _required_accel_for_speed(
        v_ego, curve_target, curve_distance, curve_time)

    speed_target = max(0.0, _finite(speed_limit_target, v_ego))
    speed_distance = _positive_or_inf(speed_limit_distance_m)
    speed_pressure = 0.0
    speed_required_accel = 0.0
    if speed_limit_active and speed_target < v_ego:
      time_to_limit = (speed_distance / max(v_ego, 0.1)
                       if math.isfinite(speed_distance) else 0.0)
      speed_ratio = float(clip(
        (v_ego - speed_target) * 3.6 / SOURCE_SPEED_GAP_FULL_LIFT_KPH,
        0.0, 1.0))
      time_ratio = (1.0 if not math.isfinite(speed_distance) else float(clip(
        (SPEED_LIMIT_APPROACH_START_S - time_to_limit) /
        (SPEED_LIMIT_APPROACH_START_S - SPEED_LIMIT_FULL_LIFT_S), 0.0, 1.0)))
      speed_pressure = speed_ratio * time_ratio
      speed_required_accel = _required_accel_for_speed(
        v_ego, speed_target, speed_distance, time_to_limit)

    self.source_pressures = {
      "lead": float(lead_pressure),
      "curve": float(curve_pressure),
      "speed_limit": float(speed_pressure),
    }
    self.dominant_source = max(self.source_pressures, key=self.source_pressures.get)
    pressure = self.source_pressures[self.dominant_source]
    if pressure <= 0.0:
      self.dominant_source = "none"
    required_values = [value for value, source_pressure in (
      (lead_required_accel, lead_pressure),
      (curve_required_accel, curve_pressure),
      (speed_required_accel, speed_pressure),
    ) if source_pressure > 0.0]
    self.required_decel_ms2 = min(required_values) if required_values else 0.0
    self.natural_decel_ms2 = max(0.0, _finite(natural_decel_ms2))
    self.natural_decel_confidence = float(clip(
      _finite(natural_decel_confidence), 0.0, 1.0))
    # Do not suppress the BRAKE advisory entirely while the vehicle model is
    # still learning. Start with conservative thresholds, then blend toward
    # the normal thresholds as confidence reaches the trusted level.
    confidence_ratio = float(clip(
      self.natural_decel_confidence / BRAKE_SHADOW_MIN_CONFIDENCE, 0.0, 1.0))
    self.brake_bootstrap = bool(
      self.natural_decel_confidence < BRAKE_SHADOW_MIN_CONFIDENCE)
    self.brake_min_pressure = (
      BRAKE_BOOTSTRAP_MIN_PRESSURE + confidence_ratio *
      (0.20 - BRAKE_BOOTSTRAP_MIN_PRESSURE))
    self.brake_decel_margin_ms2 = (
      BRAKE_BOOTSTRAP_DECEL_MARGIN_MS2 + confidence_ratio *
      (BRAKE_SHADOW_DECEL_MARGIN_MS2 - BRAKE_BOOTSTRAP_DECEL_MARGIN_MS2))
    shadow_candidate = bool(
      not fcw and pressure >= self.brake_min_pressure and
      self.required_decel_ms2 <
      -self.natural_decel_ms2 - self.brake_decel_margin_ms2)
    self._update_brake_shadow(shadow_candidate, brake_alert_enabled)
    self.last_pressure = pressure

    meaningful_closing = bool(v_rel < -0.3 or self.filtered_rel_accel < -0.4)
    urgent = bool(
      fcw or
      (plausible_lead and v_rel < -0.3 and
       (self.lead_distance_m < 8.0 or self.ttc_s < 3.0)) or
      (self.distance_margin_m < 0.0 and v_rel < -0.3) or
      (plausible_lead and meaningful_closing and
       self.time_to_gap_s <= PREDICTIVE_COAST_IMMEDIATE_TTG_S))

    if urgent:
      self.phase = "brake_needed"
      self.pedal_scale = 0.0
      self.intervening = True
      self.brake_latched = False
      self.risk_elapsed = PREDICTIVE_COAST_CONFIRM_S
      self.clear_elapsed = 0.0
      return self.pedal_scale

    lead_release_safe = bool(
      not plausible_lead or
      (self.time_to_gap_s >= PREDICTIVE_COAST_EXIT_TTG_S and
       self.distance_margin_m >= PREDICTIVE_COAST_EXIT_MARGIN_M and
       v_rel >= -0.2))
    release_safe = bool(pressure <= 0.0 and lead_release_safe)
    if pressure > 0.0:
      self.risk_elapsed = min(PREDICTIVE_COAST_CONFIRM_S,
                              self.risk_elapsed + self.dt)
      self.clear_elapsed = 0.0
    else:
      self.risk_elapsed = 0.0
      if release_safe:
        self.clear_elapsed = min(PREDICTIVE_COAST_CLEAR_S,
                                 self.clear_elapsed + self.dt)
      else:
        self.clear_elapsed = 0.0

    confirmed = self.risk_elapsed >= PREDICTIVE_COAST_CONFIRM_S - 1e-9
    if confirmed or self.intervening:
      if pressure > 0.0:
        suffix = "coast" if pressure >= 0.65 else "pre_coast"
        self.phase = "{}_{}".format(self.dominant_source, suffix)
        target_scale = 1.0 - pressure
        if _finite(requested_accel) <= 0.0:
          # The longitudinal planner has already asked for no positive pedal.
          self.pedal_scale = 0.0
        else:
          self._rate_scale(target_scale)
        self.intervening = True
        self.brake_latched = False
        return self.pedal_scale

      if self.clear_elapsed < PREDICTIVE_COAST_CLEAR_S:
        self.phase = "hysteresis_hold"
        self.intervening = True
        return self.pedal_scale

      opening_fast = bool(plausible_lead and v_rel > 0.5 and
                          self.distance_margin_m > PREDICTIVE_COAST_EXIT_MARGIN_M and
                          curve_pressure <= 0.0 and speed_pressure <= 0.0)
      self.brake_latched = False
      return self._recover(fast=opening_fast)

    self.phase = "idle"
    self.pedal_scale = 1.0
    self.intervening = False
    self.brake_latched = False
    return self.pedal_scale
