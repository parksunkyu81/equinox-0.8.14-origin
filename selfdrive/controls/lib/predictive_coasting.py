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
# In stop-and-go traffic, a clearly receding lead should release a completed
# pedal lift sooner than the normal highway-sized 2 m / 0.5 s hysteresis. The
# entry and urgent-stop thresholds remain unchanged.
PREDICTIVE_COAST_LOW_SPEED_QUICK_EXIT_KPH = 15.0
PREDICTIVE_COAST_QUICK_EXIT_VREL_MS = 0.80
PREDICTIVE_COAST_QUICK_EXIT_MARGIN_M = 0.25
PREDICTIVE_COAST_QUICK_CLEAR_S = 0.15
PREDICTIVE_COAST_QUICK_RISE_S = 0.35
PREDICTIVE_COAST_BRAKE_RELEASE_CLEAR_S = 0.15
# A lead that is already pulling away must never be treated as an immediate
# closing hazard because of a stale relative-acceleration filter. 35 km/h is
# the stop-and-go/city range where a zero-pedal hold is felt most strongly.
PREDICTIVE_COAST_OPENING_RELEASE_VREL_MS = 0.50
PREDICTIVE_COAST_OPENING_HOLD_VREL_MS = 0.20
PREDICTIVE_COAST_STRONG_OPENING_VREL_MS = 1.20
PREDICTIVE_COAST_QUICK_EXIT_MAX_KPH = 35.0
# Escape a stale zero hysteresis ceiling when the longitudinal controller has
# rebuilt a substantial positive request and the tracked lead is no longer
# closing. This is deliberately stricter than the normal release path: the
# lead must remain beyond the desired gap for the existing 0.5 s clear period.
PREDICTIVE_COAST_POSITIVE_DEMAND_RELEASE_ACCEL_MS2 = 0.50
PREDICTIVE_COAST_POSITIVE_DEMAND_RELEASE_MARGIN_M = 0.75
PREDICTIVE_COAST_POSITIVE_DEMAND_RELEASE_VREL_MS = 0.0
PREDICTIVE_COAST_LEARNED_LOW_SPEED_MAX_KPH = 35.0
PREDICTIVE_COAST_LEARNED_CLOSING_VREL_MS = -0.10
PREDICTIVE_COAST_LEARNED_OFFSET_MAX_S = 0.10
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
  """Return when a *currently closing* lead reaches the desired gap.

  Relative acceleration is noisy and filtered, so using it to predict a
  reversal while the measured lead is already opening can turn an improving
  gap into a false zero-time hazard. Re-evaluate on the next frame after the
  measured relative speed has actually become closing instead.
  """
  margin = _finite(distance_margin)
  rel_speed = _finite(v_rel)
  rel_accel = _finite(a_rel)
  # Only a clearly opening lead invalidates the filtered closing prediction.
  # Near-zero relative speed can still be the beginning of a real hard brake,
  # so retain acceleration-based prediction until opening exceeds 0.5 m/s.
  if rel_speed >= PREDICTIVE_COAST_OPENING_RELEASE_VREL_MS:
    return math.inf
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
    self.learned_low_speed_offset_s = 0.0
    self.learned_low_speed_offset_active = False
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
    self.quick_release_active = False
    self.opening_release_active = False
    self.positive_demand_release_active = False
    self.launch_floor_release_active = False
    self.recovery_floor_release_active = False
    self.brake_release_active = False

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

  def _recover(self, fast=False, quick_low_speed=False):
    self.phase = "recover"
    self.risk_elapsed = 0.0
    self.last_pressure = 0.0
    rise_time = (PREDICTIVE_COAST_QUICK_RISE_S if quick_low_speed else
                 PREDICTIVE_COAST_FAST_RISE_S if fast else
                 PREDICTIVE_COAST_SCALE_RISE_S)
    self._rate_scale(1.0, rise_time)
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
             natural_decel_confidence=0.0, brake_alert_enabled=False,
             lead_loss_recovery_active=False,
             launch_boost_floor_active=False,
             positive_recovery_active=False,
             learned_low_speed_coast_offset_s=0.0):
    if not enabled or not control_active or not can_valid:
      self.reset()
      return self.pedal_scale

    if gas_pressed:
      # Explicit accelerator input always wins and clears an automatic hold.
      self.reset()
      return self.pedal_scale

    v_ego = max(0.0, _finite(v_ego))
    self.driver_brake_pressed = bool(brake_pressed)
    self.positive_demand_release_active = False
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
      self.quick_release_active = False
      self.opening_release_active = False
      self.launch_floor_release_active = False
      self.recovery_floor_release_active = False
      self.brake_release_active = False
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
      learned_offset = float(clip(
        _finite(learned_low_speed_coast_offset_s), 0.0,
        PREDICTIVE_COAST_LEARNED_OFFSET_MAX_S))
      self.learned_low_speed_offset_active = bool(
        v_ego * 3.6 <= PREDICTIVE_COAST_LEARNED_LOW_SPEED_MAX_KPH and
        v_rel < PREDICTIVE_COAST_LEARNED_CLOSING_VREL_MS and learned_offset > 0.0)
      self.learned_low_speed_offset_s = (learned_offset
                                         if self.learned_low_speed_offset_active else 0.0)
      tr += self.learned_low_speed_offset_s
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
      self.learned_low_speed_offset_s = 0.0
      self.learned_low_speed_offset_active = False
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

    # Never declare an urgent lead hazard from the filtered acceleration alone.
    # The filter can remain negative for several seconds after the lead has
    # started pulling away (the exact failure seen in the 2026-08-12 logs).
    meaningful_closing = bool(v_rel < -0.3)
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
      self.quick_release_active = False
      self.opening_release_active = False
      self.launch_floor_release_active = False
      self.recovery_floor_release_active = False
      self.brake_release_active = False
      return self.pedal_scale

    # A false lead here is not a one-frame camera dropout: radard has already
    # completed its bounded hold. Let the dedicated lead-loss assist own the
    # smooth positive ramp instead of retaining an obsolete zero ceiling.
    lead_loss_release = bool(
      lead_loss_recovery_active and not plausible_lead and pressure <= 0.0 and
      curve_pressure <= 0.0 and speed_pressure <= 0.0)
    if lead_loss_release:
      self.phase = "lead_loss_recover"
      self.pedal_scale = 1.0
      self.intervening = False
      self.brake_latched = False
      self.risk_elapsed = 0.0
      self.clear_elapsed = PREDICTIVE_COAST_CLEAR_S
      self.quick_release_active = False
      self.opening_release_active = False
      self.launch_floor_release_active = False
      self.recovery_floor_release_active = False
      self.brake_release_active = False
      return self.pedal_scale

    # A safely armed launch floor already proves that the same lead is moving
    # away, the driver is not braking, and curve/speed/FCW gates are clear.
    # Do not multiply that independently ramped floor by a stale coasting zero.
    launch_floor_release = bool(
      launch_boost_floor_active and plausible_lead and
      v_rel >= PREDICTIVE_COAST_OPENING_HOLD_VREL_MS and pressure <= 0.0 and
      curve_pressure <= 0.0 and speed_pressure <= 0.0 and not fcw)
    if launch_floor_release:
      self.phase = "launch_floor_release"
      self.pedal_scale = 1.0
      self.intervening = False
      self.brake_latched = False
      self.risk_elapsed = 0.0
      self.clear_elapsed = PREDICTIVE_COAST_CLEAR_S
      self.quick_release_active = True
      self.opening_release_active = True
      self.launch_floor_release_active = True
      self.recovery_floor_release_active = False
      self.brake_release_active = False
      return self.pedal_scale

    # The dedicated recovery modes have stricter entry gates than predictive
    # coasting (fresh plan, PID state, no brake/curve/speed/FCW, valid CAN and
    # either a safely receding lead or a confirmed lead-loss transition). Once
    # one is active, a stale zero coasting scale must not nullify its floor.
    recovery_floor_release = bool(
      positive_recovery_active and pressure <= 0.0 and
      curve_pressure <= 0.0 and speed_pressure <= 0.0 and not fcw)
    if recovery_floor_release:
      self.phase = "recovery_floor_release"
      self.pedal_scale = 1.0
      self.intervening = False
      self.brake_latched = False
      self.risk_elapsed = 0.0
      self.clear_elapsed = PREDICTIVE_COAST_CLEAR_S
      self.quick_release_active = False
      self.opening_release_active = bool(
        plausible_lead and v_rel >= PREDICTIVE_COAST_OPENING_HOLD_VREL_MS)
      self.launch_floor_release_active = False
      self.recovery_floor_release_active = True
      self.brake_release_active = False
      return self.pedal_scale

    quick_exit_max_kph = (
      PREDICTIVE_COAST_LEARNED_LOW_SPEED_MAX_KPH
      if _finite(learned_low_speed_coast_offset_s) > 0.0 else
      PREDICTIVE_COAST_LOW_SPEED_QUICK_EXIT_KPH)
    quick_release_candidate = bool(
      plausible_lead and
      v_ego * 3.6 <= quick_exit_max_kph and
      pressure <= 0.0 and v_rel >= PREDICTIVE_COAST_QUICK_EXIT_VREL_MS and
      self.distance_margin_m >= PREDICTIVE_COAST_QUICK_EXIT_MARGIN_M and
      self.time_to_gap_s >= PREDICTIVE_COAST_EXIT_TTG_S and
      curve_pressure <= 0.0 and speed_pressure <= 0.0)
    # Once quick recovery starts, use wider exit thresholds so a small radar
    # fluctuation cannot freeze the scale near zero again. Any renewed coast
    # pressure, closing lead, curve, or speed-limit request cancels it.
    quick_release_hold = bool(
      self.quick_release_active and plausible_lead and
      v_ego * 3.6 <= quick_exit_max_kph and pressure <= 0.0 and
      v_rel > PREDICTIVE_COAST_OPENING_HOLD_VREL_MS and
      self.distance_margin_m > 0.05 and
      curve_pressure <= 0.0 and speed_pressure <= 0.0)
    quick_lead_release = bool(quick_release_candidate or quick_release_hold)
    opening_release = bool(
      plausible_lead and pressure <= 0.0 and
      v_rel >= PREDICTIVE_COAST_OPENING_RELEASE_VREL_MS and
      curve_pressure <= 0.0 and speed_pressure <= 0.0)
    strong_opening_release = bool(
      opening_release and
      v_ego * 3.6 <= PREDICTIVE_COAST_QUICK_EXIT_MAX_KPH and
      v_rel >= PREDICTIVE_COAST_STRONG_OPENING_VREL_MS)
    self.opening_release_active = opening_release
    positive_demand_release = bool(
      plausible_lead and pressure <= 0.0 and
      _finite(requested_accel) >= PREDICTIVE_COAST_POSITIVE_DEMAND_RELEASE_ACCEL_MS2 and
      self.distance_margin_m >= PREDICTIVE_COAST_POSITIVE_DEMAND_RELEASE_MARGIN_M and
      v_rel >= PREDICTIVE_COAST_POSITIVE_DEMAND_RELEASE_VREL_MS and
      curve_pressure <= 0.0 and speed_pressure <= 0.0 and not fcw)
    self.positive_demand_release_active = positive_demand_release
    self.launch_floor_release_active = False
    self.recovery_floor_release_active = False
    self.brake_release_active = False
    lead_release_safe = bool(
      not plausible_lead or
      opening_release or
      positive_demand_release or
      (self.time_to_gap_s >= PREDICTIVE_COAST_EXIT_TTG_S and
       self.distance_margin_m >= PREDICTIVE_COAST_EXIT_MARGIN_M and
       v_rel >= -0.2))
    release_safe = bool(pressure <= 0.0 and
                        (lead_release_safe or quick_lead_release))
    clear_required = (PREDICTIVE_COAST_QUICK_CLEAR_S if quick_lead_release else
                       PREDICTIVE_COAST_CLEAR_S)
    brake_release_safe = bool(
      self.brake_latched and pressure <= 0.0 and
      curve_pressure <= 0.0 and speed_pressure <= 0.0 and
      (not plausible_lead or lead_release_safe))
    if brake_release_safe:
      clear_required = min(clear_required, PREDICTIVE_COAST_BRAKE_RELEASE_CLEAR_S)
      self.brake_release_active = True
    if not quick_lead_release:
      self.quick_release_active = False
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
        self.quick_release_active = False
        self.opening_release_active = False
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

      # Once the measured lead is clearly pulling away there is no reason to
      # hold the previous zero ceiling for another hysteresis interval. Start
      # a bounded positive recovery on this frame; closing/curve/speed/FCW
      # pressure on any later frame still cancels it immediately above.
      if strong_opening_release:
        self.quick_release_active = quick_lead_release
        self.brake_latched = False
        return self._recover(fast=True, quick_low_speed=quick_lead_release)

      if self.clear_elapsed < clear_required:
        self.phase = "hysteresis_hold"
        self.intervening = True
        return self.pedal_scale

      opening_fast = opening_release
      if brake_release_safe:
        opening_fast = True
      self.brake_latched = False
      if quick_release_candidate or quick_release_hold:
        self.quick_release_active = True
      return self._recover(fast=opening_fast,
                           quick_low_speed=quick_lead_release)

    self.phase = "idle"
    self.pedal_scale = 1.0
    self.intervening = False
    self.brake_latched = False
    self.quick_release_active = False
    self.opening_release_active = False
    self.launch_floor_release_active = False
    self.recovery_floor_release_active = False
    self.brake_release_active = False
    return self.pedal_scale
