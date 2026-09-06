import math

from common.conversions import Conversions as CV
from common.numpy_fast import clip


# Two different floors, which used to be one number and contradicted the rest of
# the feature because of it.
#
# CURVE_PEDAL_MIN_SPEED_KPH gates whether this coordinator runs at all: below it
# there is not enough throttle in play for lifting off to mean anything. It was
# 40, while the curve target the limiter asks for is MIN_CURVE_SPEED = 30 km/h --
# so the half that computes a target aimed at 30 and the half that acts on it
# refused to work below 40, and they could never meet. Measured on
# 2026-09-06--09-46-21: the limiter confirmed a curve on 903 model frames, at a
# speed of p10 9.0 / p50 21.1 / p90 27.3 km/h, and this gate rejected 100% of
# them -- curvDriving was active for 0.00% of the drive. On
# 2026-09-05--17-23-32 it rejected 98.6% and curvDriving ran 2.37%, which is the
# sliver where a curve tight enough to need 30 km/h is still being approached
# above 40. 20 km/h sits below the speeds curves are actually taken at here and
# still well above the 10 km/h GM lateral-control floor.
CURVE_PEDAL_MIN_SPEED_KPH = 20.0
# CURVE_PLAN_MIN_SPEED_KPH floors the speed this coordinator will command. It has
# to sit at or below MIN_CURVE_SPEED or the target gets clamped back up on its
# way out -- the old code raised a 30 km/h request to 40 in three separate
# places (the recommendation, the entry floor and the output clip), so even the
# engagements that survived the gate above could not deliver what was asked.
CURVE_PLAN_MIN_SPEED_KPH = 30.0
# How far below the speed it entered at the car may be asked to slow. The limiter
# now returns a graded target rather than a fixed 30 km/h, so a cap of 10 km/h
# would silently truncate any real curve that needs more than that; it exists to
# reject a pathological request, not to shape ordinary ones.
CURVE_ENTRY_DROP_KPH = 25.0
CURVE_PLAN_FALL_KPH_S = 5.0
CURVE_PLAN_RISE_KPH_S = 10.0
CURVE_EXIT_CONFIRM_S = 0.80
CURVE_APPROACH_START_S = 2.50
CURVE_ENTRY_TIME_S = 0.80
CURVE_ACCEL_FALL_JERK = 1.50
CURVE_ACCEL_RISE_JERK = 1.20
CURVE_SPEED_GAP_FULL_LIFT_KPH = 10.0


def _finite(value, default=0.0):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return float(default)
  return value if math.isfinite(value) else float(default)


def _rate_limit(value, target, fall_rate, rise_rate, dt):
  delta = float(target) - float(value)
  return float(value) + clip(delta, -float(fall_rate) * dt, float(rise_rate) * dt)


class CurvePedalCoordinator:
  """Coordinates future-curve lift-off with manual-brake pedal handoff."""

  def __init__(self, dt=0.01):
    self.dt = max(1e-3, float(dt))
    self.reset()

  def reset(self):
    self.curve_active = False
    self.exit_elapsed = CURVE_EXIT_CONFIRM_S
    self.entry_speed_kph = 0.0
    self.plan_speed_kph = 0.0
    self.recommended_speed_kph = 0.0
    self.selected_time_s = None
    self.brake_latched = False
    self.pedal_intervening = False
    self.last_output_accel = 0.0
    self.last_lift_ratio = 0.0

  @property
  def phase(self):
    if self.curve_active:
      if self.brake_latched:
        return "brake_hold"
      if self.selected_time_s is not None and self.selected_time_s > CURVE_ENTRY_TIME_S:
        return "approach"
      return "active"
    if self.exit_elapsed < CURVE_EXIT_CONFIRM_S:
      return "exit"
    return "idle"

  @property
  def engaged(self):
    return self.curve_active or self.exit_elapsed < CURVE_EXIT_CONFIRM_S

  def update_curve(self, detected, v_ego, cruise_speed_kph,
                   recommended_speed_ms, selected_time_s=None):
    v_kph = max(0.0, _finite(v_ego) * CV.MS_TO_KPH)
    cruise_kph = max(CURVE_PLAN_MIN_SPEED_KPH, _finite(cruise_speed_kph, v_kph))
    recommended_kph = max(CURVE_PLAN_MIN_SPEED_KPH,
                          _finite(recommended_speed_ms, cruise_kph * CV.KPH_TO_MS) * CV.MS_TO_KPH)
    detected = bool(detected and v_kph >= CURVE_PEDAL_MIN_SPEED_KPH and recommended_kph < cruise_kph)
    selected_time = (None if selected_time_s is None else max(0.0, _finite(selected_time_s)))
    # A confirmed curve can first appear up to five seconds ahead. Observe it,
    # but do not lower vCruise or pedal output until the 2.5 s approach window;
    # otherwise the driver feels an unexplained lift while the road is straight.
    within_approach = selected_time is None or selected_time <= CURVE_APPROACH_START_S
    activate_curve = bool(detected and (self.curve_active or self.engaged or within_approach))

    if activate_curve:
      # A short false-negative inside the exit hysteresis is the same curve, so
      # preserve the original entry speed instead of subtracting
      # CURVE_ENTRY_DROP_KPH again from a later, already-reduced speed.
      if not self.curve_active and self.exit_elapsed >= CURVE_EXIT_CONFIRM_S:
        self.entry_speed_kph = v_kph
        self.plan_speed_kph = min(cruise_kph, max(v_kph, CURVE_PLAN_MIN_SPEED_KPH))
      self.curve_active = True
      self.exit_elapsed = 0.0
      self.recommended_speed_kph = recommended_kph
      self.selected_time_s = selected_time
    elif self.curve_active:
      self.curve_active = False
      self.exit_elapsed = min(CURVE_EXIT_CONFIRM_S, self.dt)
      self.selected_time_s = None
    elif self.exit_elapsed < CURVE_EXIT_CONFIRM_S:
      next_exit_elapsed = self.exit_elapsed + self.dt
      self.exit_elapsed = (CURVE_EXIT_CONFIRM_S
                           if next_exit_elapsed >= CURVE_EXIT_CONFIRM_S - 1e-9
                           else next_exit_elapsed)

    if (not activate_curve and not self.engaged and self.plan_speed_kph <= 0.0):
      self.entry_speed_kph = 0.0
      self.recommended_speed_kph = 0.0
      self.brake_latched = False
      return None

    if self.curve_active:
      entry_floor = max(CURVE_PLAN_MIN_SPEED_KPH, self.entry_speed_kph - CURVE_ENTRY_DROP_KPH)
      desired_plan = max(self.recommended_speed_kph, entry_floor)
    else:
      desired_plan = cruise_kph

    if self.plan_speed_kph <= 0.0:
      self.plan_speed_kph = min(cruise_kph, max(v_kph, CURVE_PLAN_MIN_SPEED_KPH))
    self.plan_speed_kph = _rate_limit(
      self.plan_speed_kph, desired_plan,
      CURVE_PLAN_FALL_KPH_S, CURVE_PLAN_RISE_KPH_S, self.dt)
    self.plan_speed_kph = clip(self.plan_speed_kph, CURVE_PLAN_MIN_SPEED_KPH, cruise_kph)

    recovered = not self.engaged and abs(self.plan_speed_kph - cruise_kph) < 0.1
    if recovered:
      self.plan_speed_kph = 0.0
      self.entry_speed_kph = 0.0
      self.recommended_speed_kph = 0.0
      self.brake_latched = False

    # Continue publishing the rising target after curve exit until it reaches
    # cruise. This prevents the longitudinal planner's target from jumping in
    # one frame even though pedal output itself is rate-limited.
    return self.plan_speed_kph * CV.KPH_TO_MS if not recovered else None

  def _curve_lift_ratio(self, v_ego):
    if not self.curve_active:
      return 0.0
    v_kph = max(0.0, _finite(v_ego) * CV.MS_TO_KPH)
    speed_ratio = clip((v_kph - self.recommended_speed_kph) /
                       CURVE_SPEED_GAP_FULL_LIFT_KPH, 0.0, 1.0)
    if self.selected_time_s is None:
      time_ratio = 1.0
    else:
      time_ratio = clip((CURVE_APPROACH_START_S - self.selected_time_s) /
                        (CURVE_APPROACH_START_S - CURVE_ENTRY_TIME_S), 0.0, 1.0)
      if self.selected_time_s <= CURVE_ENTRY_TIME_S:
        time_ratio = 1.0
    return float(speed_ratio * time_ratio)

  def update_accel(self, raw_accel, active, v_ego, brake_pressed, gas_pressed,
                   urgent_safety=False):
    raw = _finite(raw_accel)
    if not active:
      self.brake_latched = False
      self.pedal_intervening = False
      self.last_output_accel = raw
      return raw

    if gas_pressed:
      # LongControl already yields to the driver. Do not retain an automatic
      # curve hold after an explicit accelerator override.
      self.brake_latched = False
      self.pedal_intervening = False
      self.last_output_accel = raw
      return raw

    if brake_pressed:
      if self.engaged:
        self.brake_latched = True
      self.pedal_intervening = bool(self.engaged)
      self.last_output_accel = 0.0
      return 0.0

    if urgent_safety or raw <= 0.0:
      # Never retain positive pedal when the lead/MPC/safety planner asks for
      # zero or deceleration. Smooth lift-off is only for a clear-road future
      # curve while the underlying longitudinal request remains positive.
      self.pedal_intervening = True
      self.last_output_accel = min(raw, 0.0)
      return self.last_output_accel

    lift_ratio = self._curve_lift_ratio(v_ego)
    self.last_lift_ratio = lift_ratio
    brake_hold = bool(self.brake_latched and self.engaged)
    curve_target = raw
    if brake_hold:
      curve_target = 0.0
    elif lift_ratio > 0.0 and raw > 0.0:
      curve_target = raw * (1.0 - lift_ratio)

    intervention = bool(brake_hold or lift_ratio > 0.0 or
                        (self.pedal_intervening and self.engaged))
    if intervention and curve_target < self.last_output_accel:
      output = max(curve_target, self.last_output_accel - CURVE_ACCEL_FALL_JERK * self.dt)
    elif (self.pedal_intervening or self.phase == "exit") and curve_target > self.last_output_accel:
      output = min(curve_target, self.last_output_accel + CURVE_ACCEL_RISE_JERK * self.dt)
    else:
      output = curve_target

    self.pedal_intervening = bool(intervention or self.phase == "exit" or
                                  output < raw - 1e-4)
    if not self.engaged and output >= raw - 1e-4:
      self.pedal_intervening = False
      self.brake_latched = False
    self.last_output_accel = float(output)
    return self.last_output_accel
