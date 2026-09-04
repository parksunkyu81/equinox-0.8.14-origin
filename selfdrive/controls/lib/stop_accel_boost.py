"""Stopped-lead launch support for the gas-interceptor Equinox.

Measured on 2026-09-04--00-09-20 (19.3 min, 18.6 min engaged, 12 launches from
standstill), launch quality was bimodal and the split was entirely explained by
whether the latch below survived the launch:

  latch held >= 6.4 s   7 launches   10 km/h reached in 1.7-2.7 s
  latch cancelled early 5 launches   10 km/h reached in 5.4-10.3 s, or never

The cancellations were self-inflicted. The fixed 1.40 factor and the fixed
0.36 m/s^2 floor pushed measured acceleration to 1.1-1.4 m/s^2 against a
planner request of 0.4-0.7, so the ego briefly out-accelerated a lead that was
itself still launching. `lead_relative_speed` dipped to -0.30..-0.50 for a
fraction of a second and the old instantaneous `unsafe_approach` test cancelled
the latch on that single frame -- with no distance term at all. Every one of
the six low-speed cancellations happened at 5.7-11.7 m of gap that was opening
again within half a second.

After the cancel nothing held the pedal: the PID also outputs ~0 because
measured accel is still above the planned target, so the command sat at exactly
0.000 for 1.2-2.6 s while the lead pulled away. That dead stretch is what the
traffic behind reacts to, and it is why the driver had to press the gas 60
times in 19 minutes (38 of them below 10 km/h).

So the launch is now shaped by how much room the lead has actually left rather
than by fixed constants:

  * The boost factor and the launch floor scale with the lead's opening rate,
    bounded by the available gap. A lead the car is already matching gets
    almost no boost, which removes the overshoot that caused the cancels.
  * Closing is judged by time-to-gap, not by an instantaneous threshold, and
    must persist. A genuinely fast closure still cancels immediately.
  * The floor covers the whole launch band instead of stopping at 8 km/h, and
    tapers to zero between 12 and 20 km/h, so by the time the latch releases at
    the 25 km/h cutoff there is no step left to fall off.
  * The multiplier is capped in absolute terms, so it can fill in a weak launch
    request without amplifying an already strong one.
"""

STOP_ACCEL_BOOST_MIN_SPEED_KPH = 1.0
STOP_ACCEL_BOOST_MIN_SPEED_MS = STOP_ACCEL_BOOST_MIN_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_MAX_SPEED_KPH = 25.0
STOP_ACCEL_BOOST_MAX_SPEED_MS = STOP_ACCEL_BOOST_MAX_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FACTOR = 1.40
STOP_ACCEL_ZERO_EPS = 1e-3

# How much of the full boost authority the current lead picture justifies.
# Opening rate is the launch signal -- at a light the gap is a normal following
# gap by construction, and what says "go" is the lead pulling away from us. The
# gap only ever removes authority, never adds it.
STOP_ACCEL_BOOST_OPENING_FULL_VREL_MS = 1.00
STOP_ACCEL_BOOST_GAP_FULL_MARGIN_M = 3.00

# The multiplier exists to fill in a weak launch request, not to amplify a
# strong one. Left uncapped it took a 1.03 m/s^2 planner request to 1.44, which
# is the overshoot that made the car overtake its own still-launching lead. It
# may raise a request up to a brisk-but-comfortable launch and no further; a
# planner request already above this is passed through untouched.
STOP_ACCEL_BOOST_MAX_ACCEL = 1.10

# A confirmed, safely receding lead may need a small launch request before the
# normal longitudinal PID becomes positive. Ramp to the known-effective
# Equinox pedal floor instead of multiplying zero by the boost factor.
#
# The floor used to stop at 8 km/h, which put its edge in the middle of the
# launch. It now covers the band the boost itself covers and fades out over the
# top of it. STOP_ACCEL_BOOST_FLOOR_GAP_ACCEL is sized so the fully justified
# floor (0.36 + 0.35 = 0.71 m/s^2, about 0.13 of pedal) matches the plateau the
# successful launches in the log actually held.
STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH = 20.0
STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS = STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FLOOR_FADE_SPEED_KPH = 12.0
STOP_ACCEL_BOOST_FLOOR_ACCEL = 0.36
STOP_ACCEL_BOOST_FLOOR_GAP_ACCEL = 0.35
STOP_ACCEL_BOOST_FLOOR_RAMP_S = 0.12
# Absolute ceiling on the floor, grade compensation included. The gap term and
# the hill term answer the same question -- "the floor is not moving the car" --
# from opposite ends, and stacking them at full scale would put 0.95 m/s^2 of
# unconditional launch floor on the road. 0.80 leaves the hill term its full
# 0.24 of headroom while the gap is quiet and squeezes it out once the gap has
# already commanded a strong launch.
STOP_ACCEL_BOOST_FLOOR_TOTAL_MAX = 0.80
STOP_ACCEL_BOOST_FLOOR_MIN_VREL_MS = 0.20
STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_SPEED_MS = 0.50
STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_DISTANCE_M = 2.50

# A driver pedal press during a confirmed lead launch is an intent to move, not
# a request to forget the launch. Automatic pedal output remains suppressed
# while the driver is pressing the pedal, but the controller is allowed to keep
# its launch state warm for an immediate handoff on release.
STOP_ACCEL_HANDOFF_MIN_LEAD_SPEED_MS = 0.30
STOP_ACCEL_HANDOFF_MIN_VREL_MS = 0.05
STOP_ACCEL_HANDOFF_MIN_LEAD_DISTANCE_M = 2.50

# On an incline the normal launch floor can be consumed by grade resistance.
# Begin with the normal floor immediately, then add a bounded correction only
# when measured acceleration remains low while the lead continues to open.
STOP_ACCEL_HILL_RESPONSE_DELAY_S = 0.20
STOP_ACCEL_HILL_RESPONSE_AEGO_MS2 = 0.05
STOP_ACCEL_HILL_EXTRA_ACCEL_MAX = 0.24
STOP_ACCEL_HILL_EXTRA_RAMP_MS3 = 0.80
STOP_ACCEL_HILL_EXTRA_DECAY_MS3 = 1.60

# Cancel a latched launch if the same lead stops again or the ego vehicle is
# closing too quickly. A zero lead distance means that vision currently has no
# lead; in that case the normal longitudinal planner remains authoritative.
#
# "Closing too quickly" is a question about the gap, not about one frame of
# relative speed. -0.30 m/s against 6 m of room is 20 s away from anything and
# was cancelling six launches per drive; -0.30 m/s against 1 m of room is not.
# So the same threshold now has to survive a confirmation window and imply a
# short time-to-gap, while a genuinely fast closure cancels on the spot.
STOP_ACCEL_BOOST_LEAD_STOP_SPEED_MS = 0.20
STOP_ACCEL_BOOST_CLOSING_REL_SPEED_MS = -0.30
STOP_ACCEL_BOOST_HARD_CLOSING_REL_SPEED_MS = -1.00
STOP_ACCEL_BOOST_CLOSING_CONFIRM_S = 0.35
STOP_ACCEL_BOOST_CLOSING_TIME_TO_GAP_S = 3.0
STOP_ACCEL_BOOST_SAFE_GAP_M = 3.0
STOP_ACCEL_BOOST_SAFE_GAP_TR_S = 0.6
STOP_ACCEL_BOOST_MIN_LEAD_DISTANCE_M = 1.5
STOP_ACCEL_BOOST_LEAD_DROPOUT_HOLD_S = 0.15


def _clip01(value):
  return 0.0 if value < 0.0 else (1.0 if value > 1.0 else float(value))


def stop_accel_boost_safe_gap(v_ego):
  """Gap below which a closing lead is worth reacting to during a launch."""
  return (STOP_ACCEL_BOOST_SAFE_GAP_M +
          max(0.0, float(v_ego)) * STOP_ACCEL_BOOST_SAFE_GAP_TR_S)


def stop_accel_boost_closing_unsafe(v_ego, lead_distance, lead_relative_speed):
  """True when this closing rate really is about to consume the launch gap.

  Kept separate from the latch so the threshold can be reasoned about, and
  tested, without a state machine around it.
  """
  lead_relative_speed = float(lead_relative_speed)
  if lead_relative_speed > STOP_ACCEL_BOOST_CLOSING_REL_SPEED_MS:
    return False
  margin = float(lead_distance) - stop_accel_boost_safe_gap(v_ego)
  if margin <= 0.0:
    return True
  return (margin / -lead_relative_speed) <= STOP_ACCEL_BOOST_CLOSING_TIME_TO_GAP_S


def stop_accel_boost_pressure(v_ego, lead_distance, lead_relative_speed,
                              lead_valid=True):
  """Return 0..1: how much launch authority the lead picture justifies.

  The opening rate asks for acceleration and the remaining gap caps it, so a
  lead the car has already caught gets no boost even when the gap is large,
  and a large opening rate gets no boost when the gap is small. Without a lead
  there is nothing to justify a launch, so this is zero and the factor falls
  back to 1.0 (the unboosted planner request).
  """
  if not lead_valid or float(lead_distance) <= 0.0:
    return 0.0
  opening = _clip01(float(lead_relative_speed) /
                    STOP_ACCEL_BOOST_OPENING_FULL_VREL_MS)
  margin = float(lead_distance) - stop_accel_boost_safe_gap(v_ego)
  room = _clip01(margin / STOP_ACCEL_BOOST_GAP_FULL_MARGIN_M)
  return min(opening, room)


def stop_accel_boost_factor(pressure):
  """Taper the launch multiplier with the justified authority.

  A fixed 1.40 is what drove measured acceleration to 1.4 m/s^2 against a 0.7
  request and made the car overtake its own lead. Scaling it means the boost is
  strongest exactly when the lead is running away and disappears as the car
  catches up, which is both smoother and self-stabilising.
  """
  return 1.0 + (STOP_ACCEL_BOOST_FACTOR - 1.0) * _clip01(pressure)


def stop_accel_boost_floor_speed_scale(v_ego):
  """Fade the launch floor out over the top of its speed band."""
  speed_kph = max(0.0, float(v_ego)) * 3.6
  if speed_kph <= STOP_ACCEL_BOOST_FLOOR_FADE_SPEED_KPH:
    return 1.0
  if speed_kph >= STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH:
    return 0.0
  span = (STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH -
          STOP_ACCEL_BOOST_FLOOR_FADE_SPEED_KPH)
  return 1.0 - (speed_kph - STOP_ACCEL_BOOST_FLOOR_FADE_SPEED_KPH) / span


def speed_limit_decel_requested(speed_limit_active, speed_limit_target,
                                v_ego, margin=0.05):
  """True only when the published limit actually asks the car to slow."""
  return bool(
    speed_limit_active and
    float(speed_limit_target) < float(v_ego) - float(margin))


def boost_floor_context_allowed(floor_allowed, *, can_valid, radar_valid,
                                radar_error, driver_aware, curv_driving,
                                curve_active, speed_limit_active,
                                speed_limit_target, v_ego, fcw, plan_valid,
                                plan_age, plan_full, plan_source_lead):
  """Final non-lead safety gate for the already validated launch floor.

  A published speed limit must not block a launch merely because it exists;
  it blocks only when the target is actually below current speed. Lead motion,
  distance, brake and driver-gas gates are owned by StopAccelBoostLatch.
  """
  speed_limit_decel = speed_limit_decel_requested(
    speed_limit_active, speed_limit_target, v_ego)
  return bool(
    floor_allowed and can_valid and radar_valid and not radar_error and
    driver_aware and not curv_driving and not curve_active and
    not speed_limit_decel and not fcw and plan_valid and
    0.0 <= float(plan_age) <= 0.25 and plan_full and plan_source_lead)


class StopAccelBoostLatch:
  """Hold a confirmed lead launch from 1 km/h until the 25 km/h cutoff.

  A qualified driver launch keeps the latch and ramp warm while automatic pedal
  output is suppressed. This prevents a gas press/release from introducing a
  second launch-detection and PID-ramp delay.
  """

  def __init__(self, dt=0.01):
    self.dt = max(1e-3, float(dt))
    self.latched = False
    self.floor_elapsed = 0.0
    self.floor_allowed = False
    self.floor_accel = 0.0
    self.driver_launch_handoff = False
    self.hill_low_response_elapsed = 0.0
    self.hill_extra_accel = 0.0
    self.lead_missing_elapsed = 0.0
    self.closing_elapsed = 0.0
    self.pressure = 0.0
    self.boost_factor = 1.0
    self._v_ego = 0.0

  def _clear_latch(self):
    self.latched = False
    self.floor_elapsed = 0.0
    self.floor_allowed = False
    self.floor_accel = 0.0
    self.driver_launch_handoff = False
    self.hill_low_response_elapsed = 0.0
    self.hill_extra_accel = 0.0
    self.lead_missing_elapsed = 0.0

  def update(self, system_ready, launch_detected, v_ego, brake_pressed=False,
             gas_pressed=False, lead_speed=0.0, lead_relative_speed=0.0,
             lead_distance=0.0):
    v_ego = float(v_ego)
    lead_speed = float(lead_speed)
    lead_relative_speed = float(lead_relative_speed)
    lead_distance = float(lead_distance)

    lead_valid = lead_distance > 0.0
    self.pressure = stop_accel_boost_pressure(
      v_ego, lead_distance, lead_relative_speed, lead_valid)
    self.boost_factor = stop_accel_boost_factor(self.pressure)

    lead_stopped_again = (lead_valid and
                          lead_speed <= STOP_ACCEL_BOOST_LEAD_STOP_SPEED_MS and
                          lead_relative_speed <= 0.0)

    # Marginal closing has to persist before it counts; a real closure does not.
    if lead_valid and stop_accel_boost_closing_unsafe(
        v_ego, lead_distance, lead_relative_speed):
      self.closing_elapsed += self.dt
    else:
      self.closing_elapsed = 0.0
    unsafe_approach = lead_valid and (
      (lead_distance <= STOP_ACCEL_BOOST_MIN_LEAD_DISTANCE_M and lead_relative_speed <= 0.0) or
      lead_relative_speed <= STOP_ACCEL_BOOST_HARD_CLOSING_REL_SPEED_MS or
      self.closing_elapsed >= STOP_ACCEL_BOOST_CLOSING_CONFIRM_S)

    hard_cancel = bool(not system_ready or brake_pressed or
        v_ego >= STOP_ACCEL_BOOST_MAX_SPEED_MS or
        lead_stopped_again or unsafe_approach)
    if hard_cancel:
      self._clear_latch()
    else:
      if self.latched and not lead_valid:
        self.lead_missing_elapsed += self.dt
        if self.lead_missing_elapsed > STOP_ACCEL_BOOST_LEAD_DROPOUT_HOLD_S:
          self._clear_latch()
      else:
        self.lead_missing_elapsed = 0.0

      if launch_detected:
        # This may latch while stationary, but active remains false below 1 km/h.
        # It lets a driver-initiated launch receive boost without another delay.
        self.latched = True
        self.lead_missing_elapsed = 0.0

    handoff_lead_opening = bool(
      lead_valid and
      lead_distance >= STOP_ACCEL_HANDOFF_MIN_LEAD_DISTANCE_M and
      lead_speed >= STOP_ACCEL_HANDOFF_MIN_LEAD_SPEED_MS and
      lead_relative_speed >= STOP_ACCEL_HANDOFF_MIN_VREL_MS)
    handoff_can_continue = bool(
      self.driver_launch_handoff and self.latched and gas_pressed and
      not brake_pressed and v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS)
    self.driver_launch_handoff = bool(
      handoff_can_continue or
      (self.latched and gas_pressed and not brake_pressed and
       v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS and handoff_lead_opening))

    floor_lead_opening = bool(
      lead_valid and
      lead_distance >= STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_DISTANCE_M and
      lead_speed >= STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_SPEED_MS and
      lead_relative_speed >= STOP_ACCEL_BOOST_FLOOR_MIN_VREL_MS)
    self.floor_allowed = bool(
      self.latched and not gas_pressed and not brake_pressed and floor_lead_opening and
      STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS)

    # Precharge only after the driver has declared launch intent and the same
    # lead is safely opening. No automatic command is emitted while gasPressed;
    # this solely removes the otherwise empty ramp after pedal release.
    floor_precharge_allowed = bool(
      self.latched and not brake_pressed and floor_lead_opening and
      v_ego < STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS and
      (self.floor_allowed or self.driver_launch_handoff))
    if floor_precharge_allowed:
      self.floor_elapsed = min(STOP_ACCEL_BOOST_FLOOR_RAMP_S,
                               self.floor_elapsed + self.dt)
    elif not self.driver_launch_handoff:
      # Preserve a completed handoff ramp across a brief vision-lead wobble
      # while the driver still owns the pedal. Current safety gates are checked
      # again before floor_allowed can emit any automatic acceleration.
      self.floor_elapsed = 0.0

    self._v_ego = v_ego
    self.floor_accel = self._base_floor() if self.floor_allowed else 0.0

    return bool(self.latched and not gas_pressed and not brake_pressed and
                STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS)

  def _base_floor(self):
    """Ramped launch floor, sized by the justified authority and the speed band.

    The fixed 0.36 m/s^2 came out to about 0.067 of pedal, barely above this
    car's 0.060 interceptor deadzone, so on its own it never actually moved the
    car -- every bit of real launch acceleration came from the PID and got lost
    with it. The gap term takes the fully justified floor to 0.71 m/s^2, which
    is the pedal plateau the successful launches held.
    """
    ramp = min(1.0, self.floor_elapsed / STOP_ACCEL_BOOST_FLOOR_RAMP_S)
    accel = (STOP_ACCEL_BOOST_FLOOR_ACCEL +
             STOP_ACCEL_BOOST_FLOOR_GAP_ACCEL * self.pressure)
    return accel * ramp * stop_accel_boost_floor_speed_scale(self._v_ego)

  def update_hill_response(self, context_safe, a_ego):
    """Update grade compensation only while the launch floor can be emitted."""
    if self.floor_allowed and context_safe and float(a_ego) < STOP_ACCEL_HILL_RESPONSE_AEGO_MS2:
      self.hill_low_response_elapsed += self.dt
      if self.hill_low_response_elapsed >= STOP_ACCEL_HILL_RESPONSE_DELAY_S:
        self.hill_extra_accel = min(
          STOP_ACCEL_HILL_EXTRA_ACCEL_MAX,
          self.hill_extra_accel + STOP_ACCEL_HILL_EXTRA_RAMP_MS3 * self.dt)
    else:
      self.hill_low_response_elapsed = 0.0
      self.hill_extra_accel = max(
        0.0, self.hill_extra_accel - STOP_ACCEL_HILL_EXTRA_DECAY_MS3 * self.dt)

    self.floor_accel = (
      min(STOP_ACCEL_BOOST_FLOOR_TOTAL_MAX,
          self._base_floor() + self.hill_extra_accel)
      if self.floor_allowed and context_safe else 0.0)
    return self.floor_accel


def apply_stop_accel_boost(requested_accel, v_ego, boost_active, accel_limits,
                           launch_floor_accel=0.0,
                           boost_factor=STOP_ACCEL_BOOST_FACTOR):
  """Apply launch boost and an independently safety-gated ramped floor.

  `boost_factor` is the lead-tapered multiplier from StopAccelBoostLatch. It is
  never allowed to exceed the fixed ceiling it replaced, and never to reduce
  the planner's own request.
  """
  accel = float(requested_accel)
  factor = min(STOP_ACCEL_BOOST_FACTOR, max(1.0, float(boost_factor)))
  if (boost_active and
      STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS and
      accel > 0.0):
    accel = min(accel * factor, max(accel, STOP_ACCEL_BOOST_MAX_ACCEL))
  # The floor repairs only the observed zero-request launch hole. Never turn a
  # meaningful planner deceleration request into positive pedal.
  if boost_active and accel >= -STOP_ACCEL_ZERO_EPS:
    accel = max(accel, max(0.0, float(launch_floor_accel)))

  return min(float(accel_limits[1]), max(float(accel_limits[0]), accel))


def pedal_command_allowed(v_ego, normal_min_speed_kph=1.0):
  """Never send an automatic pedal command below 1 km/h."""
  return float(v_ego) >= float(normal_min_speed_kph) / 3.6
