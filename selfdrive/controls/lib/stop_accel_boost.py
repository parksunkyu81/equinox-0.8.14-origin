STOP_ACCEL_BOOST_MIN_SPEED_KPH = 1.0
STOP_ACCEL_BOOST_MIN_SPEED_MS = STOP_ACCEL_BOOST_MIN_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_MAX_SPEED_KPH = 25.0
STOP_ACCEL_BOOST_MAX_SPEED_MS = STOP_ACCEL_BOOST_MAX_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FACTOR = 1.30
STOP_ACCEL_ZERO_EPS = 1e-3
# A confirmed, safely receding lead may need a small launch request before the
# normal longitudinal PID becomes positive. Ramp to the known-effective
# Equinox pedal floor instead of multiplying zero by the boost factor.
STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH = 8.0
STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS = STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FLOOR_ACCEL = 0.36
STOP_ACCEL_BOOST_FLOOR_RAMP_S = 0.12
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
STOP_ACCEL_BOOST_LEAD_STOP_SPEED_MS = 0.20
STOP_ACCEL_BOOST_CLOSING_REL_SPEED_MS = -0.30
STOP_ACCEL_BOOST_MIN_LEAD_DISTANCE_M = 1.5
STOP_ACCEL_BOOST_LEAD_DROPOUT_HOLD_S = 0.15


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
    lead_stopped_again = (lead_valid and
                          lead_speed <= STOP_ACCEL_BOOST_LEAD_STOP_SPEED_MS and
                          lead_relative_speed <= 0.0)
    unsafe_approach = lead_valid and (
      (lead_distance <= STOP_ACCEL_BOOST_MIN_LEAD_DISTANCE_M and lead_relative_speed <= 0.0) or
      lead_relative_speed <= STOP_ACCEL_BOOST_CLOSING_REL_SPEED_MS)

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

    base_floor = STOP_ACCEL_BOOST_FLOOR_ACCEL * min(
      1.0, self.floor_elapsed / STOP_ACCEL_BOOST_FLOOR_RAMP_S)
    self.floor_accel = base_floor if self.floor_allowed else 0.0

    return bool(self.latched and not gas_pressed and not brake_pressed and
                STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS)

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

    base_floor = STOP_ACCEL_BOOST_FLOOR_ACCEL * min(
      1.0, self.floor_elapsed / STOP_ACCEL_BOOST_FLOOR_RAMP_S)
    self.floor_accel = (base_floor + self.hill_extra_accel
                        if self.floor_allowed and context_safe else 0.0)
    return self.floor_accel


def apply_stop_accel_boost(requested_accel, v_ego, boost_active, accel_limits,
                           launch_floor_accel=0.0):
  """Apply launch boost and an independently safety-gated ramped floor."""
  accel = float(requested_accel)
  if (boost_active and
      STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS and
      accel > 0.0):
    accel *= STOP_ACCEL_BOOST_FACTOR
  # The floor repairs only the observed zero-request launch hole. Never turn a
  # meaningful planner deceleration request into positive pedal.
  if boost_active and accel >= -STOP_ACCEL_ZERO_EPS:
    accel = max(accel, max(0.0, float(launch_floor_accel)))

  return min(float(accel_limits[1]), max(float(accel_limits[0]), accel))


def pedal_command_allowed(v_ego, normal_min_speed_kph=1.0):
  """Never send an automatic pedal command below 1 km/h."""
  return float(v_ego) >= float(normal_min_speed_kph) / 3.6
