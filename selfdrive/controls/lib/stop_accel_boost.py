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
STOP_ACCEL_BOOST_FACTOR = 1.54
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
#
# 2026-11-17: factor 1.40 -> 1.54 and this cap 1.10 -> 1.21 (+10%) because the
# start of the launch still felt sluggish. Raising the factor alone would have
# done nothing above the old cap. Watch for mid-launch cancels after this.
STOP_ACCEL_BOOST_MAX_ACCEL = 1.21

# A confirmed, safely receding lead may need a small launch request before the
# normal longitudinal PID becomes positive. Ramp to the known-effective
# Equinox pedal floor instead of multiplying zero by the boost factor.
#
# The floor used to stop at 8 km/h, which put its edge in the middle of the
# launch. It now covers the band the boost itself covers and fades out over the
# top of it. STOP_ACCEL_BOOST_FLOOR_GAP_ACCEL is sized so the fully justified
# floor (0.36 + 0.35 = 0.71 m/s^2, about 0.13 of pedal) matches the plateau the
# successful launches in the log actually held.
#
# 2026-11-17: both floor terms raised 10% (0.40 + 0.385 = 0.785 m/s^2, about
# 0.146 of pedal) for a firmer first second of the launch. With the 0.060 pedal
# deadzone this is roughly +18% of effective push, not +10%.
STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH = 20.0
STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS = STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FLOOR_FADE_SPEED_KPH = 12.0
STOP_ACCEL_BOOST_FLOOR_ACCEL = 0.40
STOP_ACCEL_BOOST_FLOOR_GAP_ACCEL = 0.385
STOP_ACCEL_BOOST_FLOOR_RAMP_S = 0.12
# Absolute ceiling on the floor, grade compensation included. The gap term and
# the hill term answer the same question -- "the floor is not moving the car" --
# from opposite ends, and stacking them at full scale would put 1.03 m/s^2 of
# unconditional launch floor on the road. 0.88 (0.80 before the 2026-11-17
# +10%) leaves the hill term its full 0.24 of headroom while the gap is quiet
# and squeezes it out once the gap has already commanded a strong launch.
STOP_ACCEL_BOOST_FLOOR_TOTAL_MAX = 0.88
STOP_ACCEL_BOOST_FLOOR_MIN_VREL_MS = 0.20
STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_SPEED_MS = 0.50
STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_DISTANCE_M = 2.50

# Lead-matching floor (2026-11-19 redesign).
#
# On the 11-19 15:42 drive (17 automatic standstill launches) the floor was
# the only thing moving the car below 10 km/h: the speed PID sat at its zero
# limit (vPid 1-3 km/h below vEgo, P term -0.3..-0.6) because the MPC was
# holding a ~6 m gap. The floor itself was sized by the opening rate, and the
# opening rate stays small exactly while the car keeps up -- so the command
# averaged 0.6-0.8 m/s^2 and the car 0.5-1.0, against a lead accelerating at
# 0.9-1.6 m/s^2. By 10 km/h (1.9-5.0 s after moving) the lead was 9-15 m ahead
# and doing 13-24 km/h. The planner cannot see that acceleration either:
# radard publishes aLeadK = 0 for vision leads.
#
# So the floor now follows the lead's own acceleration, taken from the model
# (modelV2.leadsV3[0].a[0], low-passed), plus a share of the opening rate so
# a gap that has already opened is closed rather than just held. Both are
# scaled by the remaining room over the safe gap, which goes to zero before
# the lead is reached, and the existing closing / re-stop cancels stay.
STOP_ACCEL_BOOST_LEAD_ACCEL_MAX = 1.5      # input clip on the model's a_lead
STOP_ACCEL_BOOST_LEAD_ACCEL_TAU_S = 0.30   # low-pass on the model's a_lead
STOP_ACCEL_BOOST_MATCH_VREL_GAIN = 0.30    # m/s^2 per m/s of opening
STOP_ACCEL_BOOST_MATCH_VREL_MAX_MS = 2.0
STOP_ACCEL_BOOST_MATCH_GAP_FULL_MARGIN_M = 2.0
STOP_ACCEL_BOOST_MATCH_MAX = 1.30
STOP_ACCEL_BOOST_MATCH_RISE_MS3 = 2.0

# Once the floor has engaged in a launch it holds while the lead is not
# closing, instead of while it is opening by 0.2 m/s. On the same drive the
# command fell to exactly 0 for 0.3-0.5 s at a time in 4 of 17 launches,
# every time vRel wobbled just under 0.2 because the car had caught up to a
# lead still creeping at 1-2 m/s. When the floor does drop out it now eases
# off instead of stepping to zero.
STOP_ACCEL_BOOST_FLOOR_HOLD_VREL_MS = -0.10
STOP_ACCEL_BOOST_FLOOR_HOLD_MIN_LEAD_ACCEL = -0.20
STOP_ACCEL_BOOST_FLOOR_FALL_MS3 = 2.0

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

# Standstill auto launch (Community toggle "StandstillAutoLaunch", off by
# default). Without it nothing sends pedal below 1 km/h and the driver has to
# start the car. On the 11-18 drives that meant 64 of 75 stop launches were the
# driver's gas press, 0.25 s after the lead started to move: the car stops about
# 3.6 m behind the lead, the MPC wants 5.5 m at standstill, so the plan only asks
# to go once the lead has opened ~2 m more. With the toggle on, a confirmed lead
# launch may start the car from standstill, but only while the same lead is
# clearly pulling away from a gap that is not tiny.
STANDSTILL_LAUNCH_MIN_LEAD_DISTANCE_M = 2.5
STANDSTILL_LAUNCH_MIN_LEAD_SPEED_MS = 0.5
STANDSTILL_LAUNCH_MIN_VREL_MS = 0.3


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


def stop_accel_boost_match_floor(v_ego, lead_distance, lead_relative_speed, lead_accel):
  """Floor that keeps pace with a launching lead (0 without room to use it)."""
  if float(lead_distance) <= 0.0:
    return 0.0
  margin = float(lead_distance) - stop_accel_boost_safe_gap(v_ego)
  room = _clip01(margin / STOP_ACCEL_BOOST_MATCH_GAP_FULL_MARGIN_M)
  a_lead = min(STOP_ACCEL_BOOST_LEAD_ACCEL_MAX, max(0.0, float(lead_accel)))
  opening = min(STOP_ACCEL_BOOST_MATCH_VREL_MAX_MS, max(0.0, float(lead_relative_speed)))
  return min(STOP_ACCEL_BOOST_MATCH_MAX,
             room * (a_lead + STOP_ACCEL_BOOST_MATCH_VREL_GAIN * opening))


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


# Plan sources that count as "the plan is following a lead". The MPC picks the
# smallest of its lead0 / lead1 / cruise obstacles. With vision leads, lead1 is
# the model's second hypothesis and at a stop it is almost always the same car
# as lead0 (same distance, same speed), so the source flips between the two on
# noise. Both are lead plans; only "cruise" is not.
LEAD_PLAN_SOURCES = ('lead0', 'lead1')

# 2026-11-19 drives: in all 10 automatic launches the floor never reached the
# pedal -- the command climbed 0 -> 0.8 m/s^2 over ~2 s on the PID alone, the
# car broke away only at ~0.7, and at 5 km/h with the latch on the command was
# as low as 0.05 (floor minimum 0.40). The latch itself was active at standstill,
# which needs a lead pulling away -- so floor_allowed was true and this gate was
# what refused it. The rlog could not say which term; launch_floor_diag.py logs
# that now.
#
# The curve terms are also relaxed here. A curve only matters to the floor if
# it asks for a speed below where the floor can take the car: the floor fades
# out by STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_KPH (20 km/h), and curve targets start
# at MIN_CURVE_SPEED (30 km/h). A curve "active" flag at a junction stop was
# blocking the launch for a slowdown that could never apply to it.
def boost_floor_context_block_reasons(*, can_valid, radar_valid,
                                      radar_error, driver_aware, curv_driving,
                                      curve_active, curve_target, speed_limit_active,
                                      speed_limit_target, v_ego, fcw, plan_valid,
                                      plan_age, plan_full, plan_source):
  """Names of the context terms that refuse the launch floor (empty: allowed)."""
  reasons = []
  if not can_valid:
    reasons.append('can')
  if not radar_valid:
    reasons.append('radar_invalid')
  if radar_error:
    reasons.append('radar_error')
  if not driver_aware:
    reasons.append('driver')
  if curv_driving or curve_active:
    floor_reach = max(float(v_ego), STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS)
    if float(curve_target) < floor_reach:
      reasons.append('curve')
  if speed_limit_decel_requested(speed_limit_active, speed_limit_target, v_ego):
    reasons.append('speed_limit')
  if fcw:
    reasons.append('fcw')
  if not plan_valid:
    reasons.append('plan_invalid')
  if not (0.0 <= float(plan_age) <= 0.25):
    reasons.append('plan_age')
  if not plan_full:
    reasons.append('plan_short')
  if str(plan_source) not in LEAD_PLAN_SOURCES:
    reasons.append('plan_source')
  return reasons


def boost_floor_context_allowed(floor_allowed, **context):
  """Final non-lead safety gate for the already validated launch floor.

  A published speed limit must not block a launch merely because it exists;
  it blocks only when the target is actually below current speed. Lead motion,
  distance, brake and driver-gas gates are owned by StopAccelBoostLatch.
  """
  return bool(floor_allowed and not boost_floor_context_block_reasons(**context))


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
    self.standstill_launch = False
    self.floor_engaged = False
    self.lead_accel = 0.0
    self.match_floor = 0.0
    self._floor_prev = 0.0
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
    self.floor_engaged = False
    self.match_floor = 0.0
    self._floor_prev = 0.0

  def update(self, system_ready, launch_detected, v_ego, brake_pressed=False,
             gas_pressed=False, lead_speed=0.0, lead_relative_speed=0.0,
             lead_distance=0.0, standstill_launch_enabled=False, lead_accel=0.0):
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

    # Model lead acceleration, low-passed; zero without a lead.
    lead_accel = float(lead_accel) if lead_valid else 0.0
    alpha = self.dt / (self.dt + STOP_ACCEL_BOOST_LEAD_ACCEL_TAU_S)
    self.lead_accel += alpha * (lead_accel - self.lead_accel)

    # Enter on a clearly opening lead; once engaged in this launch, hold
    # while the lead is merely not closing (see FLOOR_HOLD_VREL_MS).
    # The hold is for a lead that keeps going, not one that is slowing down.
    holding = bool(self.floor_engaged and
                   self.lead_accel > STOP_ACCEL_BOOST_FLOOR_HOLD_MIN_LEAD_ACCEL)
    min_vrel = (STOP_ACCEL_BOOST_FLOOR_HOLD_VREL_MS if holding
                else STOP_ACCEL_BOOST_FLOOR_MIN_VREL_MS)
    floor_lead_opening = bool(
      lead_valid and
      lead_distance >= STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_DISTANCE_M and
      lead_speed >= STOP_ACCEL_BOOST_FLOOR_MIN_LEAD_SPEED_MS and
      lead_relative_speed >= min_vrel)
    self.match_floor = stop_accel_boost_match_floor(
      v_ego, lead_distance, lead_relative_speed, self.lead_accel)
    # Below 1 km/h only with the toggle on, and only while the lead is clearly
    # pulling away. A latch that was already carrying the car keeps it until
    # the next stop: once moving, the normal >= 1 km/h band takes over.
    self.standstill_launch = bool(
      standstill_launch_enabled and self.latched and
      not gas_pressed and not brake_pressed and
      v_ego < STOP_ACCEL_BOOST_MIN_SPEED_MS and
      lead_valid and
      lead_distance >= STANDSTILL_LAUNCH_MIN_LEAD_DISTANCE_M and
      lead_speed >= STANDSTILL_LAUNCH_MIN_LEAD_SPEED_MS and
      lead_relative_speed >= STANDSTILL_LAUNCH_MIN_VREL_MS)
    in_band = STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_FLOOR_MAX_SPEED_MS
    self.floor_allowed = bool(
      self.latched and not gas_pressed and not brake_pressed and floor_lead_opening and
      (in_band or self.standstill_launch))
    if self.floor_allowed:
      self.floor_engaged = True

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
                (STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS or
                 self.standstill_launch))

  def _base_floor(self):
    """Ramped launch floor, sized by the justified authority and the speed band.

    The fixed 0.36 m/s^2 came out to about 0.067 of pedal, barely above this
    car's 0.060 interceptor deadzone, so on its own it never actually moved the
    car -- every bit of real launch acceleration came from the PID and got lost
    with it. The gap term takes the fully justified floor to 0.785 m/s^2, 10%
    above the 0.71 pedal plateau the successful launches held.
    """
    ramp = min(1.0, self.floor_elapsed / STOP_ACCEL_BOOST_FLOOR_RAMP_S)
    accel = (STOP_ACCEL_BOOST_FLOOR_ACCEL +
             STOP_ACCEL_BOOST_FLOOR_GAP_ACCEL * self.pressure)
    return accel * ramp * stop_accel_boost_floor_speed_scale(self._v_ego)

  def _match_floor(self):
    """Lead-matching floor, on the same ramp and speed band as the base floor."""
    ramp = min(1.0, self.floor_elapsed / STOP_ACCEL_BOOST_FLOOR_RAMP_S)
    return self.match_floor * ramp * stop_accel_boost_floor_speed_scale(self._v_ego)

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

    if self.floor_allowed and context_safe:
      # The hill term tops up the base floor under its old 0.88 ceiling; the
      # lead-matching floor stands on its own under STOP_ACCEL_BOOST_MATCH_MAX.
      # The base floor keeps its proven 0.12 s ramp; anything above it from
      # the lead-matching floor rises at MATCH_RISE_MS3 so the extra push
      # comes in as a roll-on, not a second lurch off the line.
      base = min(STOP_ACCEL_BOOST_FLOOR_TOTAL_MAX, self._base_floor() + self.hill_extra_accel)
      match = min(self._match_floor(),
                  max(base, self._floor_prev) + STOP_ACCEL_BOOST_MATCH_RISE_MS3 * self.dt)
      self.floor_accel = max(base, match)
    elif self.latched:
      # Ease off rather than step to zero; every hard cancel clears the latch
      # and so still drops the floor at once.
      self.floor_accel = max(0.0, self._floor_prev - STOP_ACCEL_BOOST_FLOOR_FALL_MS3 * self.dt)
    else:
      self.floor_accel = 0.0
    self._floor_prev = self.floor_accel
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


def pedal_command_allowed(v_ego, normal_min_speed_kph=1.0, standstill_launch=False):
  """No automatic pedal command below 1 km/h, except a standstill launch that
  StopAccelBoostLatch confirmed with the StandstillAutoLaunch toggle on."""
  return bool(standstill_launch) or float(v_ego) >= float(normal_min_speed_kph) / 3.6
