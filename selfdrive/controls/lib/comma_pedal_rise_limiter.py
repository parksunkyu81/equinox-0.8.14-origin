"""Rise-rate limiting and post-brake suppression for the comma-pedal command.

This car has a gas interceptor and no brake actuation: GM's
get_pid_accel_limits clamps accel_min to 0.0, so openpilot can only add
acceleration and the driver supplies every deceleration. Acceleration is
therefore a debt that is repaid with the driver's brake pedal, which makes the
cost asymmetric -- rising slowly costs a little speed, rising fast costs a
brake press. So the rise is limited here and the fall is not.

Measured across five drives on 2026-09-03 (16.1 min engaged, 65 driver brake
presses while engaged and moving):

  * 37 of the 50 stretches where the pedal was applying gas ended with the
    driver braking -- a 26% success rate.
  * In 78% of those the pedal was still holding or increasing when the brake
    went down. 13 of them had run 0.000 -> ~0.30 in about 1.5 s, i.e. ~0.20/s.
  * Peak pedal before a brake press was 0.22-0.32 against the 0.85 ceiling,
    and the response gain was a correctly applied 0.85 throughout. The
    magnitude was never the problem; the ramp was. Lowering the gain further
    only makes the car slow without changing the ramp -- that is what put the
    removed DrivingStyleAI learner on its 0.85 floor (see 04a68e08).
  * After a brake release the pedal came back within 0.3 s 72% of the time
    (median 0.16 s), which is what forces the driver to keep reapplying the
    brake.
  * 82% of the driver's brake applications were gentler than -1.0 m/s^2
    (median -0.67), so a good share of them are within reach of coasting if
    the pedal simply backs off earlier.

Below 8 km/h the picture inverts: 26% of brake releases happen there and the
driver re-accelerates immediately (median 0.00-0.15 s) because they are moving
off from a stop. Suppressing that would make launches sluggish, so the hold
collapses to a token value under LOW_SPEED_KPH.
"""

import math

from common.realtime import DT_CTRL


# Measured failure ramp was ~0.20/s. Replaying all five drives through this
# limiter, against the 37 brake presses that had the pedal applying gas, and
# measuring the cost as the drop in total pedal delivered while engaged:
#
#   rise    0 -> 0.30   events improved   pedal total
#   0.06/s    5.0 s      13/37 (35%)        -17.8%
#   0.08/s    3.8 s      13/37 (35%)        -13.7%
#   0.12/s    2.5 s      12/37 (32%)         -9.1%
#   0.16/s    1.9 s      11/37 (30%)         -6.6%
#   0.20/s    1.5 s       8/37 (22%)         -4.9%
#
# Below 0.12 the benefit stops improving while the cost keeps climbing, so
# 0.12 is the knee. Note the replay is open loop: it feeds the recorded pedal
# request through the limiter, but on the road a slower ramp leaves more speed
# error, so the planner asks for more and the real loss is smaller than the
# table's. Steady cruising is unaffected either way -- the pedal is already up
# by then -- so what this removes is the lunge, not the cruising speed.
PEDAL_RISE_PER_S = 0.12

# 0.12/s buys its brake presses by slowing the approach to a lead. Applying it
# with no lead in front buys nothing and only costs response.
#
# Re-measured on 2026-09-04--00-09-20 (6.4 min of automatic pedal): the final
# command sat more than 0.05 below what the controller asked for 53.5 s, in
# stretches up to 4.4 s, and five of the seven longest of those had no radar
# lead at all -- 21-50 km/h on open road, request 0.35 delivered as 0.28.
# Overall only 77.9% of the requested pedal was reaching the interceptor.
#
# So the rate is now a function of what is actually in front:
#
#   no lead, or a lead beyond FREE_GAP_M      PEDAL_RISE_FREE_PER_S
#   lead opening with room above desired gap  interpolated
#   lead close, or closing                    PEDAL_RISE_PER_S (unchanged)
#
# The cases the 0.12 measurement was taken from -- ramping toward a lead the
# car is about to have to brake for -- are exactly the ones that keep it.
PEDAL_RISE_FREE_PER_S = 0.35
PEDAL_RISE_FREE_GAP_M = 45.0
PEDAL_RISE_GAP_FULL_MARGIN_M = 8.0
PEDAL_RISE_OPENING_FULL_VREL_MS = 0.60
PEDAL_RISE_STANDSTILL_GAP_M = 4.5
PEDAL_RISE_DEFAULT_TR_S = 1.4
# Room is measured against the gap below which a ramp would actually risk a
# brake press, not against the driver's comfort following time. Measuring it
# against the comfort gap is what made this useless: with the 'long' profile
# and cruise gap 4 the desired gap at 20 km/h is ~12 m, so a lead sitting at a
# perfectly normal 12 m counted as having no room at all and the limit stayed
# at 0.12/s no matter how hard the lead was pulling away. A shorter driver TR
# still tightens this; a longer one no longer loosens the safety margin.
PEDAL_RISE_ROOM_TR_S = 0.9

# A brake application is the driver stating they want to be slower here.
# Hold the pedal down for a moment afterwards instead of resuming in 0.16 s,
# then come back at a reduced rate rather than snapping to the request.
#
# This scores far lower than the rise limit on the same replay (1.5 s / 3.0 s
# scored 5/37 for -7.4% on its own) but that metric only counts pedal height
# before the *next* brake press, which is not what makes the immediate resume
# unpleasant to sit behind. Kept at a moderate 1.0 s / 2.0 s: enough to stop
# the car chasing the driver off the brake, small enough that the combined
# cost stays near the knee (32% of events improved for -16.7% pedal total).
POST_BRAKE_HOLD_S = 1.0
POST_BRAKE_RAMP_S = 2.0
POST_BRAKE_RISE_SCALE = 0.5

# Under this speed a brake release means "move off", not "stay slow".
LOW_SPEED_KPH = 8.0
LOW_SPEED_POST_BRAKE_HOLD_S = 0.2


def _finite(value, default=0.0):
  try:
    value = float(value)
    return value if math.isfinite(value) else float(default)
  except (TypeError, ValueError):
    return float(default)


def _clip01(value):
  return 0.0 if value < 0.0 else (1.0 if value > 1.0 else float(value))


def pedal_rise_rate(v_ego=0.0, lead_valid=False, lead_distance=0.0,
                    lead_rel_speed=0.0, desired_tr=PEDAL_RISE_DEFAULT_TR_S):
  """Return the allowed pedal rise rate for the current lead picture.

  Room and opening rate both have to be there. A large gap to a lead that is
  braking is not room, and a lead pulling away from two metres is not either.
  """
  v_ego = max(0.0, _finite(v_ego, 0.0))
  lead_distance = max(0.0, _finite(lead_distance, 0.0))
  if not bool(lead_valid) or lead_distance <= 0.0:
    return PEDAL_RISE_FREE_PER_S
  if lead_distance >= PEDAL_RISE_FREE_GAP_M:
    return PEDAL_RISE_FREE_PER_S

  room_tr = min(PEDAL_RISE_ROOM_TR_S,
                max(0.0, _finite(desired_tr, PEDAL_RISE_DEFAULT_TR_S)))
  room_gap = PEDAL_RISE_STANDSTILL_GAP_M + v_ego * room_tr
  room = _clip01((lead_distance - room_gap) / PEDAL_RISE_GAP_FULL_MARGIN_M)
  opening = _clip01(_finite(lead_rel_speed, 0.0) /
                    PEDAL_RISE_OPENING_FULL_VREL_MS)
  freedom = min(room, opening)
  return PEDAL_RISE_PER_S + (PEDAL_RISE_FREE_PER_S - PEDAL_RISE_PER_S) * freedom


class CommaPedalRiseLimiter:
  """Limit how fast the pedal command may rise; never how fast it may fall."""

  def __init__(self, dt=DT_CTRL):
    self.dt = max(1e-3, _finite(dt, DT_CTRL))
    self.reset()

  def reset(self):
    self.pedal = 0.0
    # Seconds since the driver last released the brake. inf means "not in a
    # post-brake window", which is also the state after a gas override.
    self.post_brake_elapsed = math.inf
    self.rise_rate = PEDAL_RISE_PER_S
    self.base_rise_rate = PEDAL_RISE_PER_S
    self.holding = False
    self.bypassed = False
    self._brake_pressed_prev = False

  def _hold_s(self, v_ego_kph):
    return (LOW_SPEED_POST_BRAKE_HOLD_S if v_ego_kph < LOW_SPEED_KPH
            else POST_BRAKE_HOLD_S)

  def update(self, target, v_ego=0.0, brake_pressed=False, gas_pressed=False,
             bypass=False, lead_valid=False, lead_distance=0.0,
             lead_rel_speed=0.0, desired_tr=PEDAL_RISE_DEFAULT_TR_S):
    """Return the pedal command to send.

    target       the pedal the controller wants, already 0.0 whenever the pedal
                 is not allowed to act
    bypass       a deliberate, separately gated assist owns the pedal outright
                 (launch boost or one of the recovery floors); those carry their
                 own confirmation logic and measured 0-3% of brake presses, so
                 they are not slowed down here
    lead_*       the tracked lead, used only to decide how fast the command may
                 rise; with no lead the limit relaxes to PEDAL_RISE_FREE_PER_S
    """
    target = max(0.0, _finite(target, 0.0))
    v_ego_kph = max(0.0, _finite(v_ego, 0.0)) * 3.6
    brake_pressed = bool(brake_pressed)
    gas_pressed = bool(gas_pressed)
    self.base_rise_rate = pedal_rise_rate(
      v_ego=v_ego, lead_valid=lead_valid, lead_distance=lead_distance,
      lead_rel_speed=lead_rel_speed, desired_tr=desired_tr)

    if brake_pressed:
      # The pedal is already zero while braking; arm the window so the timer
      # starts from the release, not from now.
      self.pedal = 0.0
      self.post_brake_elapsed = 0.0
      self.rise_rate = 0.0
      self.holding = True
      self.bypassed = False
      self._brake_pressed_prev = True
      return 0.0

    if self._brake_pressed_prev:
      self.post_brake_elapsed = 0.0
    self._brake_pressed_prev = False

    if gas_pressed:
      # The driver is asking for acceleration themselves. Drop the suppression
      # so the system is not fighting them on the way out of it, and follow the
      # request (which is 0.0 while they hold the pedal) without a ramp.
      self.post_brake_elapsed = math.inf
      self.pedal = target
      self.rise_rate = self.base_rise_rate
      self.holding = False
      self.bypassed = False
      return self.pedal

    if math.isfinite(self.post_brake_elapsed):
      self.post_brake_elapsed += self.dt

    if bypass:
      self.pedal = target
      self.rise_rate = math.inf
      self.holding = False
      self.bypassed = True
      return self.pedal
    self.bypassed = False

    hold_s = self._hold_s(v_ego_kph)
    if self.post_brake_elapsed < hold_s:
      rate = 0.0
    elif self.post_brake_elapsed < hold_s + POST_BRAKE_RAMP_S:
      rate = self.base_rise_rate * POST_BRAKE_RISE_SCALE
    else:
      rate = self.base_rise_rate
      self.post_brake_elapsed = math.inf

    if target <= self.pedal:
      # Backing off is never delayed.
      self.pedal = target
    else:
      self.pedal = min(target, self.pedal + rate * self.dt)

    self.rise_rate = rate
    self.holding = bool(rate <= 0.0 and target > self.pedal)
    return self.pedal
