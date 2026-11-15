"""Lift the pedal shortly BEFORE a corner the steering cannot hold.

Background (see STEER_SAT_* in controlsd.py): in tight city corners the GM LKA
torque (300, the EPS limit -- raising it faults the EPS) runs out and the
wheel stops around 80-100 deg while the plan still asks for more. The only way
to get more turn out of the same torque is less speed: lateral demand is
v^2 * curvature, and the tyre aligning torque the EPS has to beat grows with
it. This car has no brake authority (pedal interceptor only), so the lever is
taking the pedal off.

controlsd already does that, but only AFTER the command has been pinned at
100% for 0.4 s, i.e. already inside the corner. This looks at the lateral plan
up to LOOKAHEAD_S ahead and lifts before the corner instead.

Guards against it becoming annoying:
  1. False triggers on a straight road: the predicted demand must stay above
     TRIGGER_LAT_ACCEL, in the SAME turn direction, for CONFIRM_S of fresh plan
     frames, with a valid MPC plan, lateral control engaged and no lane change.
     2.0 m/s^2 is above the p99 (1.94) of all engaged driving on this car, so
     ordinary curves never reach it.
  2. Lifting too early: only demand within LOOKAHEAD_S (2.0 s) counts, so the
     car does not crawl toward a corner that is still far away.
  3. Not resuming after the corner: released as soon as the corner opens up --
     the curvature in the next RELEASE_LOOKAHEAD_S falls below
     RELEASE_CURV_FRACTION of the tightest curvature seen in this corner for
     RELEASE_S -- and never held longer than MAX_ACTIVE_S. Curvature, not
     lateral demand, decides the release: demand falls as the car slows, and
     releasing on it put the pedal back mid-corner and drove the car straight
     back into the torque limit.
  Plus: the driver pressing the gas cancels it at once and it stays off for the
  rest of that corner, so it never fights the driver.

It only ever removes pedal; it commands no deceleration and never touches
steering.
"""

from selfdrive.modeld.constants import T_IDXS

ENTRY_MIN_MS = 15.0 / 3.6
KEEP_MIN_MS = 10.0 / 3.6      # once lifting, coasting below 15 km/h is fine
MAX_MS = 60.0 / 3.6
TRIGGER_LAT_ACCEL = 2.0       # m/s^2
RELEASE_LAT_ACCEL = 1.6       # m/s^2, "this corner is over" for re-arming
RELEASE_LOOKAHEAD_S = 0.5
RELEASE_CURV_FRACTION = 0.5
LOOKAHEAD_S = 2.0
CONFIRM_S = 0.30
RELEASE_S = 0.20
MAX_ACTIVE_S = 10.0          # backstop only; the curvature release ends normal corners
PLAN_DT = 0.05                # lateralPlan is 20 Hz


def peak_demand(v_ego, curvatures, lookahead_s=LOOKAHEAD_S):
  """Largest |v^2 * curvature| in the plan within lookahead_s, and its sign."""
  peak, sign = 0.0, 0.0
  v2 = float(v_ego) ** 2
  for t, k in zip(T_IDXS, curvatures):
    if t > lookahead_s:
      break
    try:
      demand = v2 * float(k)
    except (TypeError, ValueError):
      continue
    if demand != demand:  # NaN
      continue
    if abs(demand) > peak:
      peak, sign = abs(demand), (1.0 if demand > 0 else -1.0)
  return peak, sign


def peak_curvature(curvatures, lookahead_s):
  peak = 0.0
  for t, k in zip(T_IDXS, curvatures):
    if t > lookahead_s:
      break
    try:
      k = abs(float(k))
    except (TypeError, ValueError):
      continue
    if k == k and k > peak:
      peak = k
  return peak


class PreCornerCoast:
  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.peak = 0.0
    self._sign = 0.0
    self._confirm_s = 0.0
    self._release_s = 0.0
    self._active_s = 0.0
    self._corner_curv = 0.0
    self._in_corner = False
    # Set by a gas press or the time cap; cleared once the corner is over.
    self._blocked = False

  def _deactivate(self, block=False):
    self.active = False
    self._confirm_s = 0.0
    self._release_s = 0.0
    self._active_s = 0.0
    self._in_corner = False
    if block:
      self._blocked = True

  def update(self, plan_updated, control_active, v_ego, curvatures, plan_valid,
             lane_change_active, gas_pressed, brake_pressed, dt):
    """Call every control frame. Returns True while the pedal should be lifted."""
    min_speed = KEEP_MIN_MS if self.active else ENTRY_MIN_MS
    eligible = bool(control_active and plan_valid and not lane_change_active and
                    not brake_pressed and min_speed <= v_ego <= MAX_MS)

    if gas_pressed:
      # The driver wants to go: get out of the way for the rest of this corner.
      self._deactivate(block=self.active or self._confirm_s > 0.0 or self._blocked)
      return False

    if not eligible:
      self._deactivate()
      self._sign = 0.0
      return False

    if self.active:
      self._active_s += dt
      if self._active_s > MAX_ACTIVE_S:
        self._deactivate(block=True)
        return False

    if not plan_updated:
      return self.active

    peak, sign = peak_demand(v_ego, curvatures)
    self.peak = peak

    # The corner is over: allow the next one.
    if peak < RELEASE_LAT_ACCEL:
      self._blocked = False

    if self.active:
      self._corner_curv = max(self._corner_curv,
                              peak_curvature(curvatures, LOOKAHEAD_S))
      near_curv = peak_curvature(curvatures, RELEASE_LOOKAHEAD_S)
      if near_curv >= RELEASE_CURV_FRACTION * self._corner_curv:
        self._in_corner = True
      if self._in_corner:
        # Inside the corner: resume once it opens up right in front of the car.
        corner_over = near_curv < RELEASE_CURV_FRACTION * self._corner_curv
      else:
        # Still approaching: release only if the predicted corner went away
        # (a prediction that did not hold up).
        corner_over = peak < RELEASE_LAT_ACCEL
      if corner_over:
        self._release_s += PLAN_DT
        if self._release_s >= RELEASE_S - 1e-9:
          self._deactivate()
      else:
        self._release_s = 0.0
      return self.active

    if self._blocked:
      return False

    if peak >= TRIGGER_LAT_ACCEL and (self._sign == 0.0 or sign == self._sign):
      self._confirm_s += PLAN_DT
      self._sign = sign
    else:
      self._confirm_s = 0.0
      self._sign = sign if peak >= TRIGGER_LAT_ACCEL else 0.0
      if peak >= TRIGGER_LAT_ACCEL:
        self._confirm_s = PLAN_DT

    if self._confirm_s >= CONFIRM_S - 1e-9:
      self.active = True
      self._release_s = 0.0
      self._active_s = 0.0
      self._corner_curv = peak_curvature(curvatures, LOOKAHEAD_S)
    return self.active
