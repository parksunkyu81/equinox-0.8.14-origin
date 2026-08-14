"""Bounded Equinox dynamic torque-authority scheduling.

The live-torque learner identifies the vehicle. This module decides how much
of the learned authority may be used for the current speed and corner. Keeping
those jobs separate prevents transient corner assistance from contaminating
the persistent learned values.
"""

import json

from common.numpy_fast import clip, interp


AUTHORITY_SPEED_BP = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0,
                      45.0, 60.0, 80.0, 100.0, 110.0, 130.0]

# The previous profile could lower the learned factor by 16% and raise
# friction by 20%. With a fully learned Equinox this added too much authority
# on top of an already valid model. The new envelope is intentionally modest;
# the speed-dependent ceiling below reduces the actually applied change again.
AUTHORITY_LAT_FACTOR_SCALE_V = [1.000, 0.985, 0.960, 0.945, 0.940, 0.940,
                                0.945, 0.955, 0.970, 0.995, 1.000, 1.000, 1.000]
AUTHORITY_FRICTION_SCALE_V = [1.000, 1.015, 1.040, 1.060, 1.080, 1.080,
                              1.075, 1.065, 1.045, 1.010, 1.000, 1.000, 1.000]
AUTHORITY_LAT_FACTOR_DOWN_V = [0.000, 0.015, 0.040, 0.055, 0.060, 0.060,
                               0.050, 0.030, 0.010, 0.000, 0.000, 0.000, 0.000]
AUTHORITY_FRICTION_UP_V = [0.000, 0.015, 0.040, 0.060, 0.080, 0.080,
                           0.070, 0.045, 0.020, 0.000, 0.000, 0.000, 0.000]

# Even a strong corner may only use part of the profile. This bounds the
# learned 1.948/0.168 values seen in the 2026-08-13 log to roughly 1.872/0.177
# at 25-30 km/h instead of the previous 1.75/0.202 extreme.
AUTHORITY_MAX_BP = [0.0, 8.0, 10.0, 15.0, 25.0, 35.0, 45.0, 60.0, 80.0, 130.0]
AUTHORITY_MAX_V = [0.00, 0.00, 0.30, 0.55, 0.65, 0.62, 0.55, 0.50, 0.12, 0.00]

CORNER_CURVATURE_BP = [0.00030, 0.00180]
CORNER_LAT_ACCEL_BP = [0.08, 0.85]
LOW_SPEED_GATE_BP = [0.0, 8.0, 10.0, 30.0, 35.0, 40.0, 45.0, 50.0]
LOW_SPEED_GATE_V = [0.0, 0.0, 1.00, 1.00, 0.90, 0.60, 0.30, 0.0]
MID_SPEED_GATE_BP = [35.0, 40.0, 45.0, 55.0, 60.0, 70.0]
MID_SPEED_GATE_V = [0.15, 0.30, 0.40, 0.40, 0.20, 0.0]
HIGH_SPEED_GATE_BP = [45.0, 60.0, 80.0, 110.0, 130.0]
HIGH_SPEED_GATE_V = [0.0, 0.10, 0.40, 0.55, 0.55]

BOOST_RISE_STEP = 0.025
BOOST_FALL_STEP = 0.060
BOOST_HOLD_FRAMES = 15  # about 0.15 s at the 100 Hz controls rate
BOOST_HOLD_CAP = 0.30
STEERING_PRESSED_MULT = 0.35
STEER_LIMITED_MULT = 0.75
STRONG_RATE_LIMITED_MULT = 0.45
TORQUE_SLEW_ACTIVE_MULT = 0.65

DIRECTION_REVERSAL_MIN_CURVATURE = 0.00035
DIRECTION_REVERSAL_DAMP_FRAMES = 30  # about 0.30 s
DIRECTION_REVERSAL_MULT = 0.20
DIRECTION_REVERSAL_BOOST_CAP = 0.10
DIRECTION_MEMORY_FRAMES = 50  # do not treat a new corner after a straight as a reversal

LAT_FACTOR_ABS_MIN = 1.75
LAT_FACTOR_ABS_MAX = 2.42
FRICTION_ABS_MIN = 0.165
FRICTION_ABS_MAX = 0.305

# Closed-loop response compensation. Initial response values come from the
# 2026-08-14 Equinox log and are subsequently adapted independently per bin.
RESPONSE_SPEED_UPPER_KPH = [20.0, 30.0, 40.0, 60.0, 80.0]
RESPONSE_INITIAL_V = [0.846, 0.914, 0.867, 0.843, 0.906]
RESPONSE_MAX_SCALE_V = [1.06, 1.04, 1.09, 1.12, 1.06]
RESPONSE_TARGET = 0.97
# Lateral acceleration is curvature * speed^2, so one global minimum excluded
# valid low-speed corners even after long drives.  The 2026-08-14 18:00+ EON
# log showed path-stable 10-20 km/h corner demand concentrated at 0.010-0.023;
# the previous 0.035 floor rejected every one of those samples.  Keep a 0.010
# noise floor there while retaining the sign, driver, path and limiter gates.
RESPONSE_MIN_DESIRED_LAT_ACCEL_V = [0.010, 0.060, 0.085, 0.100, 0.110]
RESPONSE_STABLE_SECONDS = 0.30
RESPONSE_SETPOINT_ABS_TOL = 0.020
RESPONSE_SETPOINT_REL_TOL = 0.080
RESPONSE_EMA_ALPHA = 0.015


class LateralResponseCompensator:
  """Learn bounded actual/desired lateral response in independent speed bins."""

  def __init__(self, serialized_state=None):
    self.responses = list(RESPONSE_INITIAL_V)
    self.counts = [0] * len(RESPONSE_SPEED_UPPER_KPH)
    self.stable_seconds = 0.0
    self.last_direction = 0
    self.last_desired = 0.0
    self.last_bin_index = 0
    self.scale = 1.0
    self.ratio = 1.0
    self.bin_index = 0
    self.stable = False
    self.frozen = True
    self.update_count = 0
    self.dirty = False
    self.load(serialized_state)

  def reset_transient(self):
    self.stable_seconds = 0.0
    self.last_direction = 0
    self.last_desired = 0.0
    self.last_bin_index = self.bin_index
    self.scale = 1.0
    self.ratio = 1.0
    self.stable = False
    self.frozen = True

  @staticmethod
  def _bin_for_speed(v_kph):
    speed = float(v_kph)
    for index, upper in enumerate(RESPONSE_SPEED_UPPER_KPH):
      if speed < upper:
        return index
    return len(RESPONSE_SPEED_UPPER_KPH) - 1

  def load(self, serialized_state):
    if not serialized_state:
      return
    try:
      if isinstance(serialized_state, bytes):
        serialized_state = serialized_state.decode('utf8')
      state = json.loads(serialized_state)
      responses = state.get('responses', [])
      counts = state.get('counts', [])
      if len(responses) == len(self.responses):
        self.responses = [float(clip(float(v), 0.72, 1.08)) for v in responses]
      if len(counts) == len(self.counts):
        self.counts = [max(0, int(v)) for v in counts]
      self.update_count = max(0, int(state.get('updateCount', sum(self.counts))))
    except Exception:
      pass

  def serialize(self):
    return json.dumps({
      'version': 1,
      'speedUpperBoundsKph': RESPONSE_SPEED_UPPER_KPH,
      'responses': [round(v, 6) for v in self.responses],
      'counts': self.counts,
      'updateCount': self.update_count,
    }, separators=(',', ':'))

  def update(self, v_kph, desired_lat, actual_lat, dt=0.01, steering_pressed=False,
             steer_limited=False, rate_limited=False, reversal_active=False,
             path_unstable=False):
    speed = max(0.0, float(v_kph))
    desired = float(desired_lat)
    actual = float(actual_lat)
    self.bin_index = self._bin_for_speed(speed)
    min_desired_lat_accel = RESPONSE_MIN_DESIRED_LAT_ACCEL_V[self.bin_index]
    bin_changed = self.bin_index != self.last_bin_index
    direction = 1 if desired > 0.0 else (-1 if desired < 0.0 else 0)
    # Model demand naturally moves a few percent on a real constant-radius
    # corner. Accept that motion while still rejecting an entry transition or
    # an abrupt path change.
    setpoint_stable = abs(desired - self.last_desired) <= max(
      RESPONSE_SETPOINT_ABS_TOL, RESPONSE_SETPOINT_REL_TOL * abs(desired))
    self.frozen = bool(speed < 10.0 or speed > 80.0 or steering_pressed or steer_limited or
                       rate_limited or reversal_active or path_unstable or
                       abs(desired) < min_desired_lat_accel or
                       direction == 0 or actual * desired <= 0.0)

    if self.frozen:
      self.stable_seconds = 0.0
      self.last_direction = direction
    else:
      if direction != self.last_direction or bin_changed or not setpoint_stable:
        self.stable_seconds = 0.0
      self.last_direction = direction
      self.stable_seconds += max(0.0, float(dt))
    self.last_desired = desired
    self.last_bin_index = self.bin_index

    self.stable = bool(not self.frozen and self.stable_seconds >= RESPONSE_STABLE_SECONDS)
    raw_ratio = float(clip(abs(actual) / max(abs(desired), 1e-3), 0.55, 1.20))
    if self.stable:
      idx = self.bin_index
      self.responses[idx] += RESPONSE_EMA_ALPHA * (raw_ratio - self.responses[idx])
      self.responses[idx] = float(clip(self.responses[idx], 0.72, 1.08))
      self.counts[idx] += 1
      self.update_count += 1
      self.dirty = True

    self.ratio = float(self.responses[self.bin_index])
    max_scale = float(RESPONSE_MAX_SCALE_V[self.bin_index])
    requested_scale = float(clip(RESPONSE_TARGET / max(self.ratio, 0.01), 1.0, max_scale))
    # Do not add learned authority while the sample is unsafe. Ramp down much
    # faster than ramp-up so model wobble cannot be amplified.
    target_scale = 1.0 if self.frozen else requested_scale
    step = 0.025 if target_scale < self.scale else 0.0025
    self.scale += float(clip(target_scale - self.scale, -step, step))
    self.scale = float(clip(self.scale, 1.0, max_scale))
    return self.scale

  def diagnostics(self):
    return {
      'scale': float(self.scale),
      'ratio': float(self.ratio),
      'bin': int(self.bin_index),
      'stable': bool(self.stable),
      'frozen': bool(self.frozen),
      'updateCount': int(self.update_count),
    }


def authority_confidence(total_points):
  """Return partial cold-start authority and progressively unlock the rest."""
  points = max(0.0, float(total_points))
  return float(interp(points, [0.0, 500.0, 2500.0], [0.35, 0.60, 1.00]))


def authority_ceiling(v_kph):
  return float(clip(interp(max(0.0, float(v_kph)), AUTHORITY_MAX_BP, AUTHORITY_MAX_V), 0.0, 1.0))


def corner_strength(desired_curvature, desired_lateral_accel):
  """Return demand-only corner strength without actuator feedback.

  Deliberately excluding the previous steering output prevents an applied
  correction from feeding back into the boost request and sustaining a weave.
  """
  curv_w = float(interp(abs(float(desired_curvature)), CORNER_CURVATURE_BP, [0.0, 1.0]))
  lat_w = float(interp(abs(float(desired_lateral_accel)), CORNER_LAT_ACCEL_BP, [0.0, 1.0]))
  return float(clip(max(curv_w, lat_w), 0.0, 1.0))


def effective_torque_params(base_lat_factor, base_friction, v_kph, corner_blend,
                            total_points):
  """Return bounded effective parameters without modifying the learned base."""
  base_lat = float(clip(float(base_lat_factor), LAT_FACTOR_ABS_MIN, LAT_FACTOR_ABS_MAX))
  base_fric = float(clip(float(base_friction), FRICTION_ABS_MIN, FRICTION_ABS_MAX))
  speed = max(0.0, float(v_kph))
  requested = min(float(clip(float(corner_blend), 0.0, 1.0)), authority_ceiling(speed))
  blend = requested * authority_confidence(total_points)

  lat_scale = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_LAT_FACTOR_SCALE_V))
  friction_scale = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_FRICTION_SCALE_V))
  lat_down = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_LAT_FACTOR_DOWN_V))
  friction_up = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_FRICTION_UP_V))

  effective_lat = base_lat + (base_lat * lat_scale - base_lat) * blend
  effective_fric = base_fric + (base_fric * friction_scale - base_fric) * blend

  lat_min = max(LAT_FACTOR_ABS_MIN, base_lat * (1.0 - lat_down))
  friction_max = min(FRICTION_ABS_MAX, base_fric * (1.0 + friction_up))
  effective_lat = float(clip(effective_lat, lat_min, base_lat))
  effective_fric = float(clip(effective_fric, base_fric, friction_max))
  return effective_lat, effective_fric, blend


class DynamicTorqueAuthorityScheduler:
  """Stateful boost ramp, short hold, and direction-reversal damping."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.boost = 0.0
    self.hold_frames = 0
    self.last_direction = 0
    self.direction_neutral_frames = 0
    self.direction_damp_frames = 0

  def update(self, v_kph, desired_curvature, desired_lateral_accel,
             steering_pressed=False, strong_driver_override=False,
             steer_limited=False, strong_rate_limited=False,
             torque_slew_active=False, output_reversal_active=False,
             path_unstable=False):
    speed = max(0.0, float(v_kph))
    curvature = float(desired_curvature)
    strength = corner_strength(curvature, desired_lateral_accel)
    ceiling = authority_ceiling(speed)
    low_gate = float(clip(interp(speed, LOW_SPEED_GATE_BP, LOW_SPEED_GATE_V), 0.0, 1.0))
    mid_gate = float(clip(interp(speed, MID_SPEED_GATE_BP, MID_SPEED_GATE_V), 0.0, 1.0))
    high_gate = float(clip(interp(speed, HIGH_SPEED_GATE_BP, HIGH_SPEED_GATE_V), 0.0, 1.0))

    direction = 0
    if abs(curvature) >= DIRECTION_REVERSAL_MIN_CURVATURE:
      direction = 1 if curvature > 0.0 else -1
      self.direction_neutral_frames = 0
    else:
      self.direction_neutral_frames += 1
      if self.direction_neutral_frames >= DIRECTION_MEMORY_FRAMES:
        self.last_direction = 0
    reversal = bool(direction and self.last_direction and direction != self.last_direction)
    if reversal:
      self.direction_damp_frames = DIRECTION_REVERSAL_DAMP_FRAMES
      self.hold_frames = 0
      self.boost = min(self.boost, DIRECTION_REVERSAL_BOOST_CAP)
    if direction:
      self.last_direction = direction

    direction_damping = self.direction_damp_frames > 0
    if direction_damping:
      self.direction_damp_frames -= 1

    # No forced minimum boost: authority is proportional only to the model's
    # current curvature/lateral-acceleration demand.
    target = strength * max(low_gate, mid_gate, high_gate)
    if path_unstable:
      target = 0.0
      self.hold_frames = 0
      self.boost = min(self.boost, DIRECTION_REVERSAL_BOOST_CAP)
    elif strong_driver_override:
      target = 0.0
      self.hold_frames = 0
      self.boost = min(self.boost, DIRECTION_REVERSAL_BOOST_CAP)
    elif steering_pressed:
      target *= STEERING_PRESSED_MULT
    if strong_rate_limited:
      target *= STRONG_RATE_LIMITED_MULT
    elif steer_limited:
      target *= STEER_LIMITED_MULT
    if torque_slew_active:
      target *= TORQUE_SLEW_ACTIVE_MULT
    if direction_damping:
      target = min(target * DIRECTION_REVERSAL_MULT, DIRECTION_REVERSAL_BOOST_CAP)
    if output_reversal_active:
      # Output sign changes can be caused by closed-loop correction even when
      # planner curvature keeps the same sign.  Drop held authority immediately
      # so the output guard is not fighting a stale corner boost.
      self.hold_frames = 0
      self.boost = min(self.boost, DIRECTION_REVERSAL_BOOST_CAP)
      target = min(target, DIRECTION_REVERSAL_BOOST_CAP)
    target = min(target, ceiling)

    if target > 0.10 and not direction_damping and not output_reversal_active:
      self.hold_frames = BOOST_HOLD_FRAMES
    elif self.hold_frames > 0 and target < self.boost:
      self.hold_frames -= 1
      target = max(target, min(self.boost, BOOST_HOLD_CAP) * max(low_gate, mid_gate, high_gate))

    if target > self.boost:
      self.boost = min(target, self.boost + BOOST_RISE_STEP)
    else:
      self.boost = max(target, self.boost - BOOST_FALL_STEP)
    self.boost = float(clip(self.boost, 0.0, ceiling))

    return {
      'authorityRequest': self.boost,
      'authorityCeiling': ceiling,
      'cornerStrength': strength,
      'lowGate': low_gate,
      'midGate': mid_gate,
      'highGate': high_gate,
      'directionReversal': reversal,
      'directionDamping': direction_damping,
      'outputReversalActive': bool(output_reversal_active),
      'pathUnstable': bool(path_unstable),
      'holdFrames': self.hold_frames,
    }
