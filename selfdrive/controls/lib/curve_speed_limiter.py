import math

from common.numpy_fast import clip
from selfdrive.modeld.constants import T_IDXS


CURVE_SPEED_DISABLED = 255.0
CURVE_DECEL_MPS2 = 1.2
CURVE_ACTIVATION_MARGIN_MS = 0.5
CURVE_CONFIRM_FRAMES = 2
CURVE_INVALID_HOLD_FRAMES = 4
CURVE_TIGHTEN_RC = 0.20
CURVE_RELEASE_RC = 1.50
CURVATURE_FLOOR = 1e-4
CURVE_PLAN_DT = 0.05


def _smoothed_abs_curvatures(curvatures):
  """Three-point smoothing without cancelling opposite-direction curves."""
  values = [abs(float(v)) for v in curvatures]
  if len(values) < 3:
    return values

  smoothed = values[:]
  for i in range(1, len(values) - 1):
    smoothed[i] = 0.25 * values[i - 1] + 0.50 * values[i] + 0.25 * values[i + 1]
  return smoothed


def calculate_curve_speed(curvatures, v_ego, cruise_speed, min_curve_speed,
                          curvature_factor, time_idxs=T_IDXS):
  """Return a present-time speed ceiling using the complete MPC horizon.

  Each future curvature produces a safe speed at that point. Comfortable
  deceleration over the distance to that point is then added back to obtain
  the speed allowed now. This reacts early without applying the final corner
  speed to a bend that is still at the end of the horizon.
  """
  try:
    values = [float(v) for v in curvatures]
    v_ego = float(v_ego)
    cruise_speed = float(cruise_speed)
    min_curve_speed = float(min_curve_speed)
    curvature_factor = float(curvature_factor)
  except (TypeError, ValueError):
    return CURVE_SPEED_DISABLED, False

  if (len(values) == 0 or len(values) > len(time_idxs) or
      not all(math.isfinite(v) for v in values) or
      not all(math.isfinite(v) for v in (v_ego, cruise_speed, min_curve_speed, curvature_factor)) or
      v_ego < 0.0 or cruise_speed <= 0.0 or min_curve_speed <= 0.0 or curvature_factor <= 0.0):
    return CURVE_SPEED_DISABLED, False

  # Preserve the original Equinox lateral-acceleration profile, with a lower
  # bound for speeds above the range described by the original linear fit.
  a_y_max = clip(2.975 - v_ego * 0.0375, 1.85, 2.975)
  smoothed_curvatures = _smoothed_abs_curvatures(values)

  allowed_now = CURVE_SPEED_DISABLED
  for curvature, t in zip(smoothed_curvatures, time_idxs):
    curve_speed = math.sqrt(a_y_max / max(curvature, CURVATURE_FLOOR)) * curvature_factor
    curve_speed = max(curve_speed, min_curve_speed)

    # Approximate distance using the current measured speed. A 1 m/s floor
    # keeps the calculation well-defined while stopped.
    distance = max(v_ego, 1.0) * max(float(t), 0.0)
    speed_now = math.sqrt(curve_speed ** 2 + 2.0 * CURVE_DECEL_MPS2 * distance)
    allowed_now = min(allowed_now, speed_now)

  if allowed_now >= cruise_speed - CURVE_ACTIVATION_MARGIN_MS:
    return CURVE_SPEED_DISABLED, True
  return max(min_curve_speed, allowed_now), True


class CurveSpeedLimiter:
  """Stateful confirmation and asymmetric filtering for curve speed limits."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.speed_ms = CURVE_SPEED_DISABLED
    self.curve_frames = 0
    self.invalid_frames = 0

  def update(self, curvatures, v_ego, cruise_speed, min_curve_speed,
             curvature_factor, plan_valid=True):
    raw_speed, values_valid = calculate_curve_speed(
      curvatures, v_ego, cruise_speed, min_curve_speed, curvature_factor)
    values_valid = bool(plan_valid and values_valid)

    if not values_valid:
      self.invalid_frames += 1
      self.curve_frames = 0
      # Briefly hold the last safe limit across isolated dropped plans.
      if self.invalid_frames <= CURVE_INVALID_HOLD_FRAMES:
        return self.speed_ms
      raw_speed = CURVE_SPEED_DISABLED
    else:
      self.invalid_frames = 0

    curve_detected = raw_speed < CURVE_SPEED_DISABLED
    self.curve_frames = self.curve_frames + 1 if curve_detected else 0

    if self.speed_ms >= CURVE_SPEED_DISABLED:
      if self.curve_frames < CURVE_CONFIRM_FRAMES:
        return CURVE_SPEED_DISABLED
      self.speed_ms = float(cruise_speed)

    target = raw_speed if curve_detected else float(cruise_speed)
    rc = CURVE_TIGHTEN_RC if target < self.speed_ms else CURVE_RELEASE_RC
    alpha = CURVE_PLAN_DT / (rc + CURVE_PLAN_DT)
    self.speed_ms += alpha * (target - self.speed_ms)
    self.speed_ms = max(float(min_curve_speed), min(float(cruise_speed), self.speed_ms))

    if not curve_detected and self.speed_ms >= float(cruise_speed) - CURVE_ACTIVATION_MARGIN_MS:
      self.speed_ms = CURVE_SPEED_DISABLED

    return self.speed_ms
