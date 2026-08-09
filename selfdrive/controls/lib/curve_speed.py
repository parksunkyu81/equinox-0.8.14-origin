import math

import numpy as np


CURVE_SPEED_SENTINEL = 255.0


def calculate_curve_speed(curvatures, v_ego, cruise_speed_ms, curvature_factor, min_curve_speed):
  """Return a stable curve speed target in m/s, or None for invalid input.

  LateralPlan contains 17 curvature samples covering the next 2.5 seconds.
  A three-sample median rejects a single noisy point, while taking the maximum
  of the filtered samples keeps both the current bend and an approaching bend.
  """
  values = np.asarray(curvatures, dtype=np.float64)
  if values.ndim != 1 or values.size < 3 or not np.all(np.isfinite(values)):
    return None

  abs_curvatures = np.abs(values)
  filtered = np.asarray([
    np.median(abs_curvatures[i:i + 3]) for i in range(abs_curvatures.size - 2)
  ])
  representative_curvature = max(float(np.max(filtered)), 1e-4)

  # Use the higher of current and requested speeds for an anticipatory limit.
  # Capping the reference at 130 km/h keeps the empirical lateral-acceleration
  # formula inside the range for which it was designed.
  speed_reference = max(float(v_ego), min(float(cruise_speed_ms), 130.0 / 3.6))
  lateral_accel_max = float(np.clip(2.975 - speed_reference * 0.0375, 1.2, 3.0))
  factor = float(np.clip(curvature_factor, 0.5, 1.5))

  model_speed = math.sqrt(lateral_accel_max / representative_curvature) * 0.85 * factor
  if not math.isfinite(model_speed) or model_speed <= 0.0:
    return None

  return max(float(model_speed), float(min_curve_speed))
