import math


GM_MIN_STEER_SPEED_KPH = 10.0
GM_MIN_STEER_SPEED_MS = GM_MIN_STEER_SPEED_KPH / 3.6


# Strong low/mid-speed steering response for Equinox, then taper back for
# highway stability. Steering becomes active from 10 kph, peaks through
# 30-40 kph, and progressively returns to the stable 7/17 envelope by 70 kph.
STEER_DELTA_BP_KPH = (0.0, 10.0, 20.0, 30.0, 40.0, 45.0, 60.0, 70.0, 130.0)
STEER_DELTA_UP_V = (7.0, 11.0, 13.0, 14.0, 14.0, 13.0, 9.0, 7.0, 7.0)
STEER_DELTA_DOWN_V = (17.0, 18.0, 19.0, 20.0, 20.0, 19.0, 18.0, 17.0, 17.0)

STEER_DELTA_UP_MAX = 14.0
STEER_DELTA_DOWN_MAX = 20.0
STEER_DELTA_UP_SAFE = 7.0
STEER_DELTA_DOWN_SAFE = 17.0


def _interp(value, breakpoints, values):
  if value <= breakpoints[0]:
    return float(values[0])
  if value >= breakpoints[-1]:
    return float(values[-1])

  for i in range(1, len(breakpoints)):
    if value <= breakpoints[i]:
      x0, x1 = float(breakpoints[i - 1]), float(breakpoints[i])
      y0, y1 = float(values[i - 1]), float(values[i])
      return y0 + (value - x0) * (y1 - y0) / max(x1 - x0, 1e-6)
  return float(values[-1])


def steer_delta_limits_kph(v_kph):
  """Return speed-dependent GM torque deltas for the 50 Hz steering command."""
  try:
    speed = float(v_kph)
  except (TypeError, ValueError):
    return STEER_DELTA_UP_SAFE, STEER_DELTA_DOWN_SAFE

  if not math.isfinite(speed):
    return STEER_DELTA_UP_SAFE, STEER_DELTA_DOWN_SAFE

  speed = max(0.0, speed)
  delta_up = _interp(speed, STEER_DELTA_BP_KPH, STEER_DELTA_UP_V)
  delta_down = _interp(speed, STEER_DELTA_BP_KPH, STEER_DELTA_DOWN_V)
  return min(delta_up, STEER_DELTA_UP_MAX), min(delta_down, STEER_DELTA_DOWN_MAX)


def steer_delta_limits_ms(v_ego):
  try:
    return steer_delta_limits_kph(float(v_ego) * 3.6)
  except (TypeError, ValueError):
    return STEER_DELTA_UP_SAFE, STEER_DELTA_DOWN_SAFE
