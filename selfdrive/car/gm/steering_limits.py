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


def steer_delta_limits_kph(v_kph):
  """Return speed-dependent GM torque deltas for the 50 Hz steering command.

  Both curves share one breakpoint table, so the segment is found once and
  used twice. The tables already hold floats, so the float() call the old
  helper made on all four endpoints of every comparison was pure overhead.
  The interpolation expression is unchanged, down to the operator order.
  """
  try:
    speed = float(v_kph)
  except (TypeError, ValueError):
    return STEER_DELTA_UP_SAFE, STEER_DELTA_DOWN_SAFE

  if not math.isfinite(speed):
    return STEER_DELTA_UP_SAFE, STEER_DELTA_DOWN_SAFE

  speed = max(0.0, speed)

  bp = STEER_DELTA_BP_KPH
  if speed <= bp[0]:
    delta_up, delta_down = STEER_DELTA_UP_V[0], STEER_DELTA_DOWN_V[0]
  elif speed >= bp[-1]:
    delta_up, delta_down = STEER_DELTA_UP_V[-1], STEER_DELTA_DOWN_V[-1]
  else:
    i = 1
    while speed > bp[i]:
      i += 1
    x0 = bp[i - 1]
    span = max(bp[i] - x0, 1e-6)
    up0, down0 = STEER_DELTA_UP_V[i - 1], STEER_DELTA_DOWN_V[i - 1]
    delta_up = up0 + (speed - x0) * (STEER_DELTA_UP_V[i] - up0) / span
    delta_down = down0 + (speed - x0) * (STEER_DELTA_DOWN_V[i] - down0) / span

  return min(delta_up, STEER_DELTA_UP_MAX), min(delta_down, STEER_DELTA_DOWN_MAX)


def steer_delta_limits_ms(v_ego):
  try:
    return steer_delta_limits_kph(float(v_ego) * 3.6)
  except (TypeError, ValueError):
    return STEER_DELTA_UP_SAFE, STEER_DELTA_DOWN_SAFE
