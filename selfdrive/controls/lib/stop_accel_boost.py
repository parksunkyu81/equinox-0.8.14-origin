STOP_ACCEL_BOOST_MIN_SPEED_KPH = 1.0
STOP_ACCEL_BOOST_MIN_SPEED_MS = STOP_ACCEL_BOOST_MIN_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FACTOR = 1.10

# The current Equinox pedal table maps 0.36 m/s^2 to about 0.067 pedal at
# launch, just above the measured 0.060 command needed for a reliable response.
STOP_ACCEL_LAUNCH_ACCEL = 0.36
STOP_ACCEL_ZERO_EPS = 1e-3


def apply_stop_accel_boost(requested_accel, v_ego, boost_active, accel_limits):
  """Release a confirmed launch and apply 10% boost inside existing limits."""
  accel = float(requested_accel)
  if boost_active:
    # Never turn a braking request into acceleration. The launch floor only
    # replaces an effectively-zero request while crossing the pedal deadzone.
    if v_ego < STOP_ACCEL_BOOST_MIN_SPEED_MS and abs(accel) <= STOP_ACCEL_ZERO_EPS:
      accel = STOP_ACCEL_LAUNCH_ACCEL
    elif v_ego >= STOP_ACCEL_BOOST_MIN_SPEED_MS and accel > 0.0:
      accel *= STOP_ACCEL_BOOST_FACTOR

  return min(float(accel_limits[1]), max(float(accel_limits[0]), accel))


def pedal_command_allowed(v_ego, launch_active, normal_min_speed_kph=1.0):
  """Allow normal pedal at >=1 km/h, or only a confirmed launch below it."""
  return float(v_ego) >= float(normal_min_speed_kph) / 3.6 or bool(launch_active)
