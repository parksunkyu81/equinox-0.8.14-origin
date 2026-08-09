STOP_ACCEL_BOOST_MIN_SPEED_KPH = 1.0
STOP_ACCEL_BOOST_MIN_SPEED_MS = STOP_ACCEL_BOOST_MIN_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_MAX_SPEED_KPH = 25.0
STOP_ACCEL_BOOST_MAX_SPEED_MS = STOP_ACCEL_BOOST_MAX_SPEED_KPH / 3.6
STOP_ACCEL_BOOST_FACTOR = 1.20
STOP_ACCEL_ZERO_EPS = 1e-3

# Cancel a latched launch if the same lead stops again or the ego vehicle is
# closing too quickly. A zero lead distance means that vision currently has no
# lead; in that case the normal longitudinal planner remains authoritative.
STOP_ACCEL_BOOST_LEAD_STOP_SPEED_MS = 0.20
STOP_ACCEL_BOOST_CLOSING_REL_SPEED_MS = -0.30
STOP_ACCEL_BOOST_MIN_LEAD_DISTANCE_M = 3.0


class StopAccelBoostLatch:
  """Hold a confirmed lead launch from 1 km/h until the 25 km/h cutoff."""

  def __init__(self):
    self.latched = False

  def update(self, system_ready, launch_detected, v_ego, brake_pressed=False,
             gas_pressed=False, lead_speed=0.0, lead_relative_speed=0.0,
             lead_distance=0.0):
    v_ego = float(v_ego)
    lead_speed = float(lead_speed)
    lead_relative_speed = float(lead_relative_speed)
    lead_distance = float(lead_distance)

    lead_valid = lead_distance > 0.0
    lead_stopped_again = lead_valid and lead_speed <= STOP_ACCEL_BOOST_LEAD_STOP_SPEED_MS
    unsafe_approach = lead_valid and (lead_distance <= STOP_ACCEL_BOOST_MIN_LEAD_DISTANCE_M or
                                      lead_relative_speed <= STOP_ACCEL_BOOST_CLOSING_REL_SPEED_MS)

    if (not system_ready or brake_pressed or
        v_ego >= STOP_ACCEL_BOOST_MAX_SPEED_MS or
        lead_stopped_again or unsafe_approach):
      self.latched = False
    elif launch_detected:
      # This may latch while stationary, but active remains false below 1 km/h.
      # It lets a driver-initiated launch receive boost without another delay.
      self.latched = True

    return bool(self.latched and not gas_pressed and not brake_pressed and
                STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS)


def apply_stop_accel_boost(requested_accel, v_ego, boost_active, accel_limits):
  """Apply 20% boost only in the measured 1-25 km/h launch window."""
  accel = float(requested_accel)
  if (boost_active and
      STOP_ACCEL_BOOST_MIN_SPEED_MS <= v_ego < STOP_ACCEL_BOOST_MAX_SPEED_MS and
      accel > 0.0):
    accel *= STOP_ACCEL_BOOST_FACTOR

  return min(float(accel_limits[1]), max(float(accel_limits[0]), accel))


def pedal_command_allowed(v_ego, normal_min_speed_kph=1.0):
  """Never send an automatic pedal command below 1 km/h."""
  return float(v_ego) >= float(normal_min_speed_kph) / 3.6
