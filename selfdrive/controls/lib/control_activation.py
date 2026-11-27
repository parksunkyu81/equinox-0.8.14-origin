from common.conversions import Conversions as CV

# Lateral control turns on only once the car is this far above minSteerSpeed,
# and turns off only when it drops below minSteerSpeed itself.
#
# Without the gap, a car hovering at exactly 10.0 / 9.9 km/h switched LKAS on
# and off within a fraction of a second. Every Equinox EPS fault in a week of
# drives (2026-11-18..27) sat on that edge, with latActive on for 0.19-1.0 s;
# on 2026-11-27 one became a permanent EPS fault (DTC U056B, "invalid data
# from the front view camera") that only an ignition cycle cleared.
STEER_SPEED_ON_MARGIN_MS = 2.0 * CV.KPH_TO_MS


class SteerSpeedGate:
  """minSteerSpeed with hysteresis: on at minSteerSpeed + margin, off below it."""
  def __init__(self, min_steer_speed, on_margin=STEER_SPEED_ON_MARGIN_MS):
    self.off_speed = float(min_steer_speed)
    self.on_speed = self.off_speed + float(on_margin)
    self.ok = False

  def update(self, v_ego):
    self.ok = v_ego >= (self.off_speed if self.ok else self.on_speed)
    return self.ok


def apply_control_activation(control, active, car_state, car_params, longitudinal_override,
                             lat_active=None, steer_speed_ok=None):
  """Populate actuator activation flags on every CarControl message instance.

  lat_active: the latActive this frame already worked out for an earlier
  CarControl (it depends only on active, the car state and the car params), so
  the second message of the frame does not re-read six car-state fields.
  steer_speed_ok: the SteerSpeedGate result for this frame; without it the
  plain vEgo >= minSteerSpeed check is used.
  """
  if lat_active is not None:
    control.latActive = bool(lat_active)
  else:
    if steer_speed_ok is None:
      steer_speed_ok = car_state.vEgo >= car_params.minSteerSpeed
    control.latActive = bool(
      active and
      not car_state.steerFaultTemporary and
      not car_state.steerFaultPermanent and
      steer_speed_ok and
      not car_state.standstill and
      abs(car_state.steeringAngleDeg) < car_params.maxSteeringAngleDeg
    )
  control.longActive = bool(
    active and not longitudinal_override and car_params.openpilotLongitudinalControl
  )
  return control.latActive, control.longActive
