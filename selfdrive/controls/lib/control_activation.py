def apply_control_activation(control, active, car_state, car_params, longitudinal_override,
                             lat_active=None):
  """Populate actuator activation flags on every CarControl message instance.

  lat_active: the latActive this frame already worked out for an earlier
  CarControl (it depends only on active, the car state and the car params), so
  the second message of the frame does not re-read six car-state fields.
  """
  if lat_active is not None:
    control.latActive = bool(lat_active)
  else:
    control.latActive = bool(
      active and
      not car_state.steerFaultTemporary and
      not car_state.steerFaultPermanent and
      car_state.vEgo >= car_params.minSteerSpeed and
      not car_state.standstill and
      abs(car_state.steeringAngleDeg) < car_params.maxSteeringAngleDeg
    )
  control.longActive = bool(
    active and not longitudinal_override and car_params.openpilotLongitudinalControl
  )
  return control.latActive, control.longActive
