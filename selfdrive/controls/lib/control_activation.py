def apply_control_activation(control, active, car_state, car_params, longitudinal_override):
  """Populate actuator activation flags on every CarControl message instance."""
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
