import math


GM_PEDAL_COMMAND_MAX = 0.85


def gm_pedal_control_allowed(controls_active, pedal_mode_enabled, brake_pressed,
                             gas_pressed, speed_allowed):
  return bool(
    controls_active and pedal_mode_enabled and not brake_pressed and
    not gas_pressed and speed_allowed
  )


def limit_gm_pedal_accel(requested_accel, comfort_accel_cap, blocked):
  """Apply the positive planner cap before converting acceleration to pedal."""
  requested_accel = float(requested_accel)
  comfort_accel_cap = float(comfort_accel_cap)
  if not math.isfinite(requested_accel) or not math.isfinite(comfort_accel_cap):
    return 0.0

  comfort_accel_cap = max(comfort_accel_cap, 0.0)
  limited_accel = min(requested_accel, comfort_accel_cap) if requested_accel > 0.0 else requested_accel
  return min(limited_accel, 0.0) if blocked else limited_accel


def calculate_gm_pedal_command(accel, accel_multiplier,
                               command_max=GM_PEDAL_COMMAND_MAX):
  """Convert an already limited acceleration request to a pedal command."""
  accel = float(accel)
  accel_multiplier = float(accel_multiplier)
  if not math.isfinite(accel) or not math.isfinite(accel_multiplier):
    return 0.0
  return min(max(max(accel, 0.0) * accel_multiplier, 0.0), float(command_max))
