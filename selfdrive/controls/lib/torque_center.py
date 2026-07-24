#!/usr/bin/env python3
import math


CENTER_OFFSET_ABS_MAX = 0.10
CENTER_OFFSET_APPLY_RATE_PER_S = 0.010
CENTER_OFFSET_RELEASE_RATE_PER_S = 0.050

CENTER_I_UNWIND_ENABLED = True
CENTER_I_DESIRED_LAT_ACCEL_MAX = 0.08
CENTER_I_ACTUAL_LAT_ACCEL_MAX = 0.12
CENTER_I_ERROR_DEADBAND = 0.025
CENTER_I_UNWIND_TIME_CONSTANT_S = 5.0
CENTER_I_ZERO_THRESHOLD = 1e-4


def limit_center_offset(current, requested, dt):
  try:
    target = float(requested)
    target = max(-CENTER_OFFSET_ABS_MAX, min(target, CENTER_OFFSET_ABS_MAX)) if math.isfinite(target) else 0.0
  except Exception:
    target = 0.0

  try:
    current = float(current)
    if not math.isfinite(current):
      current = 0.0
  except Exception:
    current = 0.0

  releasing = (
    abs(target) < abs(current) or
    (abs(current) > 1e-9 and target * current <= 0.0)
  )
  rate = CENTER_OFFSET_RELEASE_RATE_PER_S if releasing else CENTER_OFFSET_APPLY_RATE_PER_S
  max_step = max(0.0, float(rate) * max(0.0, float(dt)))
  applied = max(current - max_step, min(target, current + max_step))
  if abs(applied) < 1e-9:
    applied = 0.0
  return target, applied, abs(applied - target) > 1e-9


def unwind_center_integral(integral, v_ego, desired_lateral_accel,
                           actual_lateral_accel, error, dt,
                           steering_pressed=False, steer_limited=False):
  integral = float(integral)
  near_straight = (
    CENTER_I_UNWIND_ENABLED and
    float(v_ego) >= 5.0 and
    abs(float(desired_lateral_accel)) <= CENTER_I_DESIRED_LAT_ACCEL_MAX and
    abs(float(actual_lateral_accel)) <= CENTER_I_ACTUAL_LAT_ACCEL_MAX and
    not bool(steering_pressed) and
    not bool(steer_limited)
  )
  if not near_straight:
    return integral, False

  error_value = float(error)
  stale_integral = (
    abs(error_value) <= CENTER_I_ERROR_DEADBAND or
    integral * error_value < 0.0
  )
  if not stale_integral or abs(integral) <= CENTER_I_ZERO_THRESHOLD:
    return integral, False

  decay = math.exp(-max(0.0, float(dt)) / max(CENTER_I_UNWIND_TIME_CONSTANT_S, float(dt), 1e-6))
  integral *= decay
  if abs(integral) <= CENTER_I_ZERO_THRESHOLD:
    integral = 0.0
  return integral, True
