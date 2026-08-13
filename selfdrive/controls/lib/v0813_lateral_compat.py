"""Lateral-control compatibility helpers for the official v0.8.13 model.

The live torque learner identifies the physical car from applied steering and
measured motion, so its learned parameters remain model independent. These
helpers normalize only the model-derived steering demand before it reaches the
controller and transient torque-authority scheduler.
"""

import math

from common.numpy_fast import clip, interp


V0813_EXTRA_STEER_DELAY_S = 0.20
V0813_MAX_STEER_DELAY_S = 1.00

V0813_CURV_GUARD_ON_KPH = 55.0
V0813_CURV_DELTA_BP = [55.0, 70.0, 90.0, 110.0, 130.0]
V0813_CURV_DELTA_V = [0.00062, 0.00052, 0.00038, 0.00024, 0.00016]
V0813_CURV_RATE_BP = [55.0, 70.0, 90.0, 110.0, 130.0]
V0813_CURV_RATE_V = [0.020, 0.017, 0.012, 0.008, 0.005]
# A real time constant is used instead of a per-control-frame blend. The model
# publishes at 20 Hz while controls runs at 100 Hz; this keeps filtering
# consistent across the five repeated control frames for each model plan.
V0813_CURV_RC_BP = [55.0, 70.0, 90.0, 110.0, 130.0]
V0813_CURV_RC_V = [0.025, 0.045, 0.070, 0.100, 0.130]
V0813_CONTROL_DT_S = 0.01
V0813_SIGN_FLIP_MIN_CURVATURE = 0.00045
V0813_LIMIT_DELTA_SHRINK = 0.55
V0813_LIMIT_ALPHA_SHRINK = 0.75


def compensated_steer_delay(steer_actuator_delay):
  """Return the v0.8.13 official plan lookahead, including other delays."""
  try:
    base_delay = float(steer_actuator_delay)
  except (TypeError, ValueError):
    base_delay = 0.0
  if not math.isfinite(base_delay):
    base_delay = 0.0
  return float(clip(base_delay + V0813_EXTRA_STEER_DELAY_S,
                    0.01, V0813_MAX_STEER_DELAY_S))


class V0813CurvatureGuard:
  """Smooth v0.8.13 high-speed curvature without weakening steady corners."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.previous_curvature = 0.0
    self.previous_rate = 0.0
    self.last_raw_curvature = 0.0
    self.last_filtered_curvature = 0.0
    self.last_alpha = 1.0
    self.last_direction_reversal = False
    self.last_active = False

  @staticmethod
  def _finite(value):
    try:
      value = float(value)
      return value if math.isfinite(value) else 0.0
    except (TypeError, ValueError):
      return 0.0

  def update(self, v_kph, desired_curvature, desired_curvature_rate,
             limited_hold=False):
    speed = max(0.0, self._finite(v_kph))
    raw_curvature = self._finite(desired_curvature)
    raw_rate = self._finite(desired_curvature_rate)
    self.last_raw_curvature = raw_curvature

    if speed < V0813_CURV_GUARD_ON_KPH:
      self.previous_curvature = raw_curvature
      self.previous_rate = raw_rate
      self.last_filtered_curvature = raw_curvature
      self.last_alpha = 1.0
      self.last_direction_reversal = False
      self.last_active = False
      return raw_curvature, raw_rate, False

    previous = self.previous_curvature
    reversal = bool(
      abs(previous) >= V0813_SIGN_FLIP_MIN_CURVATURE and
      abs(raw_curvature) >= V0813_SIGN_FLIP_MIN_CURVATURE and
      previous * raw_curvature < 0.0)
    target = 0.0 if reversal else raw_curvature
    target_rate = 0.0 if reversal else raw_rate

    delta_max = float(interp(speed, V0813_CURV_DELTA_BP, V0813_CURV_DELTA_V))
    rate_max = float(interp(speed, V0813_CURV_RATE_BP, V0813_CURV_RATE_V))
    rc = float(interp(speed, V0813_CURV_RC_BP, V0813_CURV_RC_V))
    alpha = V0813_CONTROL_DT_S / max(V0813_CONTROL_DT_S + rc, V0813_CONTROL_DT_S)
    if limited_hold:
      delta_max *= V0813_LIMIT_DELTA_SHRINK
      rate_max *= V0813_LIMIT_DELTA_SHRINK
      alpha *= V0813_LIMIT_ALPHA_SHRINK

    # Apply the time-domain filter first, then cap the resulting output step.
    # Clipping the input before filtering would multiply the two limits and
    # delay a legitimate highway curve by more than half a second.
    filtered_candidate = float(previous + alpha * (target - previous))
    filtered_curvature = float(clip(
      filtered_candidate, previous - delta_max, previous + delta_max))
    filtered_rate = float(clip(target_rate, -rate_max, rate_max))

    self.previous_curvature = filtered_curvature
    self.previous_rate = filtered_rate
    self.last_filtered_curvature = filtered_curvature
    self.last_alpha = float(alpha)
    self.last_direction_reversal = reversal
    self.last_active = True
    return filtered_curvature, filtered_rate, True

  def diagnostics(self):
    return {
      "modelCurvatureGuardActive": bool(self.last_active),
      "modelCurvatureRaw": float(self.last_raw_curvature),
      "modelCurvatureFiltered": float(self.last_filtered_curvature),
      "modelCurvatureFilterAlpha": float(self.last_alpha),
      "modelCurvatureDirectionReversal": bool(self.last_direction_reversal),
      "modelSteerDelayCompensation": float(V0813_EXTRA_STEER_DELAY_S),
    }
