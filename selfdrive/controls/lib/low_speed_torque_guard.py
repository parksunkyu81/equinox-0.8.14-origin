"""Low-speed torque output reversal protection for the Equinox.

The planner curvature and the closed-loop torque correction can change sign for
different reasons.  This guard therefore uses the last torque actually applied
by CarController as its primary reference.  A meaningful reversal must unwind
through neutral before authority is allowed to build in the opposite direction.
"""

import math

from common.numpy_fast import clip, interp


LS_REVERSAL_MIN_KPH = 10.0
LS_REVERSAL_RESET_KPH = 9.5
LS_REVERSAL_MAX_KPH = 30.0
LS_REVERSAL_SPEED_BP = [10.0, 20.0, 28.0, 30.0]
LS_REVERSAL_SPEED_WEIGHT = [1.0, 1.0, 0.3, 0.0]

LS_REVERSAL_APPLIED_MIN = 0.10
LS_REVERSAL_TARGET_MIN = 0.15
LS_REVERSAL_NEUTRAL = 0.05
LS_REVERSAL_CONFIRM_FRAMES = 12       # 120 ms at the 100 Hz controls rate
LS_REVERSAL_FAST_CONFIRM_FRAMES = 6   # sustained planner S-curve reversal
LS_REVERSAL_RAMP_FRAMES = 25          # 250 ms guarded opposite-direction rise
LS_REVERSAL_REPEAT_WINDOW_FRAMES = 50
LS_REVERSAL_REPEAT_SUPPRESS_FRAMES = 30

LS_REVERSAL_CURVATURE_MIN = 0.00045
LS_REVERSAL_FAST_CURVATURE = 0.0015
LS_REVERSAL_FAST_LAT_ACCEL = 0.15

LS_REVERSAL_UNWIND_BP = [10.0, 20.0, 28.0, 30.0]
LS_REVERSAL_UNWIND_V = [0.10, 0.12, 0.11, 0.10]
LS_REVERSAL_RISE_BP = [10.0, 20.0, 28.0, 30.0]
LS_REVERSAL_RISE_V = [0.025, 0.040, 0.065, 0.090]


class LowSpeedTorqueReversalGuard:
  FOLLOW = 0
  UNWIND_TO_ZERO = 1
  CONFIRM_NEW_DIRECTION = 2
  RAMP_OPPOSITE = 3
  DRIVER_BYPASS = 4

  def __init__(self):
    self.reset()

  @staticmethod
  def _finite(value, default=0.0):
    try:
      value = float(value)
      return value if math.isfinite(value) else float(default)
    except (TypeError, ValueError):
      return float(default)

  @staticmethod
  def _direction(value, threshold):
    if value >= threshold:
      return 1
    if value <= -threshold:
      return -1
    return 0

  @staticmethod
  def _move_towards(value, target, step):
    return float(clip(target, value - abs(step), value + abs(step)))

  def reset(self, applied_steer=0.0):
    applied = float(clip(self._finite(applied_steer), -1.0, 1.0))
    self.state = self.FOLLOW
    self.pending_direction = 0
    self.confirm_frames = 0
    self.confirm_required_frames = LS_REVERSAL_CONFIRM_FRAMES
    self.ramp_required_frames = LS_REVERSAL_RAMP_FRAMES
    self.ramp_frames = 0
    self.frames_since_reversal = LS_REVERSAL_REPEAT_WINDOW_FRAMES + 1
    self.rapid_reversal_count = 0
    self.suppress_frames = 0
    self.reversal_count = 0
    self.last_curvature_direction = 0
    self.last_raw_steer = 0.0
    self.last_guarded_steer = applied
    self.last_applied_steer = applied
    self.last_limited = False
    self.last_speed_weight = 0.0

  @property
  def active(self):
    return self.state in (self.UNWIND_TO_ZERO,
                          self.CONFIRM_NEW_DIRECTION,
                          self.RAMP_OPPOSITE)

  @property
  def boost_suppressed(self):
    return bool(self.active or self.suppress_frames > 0)

  def _note_reversal(self, new_direction, fast_confirmation, applied_steer):
    if self.frames_since_reversal <= LS_REVERSAL_REPEAT_WINDOW_FRAMES:
      self.rapid_reversal_count += 1
    else:
      self.rapid_reversal_count = 1
    if self.rapid_reversal_count >= 2:
      self.suppress_frames = max(self.suppress_frames,
                                 LS_REVERSAL_REPEAT_SUPPRESS_FRAMES)

    self.frames_since_reversal = 0
    self.reversal_count += 1
    self.state = self.UNWIND_TO_ZERO
    self.pending_direction = int(new_direction)
    self.confirm_frames = 1
    base_confirm = (LS_REVERSAL_FAST_CONFIRM_FRAMES if fast_confirmation
                    else LS_REVERSAL_CONFIRM_FRAMES)
    fade = 0.4 + (0.6 * self.last_speed_weight)
    self.confirm_required_frames = max(3, int(round(base_confirm * fade)))
    self.ramp_required_frames = max(8, int(round(LS_REVERSAL_RAMP_FRAMES * fade)))
    self.ramp_frames = 0
    self.last_guarded_steer = float(clip(applied_steer, -1.0, 1.0))

  def update(self, v_kph, requested_steer, applied_steer,
             desired_curvature=0.0, desired_lateral_accel=0.0,
             steering_pressed=False, active=True):
    speed = max(0.0, self._finite(v_kph))
    target = float(clip(self._finite(requested_steer), -1.0, 1.0))
    applied = float(clip(self._finite(applied_steer), -1.0, 1.0))
    curvature = self._finite(desired_curvature)
    lateral_accel = self._finite(desired_lateral_accel)
    self.last_raw_steer = target
    self.last_applied_steer = applied
    self.last_limited = False

    self.frames_since_reversal += 1
    if self.suppress_frames > 0:
      self.suppress_frames -= 1

    curvature_direction = self._direction(curvature, LS_REVERSAL_CURVATURE_MIN)
    curvature_reversal = bool(curvature_direction and self.last_curvature_direction and
                              curvature_direction != self.last_curvature_direction)
    if curvature_direction:
      self.last_curvature_direction = curvature_direction
    fast_confirmation = bool(
      curvature_reversal and
      (abs(curvature) >= LS_REVERSAL_FAST_CURVATURE or
       abs(lateral_accel) >= LS_REVERSAL_FAST_LAT_ACCEL)
    )

    if not active or speed < LS_REVERSAL_RESET_KPH:
      self.reset(applied)
      self.last_raw_steer = target
      self.last_guarded_steer = 0.0
      self.last_limited = bool(abs(target) > 1e-6)
      return 0.0

    if steering_pressed:
      self.reset(applied)
      self.state = self.DRIVER_BYPASS
      self.last_raw_steer = target
      self.last_guarded_steer = target
      return target

    if self.state == self.DRIVER_BYPASS:
      self.state = self.FOLLOW

    self.last_speed_weight = float(clip(interp(
      speed, LS_REVERSAL_SPEED_BP, LS_REVERSAL_SPEED_WEIGHT), 0.0, 1.0))
    if speed < LS_REVERSAL_MIN_KPH:
      # The GM EPS does not accept torque below 10 km/h.  Keep short-lived
      # state across the 9.5-10.0 km/h hysteresis band, but never emit torque.
      self.last_guarded_steer = 0.0
      self.last_limited = bool(abs(target) > 1e-6)
      return 0.0
    if speed >= LS_REVERSAL_MAX_KPH:
      self.state = self.FOLLOW
      self.pending_direction = 0
      self.confirm_frames = 0
      self.ramp_frames = 0
      self.last_guarded_steer = target
      return target

    # Strong reversals remain protected up to 30 km/h, while the trigger
    # threshold rises as protection fades above 20 km/h.
    target_threshold = float(interp(
      self.last_speed_weight, [0.0, 1.0], [0.55, LS_REVERSAL_TARGET_MIN]))
    applied_threshold = float(interp(
      self.last_speed_weight, [0.0, 1.0], [0.35, LS_REVERSAL_APPLIED_MIN]))
    target_direction = self._direction(target, target_threshold)
    applied_direction = self._direction(applied, applied_threshold)
    guarded_direction = self._direction(self.last_guarded_steer,
                                        LS_REVERSAL_APPLIED_MIN)
    reference_direction = applied_direction or guarded_direction

    if (self.state == self.FOLLOW and target_direction and reference_direction and
        target_direction != reference_direction):
      self._note_reversal(target_direction, fast_confirmation, applied)

    if self.state == self.UNWIND_TO_ZERO:
      if target_direction and target_direction != self.pending_direction:
        self._note_reversal(target_direction, fast_confirmation, applied)
      elif target_direction == self.pending_direction:
        self.confirm_frames += 1
      elif self.confirm_frames > 0:
        self.confirm_frames -= 1

      unwind_step = float(interp(speed, LS_REVERSAL_UNWIND_BP,
                                 LS_REVERSAL_UNWIND_V))
      self.last_guarded_steer = self._move_towards(
        self.last_guarded_steer, 0.0, unwind_step)
      if abs(applied) <= LS_REVERSAL_NEUTRAL and \
          abs(self.last_guarded_steer) <= LS_REVERSAL_NEUTRAL:
        self.last_guarded_steer = 0.0
        if self.confirm_frames >= self.confirm_required_frames:
          self.state = self.RAMP_OPPOSITE
          self.ramp_frames = 0
        else:
          self.state = self.CONFIRM_NEW_DIRECTION

    elif self.state == self.CONFIRM_NEW_DIRECTION:
      self.last_guarded_steer = 0.0
      if target_direction and target_direction != self.pending_direction:
        self._note_reversal(target_direction, fast_confirmation, applied)
      elif target_direction == self.pending_direction:
        self.confirm_frames += 1
      elif self.confirm_frames > 0:
        self.confirm_frames -= 1
      if self.confirm_frames >= self.confirm_required_frames:
        self.state = self.RAMP_OPPOSITE
        self.ramp_frames = 0

    elif self.state == self.RAMP_OPPOSITE:
      if target_direction and target_direction != self.pending_direction:
        self._note_reversal(target_direction, fast_confirmation, applied)
      else:
        increasing = bool(target_direction == self.pending_direction and
                          abs(target) > abs(self.last_guarded_steer))
        if increasing:
          step = float(interp(speed, LS_REVERSAL_RISE_BP,
                              LS_REVERSAL_RISE_V))
        else:
          step = float(interp(speed, LS_REVERSAL_UNWIND_BP,
                              LS_REVERSAL_UNWIND_V))
        self.last_guarded_steer = self._move_towards(
          self.last_guarded_steer, target, step)
        self.ramp_frames += 1
        if self.ramp_frames >= self.ramp_required_frames:
          self.state = self.FOLLOW
          self.pending_direction = 0
          self.confirm_frames = 0

    else:
      self.last_guarded_steer = target

    self.last_limited = bool(abs(self.last_guarded_steer - target) > 1e-6)
    return float(self.last_guarded_steer)

  def diagnostics(self):
    return {
      'active': bool(self.active),
      'state': int(self.state),
      'rawSteer': float(self.last_raw_steer),
      'guardedSteer': float(self.last_guarded_steer),
      'appliedSteer': float(self.last_applied_steer),
      'confirmMs': int(self.confirm_frames * 10),
      'reversalCount': int(self.reversal_count),
      'boostSuppressed': bool(self.boost_suppressed),
      'limited': bool(self.last_limited),
      'speedWeight': float(self.last_speed_weight),
    }
