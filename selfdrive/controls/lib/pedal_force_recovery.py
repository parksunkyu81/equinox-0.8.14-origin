# The historical deadzone calibration in this fork used 0.060 as the first
# pedal command that reliably produces a vehicle response. 0.36 m/s^2 maps to
# at least that command across the current GM speed-dependent multiplier table.
PEDAL_FORCE_RECOVERY_ACCEL = 0.36
PEDAL_FORCE_RECOVERY_ACCEL_EPS = 1e-3
PEDAL_FORCE_RECOVERY_PEDAL_FLOOR = 0.060
PEDAL_FORCE_RECOVERY_SPEED_ERROR = 0.30
PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR = 0.05
PEDAL_FORCE_RECOVERY_REARM_SECONDS = 0.5


def recovery_speed_demand(speed_error, future_speed_error, injected_fault=False):
  normal_demand = speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and \
                  future_speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR
  injected_demand = bool(injected_fault) and \
                    future_speed_error >= PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR
  return normal_demand or injected_demand


def bench_fault_state(previous_mode, recovery_completed, requested_mode):
  """Return normalized bench fault state for off, held, one-shot, or production modes.

  Mode 1 holds accel at zero with recovery blocked. Mode 2 permits exactly one
  simulator-assisted recovery activation. Mode 3 holds accel at zero while
  retaining the unmodified production recovery gates.
  """
  mode = min(3, max(0, int(requested_mode)))
  completed = bool(recovery_completed) if mode == int(previous_mode) else False
  force_accel_zero = mode in (1, 2, 3) and not (mode == 2 and completed)
  recovery_enabled = mode != 1
  return mode, completed, force_accel_zero, recovery_enabled


def recovery_log_trigger(recovery_active, controls_active, adaptive_cruise,
                         brake_pressed, gas_pressed, standstill, raw_accel):
  """Capture every zero-accel occurrence while ACC is actively driving.

  Logging deliberately does not require a valid/fresh plan or a minimum speed
  error. Those values are evidence used after capture; making them trigger
  gates hid the very events needed to diagnose why recovery did not run.
  """
  zero_accel_while_driving = bool(controls_active) and bool(adaptive_cruise) and \
    not bool(brake_pressed) and not bool(gas_pressed) and not bool(standstill) and \
    abs(float(raw_accel)) <= PEDAL_FORCE_RECOVERY_ACCEL_EPS
  return bool(recovery_active) or zero_accel_while_driving


class PedalForceRecovery:
  """Immediately replace an abnormal zero request with a positive request.

  The caller owns the definition of normal-driving demand. Once eligible, this
  class deliberately does not classify PID/P/I/F causes or wait for a timer.
  """

  def __init__(self, dt=0.01):
    self.dt = float(dt)
    self.rearm_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_REARM_SECONDS / self.dt)))
    self.active = False
    self.active_frames = 0
    self.inactive_frames = self.rearm_frames
    self.activation_count = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0

  @property
  def duration(self):
    return self.active_frames * self.dt

  def reset(self):
    self.active = False
    self.active_frames = 0
    self.inactive_frames = self.rearm_frames
    self.raw_accel = 0.0
    self.forced_accel = 0.0

  def update(self, eligible, requested_accel):
    self.raw_accel = float(requested_accel)
    force_now = bool(eligible) and self.raw_accel <= PEDAL_FORCE_RECOVERY_ACCEL_EPS

    if force_now:
      if not self.active:
        # Safety/eligibility gates still cancel the forced output immediately.
        # Only count a new event after a meaningful inactive interval so a
        # one-frame plan/state timing gap cannot inflate activation_count.
        if self.inactive_frames >= self.rearm_frames:
          self.activation_count += 1
        self.active_frames = 0
      self.active = True
      self.inactive_frames = 0
      self.active_frames += 1
      self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
    else:
      self.active = False
      self.active_frames = 0
      self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
      self.forced_accel = self.raw_accel

    return self.forced_accel
