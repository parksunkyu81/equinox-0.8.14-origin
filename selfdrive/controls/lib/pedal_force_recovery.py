# The historical deadzone calibration in this fork used 0.060 as the first
# pedal command that reliably produces a vehicle response. 0.36 m/s^2 maps to
# at least that command across the current GM speed-dependent multiplier table.
PEDAL_FORCE_RECOVERY_ACCEL = 0.36
PEDAL_FORCE_RECOVERY_ACCEL_EPS = 1e-3
PEDAL_FORCE_RECOVERY_PEDAL_FLOOR = 0.060


class PedalForceRecovery:
  """Immediately replace an abnormal zero request with a positive request.

  The caller owns the definition of normal-driving demand. Once eligible, this
  class deliberately does not classify PID/P/I/F causes or wait for a timer.
  """

  def __init__(self, dt=0.01):
    self.dt = float(dt)
    self.active = False
    self.active_frames = 0
    self.activation_count = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0

  @property
  def duration(self):
    return self.active_frames * self.dt

  def reset(self):
    self.active = False
    self.active_frames = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0

  def update(self, eligible, requested_accel):
    self.raw_accel = float(requested_accel)
    force_now = bool(eligible) and self.raw_accel <= PEDAL_FORCE_RECOVERY_ACCEL_EPS

    if force_now:
      if not self.active:
        self.activation_count += 1
        self.active_frames = 0
      self.active = True
      self.active_frames += 1
      self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
    else:
      self.active = False
      self.active_frames = 0
      self.forced_accel = self.raw_accel

    return self.forced_accel
