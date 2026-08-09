LEAD_DROPOUT_CONFIRM_SECONDS = 0.20
LEAD_DROPOUT_RECOVERY_SECONDS = 0.80
LEAD_DROPOUT_MIN_SPEED_MS = 5.0 / 3.6
LEAD_DROPOUT_SPEED_DEMAND_MS = 0.50
LEAD_DROPOUT_ACCEL_FLOOR = 0.36


class LeadDropoutRecovery:
  """Detect a confirmed loss of a lead that was constraining the MPC.

  Recovery is deliberately one-shot. A lead must be seen again before another
  dropout can arm, preventing a persistent perception outage from repeatedly
  requesting acceleration.
  """

  def __init__(self, dt):
    self.dt = float(dt)
    self.confirm_frames = max(1, int(round(LEAD_DROPOUT_CONFIRM_SECONDS / self.dt)))
    self.recovery_frames = max(1, int(round(LEAD_DROPOUT_RECOVERY_SECONDS / self.dt)))
    self.previous_lead_present = False
    self.armed = False
    self.active = False
    self.triggered = False
    self.absent_frames = 0
    self.active_frames = 0

  @property
  def absent_time(self):
    return self.absent_frames * self.dt

  def reset(self, lead_present=False):
    self.previous_lead_present = bool(lead_present)
    self.armed = False
    self.active = False
    self.triggered = False
    self.absent_frames = 0
    self.active_frames = 0

  def update(self, enabled, lead_present, lead_was_constraining,
             v_ego, v_cruise, brake_pressed, gas_pressed, standstill,
             force_decel):
    lead_present = bool(lead_present)
    self.triggered = False

    recovery_allowed = bool(enabled) and not bool(brake_pressed) and \
      not bool(gas_pressed) and not bool(standstill) and not bool(force_decel) and \
      float(v_ego) >= LEAD_DROPOUT_MIN_SPEED_MS and \
      float(v_cruise) - float(v_ego) >= LEAD_DROPOUT_SPEED_DEMAND_MS

    falling_edge = self.previous_lead_present and not lead_present

    if lead_present:
      # Seeing a lead again cancels recovery immediately and rearms the next
      # genuine lead-loss transition.
      self.armed = False
      self.active = False
      self.absent_frames = 0
      self.active_frames = 0
    elif not recovery_allowed:
      self.armed = False
      self.active = False
      self.absent_frames = 0
      self.active_frames = 0
    elif falling_edge and bool(lead_was_constraining):
      self.armed = True
      self.absent_frames = 1
    elif self.armed and not self.active:
      self.absent_frames += 1

    if self.armed and not self.active and self.absent_frames >= self.confirm_frames:
      self.active = True
      self.triggered = True
      self.active_frames = 0

    if self.active:
      self.active_frames += 1
      if self.active_frames > self.recovery_frames:
        self.active = False
        self.armed = False

    self.previous_lead_present = lead_present
    return self.active


def apply_lead_dropout_accel(requested_accel, recovery_allowed):
  if recovery_allowed:
    return max(float(requested_accel), LEAD_DROPOUT_ACCEL_FLOOR)
  return float(requested_accel)
