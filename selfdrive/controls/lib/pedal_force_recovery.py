# The historical deadzone calibration in this fork used 0.060 as the first
# pedal command that reliably produces a vehicle response. 0.36 m/s^2 maps to
# at least that command across the current GM speed-dependent multiplier table.
PEDAL_FORCE_RECOVERY_ACCEL = 0.36
PEDAL_FORCE_RECOVERY_ACCEL_EPS = 1e-3
PEDAL_FORCE_RECOVERY_PEDAL_FLOOR = 0.060
PEDAL_FORCE_RECOVERY_SPEED_ERROR = 0.30
PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR = 0.05
PEDAL_FORCE_RECOVERY_REARM_SECONDS = 0.5
PEDAL_FORCE_RECOVERY_MIN_HOLD_SECONDS = 0.30
PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS = 0.10

# A separate, conservative mode for the low-demand lead-follow stalls seen in
# real Equinox logs. Unlike hard-zero recovery it waits for a stable, receding
# lead and a comfortable time-gap margin before supplying the effective pedal
# floor. All values are deliberately below the hard recovery authority.
LEAD_COAST_ASSIST_ACCEL_EPS = 0.05
LEAD_COAST_ASSIST_SPEED_ERROR = 0.05
LEAD_COAST_ASSIST_FUTURE_ERROR = 0.10
LEAD_COAST_ASSIST_DECEL = -0.25
LEAD_COAST_ASSIST_ENTER_VREL = 0.20
LEAD_COAST_ASSIST_EXIT_VREL = -0.10
LEAD_COAST_ASSIST_ENTER_TR_MARGIN = 0.25
LEAD_COAST_ASSIST_EXIT_TR_MARGIN = 0.05
LEAD_COAST_ASSIST_DRIVER_CLEAR_SECONDS = 0.50
LEAD_COAST_ASSIST_LEAD_STABLE_SECONDS = 0.20
LEAD_COAST_ASSIST_CANDIDATE_SECONDS = 0.15
LEAD_COAST_ASSIST_RAMP_SECONDS = 0.12
LEAD_COAST_ASSIST_MAX_SECONDS = 0.80
LEAD_COAST_ASSIST_COOLDOWN_SECONDS = 0.50
LEAD_COAST_ASSIST_ACCEL_PER_PEDAL = 1.0 / 0.17

RECOVERY_MODE_NONE = 0
RECOVERY_MODE_HARD_ZERO = 1
RECOVERY_MODE_LEAD_COAST_ASSIST = 2

LEAD_ASSIST_CANCEL_NONE = 0
LEAD_ASSIST_CANCEL_SAFETY = 1
LEAD_ASSIST_CANCEL_LEAD = 2
LEAD_ASSIST_CANCEL_PID_HANDOFF = 3
LEAD_ASSIST_CANCEL_TIMEOUT = 4


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
                         brake_pressed, gas_pressed, standstill, plan_valid,
                         plan_age_ms, speed_error, future_speed_error, raw_accel):
  production_candidate = bool(controls_active) and bool(adaptive_cruise) and \
    not bool(brake_pressed) and not bool(gas_pressed) and not bool(standstill) and \
    bool(plan_valid) and 0.0 <= float(plan_age_ms) <= 250.0 and \
    float(speed_error) >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and \
    float(future_speed_error) >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and \
    float(raw_accel) <= PEDAL_FORCE_RECOVERY_ACCEL_EPS
  return bool(recovery_active) or production_candidate


class PedalForceRecovery:
  """Immediately replace an abnormal zero request with a positive request.

  The caller owns the definition of normal-driving demand. Once eligible, this
  class deliberately does not classify PID/P/I/F causes or wait for a timer.
  """

  def __init__(self, dt=0.01):
    self.dt = float(dt)
    self.rearm_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_REARM_SECONDS / self.dt)))
    self.min_hold_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_MIN_HOLD_SECONDS / self.dt)))
    self.handoff_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS / self.dt)))
    self.active = False
    self.active_frames = 0
    self.handoff_stable_frames = 0
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
    self.handoff_stable_frames = 0
    self.inactive_frames = self.rearm_frames
    self.raw_accel = 0.0
    self.forced_accel = 0.0

  def update(self, eligible, requested_accel):
    self.raw_accel = float(requested_accel)
    eligible = bool(eligible)

    # Driver/safety gates cancel a held recovery immediately. Never preserve a
    # positive pedal floor across brake, gas override, FCW, curve, stale plan,
    # disengagement, or another ineligible state selected by the caller.
    if not eligible:
      self.active = False
      self.active_frames = 0
      self.handoff_stable_frames = 0
      self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
      self.forced_accel = self.raw_accel
      return self.forced_accel

    if not self.active and self.raw_accel <= PEDAL_FORCE_RECOVERY_ACCEL_EPS:
      if self.inactive_frames >= self.rearm_frames:
        # Safety/eligibility gates still cancel the forced output immediately.
        # Only count a new event after a meaningful inactive interval so a
        # one-frame plan/state timing gap cannot inflate activation_count.
        self.activation_count += 1
      self.active = True
      self.inactive_frames = 0
      self.active_frames = 0
      self.handoff_stable_frames = 0

    if not self.active:
      self.active_frames = 0
      self.handoff_stable_frames = 0
      self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
      self.forced_accel = self.raw_accel
      return self.forced_accel

    self.active_frames += 1
    if self.raw_accel >= PEDAL_FORCE_RECOVERY_ACCEL:
      self.handoff_stable_frames += 1
    else:
      self.handoff_stable_frames = 0

    # Hold the known-effective floor long enough for the comma pedal and the
    # diesel powertrain to respond. Hand control back only after the normal PID
    # has supplied an equally strong request continuously for 100 ms.
    handoff_ready = self.active_frames > self.min_hold_frames and \
                    self.handoff_stable_frames >= self.handoff_frames
    if handoff_ready:
      self.active = False
      self.active_frames = 0
      self.handoff_stable_frames = 0
      self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
      self.forced_accel = self.raw_accel
    else:
      self.inactive_frames = 0
      self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)

    return self.forced_accel


class LeadCoastAssist:
  """Fill a lead-follow pedal deadzone only while the lead is safely receding."""

  def __init__(self, dt=0.01):
    self.dt = float(dt)
    self.driver_clear_frames_required = max(1, int(round(LEAD_COAST_ASSIST_DRIVER_CLEAR_SECONDS / self.dt)))
    self.lead_stable_frames_required = max(1, int(round(LEAD_COAST_ASSIST_LEAD_STABLE_SECONDS / self.dt)))
    self.candidate_frames_required = max(1, int(round(LEAD_COAST_ASSIST_CANDIDATE_SECONDS / self.dt)))
    self.ramp_frames = max(1, int(round(LEAD_COAST_ASSIST_RAMP_SECONDS / self.dt)))
    self.max_frames = max(1, int(round(LEAD_COAST_ASSIST_MAX_SECONDS / self.dt)))
    self.cooldown_frames_required = max(1, int(round(LEAD_COAST_ASSIST_COOLDOWN_SECONDS / self.dt)))
    self.reset()

  @property
  def duration(self):
    return self.active_frames * self.dt

  @property
  def candidate_duration(self):
    return self.candidate_frames * self.dt

  def reset(self):
    self.active = False
    self.active_frames = 0
    self.driver_clear_frames = 0
    self.lead_stable_frames = 0
    self.candidate_frames = 0
    self.cooldown_frames = self.cooldown_frames_required
    self.handoff_stable_frames = 0
    self.activation_count = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0
    self.pedal_target = 0.0
    self.filtered_v_rel = 0.0
    self.actual_tr = 0.0
    self.desired_tr = 0.0
    self.tr_margin = 0.0
    self.cancel_reason = LEAD_ASSIST_CANCEL_NONE

  def _deactivate(self, reason):
    self.active = False
    self.active_frames = 0
    self.candidate_frames = 0
    self.handoff_stable_frames = 0
    self.cooldown_frames = 0
    self.pedal_target = 0.0
    self.cancel_reason = int(reason)

  def update(self, base_safe, lead_valid, lead_v_rel, lead_distance, v_ego,
             desired_tr, speed_error, future_speed_error, a_ego,
             requested_accel, lead_measurement_updated=True):
    self.raw_accel = float(requested_accel)
    self.desired_tr = max(0.0, float(desired_tr))
    self.actual_tr = max(0.0, float(lead_distance)) / max(float(v_ego), 1.0)
    self.tr_margin = self.actual_tr - self.desired_tr

    if lead_measurement_updated:
      measured_v_rel = float(lead_v_rel)
      alpha = 0.35
      self.filtered_v_rel = measured_v_rel if self.lead_stable_frames == 0 else \
                            (1.0 - alpha) * self.filtered_v_rel + alpha * measured_v_rel

    if not base_safe:
      self.driver_clear_frames = 0
      self.lead_stable_frames = 0
      if self.active:
        self._deactivate(LEAD_ASSIST_CANCEL_SAFETY)
      else:
        self.candidate_frames = 0
        self.cooldown_frames = min(self.cooldown_frames_required, self.cooldown_frames + 1)
        self.pedal_target = 0.0
      self.forced_accel = self.raw_accel
      return self.forced_accel

    self.driver_clear_frames = min(self.driver_clear_frames_required, self.driver_clear_frames + 1)
    if lead_valid:
      self.lead_stable_frames = min(self.lead_stable_frames_required, self.lead_stable_frames + 1)
    else:
      self.lead_stable_frames = 0

    lead_enter_safe = bool(
      lead_valid and self.lead_stable_frames >= self.lead_stable_frames_required and
      self.filtered_v_rel >= LEAD_COAST_ASSIST_ENTER_VREL and
      self.tr_margin >= LEAD_COAST_ASSIST_ENTER_TR_MARGIN)
    lead_hold_safe = bool(
      lead_valid and self.filtered_v_rel > LEAD_COAST_ASSIST_EXIT_VREL and
      self.tr_margin > LEAD_COAST_ASSIST_EXIT_TR_MARGIN)
    low_demand_stall = bool(
      float(speed_error) >= LEAD_COAST_ASSIST_SPEED_ERROR and
      float(future_speed_error) >= LEAD_COAST_ASSIST_FUTURE_ERROR and
      float(a_ego) <= LEAD_COAST_ASSIST_DECEL and
      self.raw_accel <= LEAD_COAST_ASSIST_ACCEL_EPS)

    if self.active:
      if not lead_hold_safe:
        self._deactivate(LEAD_ASSIST_CANCEL_LEAD)
      elif self.raw_accel > LEAD_COAST_ASSIST_ACCEL_EPS:
        self.handoff_stable_frames += 1
        if self.handoff_stable_frames >= max(1, int(round(PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS / self.dt))):
          self._deactivate(LEAD_ASSIST_CANCEL_PID_HANDOFF)
      else:
        self.handoff_stable_frames = 0

      if self.active and self.active_frames >= self.max_frames:
        self._deactivate(LEAD_ASSIST_CANCEL_TIMEOUT)

      if self.active:
        self.active_frames += 1
        ramp_ratio = min(1.0, self.active_frames / float(self.ramp_frames))
        self.pedal_target = PEDAL_FORCE_RECOVERY_PEDAL_FLOOR * ramp_ratio
        assist_accel = self.pedal_target * LEAD_COAST_ASSIST_ACCEL_PER_PEDAL
        self.forced_accel = max(self.raw_accel, assist_accel)
        return self.forced_accel

      self.forced_accel = self.raw_accel
      return self.forced_accel

    self.cooldown_frames = min(self.cooldown_frames_required, self.cooldown_frames + 1)
    candidate = bool(
      self.driver_clear_frames >= self.driver_clear_frames_required and
      self.cooldown_frames >= self.cooldown_frames_required and
      lead_enter_safe and low_demand_stall)
    self.candidate_frames = self.candidate_frames + 1 if candidate else 0

    if self.candidate_frames >= self.candidate_frames_required:
      self.active = True
      self.active_frames = 1
      self.candidate_frames = 0
      self.handoff_stable_frames = 0
      self.activation_count += 1
      self.cancel_reason = LEAD_ASSIST_CANCEL_NONE
      self.pedal_target = PEDAL_FORCE_RECOVERY_PEDAL_FLOOR / float(self.ramp_frames)
      self.forced_accel = max(self.raw_accel,
                              self.pedal_target * LEAD_COAST_ASSIST_ACCEL_PER_PEDAL)
    else:
      self.pedal_target = 0.0
      self.forced_accel = self.raw_accel

    return self.forced_accel
