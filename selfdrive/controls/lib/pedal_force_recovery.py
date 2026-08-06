# Strict zero-accel watchdog for GM comma-pedal vehicles.
#
# This module intentionally favors false negatives over false throttle. Normal
# coasting can legitimately produce accel == 0, so recovery is allowed only
# after a previous sustained positive request, a sustained exact-zero request,
# measurable vehicle speed loss, growing speed error, and negative measured
# acceleration. The former positive-command watchdog is diagnostic-only and no
# longer changes actuator output.

PEDAL_FORCE_RECOVERY_ACCEL = 0.36
PEDAL_FORCE_RECOVERY_ACCEL_EPS = 1e-3
PEDAL_FORCE_RECOVERY_PEDAL_FLOOR = 0.060

# Strict external demand gates.
PEDAL_FORCE_RECOVERY_SPEED_ERROR = 0.55          # 1.98 km/h
PEDAL_FORCE_RECOVERY_FUTURE_SPEED_ERROR = 0.35   # 1.26 km/h
PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR = 0.05
PEDAL_FORCE_RECOVERY_PLAN_DECEL_MARGIN = 0.20
PEDAL_FORCE_RECOVERY_PLAN_LOOKAHEAD_POINTS = 10

# Strict temporal classifier. No throttle is injected during confirmation.
PEDAL_FORCE_RECOVERY_PRIOR_ACCEL = 0.08
PEDAL_FORCE_RECOVERY_PRIOR_SECONDS = 0.30
PEDAL_FORCE_RECOVERY_ZERO_CONFIRM_SECONDS = 0.80
PEDAL_FORCE_RECOVERY_ZERO_MAX_SECONDS = 1.20
PEDAL_FORCE_RECOVERY_NEGATIVE_ACCEL = -0.06
PEDAL_FORCE_RECOVERY_NEGATIVE_RATIO = 0.75
PEDAL_FORCE_RECOVERY_MIN_SPEED_LOSS = 0.20
PEDAL_FORCE_RECOVERY_MIN_ERROR_GROWTH = 0.12

# Bounded one-shot recovery and handoff.
PEDAL_FORCE_RECOVERY_MAX_FORCE_SECONDS = 0.60
PEDAL_FORCE_RECOVERY_RESPONSE_ACCEL = 0.02
PEDAL_FORCE_RECOVERY_RESPONSE_SECONDS = 0.15
PEDAL_FORCE_RECOVERY_HANDOFF_ACCEL = 0.08
PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS = 0.10
PEDAL_FORCE_RECOVERY_COOLDOWN_SECONDS = 5.0

# Conservative lead guard. Recovery is blocked with any relevant lead, not only
# an imminently dangerous one.
PEDAL_FORCE_RECOVERY_LEAD_MIN_CLEAR_DISTANCE = 80.0
PEDAL_FORCE_RECOVERY_LEAD_MIN_HEADWAY = 3.0
PEDAL_FORCE_RECOVERY_LEAD_CLOSING_SPEED = 0.10

# Compatibility constants retained for existing diagnostics/documentation. The
# positive watchdog no longer alters output.
PEDAL_POSITIVE_WATCHDOG_MIN_REQUEST = 0.01
PEDAL_POSITIVE_WATCHDOG_MIN_SPEED_ERROR = 0.30
PEDAL_POSITIVE_WATCHDOG_NEGATIVE_ACCEL = -0.05
PEDAL_POSITIVE_WATCHDOG_RESPONSE_ACCEL = 0.03
PEDAL_POSITIVE_WATCHDOG_TRIGGER_SECONDS = 0.50
PEDAL_POSITIVE_WATCHDOG_STAGE1_SECONDS = 0.60
PEDAL_POSITIVE_WATCHDOG_RESPONSE_SECONDS = 0.25
PEDAL_POSITIVE_WATCHDOG_MAX_STAGE2_SECONDS = 2.00
PEDAL_POSITIVE_WATCHDOG_COOLDOWN_SECONDS = 1.00
PEDAL_POSITIVE_WATCHDOG_STAGE1_ACCEL = 0.22
PEDAL_POSITIVE_WATCHDOG_STAGE1_PEDAL_FLOOR = 0.035

# Diagnostic block-reason bit mask. Kept local to selfdrive to avoid a cereal
# schema rebuild.
RECOVERY_BLOCK_PLAN_INVALID = 1 << 0
RECOVERY_BLOCK_CONTROLS_INACTIVE = 1 << 1
RECOVERY_BLOCK_ACC_INACTIVE = 1 << 2
RECOVERY_BLOCK_LONG_NOT_PID = 1 << 3
RECOVERY_BLOCK_BRAKE = 1 << 4
RECOVERY_BLOCK_DRIVER_GAS = 1 << 5
RECOVERY_BLOCK_STANDSTILL = 1 << 6
RECOVERY_BLOCK_LOW_SPEED = 1 << 7
RECOVERY_BLOCK_SOFT_DISABLE = 1 << 8
RECOVERY_BLOCK_FCW = 1 << 9
RECOVERY_BLOCK_NO_SPEED_DEMAND = 1 << 10
RECOVERY_BLOCK_PLAN_DECEL = 1 << 11
RECOVERY_BLOCK_CURVE_DECEL = 1 << 12
RECOVERY_BLOCK_LEAD_RISK = 1 << 13
RECOVERY_BLOCK_MANUAL_CATCHUP = 1 << 14
RECOVERY_BLOCK_JOYSTICK = 1 << 15
RECOVERY_BLOCK_BENCH_DISABLED = 1 << 16
RECOVERY_BLOCK_NO_INTERCEPTOR = 1 << 17
RECOVERY_BLOCK_DISABLED = 1 << 18

RECOVERY_BLOCK_NAMES = (
  (RECOVERY_BLOCK_PLAN_INVALID, "PLAN_INVALID"),
  (RECOVERY_BLOCK_CONTROLS_INACTIVE, "CONTROLS_INACTIVE"),
  (RECOVERY_BLOCK_ACC_INACTIVE, "ACC_INACTIVE"),
  (RECOVERY_BLOCK_LONG_NOT_PID, "LONG_NOT_PID"),
  (RECOVERY_BLOCK_BRAKE, "BRAKE"),
  (RECOVERY_BLOCK_DRIVER_GAS, "DRIVER_GAS"),
  (RECOVERY_BLOCK_STANDSTILL, "STANDSTILL"),
  (RECOVERY_BLOCK_LOW_SPEED, "LOW_SPEED"),
  (RECOVERY_BLOCK_SOFT_DISABLE, "SOFT_DISABLE"),
  (RECOVERY_BLOCK_FCW, "FCW"),
  (RECOVERY_BLOCK_NO_SPEED_DEMAND, "NO_SPEED_DEMAND"),
  (RECOVERY_BLOCK_PLAN_DECEL, "PLAN_DECEL"),
  (RECOVERY_BLOCK_CURVE_DECEL, "CURVE"),
  (RECOVERY_BLOCK_LEAD_RISK, "LEAD"),
  (RECOVERY_BLOCK_MANUAL_CATCHUP, "MANUAL_CATCHUP"),
  (RECOVERY_BLOCK_JOYSTICK, "JOYSTICK"),
  (RECOVERY_BLOCK_BENCH_DISABLED, "BENCH_DISABLED"),
  (RECOVERY_BLOCK_NO_INTERCEPTOR, "NO_INTERCEPTOR"),
  (RECOVERY_BLOCK_DISABLED, "DISABLED"),
)


def recovery_block_reason_text(reason):
  names = [name for bit, name in RECOVERY_BLOCK_NAMES if int(reason) & bit]
  return "|".join(names) if names else "NONE"


def recovery_speed_demand(speed_error, future_speed_error, injected_fault=False):
  normal_demand = float(speed_error) >= PEDAL_FORCE_RECOVERY_SPEED_ERROR and \
                  float(future_speed_error) >= PEDAL_FORCE_RECOVERY_FUTURE_SPEED_ERROR
  injected_demand = bool(injected_fault) and \
                    float(future_speed_error) >= PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR
  return normal_demand or injected_demand


def recovery_plan_decelerating(speeds, v_ego,
                               margin=PEDAL_FORCE_RECOVERY_PLAN_DECEL_MARGIN,
                               lookahead_points=PEDAL_FORCE_RECOVERY_PLAN_LOOKAHEAD_POINTS):
  """Return True when the near-term plan contains an intentional speed drop."""
  if speeds is None or len(speeds) < 2:
    return True
  end = min(len(speeds), max(2, int(lookahead_points) + 1))
  future_min = min(float(speeds[i]) for i in range(1, end))
  return future_min < float(v_ego) - float(margin)


def recovery_curve_decelerating(is_curv_driving, slow_on_curves,
                                 curve_speed_ms, v_ego,
                                 margin=PEDAL_FORCE_RECOVERY_PLAN_DECEL_MARGIN):
  """Strict mode blocks all detected curves, regardless of curve-speed policy."""
  del slow_on_curves, curve_speed_ms, v_ego, margin
  return bool(is_curv_driving)


def recovery_lead_risk(lead, v_ego):
  """Block recovery unless a detected lead is very far and not closing."""
  if lead is None or not bool(getattr(lead, "status", False)):
    return False

  d_rel = float(getattr(lead, "dRel", 0.0))
  v_rel = float(getattr(lead, "vRel", 0.0))
  clear_distance = max(PEDAL_FORCE_RECOVERY_LEAD_MIN_CLEAR_DISTANCE,
                       float(v_ego) * PEDAL_FORCE_RECOVERY_LEAD_MIN_HEADWAY)
  return d_rel <= clear_distance or v_rel < -PEDAL_FORCE_RECOVERY_LEAD_CLOSING_SPEED


def positive_pedal_ineffective_candidate(requested_accel, vehicle_accel, speed_error):
  """Diagnostic-only candidate; it never changes actuator output."""
  return float(requested_accel) >= PEDAL_POSITIVE_WATCHDOG_MIN_REQUEST and \
         float(speed_error) >= PEDAL_POSITIVE_WATCHDOG_MIN_SPEED_ERROR and \
         float(vehicle_accel) <= PEDAL_POSITIVE_WATCHDOG_NEGATIVE_ACCEL


def bench_fault_state(previous_mode, recovery_completed, requested_mode):
  mode = min(3, max(0, int(requested_mode)))
  completed = bool(recovery_completed) if mode == int(previous_mode) else False
  force_accel_zero = mode in (1, 2, 3) and not (mode == 2 and completed)
  recovery_enabled = mode != 1
  return mode, completed, force_accel_zero, recovery_enabled


def recovery_log_trigger(recovery_active, controls_active, adaptive_cruise,
                         brake_pressed, gas_pressed, standstill, raw_accel,
                         vehicle_accel=None, speed_error=None):
  del vehicle_accel, speed_error
  normal_driving = bool(controls_active) and bool(adaptive_cruise) and \
    not bool(brake_pressed) and not bool(gas_pressed) and not bool(standstill)
  zero_accel_while_driving = normal_driving and \
    abs(float(raw_accel)) <= PEDAL_FORCE_RECOVERY_ACCEL_EPS
  return bool(recovery_active) or zero_accel_while_driving


class PedalForceRecovery:
  """Strict, bounded, zero-only recovery classifier.

  Recovery requires all of the following:
    * a sustained real positive request immediately before the zero condition;
    * exact zero request for 0.8 seconds with no output modification;
    * at least 0.20 m/s measured speed loss;
    * speed error growing by at least 0.12 m/s;
    * negative measured acceleration on at least 75% of confirmation frames;
    * every external safety gate remaining true.

  The former positive-command watchdog is disabled. A confirmed event receives
  one bounded 0.36 m/s^2 / 6% pulse for at most 0.6 seconds, followed by a
  five-second cooldown. This design intentionally prefers missing a recovery to
  injecting throttle during legitimate coasting.
  """

  def __init__(self, dt=0.01):
    self.dt = float(dt)
    self.prior_frames_required = max(1, int(round(PEDAL_FORCE_RECOVERY_PRIOR_SECONDS / self.dt)))
    self.zero_confirm_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_ZERO_CONFIRM_SECONDS / self.dt)))
    self.zero_max_frames = max(self.zero_confirm_frames, int(round(PEDAL_FORCE_RECOVERY_ZERO_MAX_SECONDS / self.dt)))
    self.max_force_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_MAX_FORCE_SECONDS / self.dt)))
    self.response_frames_required = max(1, int(round(PEDAL_FORCE_RECOVERY_RESPONSE_SECONDS / self.dt)))
    self.handoff_frames_required = max(1, int(round(PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS / self.dt)))
    self.cooldown_frames_total = max(1, int(round(PEDAL_FORCE_RECOVERY_COOLDOWN_SECONDS / self.dt)))
    self.reset()

  @property
  def active(self):
    return self.zero_force_active

  @property
  def positive_candidate(self):
    return False

  @property
  def positive_watchdog_active(self):
    return False

  @property
  def watchdog_active(self):
    return self.zero_candidate_frames > 0 or self.zero_force_active

  @property
  def pedal_floor(self):
    return PEDAL_FORCE_RECOVERY_PEDAL_FLOOR if self.zero_force_active else 0.0

  @property
  def pedal_floor_active(self):
    return self.pedal_floor > 0.0

  @property
  def requested_floor_accel(self):
    return PEDAL_FORCE_RECOVERY_ACCEL if self.zero_force_active else 0.0

  @property
  def duration(self):
    return max(self.zero_candidate_frames, self.force_frames) * self.dt

  def _clear_candidate(self):
    self.zero_candidate_frames = 0
    self.zero_negative_frames = 0
    self.zero_start_v_ego = 0.0
    self.zero_start_speed_error = 0.0

  def _clear_active(self):
    self.zero_force_active = False
    self.force_frames = 0
    self.response_frames = 0
    self.handoff_frames = 0

  def _clear_all_events(self):
    self._clear_candidate()
    self._clear_active()
    self.hold_active = False

  def reset(self):
    self.zero_force_active = False
    self.hold_active = False
    self.prior_positive_frames = 0
    self.last_positive_accel = 0.0
    self.zero_candidate_frames = 0
    self.zero_negative_frames = 0
    self.zero_start_v_ego = 0.0
    self.zero_start_speed_error = 0.0
    self.force_frames = 0
    self.response_frames = 0
    self.handoff_frames = 0
    self.cooldown_frames = 0

    # Compatibility diagnostics expected by controlsd/UI.
    self.positive_stage = 0
    self.positive_candidate_frames = 0
    self.positive_stage_frames = 0
    self.positive_event_frames = 0
    self.positive_response_frames = 0
    self.positive_cooldown_frames = 0
    self.positive_activation_count = 0
    self.delivery_fault = False
    self.delivery_fault_count = 0
    self.inactive_frames = 0
    self.zero_event_frames = 0
    self.zero_active_frames = 0
    self.zero_frames = 0

    self.activation_count = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0
    self.vehicle_accel = 0.0
    self.speed_error = 0.0
    self.v_ego = 0.0

  def _cancel(self, keep_prior=False):
    self._clear_all_events()
    if not keep_prior:
      self.prior_positive_frames = 0
      self.last_positive_accel = 0.0

  def _update_prior_positive(self):
    if self.raw_accel >= PEDAL_FORCE_RECOVERY_PRIOR_ACCEL:
      self.prior_positive_frames = min(self.prior_frames_required, self.prior_positive_frames + 1)
      self.last_positive_accel = min(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
    else:
      self.prior_positive_frames = 0
      self.last_positive_accel = 0.0

  def _start_candidate(self):
    self.zero_candidate_frames = 1
    self.zero_negative_frames = 1 if self.vehicle_accel <= PEDAL_FORCE_RECOVERY_NEGATIVE_ACCEL else 0
    self.zero_start_v_ego = self.v_ego
    self.zero_start_speed_error = self.speed_error
    self.zero_event_frames = 1
    self.zero_frames = 1

  def _candidate_confirmed(self):
    if self.zero_candidate_frames < self.zero_confirm_frames:
      return False
    negative_ratio = self.zero_negative_frames / max(1, self.zero_candidate_frames)
    speed_loss = self.zero_start_v_ego - self.v_ego
    error_growth = self.speed_error - self.zero_start_speed_error
    return negative_ratio >= PEDAL_FORCE_RECOVERY_NEGATIVE_RATIO and \
           speed_loss >= PEDAL_FORCE_RECOVERY_MIN_SPEED_LOSS and \
           error_growth >= PEDAL_FORCE_RECOVERY_MIN_ERROR_GROWTH and \
           self.speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR

  def _update_candidate(self):
    if self.zero_candidate_frames == 0:
      if self.prior_positive_frames < self.prior_frames_required:
        self.forced_accel = self.raw_accel
        return self.forced_accel
      self._start_candidate()
    else:
      self.zero_candidate_frames += 1
      self.zero_event_frames = self.zero_candidate_frames
      self.zero_frames = self.zero_candidate_frames
      if self.vehicle_accel <= PEDAL_FORCE_RECOVERY_NEGATIVE_ACCEL:
        self.zero_negative_frames += 1

    # Confirmation is observation-only. Do not bridge or inject throttle here.
    self.forced_accel = self.raw_accel
    if not self._candidate_confirmed():
      # Do not let ordinary long coasting eventually accumulate into a fault.
      # A true abrupt stall must satisfy all evidence inside this short window.
      if self.zero_candidate_frames >= self.zero_max_frames:
        self._clear_candidate()
        self.prior_positive_frames = 0
        self.last_positive_accel = 0.0
      return self.forced_accel

    self._clear_candidate()
    self.zero_force_active = True
    self.force_frames = 0
    self.response_frames = 0
    self.handoff_frames = 0
    self.activation_count += 1
    self.prior_positive_frames = 0
    self.last_positive_accel = 0.0
    self.forced_accel = PEDAL_FORCE_RECOVERY_ACCEL
    return self.forced_accel

  def _update_active(self):
    self.force_frames += 1
    self.zero_active_frames = self.force_frames
    self.zero_event_frames = self.force_frames

    if self.vehicle_accel >= PEDAL_FORCE_RECOVERY_RESPONSE_ACCEL:
      self.response_frames += 1
    else:
      self.response_frames = 0

    if self.raw_accel >= PEDAL_FORCE_RECOVERY_HANDOFF_ACCEL:
      self.handoff_frames += 1
    else:
      self.handoff_frames = 0

    responded = self.response_frames >= self.response_frames_required
    pid_ready = self.handoff_frames >= self.handoff_frames_required
    timed_out = self.force_frames >= self.max_force_frames
    if responded or pid_ready or timed_out:
      self._clear_active()
      self.cooldown_frames = self.cooldown_frames_total
      self.forced_accel = self.raw_accel
      return self.forced_accel

    self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
    return self.forced_accel

  def update(self, eligible, requested_accel, vehicle_accel=0.0,
             speed_error=0.0, v_ego=0.0):
    self.raw_accel = float(requested_accel)
    self.vehicle_accel = float(vehicle_accel)
    self.speed_error = float(speed_error)
    self.v_ego = float(v_ego)
    eligible = bool(eligible)

    # Master/safety gate and intentional deceleration always win immediately.
    if not eligible or self.raw_accel < -PEDAL_FORCE_RECOVERY_ACCEL_EPS:
      self._cancel()
      self.cooldown_frames = max(0, self.cooldown_frames - 1)
      self.forced_accel = self.raw_accel
      return self.forced_accel

    if self.cooldown_frames > 0 and not self.zero_force_active:
      self.cooldown_frames -= 1
      self._clear_candidate()
      self._update_prior_positive()
      self.forced_accel = self.raw_accel
      return self.forced_accel

    if self.zero_force_active:
      return self._update_active()

    # Positive command: collect the prerequisite history and never auto-boost.
    if self.raw_accel > PEDAL_FORCE_RECOVERY_ACCEL_EPS:
      self._clear_candidate()
      self._update_prior_positive()
      self.forced_accel = self.raw_accel
      return self.forced_accel

    # Exact zero: only classify after a sustained prior positive request.
    return self._update_candidate()
