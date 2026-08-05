# The historical deadzone calibration in this fork used 0.060 as the first
# pedal command that reliably produces a vehicle response. 0.36 m/s^2 maps to
# at least that command across the current GM speed-dependent multiplier table.
PEDAL_FORCE_RECOVERY_ACCEL = 0.36
PEDAL_FORCE_RECOVERY_ACCEL_EPS = 1e-3
PEDAL_FORCE_RECOVERY_PEDAL_FLOOR = 0.060
PEDAL_FORCE_RECOVERY_SPEED_ERROR = 0.30
PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR = 0.05
PEDAL_FORCE_RECOVERY_REARM_SECONDS = 0.5
PEDAL_FORCE_RECOVERY_ZERO_HOLD_SECONDS = 0.12
PEDAL_FORCE_RECOVERY_MIN_HOLD_SECONDS = 0.30
PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS = 0.10
PEDAL_FORCE_RECOVERY_PLAN_DECEL_MARGIN = 0.25
PEDAL_FORCE_RECOVERY_PLAN_LOOKAHEAD_POINTS = 10
PEDAL_FORCE_RECOVERY_LEAD_TTC = 4.0
PEDAL_FORCE_RECOVERY_LEAD_CLOSING_SPEED = 0.5
PEDAL_FORCE_RECOVERY_LEAD_HEADWAY = 1.25
PEDAL_FORCE_RECOVERY_LEAD_DISTANCE_OFFSET = 5.0
PEDAL_FORCE_RECOVERY_HANDOFF_ACCEL = PEDAL_FORCE_RECOVERY_ACCEL

# Positive-command ineffective-pedal watchdog. This path covers the second
# failure mode: accel is positive and the pedal command is non-zero, but the
# vehicle still loses speed. Recovery is deliberately staged and bounded.
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

# Diagnostic block-reason bit mask. The values are kept local to selfdrive so
# this safety fix does not require a cereal schema rebuild.
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
  (RECOVERY_BLOCK_CURVE_DECEL, "CURVE_DECEL"),
  (RECOVERY_BLOCK_LEAD_RISK, "LEAD_RISK"),
  (RECOVERY_BLOCK_MANUAL_CATCHUP, "MANUAL_CATCHUP"),
  (RECOVERY_BLOCK_JOYSTICK, "JOYSTICK"),
  (RECOVERY_BLOCK_BENCH_DISABLED, "BENCH_DISABLED"),
  (RECOVERY_BLOCK_NO_INTERCEPTOR, "NO_INTERCEPTOR"),
)


def recovery_block_reason_text(reason):
  names = [name for bit, name in RECOVERY_BLOCK_NAMES if int(reason) & bit]
  return "|".join(names) if names else "NONE"


def recovery_speed_demand(speed_error, future_speed_error, injected_fault=False):
  # Either the current PID target or the future plan may prove that speed
  # recovery is needed. Requiring both suppressed real accel-zero events near
  # transitions between cruise and lead-limited planning.
  normal_demand = speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR or \
                  future_speed_error >= PEDAL_FORCE_RECOVERY_SPEED_ERROR
  injected_demand = bool(injected_fault) and \
                    future_speed_error >= PEDAL_FORCE_RECOVERY_INJECTED_SPEED_ERROR
  return normal_demand or injected_demand


def recovery_plan_decelerating(speeds, v_ego,
                               margin=PEDAL_FORCE_RECOVERY_PLAN_DECEL_MARGIN,
                               lookahead_points=PEDAL_FORCE_RECOVERY_PLAN_LOOKAHEAD_POINTS):
  """Return True only when the near-term plan explicitly asks to slow down."""
  if speeds is None or len(speeds) < 2:
    return True
  end = min(len(speeds), max(2, int(lookahead_points) + 1))
  # Ignore point zero because it is normally initialized from current speed and
  # can contain small estimator noise. A meaningful drop in the following
  # points is treated as an intentional coast/deceleration request.
  # capnp _DynamicListReader supports integer indexing, but not Python slices.
  # Read each future point explicitly so this works with both capnp lists and
  # ordinary Python sequences used by tests.
  future_min = min(float(speeds[i]) for i in range(1, end))
  return future_min < float(v_ego) - float(margin)


def recovery_curve_decelerating(is_curv_driving, slow_on_curves,
                                 curve_speed_ms, v_ego,
                                 margin=PEDAL_FORCE_RECOVERY_PLAN_DECEL_MARGIN):
  """Do not block merely because a road is curved; block actual curve slowing."""
  return bool(is_curv_driving) and bool(slow_on_curves) and \
         float(curve_speed_ms) < 200.0 and \
         float(curve_speed_ms) < float(v_ego) - float(margin)


def recovery_lead_risk(lead, v_ego):
  """Conservative independent guard against forcing throttle into a closing lead."""
  if lead is None or not bool(getattr(lead, "status", False)):
    return False

  d_rel = float(getattr(lead, "dRel", 0.0))
  v_rel = float(getattr(lead, "vRel", 0.0))
  if d_rel <= 0.0 or v_rel >= -PEDAL_FORCE_RECOVERY_LEAD_CLOSING_SPEED:
    return False

  closing_speed = -v_rel
  ttc = d_rel / max(closing_speed, 0.1)
  guarded_distance = max(8.0,
                         float(v_ego) * PEDAL_FORCE_RECOVERY_LEAD_HEADWAY +
                         PEDAL_FORCE_RECOVERY_LEAD_DISTANCE_OFFSET)
  return d_rel <= guarded_distance or ttc <= PEDAL_FORCE_RECOVERY_LEAD_TTC


def positive_pedal_ineffective_candidate(requested_accel, vehicle_accel, speed_error):
  """Detect a positive command that is not producing a vehicle response.

  A single noisy aEgo sample is not enough to activate recovery. This function
  only identifies candidate frames; PedalForceRecovery requires the condition
  to persist for PEDAL_POSITIVE_WATCHDOG_TRIGGER_SECONDS before changing output.
  """
  return float(requested_accel) >= PEDAL_POSITIVE_WATCHDOG_MIN_REQUEST and \
         float(speed_error) >= PEDAL_POSITIVE_WATCHDOG_MIN_SPEED_ERROR and \
         float(vehicle_accel) <= PEDAL_POSITIVE_WATCHDOG_NEGATIVE_ACCEL


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
                         brake_pressed, gas_pressed, standstill, raw_accel,
                         vehicle_accel=None, speed_error=None):
  """Capture zero-accel and positive-but-ineffective occurrences while driving."""
  normal_driving = bool(controls_active) and bool(adaptive_cruise) and \
    not bool(brake_pressed) and not bool(gas_pressed) and not bool(standstill)
  zero_accel_while_driving = normal_driving and \
    abs(float(raw_accel)) <= PEDAL_FORCE_RECOVERY_ACCEL_EPS
  positive_ineffective = False
  if normal_driving and vehicle_accel is not None and speed_error is not None:
    positive_ineffective = positive_pedal_ineffective_candidate(
      raw_accel, vehicle_accel, speed_error)
  return bool(recovery_active) or zero_accel_while_driving or positive_ineffective


class PedalForceRecovery:
  """Bounded watchdog for zero and positive-but-ineffective pedal commands.

  Zero path:
    1. Bridge the last bounded positive request for 120 ms.
    2. If zero persists, force 0.36 m/s^2 and a 6% pedal floor.

  Positive ineffective path:
    1. Observe positive accel + speed deficit + negative aEgo for 500 ms.
    2. Apply a 0.22 m/s^2 / 3.5% floor for 600 ms.
    3. If the vehicle still does not respond, apply 0.36 m/s^2 / 6%.
    4. Release after confirmed positive response, or stop the forced floor after
       two seconds and flag a suspected command-delivery fault.

  Every external safety eligibility gate has immediate authority over both
  paths. Negative accel is always treated as intentional deceleration.
  """

  def __init__(self, dt=0.01):
    self.dt = float(dt)
    self.rearm_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_REARM_SECONDS / self.dt)))
    self.zero_hold_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_ZERO_HOLD_SECONDS / self.dt)))
    self.min_hold_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_MIN_HOLD_SECONDS / self.dt)))
    self.handoff_confirm_frames = max(1, int(round(PEDAL_FORCE_RECOVERY_HANDOFF_SECONDS / self.dt)))
    self.positive_trigger_frames = max(1, int(round(PEDAL_POSITIVE_WATCHDOG_TRIGGER_SECONDS / self.dt)))
    self.positive_stage1_frames = max(1, int(round(PEDAL_POSITIVE_WATCHDOG_STAGE1_SECONDS / self.dt)))
    self.positive_response_frames_required = max(1, int(round(PEDAL_POSITIVE_WATCHDOG_RESPONSE_SECONDS / self.dt)))
    self.positive_max_stage2_frames = max(1, int(round(PEDAL_POSITIVE_WATCHDOG_MAX_STAGE2_SECONDS / self.dt)))
    self.positive_cooldown_frames_total = max(1, int(round(PEDAL_POSITIVE_WATCHDOG_COOLDOWN_SECONDS / self.dt)))

    self.hold_active = False
    self.zero_force_active = False
    self.zero_event_frames = 0
    self.zero_active_frames = 0
    self.zero_frames = 0
    self.handoff_frames = 0
    self.inactive_frames = self.rearm_frames

    self.positive_candidate_frames = 0
    self.positive_stage = 0
    self.positive_stage_frames = 0
    self.positive_event_frames = 0
    self.positive_response_frames = 0
    self.positive_cooldown_frames = 0
    self.positive_activation_count = 0
    self.delivery_fault = False
    self.delivery_fault_count = 0

    self.activation_count = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0
    self.last_positive_accel = 0.0
    self.vehicle_accel = 0.0
    self.speed_error = 0.0

  @property
  def active(self):
    # Strong recovery: zero-force path or positive watchdog stage 2.
    return self.zero_force_active or self.positive_stage == 2

  @property
  def positive_candidate(self):
    return self.positive_candidate_frames > 0 and self.positive_stage == 0

  @property
  def positive_watchdog_active(self):
    return self.positive_stage > 0

  @property
  def watchdog_active(self):
    return self.hold_active or self.zero_force_active or self.positive_stage > 0

  @property
  def pedal_floor(self):
    if self.zero_force_active or self.positive_stage == 2:
      return PEDAL_FORCE_RECOVERY_PEDAL_FLOOR
    if self.positive_stage == 1:
      return PEDAL_POSITIVE_WATCHDOG_STAGE1_PEDAL_FLOOR
    return 0.0

  @property
  def pedal_floor_active(self):
    return self.pedal_floor > 0.0

  @property
  def requested_floor_accel(self):
    if self.zero_force_active or self.positive_stage == 2:
      return PEDAL_FORCE_RECOVERY_ACCEL
    if self.positive_stage == 1:
      return PEDAL_POSITIVE_WATCHDOG_STAGE1_ACCEL
    if self.hold_active:
      return self.last_positive_accel
    return 0.0

  @property
  def duration(self):
    return max(self.zero_event_frames, self.positive_event_frames) * self.dt

  def _clear_zero_event(self):
    self.hold_active = False
    self.zero_force_active = False
    self.zero_event_frames = 0
    self.zero_active_frames = 0
    self.zero_frames = 0
    self.handoff_frames = 0

  def _clear_positive_event(self, clear_candidate=True):
    if clear_candidate:
      self.positive_candidate_frames = 0
    self.positive_stage = 0
    self.positive_stage_frames = 0
    self.positive_event_frames = 0
    self.positive_response_frames = 0

  def _clear_all_events(self):
    self._clear_zero_event()
    self._clear_positive_event()

  def reset(self):
    self._clear_all_events()
    self.inactive_frames = self.rearm_frames
    self.positive_cooldown_frames = 0
    self.activation_count = 0
    self.positive_activation_count = 0
    self.delivery_fault = False
    self.delivery_fault_count = 0
    self.raw_accel = 0.0
    self.forced_accel = 0.0
    self.last_positive_accel = 0.0
    self.vehicle_accel = 0.0
    self.speed_error = 0.0

  def _update_zero_watchdog(self):
    if not (self.hold_active or self.zero_force_active):
      if self.inactive_frames >= self.rearm_frames:
        self.activation_count += 1
      self.hold_active = True
      self.zero_event_frames = 0
      self.zero_frames = 0
      self.handoff_frames = 0

    self.inactive_frames = 0
    self.zero_event_frames += 1
    self.zero_frames += 1

    # Stage 1: bridge only the last bounded positive request for 120 ms.
    if self.hold_active:
      if self.zero_frames <= self.zero_hold_frames:
        self.forced_accel = max(self.raw_accel, self.last_positive_accel)
        return self.forced_accel

      self.hold_active = False
      self.zero_force_active = True
      self.zero_active_frames = 0
      self.handoff_frames = 0

    # Stage 2: persistent zero receives the calibrated force request.
    self.zero_active_frames += 1
    if self.raw_accel >= PEDAL_FORCE_RECOVERY_HANDOFF_ACCEL:
      self.handoff_frames += 1
    else:
      self.handoff_frames = 0

    minimum_hold_complete = self.zero_active_frames > self.min_hold_frames
    pid_handoff_confirmed = self.handoff_frames >= self.handoff_confirm_frames
    if minimum_hold_complete and pid_handoff_confirmed:
      self._clear_zero_event()
      self.inactive_frames = 1
      self.last_positive_accel = min(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
      self.forced_accel = self.raw_accel
    else:
      self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)

    return self.forced_accel

  def _update_zero_force_handoff(self):
    """Keep the strong zero-fault floor until the PID can safely take over."""
    self.zero_event_frames += 1
    self.zero_active_frames += 1
    if self.raw_accel >= PEDAL_FORCE_RECOVERY_HANDOFF_ACCEL:
      self.handoff_frames += 1
    else:
      self.handoff_frames = 0

    minimum_hold_complete = self.zero_active_frames > self.min_hold_frames
    pid_handoff_confirmed = self.handoff_frames >= self.handoff_confirm_frames
    if minimum_hold_complete and pid_handoff_confirmed:
      self._clear_zero_event()
      self.inactive_frames = 1
      self.last_positive_accel = min(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
      self.forced_accel = self.raw_accel
    else:
      self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
    return self.forced_accel

  def _update_positive_watchdog(self):
    if self.positive_cooldown_frames > 0:
      self.positive_cooldown_frames -= 1
      self.positive_candidate_frames = 0
      self.forced_accel = self.raw_accel
      return self.forced_accel

    ineffective = positive_pedal_ineffective_candidate(
      self.raw_accel, self.vehicle_accel, self.speed_error)

    if self.positive_stage == 0:
      if ineffective:
        self.positive_candidate_frames += 1
      else:
        self.positive_candidate_frames = 0
        self.delivery_fault = False

      if self.positive_candidate_frames < self.positive_trigger_frames:
        self.forced_accel = self.raw_accel
        return self.forced_accel

      self.positive_stage = 1
      self.positive_stage_frames = 0
      self.positive_event_frames = 0
      self.positive_response_frames = 0
      self.positive_candidate_frames = 0
      self.delivery_fault = False
      self.activation_count += 1
      self.positive_activation_count += 1

    self.positive_event_frames += 1
    self.positive_stage_frames += 1

    if self.vehicle_accel >= PEDAL_POSITIVE_WATCHDOG_RESPONSE_ACCEL:
      self.positive_response_frames += 1
    else:
      self.positive_response_frames = 0

    # Confirm that the vehicle actually responded before handing back to the
    # original positive PID request. One positive aEgo frame is not sufficient.
    if self.positive_response_frames >= self.positive_response_frames_required:
      self._clear_positive_event()
      self.positive_cooldown_frames = self.rearm_frames
      self.delivery_fault = False
      self.forced_accel = self.raw_accel
      return self.forced_accel

    if self.positive_stage == 1:
      self.forced_accel = max(self.raw_accel, PEDAL_POSITIVE_WATCHDOG_STAGE1_ACCEL)
      if self.positive_stage_frames >= self.positive_stage1_frames:
        self.positive_stage = 2
        self.positive_stage_frames = 0
        self.positive_response_frames = 0
        self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
      return self.forced_accel

    # Stage 2 is intentionally bounded. If 6% still produces no measurable
    # response for two seconds, a command-delivery or pedal-hardware problem is
    # more likely than an insufficient PID request. Do not keep forcing throttle
    # indefinitely; flag the fault and return authority to the raw command.
    self.forced_accel = max(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)
    if self.positive_stage_frames >= self.positive_max_stage2_frames:
      self._clear_positive_event()
      self.positive_cooldown_frames = self.positive_cooldown_frames_total
      self.delivery_fault = True
      self.delivery_fault_count += 1
      self.forced_accel = self.raw_accel

    return self.forced_accel

  def update(self, eligible, requested_accel, vehicle_accel=0.0, speed_error=0.0):
    self.raw_accel = float(requested_accel)
    self.vehicle_accel = float(vehicle_accel)
    self.speed_error = float(speed_error)
    eligible = bool(eligible)

    # Keep a bounded memory of the last real positive PID request. Never bridge
    # more than the calibrated recovery request.
    if self.raw_accel > PEDAL_FORCE_RECOVERY_ACCEL_EPS and not self.watchdog_active:
      self.last_positive_accel = min(self.raw_accel, PEDAL_FORCE_RECOVERY_ACCEL)

    # Every safety gate cancels both paths on the current 100 Hz frame.
    if not eligible:
      self._clear_all_events()
      self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
      self.positive_cooldown_frames = max(0, self.positive_cooldown_frames - 1)
      self.forced_accel = self.raw_accel
      return self.forced_accel

    # A genuine negative request is intentional deceleration and always wins.
    if self.raw_accel < -PEDAL_FORCE_RECOVERY_ACCEL_EPS:
      self._clear_all_events()
      self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
      self.forced_accel = self.raw_accel
      return self.forced_accel

    # Zero request has priority over the positive ineffective path.
    if abs(self.raw_accel) <= PEDAL_FORCE_RECOVERY_ACCEL_EPS:
      self._clear_positive_event()
      return self._update_zero_watchdog()

    # During strong zero recovery, keep the bounded floor until the PID has
    # produced a strong enough positive request for the configured handoff time.
    if self.zero_force_active:
      self._clear_positive_event()
      return self._update_zero_force_handoff()

    # A positive request ends only the short zero bridge, then enters the
    # measured-response watchdog if the vehicle is still losing speed.
    if self.hold_active:
      self._clear_zero_event()
      self.inactive_frames = 1

    self.inactive_frames = min(self.rearm_frames, self.inactive_frames + 1)
    return self._update_positive_watchdog()
