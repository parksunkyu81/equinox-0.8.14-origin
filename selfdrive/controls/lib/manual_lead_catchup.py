import math

from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.params import Params
from common.realtime import DT_CTRL


MANUAL_CATCHUP_IDLE = 0
MANUAL_CATCHUP_GAS_PRESSED = 1
MANUAL_CATCHUP_ACTIVE = 2
MANUAL_CATCHUP_BLEND_OUT = 3

# The stock MPC distance model is intentionally mirrored here instead of
# importing long_mpc.py. Importing long_mpc pulls in the generated acados
# solver, while this helper only needs the distance equation.
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 5.5

MANUAL_GAS_ARM_MAX_EGO_KPH = 8.0
CATCHUP_MAX_EGO_KPH = 30.0
MIN_PEDAL_SPEED_KPH = 1.0
MIN_LEAD_SPEED_MS = 0.8
MIN_REL_SPEED_MS = 0.3
LEAD_STOP_SPEED_MS = 0.25
MIN_ABSOLUTE_DISTANCE_M = 3.5
MIN_HANDOFF_GAP_RATIO = 0.80
MIN_ACTIVE_GAP_RATIO = 0.75
MIN_TTC_S = 4.0
MAX_ACTIVE_S = 6.0
RELEASE_SURPLUS_M = 0.8
RELEASE_REL_SPEED_MS = 0.3
RELEASE_CONFIRM_S = 0.30
MAX_STEERING_ANGLE_DEG = 15.0
BOOST_RISE_JERK = 0.8
BOOST_FALL_JERK = 1.5
CONFIG_REFRESH_S = 0.50
SAFETY_RECOVERY_BLOCK_S = 0.50


def safe_obstacle_distance(v_ego, t_follow):
  v_ego = max(0.0, float(v_ego))
  t_follow = float(clip(t_follow, 0.8, 2.5))
  return (v_ego ** 2) / (2.0 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE


class ManualLeadCatchup:
  """Distance-aware acceleration handoff after an explicit driver launch.

  This helper never permits a standstill launch. It can arm only while the
  driver is pressing the accelerator and becomes active only after the driver
  releases it with vehicle speed already above 1 km/h.
  """

  def __init__(self, dt=DT_CTRL, params=None):
    self.dt = float(dt)
    self.params = params if params is not None else Params()
    self.config_refresh_frames = max(1, int(round(CONFIG_REFRESH_S / self.dt)))
    self.release_confirm_frames = max(1, int(round(RELEASE_CONFIRM_S / self.dt)))
    self.max_active_frames = max(1, int(round(MAX_ACTIVE_S / self.dt)))
    self.safety_recovery_block_frames = max(1, int(round(SAFETY_RECOVERY_BLOCK_S / self.dt)))

    self.enabled = False
    self.max_accel = 0.70
    self._config_frame = -self.config_refresh_frames

    self.state = MANUAL_CATCHUP_IDLE
    self.prev_gas_pressed = False
    self.active_frames = 0
    self.release_frames = 0
    self.recovery_block_frames = 0
    self.accel_floor = 0.0
    self.target_accel_floor = 0.0
    self.safe_distance = 0.0
    self.gap_surplus = 0.0
    self.effective_gap_surplus = 0.0
    self.cancel_reason = ""
    self.activation_count = 0

  @property
  def active(self):
    return self.state in (MANUAL_CATCHUP_ACTIVE, MANUAL_CATCHUP_BLEND_OUT)

  @property
  def handoff_ready(self):
    return self.active

  @property
  def recovery_blocked(self):
    return self.active or self.recovery_block_frames > 0

  @property
  def duration(self):
    return self.active_frames * self.dt

  def _read_config(self, frame):
    if frame - self._config_frame < self.config_refresh_frames:
      return
    self._config_frame = int(frame)
    try:
      self.enabled = bool(self.params.get_bool("ManualLeadCatchupEnabled"))
    except Exception:
      self.enabled = False

    try:
      raw = self.params.get("ManualLeadCatchupMaxAccel")
      value = float(raw) if raw is not None else 0.70
    except (TypeError, ValueError):
      value = 0.70
    if not math.isfinite(value):
      value = 0.70
    self.max_accel = float(clip(value, 0.40, 0.70))

  @staticmethod
  def _lead_valid(lead):
    if lead is None or not bool(getattr(lead, "status", False)):
      return False
    d_rel = float(getattr(lead, "dRel", 0.0))
    return math.isfinite(d_rel) and d_rel > 0.0

  def _reset_active(self, reason="", safety=False):
    self.state = MANUAL_CATCHUP_IDLE
    self.active_frames = 0
    self.release_frames = 0
    self.accel_floor = 0.0
    self.target_accel_floor = 0.0
    self.cancel_reason = str(reason)
    if safety:
      self.recovery_block_frames = max(self.recovery_block_frames, self.safety_recovery_block_frames)

  def reset(self):
    self._reset_active()
    self.prev_gas_pressed = False
    self.recovery_block_frames = 0

  def _common_context_valid(self, controls_active, adaptive_cruise, CS, lead,
                            plan_valid, fcw, curve_blocked):
    return bool(controls_active) and bool(adaptive_cruise) and bool(plan_valid) and not bool(fcw) and \
      not bool(curve_blocked) and abs(float(getattr(CS, "steeringAngleDeg", 0.0))) <= MAX_STEERING_ANGLE_DEG and \
      not bool(getattr(CS, "brakePressed", False)) and self._lead_valid(lead)

  def pre_update(self, frame, controls_active, adaptive_cruise, CS, lead,
                 dynamic_tr, plan_valid, fcw, curve_blocked):
    """Observe the driver launch before LongControl runs.

    Returns True only after the driver has released the accelerator and every
    handoff safety gate is valid. LongControl uses this to leave stopping mode;
    the pedal remains protected by the existing >1 km/h CarController gate.
    """
    self._read_config(frame)
    if self.recovery_block_frames > 0:
      self.recovery_block_frames -= 1

    gas_pressed = bool(getattr(CS, "gasPressed", False))
    gas_released = self.prev_gas_pressed and not gas_pressed
    v_ego = float(getattr(CS, "vEgo", 0.0))
    v_ego_kph = v_ego * CV.MS_TO_KPH

    if not self.enabled:
      self._reset_active("disabled")
      self.prev_gas_pressed = gas_pressed
      return False

    context_valid = self._common_context_valid(controls_active, adaptive_cruise, CS, lead,
                                                plan_valid, fcw, curve_blocked)

    if gas_pressed:
      # Explicit driver intent is mandatory. Merely detecting lead motion can
      # never arm this state machine.
      if context_valid and v_ego_kph <= MANUAL_GAS_ARM_MAX_EGO_KPH and \
         float(getattr(lead, "vLead", 0.0)) > LEAD_STOP_SPEED_MS and \
         float(getattr(lead, "dRel", 0.0)) >= MIN_ABSOLUTE_DISTANCE_M:
        self.state = MANUAL_CATCHUP_GAS_PRESSED
        self.active_frames = 0
        self.release_frames = 0
        self.accel_floor = 0.0
        self.target_accel_floor = 0.0
        self.cancel_reason = ""
      elif self.active:
        self._reset_active("driver_gas", safety=True)

    elif gas_released and self.state == MANUAL_CATCHUP_GAS_PRESSED:
      d_rel = float(getattr(lead, "dRel", 0.0)) if self._lead_valid(lead) else 0.0
      v_lead = float(getattr(lead, "vLead", 0.0)) if self._lead_valid(lead) else 0.0
      v_rel = float(getattr(lead, "vRel", v_lead - v_ego)) if self._lead_valid(lead) else -99.0
      safe_dist = safe_obstacle_distance(v_ego, dynamic_tr)
      handoff_gap = max(MIN_ABSOLUTE_DISTANCE_M, safe_dist * MIN_HANDOFF_GAP_RATIO)

      ready = context_valid and not bool(getattr(CS, "standstill", False)) and \
        v_ego_kph > MIN_PEDAL_SPEED_KPH and v_lead >= MIN_LEAD_SPEED_MS and \
        v_rel >= MIN_REL_SPEED_MS and d_rel >= handoff_gap

      if ready:
        self.state = MANUAL_CATCHUP_ACTIVE
        self.active_frames = 0
        self.release_frames = 0
        self.accel_floor = 0.0
        self.target_accel_floor = 0.0
        self.safe_distance = safe_dist
        self.gap_surplus = d_rel - safe_dist
        self.effective_gap_surplus = self.gap_surplus
        self.activation_count += 1
      else:
        self._reset_active("handoff_gate", safety=True)

    elif self.active:
      # Gates independent of the current MPC acceleration are checked here so
      # LongControl never receives a stale handoff signal.
      if not context_valid:
        self._reset_active("context", safety=True)
      elif gas_pressed:
        self._reset_active("driver_gas", safety=True)
      elif bool(getattr(CS, "standstill", False)) or v_ego_kph <= MIN_PEDAL_SPEED_KPH:
        self._reset_active("below_pedal_speed", safety=True)
      elif v_ego_kph > CATCHUP_MAX_EGO_KPH:
        self._reset_active("speed_complete")

    elif self.state == MANUAL_CATCHUP_GAS_PRESSED:
      # The release edge is the only legal transition to active. If that edge
      # was missed because the surrounding context disappeared, disarm.
      self._reset_active("manual_launch_expired")

    self.prev_gas_pressed = gas_pressed
    return self.handoff_ready

  def apply(self, requested_accel, accel_limits, CS, lead, dynamic_tr,
            plan_valid, fcw, curve_blocked, catchup_factor=0.0):
    """Apply a distance-derived acceleration floor after LongControl."""
    raw_accel = float(requested_accel)
    if not self.active:
      return raw_accel

    v_ego = float(getattr(CS, "vEgo", 0.0))
    v_ego_kph = v_ego * CV.MS_TO_KPH
    if not plan_valid or bool(fcw) or bool(curve_blocked) or \
       bool(getattr(CS, "brakePressed", False)) or bool(getattr(CS, "gasPressed", False)) or \
       bool(getattr(CS, "standstill", False)) or v_ego_kph <= MIN_PEDAL_SPEED_KPH or \
       not self._lead_valid(lead):
      self._reset_active("runtime_gate", safety=True)
      return raw_accel

    d_rel = float(getattr(lead, "dRel", 0.0))
    v_lead = float(getattr(lead, "vLead", 0.0))
    v_rel = float(getattr(lead, "vRel", v_lead - v_ego))
    a_lead = float(getattr(lead, "aLeadK", 0.0))
    a_ego = float(getattr(CS, "aEgo", 0.0))
    if not all(math.isfinite(v) for v in (raw_accel, v_ego, d_rel, v_lead, v_rel, a_lead, a_ego)):
      self._reset_active("nonfinite", safety=True)
      return raw_accel

    self.safe_distance = safe_obstacle_distance(v_ego, dynamic_tr)
    self.gap_surplus = d_rel - self.safe_distance

    min_runtime_distance = max(MIN_ABSOLUTE_DISTANCE_M, self.safe_distance * MIN_ACTIVE_GAP_RATIO)
    closing_speed = max(-v_rel, 0.0)
    ttc = d_rel / closing_speed if closing_speed > 0.1 else float("inf")

    if raw_accel < -0.05:
      self._reset_active("mpc_decel", safety=True)
      return raw_accel
    if d_rel < min_runtime_distance:
      self._reset_active("distance", safety=True)
      return raw_accel
    if ttc < MIN_TTC_S:
      self._reset_active("ttc", safety=True)
      return raw_accel
    if v_lead <= LEAD_STOP_SPEED_MS:
      self._reset_active("lead_stopped", safety=True)
      return raw_accel
    if v_rel < -0.2:
      self._reset_active("closing", safety=True)
      return raw_accel
    if self.active_frames >= self.max_active_frames:
      self._reset_active("timeout")
      return raw_accel

    projected_gap_growth = clip(v_rel, 0.0, 3.0) + 0.5 * clip(a_lead - a_ego, -1.0, 1.0)
    self.effective_gap_surplus = self.gap_surplus + 0.5 * max(float(projected_gap_growth), 0.0)

    gap_accel_floor = interp(
      self.effective_gap_surplus,
      [0.5, 1.0, 2.0, 3.5, 6.0, 10.0],
      [0.0, 0.15, 0.25, 0.36, 0.50, 0.65],
    )
    relative_speed_boost = interp(
      v_rel,
      [0.2, 0.5, 1.0, 2.0, 3.0],
      [0.0, 0.03, 0.07, 0.12, 0.15],
    )
    speed_accel_cap = interp(
      v_ego_kph,
      [1.0, 3.0, 5.0, 10.0, 20.0],
      [0.40, 0.50, 0.58, 0.68, 0.70],
    )

    # DynamicFollow confirms the same pull-away situation. Treat it as a soft
    # confidence term only; driver intent and measured gap remain authoritative.
    catchup_scale = interp(float(clip(catchup_factor, 0.0, 1.0)), [0.0, 0.5, 1.0], [0.85, 0.95, 1.0])
    target_floor = (float(gap_accel_floor) + float(relative_speed_boost)) * float(catchup_scale)
    target_floor = min(target_floor, float(speed_accel_cap), self.max_accel)

    release_condition = self.gap_surplus <= RELEASE_SURPLUS_M and v_rel <= RELEASE_REL_SPEED_MS
    if release_condition:
      self.release_frames += 1
    else:
      self.release_frames = 0

    if self.state == MANUAL_CATCHUP_ACTIVE and self.release_frames >= self.release_confirm_frames:
      self.state = MANUAL_CATCHUP_BLEND_OUT

    if self.state == MANUAL_CATCHUP_BLEND_OUT:
      target_floor = 0.0

    self.target_accel_floor = max(0.0, float(target_floor))
    if self.target_accel_floor > self.accel_floor:
      self.accel_floor = min(self.target_accel_floor, self.accel_floor + BOOST_RISE_JERK * self.dt)
    else:
      self.accel_floor = max(self.target_accel_floor, self.accel_floor - BOOST_FALL_JERK * self.dt)

    self.active_frames += 1
    if self.state == MANUAL_CATCHUP_BLEND_OUT and self.accel_floor <= 1e-3:
      self._reset_active("gap_complete")
      return raw_accel

    upper_limit = min(float(accel_limits[1]), self.max_accel)
    assisted = max(raw_accel, self.accel_floor)
    return float(clip(assisted, float(accel_limits[0]), upper_limit))
