#!/usr/bin/env python3
import math


KPH_TO_MS = 1.0 / 3.6

PEDAL_LAUNCH_STATE_DISABLED = 0
PEDAL_LAUNCH_STATE_WAITING = 1
PEDAL_LAUNCH_STATE_CONFIRMING = 2
PEDAL_LAUNCH_STATE_ACTIVE = 3
PEDAL_LAUNCH_STATE_BLOCKED_GAP = 4
PEDAL_LAUNCH_STATE_BLOCKED_CLOSING = 5
PEDAL_LAUNCH_STATE_BLOCKED_BRAKING = 6
PEDAL_LAUNCH_STATE_NO_LEAD = 7
PEDAL_LAUNCH_STATE_SPEED = 8
PEDAL_LAUNCH_STATE_TIMEOUT = 9
PEDAL_LAUNCH_STATE_DRIVER_OVERRIDE = 10
PEDAL_LAUNCH_STATE_FORCE_DECEL = 11

PEDAL_LAUNCH_ARM_MAX_VEGO = 0.5 * KPH_TO_MS
PEDAL_LAUNCH_ARM_EXIT_VEGO = 1.0 * KPH_TO_MS
PEDAL_LAUNCH_EXIT_VEGO = 25.0 * KPH_TO_MS
PEDAL_LAUNCH_ARM_TIME = 0.5
PEDAL_LAUNCH_CONFIRM_SAMPLES = 2
PEDAL_LAUNCH_RADAR_PERIOD = 1.0 / 15.0
PEDAL_LAUNCH_MAX_RADAR_AGE = 0.20
PEDAL_LAUNCH_DEPARTURE_LATCH_TIME = 1.5
PEDAL_LAUNCH_KICK_TIME = 0.40
PEDAL_LAUNCH_TIMEOUT = 10.0

PEDAL_LAUNCH_MIN_DREL = 3.0
PEDAL_LAUNCH_MAX_ARM_DREL = 20.0
PEDAL_LAUNCH_DISTANCE_HEADWAY = 0.75
PEDAL_LAUNCH_STATIONARY_VLEAD = 0.08
PEDAL_LAUNCH_STATIONARY_VREL = 0.15
PEDAL_LAUNCH_MIN_VLEAD = 0.30
PEDAL_LAUNCH_MIN_DISTANCE_DELTA = 0.20
PEDAL_LAUNCH_MIN_OPENING_VREL = 0.10
PEDAL_LAUNCH_MAX_CLOSING_VREL = -0.20
PEDAL_LAUNCH_MAX_LEAD_DECEL = -0.30
PEDAL_LAUNCH_REFERENCE_DRIFT = 0.04
PEDAL_LAUNCH_PREDICTION_TIME = 1.0

PEDAL_LAUNCH_ACCEL_BP_KPH = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0]
PEDAL_LAUNCH_ACCEL_V = [0.70, 0.90, 1.00, 0.95, 0.75, 0.00]
PEDAL_LAUNCH_MAX_ACCEL = 1.05


def _interp(x, xp, fp):
  if x <= xp[0]:
    return float(fp[0])
  if x >= xp[-1]:
    return float(fp[-1])
  for index in range(1, len(xp)):
    if x <= xp[index]:
      ratio = (x - xp[index - 1]) / (xp[index] - xp[index - 1])
      return float(fp[index - 1] + ratio * (fp[index] - fp[index - 1]))
  return float(fp[-1])


def pedal_launch_accel_floor(v_ego):
  return _interp(max(float(v_ego), 0.0) * 3.6,
                 PEDAL_LAUNCH_ACCEL_BP_KPH,
                 PEDAL_LAUNCH_ACCEL_V)


def pedal_launch_safe_distance(v_ego):
  return PEDAL_LAUNCH_MIN_DREL + PEDAL_LAUNCH_DISTANCE_HEADWAY * max(float(v_ego), 0.0)


class PedalLaunchBoostController:
  """Detect a stopped lead departing and provide a guarded 0-25 km/h accel floor."""

  def __init__(self):
    self.reset()

  def reset(self, state=PEDAL_LAUNCH_STATE_DISABLED):
    self.active = False
    self.kick_active = False
    self.state = state
    self.safe_distance = pedal_launch_safe_distance(0.0)
    self.distance_delta = 0.0
    self.confirm_time = 0.0
    self.accel_floor = 0.0
    self.output_accel = 0.0
    self.radar_age = 0.0
    self.stopped_d_rel = None
    self.stopped_since = None
    self.confirm_samples = 0
    self.departure_latched_until = 0.0
    self.active_since = None
    self.kick_until = None

  @staticmethod
  def _lead_values(lead):
    if lead is None or not bool(getattr(lead, "status", False)):
      return None
    d_rel = float(getattr(lead, "dRel", 0.0))
    if not math.isfinite(d_rel) or d_rel <= 0.0:
      return None
    v_rel = float(getattr(lead, "vRel", 0.0))
    v_lead = float(getattr(lead, "vLeadK", getattr(lead, "vLead", 0.0)))
    a_lead = float(getattr(lead, "aLeadK", 0.0))
    if not all(math.isfinite(value) for value in (v_rel, v_lead, a_lead)):
      return None
    return d_rel, v_rel, v_lead, a_lead

  def _block(self, state, requested_accel):
    self.reset(state)
    return min(float(requested_accel), 0.0)

  def _lead_is_safe(self, d_rel, v_rel, a_lead, v_ego):
    self.safe_distance = pedal_launch_safe_distance(v_ego)
    prediction_time = PEDAL_LAUNCH_PREDICTION_TIME
    predicted_distance = d_rel + min(v_rel, 0.0) * prediction_time + \
                         0.5 * min(a_lead, 0.0) * prediction_time ** 2
    if d_rel < self.safe_distance or predicted_distance < self.safe_distance:
      return PEDAL_LAUNCH_STATE_BLOCKED_GAP
    if v_rel <= PEDAL_LAUNCH_MAX_CLOSING_VREL:
      return PEDAL_LAUNCH_STATE_BLOCKED_CLOSING
    if a_lead <= PEDAL_LAUNCH_MAX_LEAD_DECEL:
      return PEDAL_LAUNCH_STATE_BLOCKED_BRAKING
    return None

  def update(self, enabled, brake_pressed, gas_pressed, standstill, v_ego,
             lead, radar_updated, radar_age, requested_accel, accel_limit_max,
             force_decel, now):
    requested_accel = float(requested_accel)
    v_ego = max(float(v_ego), 0.0)
    now = float(now)
    self.radar_age = max(float(radar_age), 0.0)
    self.output_accel = requested_accel

    if not enabled:
      self.reset(PEDAL_LAUNCH_STATE_DISABLED)
      return requested_accel
    if force_decel:
      return self._block(PEDAL_LAUNCH_STATE_FORCE_DECEL, requested_accel)
    if gas_pressed:
      return self._block(PEDAL_LAUNCH_STATE_DRIVER_OVERRIDE, requested_accel)
    if brake_pressed and self.active:
      return self._block(PEDAL_LAUNCH_STATE_DRIVER_OVERRIDE, requested_accel)
    if v_ego >= PEDAL_LAUNCH_EXIT_VEGO:
      self.reset(PEDAL_LAUNCH_STATE_SPEED)
      return requested_accel
    if not self.active and self.stopped_d_rel is not None and \
       v_ego > PEDAL_LAUNCH_ARM_EXIT_VEGO:
      self.reset(PEDAL_LAUNCH_STATE_SPEED)
      return requested_accel

    lead_values = self._lead_values(lead)
    if lead_values is None or self.radar_age > PEDAL_LAUNCH_MAX_RADAR_AGE:
      if self.active or self.stopped_d_rel is not None:
        return self._block(PEDAL_LAUNCH_STATE_NO_LEAD, requested_accel)
      self.reset(PEDAL_LAUNCH_STATE_NO_LEAD)
      return requested_accel

    d_rel, v_rel, v_lead, a_lead = lead_values
    if self.stopped_d_rel is not None:
      self.distance_delta = d_rel - self.stopped_d_rel

    if self.active or self.stopped_d_rel is not None:
      blocked_state = self._lead_is_safe(d_rel, v_rel, a_lead, v_ego)
      if blocked_state is not None:
        return self._block(blocked_state, requested_accel)

    if self.active:
      if self.active_since is None or now - self.active_since > PEDAL_LAUNCH_TIMEOUT:
        return self._block(PEDAL_LAUNCH_STATE_TIMEOUT, requested_accel)

      self.kick_active = self.kick_until is not None and now <= self.kick_until
      opening_safely = v_rel >= PEDAL_LAUNCH_MIN_OPENING_VREL and \
                       d_rel >= self.safe_distance + 0.5
      if not self.kick_active and requested_accel <= 0.0 and not opening_safely:
        self.reset(PEDAL_LAUNCH_STATE_DISABLED)
        return requested_accel

      self.accel_floor = pedal_launch_accel_floor(v_ego)
      launch_cap = min(PEDAL_LAUNCH_MAX_ACCEL,
                       max(max(float(accel_limit_max), 0.0), self.accel_floor))
      self.output_accel = min(max(requested_accel, self.accel_floor), launch_cap)
      self.state = PEDAL_LAUNCH_STATE_ACTIVE
      return self.output_accel

    stopped = bool(standstill) or v_ego <= PEDAL_LAUNCH_ARM_MAX_VEGO
    lead_stationary = v_lead <= PEDAL_LAUNCH_STATIONARY_VLEAD and \
                      abs(v_rel) <= PEDAL_LAUNCH_STATIONARY_VREL
    arm_distance_ok = PEDAL_LAUNCH_MIN_DREL <= d_rel <= PEDAL_LAUNCH_MAX_ARM_DREL

    if self.stopped_d_rel is None:
      if stopped and lead_stationary and arm_distance_ok:
        if self.stopped_since is None:
          self.stopped_since = now
        self.state = PEDAL_LAUNCH_STATE_WAITING
        if now - self.stopped_since >= PEDAL_LAUNCH_ARM_TIME:
          self.stopped_d_rel = d_rel
          self.distance_delta = 0.0
      else:
        self.stopped_since = None
        self.state = PEDAL_LAUNCH_STATE_DISABLED
      return min(requested_accel, 0.0) if stopped else requested_accel

    self.distance_delta = d_rel - self.stopped_d_rel
    if lead_stationary and abs(self.distance_delta) <= PEDAL_LAUNCH_REFERENCE_DRIFT:
      self.stopped_d_rel = 0.995 * self.stopped_d_rel + 0.005 * d_rel
      self.distance_delta = d_rel - self.stopped_d_rel

    if radar_updated:
      distance_opened = self.distance_delta >= PEDAL_LAUNCH_MIN_DISTANCE_DELTA
      lead_departing = v_lead >= PEDAL_LAUNCH_MIN_VLEAD or \
                       (distance_opened and v_rel >= PEDAL_LAUNCH_MIN_OPENING_VREL)
      if lead_departing:
        self.confirm_samples += 1
      elif lead_stationary:
        self.confirm_samples = 0
      self.confirm_time = self.confirm_samples * PEDAL_LAUNCH_RADAR_PERIOD

      if self.confirm_samples >= PEDAL_LAUNCH_CONFIRM_SAMPLES:
        self.departure_latched_until = now + PEDAL_LAUNCH_DEPARTURE_LATCH_TIME
        self.confirm_samples = 0
        self.confirm_time = PEDAL_LAUNCH_CONFIRM_SAMPLES * PEDAL_LAUNCH_RADAR_PERIOD

    if self.departure_latched_until > now:
      self.state = PEDAL_LAUNCH_STATE_CONFIRMING
      if not brake_pressed:
        self.active = True
        self.kick_active = True
        self.active_since = now
        self.kick_until = now + PEDAL_LAUNCH_KICK_TIME
        self.state = PEDAL_LAUNCH_STATE_ACTIVE
        self.accel_floor = pedal_launch_accel_floor(v_ego)
        launch_cap = min(PEDAL_LAUNCH_MAX_ACCEL,
                         max(max(float(accel_limit_max), 0.0), self.accel_floor))
        self.output_accel = min(max(requested_accel, self.accel_floor), launch_cap)
        return self.output_accel
    elif self.departure_latched_until > 0.0:
      self.reset(PEDAL_LAUNCH_STATE_TIMEOUT)
      return min(requested_accel, 0.0) if stopped else requested_accel

    self.state = PEDAL_LAUNCH_STATE_CONFIRMING if self.confirm_samples else PEDAL_LAUNCH_STATE_WAITING
    return min(requested_accel, 0.0) if stopped else requested_accel
