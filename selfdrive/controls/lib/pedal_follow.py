from common.numpy_fast import clip, interp
from common.conversions import Conversions as CV
from common.realtime import DT_CTRL


# Pedal-only longitudinal control cannot command the brakes. Keep a comfortable
# speed-dependent target gap, predict two seconds ahead, and continuously remove
# acceleration as that predicted gap moves from the target toward the guard.
PEDAL_FOLLOW_TARGET_TR_BP_KPH = [0.0, 20.0, 40.0, 60.0, 80.0, 100.0]
PEDAL_FOLLOW_TARGET_TR_V = [1.00, 1.05, 1.10, 1.17, 1.22, 1.25]
PEDAL_FOLLOW_MIN_DISTANCE = 5.5
PEDAL_FOLLOW_GUARD_TR_MARGIN = 0.20
PEDAL_FOLLOW_GUARD_MIN_TR = 0.80
PEDAL_FOLLOW_MIN_AUTHORITY_BAND = 2.0
PEDAL_FOLLOW_PREDICTION_TIME = 2.0
PEDAL_FOLLOW_GUARD_CLOSING_TIME = 2.0
PEDAL_FOLLOW_OUTPUT_FALL_RATE = 0.80  # m/s^3
PEDAL_FOLLOW_OUTPUT_RISE_RATE = 1.10  # m/s^3
PEDAL_FOLLOW_RECOVERY_MARGIN = 2.0
PEDAL_FOLLOW_RECOVERY_MAX_CLOSING = 0.20
PEDAL_FOLLOW_RECOVERY_HYSTERESIS_MARGIN = 0.75
PEDAL_FOLLOW_RECOVERY_HYSTERESIS_MAX_CLOSING = 0.40
PEDAL_FOLLOW_RECOVERY_FRAMES = max(1, round(0.12 / DT_CTRL))
PEDAL_FOLLOW_LEAD_DROPOUT_HOLD_FRAMES = max(1, round(0.20 / DT_CTRL))
PEDAL_FOLLOW_LEAD_BRAKING_ACCEL = -0.30
PEDAL_FOLLOW_LEAD_HARD_BRAKING_ACCEL = -1.00
PEDAL_FOLLOW_LEAD_BRAKING_MARGIN = 4.0
PEDAL_FOLLOW_LEAD_BRAKING_CONFIRM_FRAMES = max(1, round(0.15 / DT_CTRL))
PEDAL_DEADZONE_FLOOR = 0.060
PEDAL_DEADZONE_CONFIRM_FRAMES = max(1, round(0.10 / DT_CTRL))
PEDAL_DEADZONE_RAMP_FRAMES = max(1, round(0.25 / DT_CTRL))
PEDAL_DEADZONE_MAX_ACTIVE_FRAMES = max(1, round(0.80 / DT_CTRL))
PEDAL_DEADZONE_RESPONSE_ACCEL = 0.20
PEDAL_DEADZONE_RESPONSE_CONFIRM_FRAMES = max(1, round(0.10 / DT_CTRL))
PEDAL_FOLLOW_URGENT_VREL = -0.80
PEDAL_FOLLOW_URGENT_TTC = 3.0
PEDAL_FOLLOW_URGENT_MIN_DISTANCE = 5.5
PEDAL_FOLLOW_URGENT_HEADWAY = 0.8
PEDAL_LAUNCH_ENTER_MAX_VEGO = 2.0 * CV.KPH_TO_MS
PEDAL_LAUNCH_EXIT_VEGO = 3.0 * CV.KPH_TO_MS
PEDAL_LAUNCH_MIN_DISTANCE = 3.0
PEDAL_LAUNCH_MIN_VLEAD = 0.15
PEDAL_LAUNCH_MIN_DISTANCE_DELTA = 0.10
PEDAL_LAUNCH_MAX_CLOSING_VREL = -0.20
PEDAL_LAUNCH_MAX_LEAD_DECEL = -0.30
PEDAL_LAUNCH_STATIONARY_VLEAD = 0.05
PEDAL_LAUNCH_STATIONARY_VREL = 0.05
PEDAL_LAUNCH_CONFIRM_FRAMES = max(1, round(0.15 / DT_CTRL))
PEDAL_LAUNCH_TIMEOUT_FRAMES = max(1, round(2.0 / DT_CTRL))


def smoothstep(edge0, edge1, value):
  if edge1 <= edge0:
    return float(value >= edge1)
  x = float(clip((float(value) - edge0) / (edge1 - edge0), 0.0, 1.0))
  return x * x * (3.0 - 2.0 * x)


def pedal_follow_target_headway(v_ego):
  return float(interp(max(float(v_ego), 0.0) * CV.MS_TO_KPH,
                      PEDAL_FOLLOW_TARGET_TR_BP_KPH,
                      PEDAL_FOLLOW_TARGET_TR_V))


def pedal_follow_geometry(v_ego, d_rel, v_rel, target_tr=None, target_distance=None):
  v_ego = max(float(v_ego), 0.0)
  v_rel = float(v_rel)
  closing_speed = max(-v_rel, 0.0)

  if target_distance is None or target_distance <= 0.0:
    if target_tr is None:
      target_tr = pedal_follow_target_headway(v_ego)
    target_distance = PEDAL_FOLLOW_MIN_DISTANCE + max(float(target_tr), 0.0) * v_ego
  else:
    target_distance = float(target_distance)
    target_tr = max((target_distance - PEDAL_FOLLOW_MIN_DISTANCE) / max(v_ego, 0.1),
                    PEDAL_FOLLOW_GUARD_MIN_TR)

  guard_tr = max(float(target_tr) - PEDAL_FOLLOW_GUARD_TR_MARGIN,
                 PEDAL_FOLLOW_GUARD_MIN_TR)
  guard_distance = PEDAL_FOLLOW_MIN_DISTANCE + guard_tr * v_ego + \
                   PEDAL_FOLLOW_GUARD_CLOSING_TIME * closing_speed
  predicted_distance = float(d_rel) + PEDAL_FOLLOW_PREDICTION_TIME * v_rel
  return target_distance, guard_distance, predicted_distance, closing_speed


def pedal_follow_urgent(lead, v_ego):
  if lead is None or not lead.status or lead.dRel <= 0.0 or lead.vRel >= 0.0:
    return False

  closing_speed = -float(lead.vRel)
  ttc = float(lead.dRel) / max(closing_speed, 0.1)
  urgent_distance = PEDAL_FOLLOW_URGENT_MIN_DISTANCE + \
                    PEDAL_FOLLOW_URGENT_HEADWAY * max(float(v_ego), 0.0)
  return lead.dRel <= urgent_distance and \
         (lead.vRel <= PEDAL_FOLLOW_URGENT_VREL or ttc <= PEDAL_FOLLOW_URGENT_TTC)


def pedal_deadzone_recovery_safe(follow_smoother, lead):
  return follow_smoother is not None and \
         follow_smoother.state == 'recovering' and \
         follow_smoother.accel_authority >= 0.80 and \
         follow_smoother.predicted_distance >= follow_smoother.target_distance and \
         lead is not None and lead.status and lead.dRel > 0.0 and \
         lead.vRel >= -0.20


class PedalLaunchController:
  def __init__(self):
    self.active = False
    self.confirm_frames = 0
    self.timeout_frames = 0
    self.stopped_d_rel = None

  def reset(self):
    self.active = False
    self.confirm_frames = 0
    self.timeout_frames = 0
    self.stopped_d_rel = None

  def update(self, enabled, brake_pressed, lead, v_ego):
    lead_valid = lead is not None and lead.status and lead.dRel > 0.0
    if not enabled or brake_pressed or not lead_valid:
      self.reset()
      return False

    v_ego = max(float(v_ego), 0.0)
    d_rel = float(lead.dRel)
    v_rel = float(lead.vRel)
    v_lead = float(getattr(lead, 'vLead', v_ego + v_rel))
    a_lead = float(getattr(lead, 'aLeadK', 0.0))

    unsafe = d_rel < PEDAL_LAUNCH_MIN_DISTANCE or \
             v_rel <= PEDAL_LAUNCH_MAX_CLOSING_VREL or \
             a_lead <= PEDAL_LAUNCH_MAX_LEAD_DECEL
    if unsafe or v_ego >= PEDAL_LAUNCH_EXIT_VEGO - 1e-3:
      self.reset()
      return False

    if self.active:
      self.timeout_frames -= 1
      if self.timeout_frames <= 0:
        self.reset()
        return False
      return True

    if v_ego >= PEDAL_LAUNCH_ENTER_MAX_VEGO - 1e-3:
      self.confirm_frames = 0
      return False

    lead_stationary = v_lead <= PEDAL_LAUNCH_STATIONARY_VLEAD and \
                      abs(v_rel) <= PEDAL_LAUNCH_STATIONARY_VREL
    if lead_stationary:
      # Follow slow radar drift while stopped, then freeze this reference as
      # soon as the lead starts moving.
      if self.stopped_d_rel is None:
        self.stopped_d_rel = d_rel
      else:
        self.stopped_d_rel = 0.95 * self.stopped_d_rel + 0.05 * d_rel
      self.confirm_frames = 0
      return False

    if self.stopped_d_rel is None:
      self.stopped_d_rel = d_rel
    distance_opened = d_rel - self.stopped_d_rel >= PEDAL_LAUNCH_MIN_DISTANCE_DELTA
    lead_departing = v_lead > PEDAL_LAUNCH_MIN_VLEAD or distance_opened
    launch_candidate = lead_departing and v_rel > PEDAL_LAUNCH_MAX_CLOSING_VREL
    self.confirm_frames = self.confirm_frames + 1 if launch_candidate else 0

    if self.confirm_frames >= PEDAL_LAUNCH_CONFIRM_FRAMES:
      self.active = True
      self.confirm_frames = 0
      self.timeout_frames = PEDAL_LAUNCH_TIMEOUT_FRAMES

    return self.active


class PedalDeadzoneBoostController:
  """Apply one short, ramped pedal floor after a confirmed safe recovery.

  The caller owns the lead/gap safety decision. A false candidate immediately
  removes the floor. Once the car responds (or the timeout expires), the boost
  cannot repeat until the candidate becomes false and rearms it.
  """
  def __init__(self):
    self.active = False
    self.completed = False
    self.confirm_frames = 0
    self.active_frames = 0
    self.response_frames = 0
    self.applied_floor = 0.0
    self.start_pedal = 0.0

  def reset(self):
    self.active = False
    self.completed = False
    self.confirm_frames = 0
    self.active_frames = 0
    self.response_frames = 0
    self.applied_floor = 0.0
    self.start_pedal = 0.0

  def _complete(self):
    self.active = False
    self.completed = True
    self.confirm_frames = 0
    self.active_frames = 0
    self.response_frames = 0
    self.applied_floor = 0.0
    self.start_pedal = 0.0

  def update(self, candidate, raw_pedal, a_ego):
    raw_pedal = max(float(raw_pedal), 0.0)
    if not candidate:
      self.reset()
      return raw_pedal

    if self.completed:
      return raw_pedal

    if raw_pedal >= PEDAL_DEADZONE_FLOOR:
      self._complete()
      return raw_pedal

    if not self.active:
      self.confirm_frames += 1
      if self.confirm_frames < PEDAL_DEADZONE_CONFIRM_FRAMES:
        return raw_pedal
      self.active = True
      self.active_frames = 0
      self.response_frames = 0
      self.start_pedal = raw_pedal

    self.active_frames += 1
    ramp = min(float(self.active_frames) / PEDAL_DEADZONE_RAMP_FRAMES, 1.0)
    self.applied_floor = self.start_pedal + (PEDAL_DEADZONE_FLOOR - self.start_pedal) * ramp

    self.response_frames = self.response_frames + 1 if float(a_ego) >= PEDAL_DEADZONE_RESPONSE_ACCEL else 0
    if self.response_frames >= PEDAL_DEADZONE_RESPONSE_CONFIRM_FRAMES or \
       self.active_frames >= PEDAL_DEADZONE_MAX_ACTIVE_FRAMES:
      self._complete()
      return raw_pedal

    return max(raw_pedal, self.applied_floor)


class PedalFollowSmoother:
  def __init__(self):
    self.state = 'cruise'
    self.accel_authority = 1.0
    self.target_distance = 0.0
    self.guard_distance = 0.0
    self.predicted_distance = 0.0
    self.recovery_frames = 0
    self.lead_missing_frames = 0
    self.lead_braking_frames = 0
    self.output_accel = None

  def reset(self):
    self.state = 'cruise'
    self.accel_authority = 1.0
    self.target_distance = 0.0
    self.guard_distance = 0.0
    self.predicted_distance = 0.0
    self.recovery_frames = 0
    self.lead_missing_frames = 0
    self.lead_braking_frames = 0
    self.output_accel = None

  def _update_limited_output(self, requested_accel, desired_accel, can_recover):
    requested_positive = max(float(requested_accel), 0.0)
    desired_accel = float(clip(desired_accel, 0.0, requested_positive))
    if self.output_accel is None:
      self.output_accel = requested_positive

    # A lower request from the planner is always authoritative.
    self.output_accel = min(self.output_accel, requested_positive)
    if desired_accel < self.output_accel:
      self.recovery_frames = 0
      self.output_accel = max(desired_accel,
                              self.output_accel - PEDAL_FOLLOW_OUTPUT_FALL_RATE * DT_CTRL)
    elif desired_accel > self.output_accel:
      self.recovery_frames = self.recovery_frames + 1 if can_recover else 0
      if self.recovery_frames >= PEDAL_FOLLOW_RECOVERY_FRAMES:
        self.output_accel = min(desired_accel,
                                self.output_accel + PEDAL_FOLLOW_OUTPUT_RISE_RATE * DT_CTRL)

    return min(float(requested_accel), self.output_accel)

  def _update_missing_lead(self, requested_accel):
    if self.output_accel is None:
      self.reset()
      return float(requested_accel)

    self.lead_missing_frames += 1
    self.lead_braking_frames = 0
    requested_positive = max(float(requested_accel), 0.0)
    self.output_accel = min(self.output_accel, requested_positive)
    if self.lead_missing_frames <= PEDAL_FOLLOW_LEAD_DROPOUT_HOLD_FRAMES:
      self.state = 'lead_dropout'
      # Radar/vision leads can flicker for a few frames. Preserve the last
      # already-limited safe output instead of turning a short dropout into an
      # additional gas cut that takes seconds to recover from.
    else:
      self.state = 'recovering'
      self.output_accel = min(requested_positive,
                              self.output_accel + PEDAL_FOLLOW_OUTPUT_RISE_RATE * DT_CTRL)
      if self.output_accel >= requested_positive:
        self.reset()
        return float(requested_accel)

    return min(float(requested_accel), self.output_accel)

  def update(self, enabled, requested_accel, lead, v_ego, target_distance=None, launch_active=False):
    requested_accel = float(requested_accel)
    lead_valid = lead is not None and lead.status and lead.dRel > 0.0

    if not enabled:
      self.reset()
      return requested_accel
    if not lead_valid:
      return self._update_missing_lead(requested_accel)

    self.lead_missing_frames = 0

    if pedal_follow_urgent(lead, v_ego):
      self.state = 'urgent'
      self.accel_authority = 0.0
      self.recovery_frames = 0
      self.output_accel = 0.0
      return min(requested_accel, 0.0)

    self.target_distance, self.guard_distance, self.predicted_distance, closing_speed = \
      pedal_follow_geometry(v_ego, lead.dRel, lead.vRel, target_distance=target_distance)

    if launch_active:
      # A separately confirmed standstill departure may bypass the normal
      # recovery debounce. Urgent closing was checked above and the
      # launch detector independently rejects braking or closing leads.
      self.state = 'launch'
      self.accel_authority = 1.0
      self.recovery_frames = 0
      self.output_accel = None
      return requested_accel

    authority_denominator = max(self.target_distance - self.guard_distance,
                                PEDAL_FOLLOW_MIN_AUTHORITY_BAND)
    self.accel_authority = float(clip(
      (self.predicted_distance - self.guard_distance) / authority_denominator, 0.0, 1.0))

    a_lead = float(getattr(lead, 'aLeadK', 0.0))
    lead_braking_candidate = a_lead <= PEDAL_FOLLOW_LEAD_BRAKING_ACCEL and \
                             self.predicted_distance <= self.target_distance + PEDAL_FOLLOW_LEAD_BRAKING_MARGIN
    self.lead_braking_frames = self.lead_braking_frames + 1 if lead_braking_candidate else 0
    lead_braking_confirmed = self.lead_braking_frames >= PEDAL_FOLLOW_LEAD_BRAKING_CONFIRM_FRAMES
    lead_hard_braking = a_lead <= PEDAL_FOLLOW_LEAD_HARD_BRAKING_ACCEL and \
                        self.predicted_distance <= self.target_distance + PEDAL_FOLLOW_LEAD_BRAKING_MARGIN
    if lead_hard_braking or lead_braking_confirmed:
      self.accel_authority = 0.0

    if requested_accel <= 0.0:
      self.recovery_frames = 0
      if self.accel_authority < 1.0:
        self.output_accel = 0.0
        self.state = 'coast'
      else:
        # A normal planner-requested coast must not create an artificial slow
        # recovery when acceleration is requested again at a safe distance.
        self.output_accel = None
        self.state = 'cruise'
      return requested_accel

    if self.accel_authority >= 1.0 and self.output_accel is None:
      self.state = 'cruise'
      return requested_accel

    desired_accel = requested_accel * self.accel_authority
    recovery_safe = self.predicted_distance >= self.guard_distance + PEDAL_FOLLOW_RECOVERY_MARGIN and \
                    closing_speed <= PEDAL_FOLLOW_RECOVERY_MAX_CLOSING
    recovery_hysteresis_safe = self.state == 'recovering' and \
                               self.predicted_distance >= self.guard_distance + PEDAL_FOLLOW_RECOVERY_HYSTERESIS_MARGIN and \
                               closing_speed <= PEDAL_FOLLOW_RECOVERY_HYSTERESIS_MAX_CLOSING
    can_recover = recovery_safe or recovery_hysteresis_safe
    previous_output = self.output_accel
    output = self._update_limited_output(requested_accel, desired_accel, can_recover)

    if self.accel_authority <= 0.01:
      self.state = 'coast'
    elif previous_output is not None and self.output_accel > previous_output:
      self.state = 'recovering'
    else:
      self.state = 'settle'

    if self.accel_authority >= 1.0 and self.output_accel >= requested_accel:
      self.output_accel = None
      self.recovery_frames = 0
      self.state = 'cruise'

    return output
