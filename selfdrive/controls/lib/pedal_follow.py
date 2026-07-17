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
PEDAL_FOLLOW_OUTPUT_RISE_RATE = 0.60  # m/s^3
PEDAL_FOLLOW_RECOVERY_MARGIN = 2.0
PEDAL_FOLLOW_RECOVERY_MAX_CLOSING = 0.20
PEDAL_FOLLOW_RECOVERY_FRAMES = max(1, round(0.30 / DT_CTRL))
PEDAL_FOLLOW_LEAD_DROPOUT_HOLD_FRAMES = max(1, round(0.50 / DT_CTRL))
PEDAL_FOLLOW_LEAD_BRAKING_ACCEL = -0.30
PEDAL_FOLLOW_LEAD_BRAKING_MARGIN = 4.0
PEDAL_FOLLOW_URGENT_VREL = -0.80
PEDAL_FOLLOW_URGENT_TTC = 3.0
PEDAL_FOLLOW_URGENT_MIN_DISTANCE = 5.5
PEDAL_FOLLOW_URGENT_HEADWAY = 0.8


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


class PedalFollowSmoother:
  def __init__(self):
    self.state = 'cruise'
    self.accel_authority = 1.0
    self.target_distance = 0.0
    self.guard_distance = 0.0
    self.predicted_distance = 0.0
    self.recovery_frames = 0
    self.lead_missing_frames = 0
    self.output_accel = None

  def reset(self):
    self.state = 'cruise'
    self.accel_authority = 1.0
    self.target_distance = 0.0
    self.guard_distance = 0.0
    self.predicted_distance = 0.0
    self.recovery_frames = 0
    self.lead_missing_frames = 0
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
    requested_positive = max(float(requested_accel), 0.0)
    self.output_accel = min(self.output_accel, requested_positive)
    if self.lead_missing_frames <= PEDAL_FOLLOW_LEAD_DROPOUT_HOLD_FRAMES:
      self.state = 'lead_dropout'
      self.output_accel = max(0.0,
                              self.output_accel - PEDAL_FOLLOW_OUTPUT_FALL_RATE * DT_CTRL)
    else:
      self.state = 'recovering'
      self.output_accel = min(requested_positive,
                              self.output_accel + PEDAL_FOLLOW_OUTPUT_RISE_RATE * DT_CTRL)
      if self.output_accel >= requested_positive:
        self.reset()
        return float(requested_accel)

    return min(float(requested_accel), self.output_accel)

  def update(self, enabled, requested_accel, lead, v_ego, target_distance=None):
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

    authority_denominator = max(self.target_distance - self.guard_distance,
                                PEDAL_FOLLOW_MIN_AUTHORITY_BAND)
    self.accel_authority = float(clip(
      (self.predicted_distance - self.guard_distance) / authority_denominator, 0.0, 1.0))

    a_lead = float(getattr(lead, 'aLeadK', 0.0))
    if a_lead <= PEDAL_FOLLOW_LEAD_BRAKING_ACCEL and \
       self.predicted_distance <= self.target_distance + PEDAL_FOLLOW_LEAD_BRAKING_MARGIN:
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
    can_recover = self.predicted_distance >= self.guard_distance + PEDAL_FOLLOW_RECOVERY_MARGIN and \
                  closing_speed <= PEDAL_FOLLOW_RECOVERY_MAX_CLOSING
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
