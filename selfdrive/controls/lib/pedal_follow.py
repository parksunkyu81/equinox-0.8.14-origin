from common.numpy_fast import clip
from common.realtime import DT_CTRL


# Pedal-only GM longitudinal control cannot command the brakes. A far-away
# slower lead must not cause repeated pedal release/reapply cycles. Enter one
# latched coast phase only inside the brake-free coast window, then continuously
# release gas until the lead has clearly opened the gap again.
PEDAL_FOLLOW_MAX_ACCEL = 1.0
PEDAL_FOLLOW_OUTPUT_FALL_RATE = 0.65  # m/s^3
PEDAL_FOLLOW_OUTPUT_RISE_RATE = 0.35  # m/s^3
PEDAL_FOLLOW_COAST_MIN_DISTANCE = 5.5
PEDAL_FOLLOW_COAST_HEADWAY = 0.90
PEDAL_FOLLOW_COAST_CLOSING_TIME = 2.0
PEDAL_FOLLOW_COAST_ENTER_VREL = -0.15
PEDAL_FOLLOW_COAST_EXIT_VREL = 0.30
PEDAL_FOLLOW_COAST_ENTER_FRAMES = max(1, round(0.25 / DT_CTRL))
PEDAL_FOLLOW_COAST_EXIT_FRAMES = max(1, round(0.80 / DT_CTRL))
PEDAL_FOLLOW_LEAD_DROPOUT_HOLD_FRAMES = max(1, round(0.50 / DT_CTRL))
PEDAL_FOLLOW_URGENT_VREL = -0.80
PEDAL_FOLLOW_URGENT_TTC = 3.0
PEDAL_FOLLOW_URGENT_MIN_DISTANCE = 5.5
PEDAL_FOLLOW_URGENT_HEADWAY = 0.8


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
    self.coast_active = False
    self.recovering = False
    self.coast_enter_frames = 0
    self.coast_exit_frames = 0
    self.lead_missing_frames = 0
    self.output_accel = None

  def reset(self):
    self.coast_active = False
    self.recovering = False
    self.coast_enter_frames = 0
    self.coast_exit_frames = 0
    self.lead_missing_frames = 0
    self.output_accel = None

  @staticmethod
  def coast_distance(v_ego, v_rel):
    closing_speed = max(-float(v_rel), 0.0)
    return PEDAL_FOLLOW_COAST_MIN_DISTANCE + \
           PEDAL_FOLLOW_COAST_HEADWAY * max(float(v_ego), 0.0) + \
           PEDAL_FOLLOW_COAST_CLOSING_TIME * closing_speed

  def update(self, enabled, requested_accel, lead, v_ego):
    requested_accel = float(requested_accel)
    lead_valid = lead is not None and lead.status and lead.dRel > 0.0
    if enabled and self.coast_active and not lead_valid and \
       self.lead_missing_frames < PEDAL_FOLLOW_LEAD_DROPOUT_HOLD_FRAMES:
      self.lead_missing_frames += 1
      if self.output_accel is None:
        self.output_accel = float(clip(requested_accel, 0.0, PEDAL_FOLLOW_MAX_ACCEL))
      self.output_accel = max(0.0, self.output_accel - PEDAL_FOLLOW_OUTPUT_FALL_RATE * DT_CTRL)
      return min(requested_accel, self.output_accel)

    if not enabled or not lead_valid:
      self.reset()
      return requested_accel

    self.lead_missing_frames = 0

    if pedal_follow_urgent(lead, v_ego):
      self.coast_active = True
      self.recovering = False
      self.coast_enter_frames = 0
      self.coast_exit_frames = 0
      self.output_accel = 0.0
      return min(requested_accel, 0.0)

    raw_v_rel = float(lead.vRel)
    coast_distance = self.coast_distance(v_ego, raw_v_rel)

    if self.coast_active:
      safe_follow_distance = PEDAL_FOLLOW_COAST_MIN_DISTANCE + \
                             PEDAL_FOLLOW_COAST_HEADWAY * max(float(v_ego), 0.0)
      can_exit = raw_v_rel >= PEDAL_FOLLOW_COAST_EXIT_VREL and \
                 float(lead.dRel) >= safe_follow_distance
      self.coast_exit_frames = self.coast_exit_frames + 1 if can_exit else 0
      if self.coast_exit_frames >= PEDAL_FOLLOW_COAST_EXIT_FRAMES:
        self.coast_active = False
        self.recovering = True
        self.coast_exit_frames = 0
    else:
      can_enter = raw_v_rel <= PEDAL_FOLLOW_COAST_ENTER_VREL and \
                  float(lead.dRel) <= coast_distance
      self.coast_enter_frames = self.coast_enter_frames + 1 if can_enter else 0
      if self.coast_enter_frames >= PEDAL_FOLLOW_COAST_ENTER_FRAMES:
        self.coast_active = True
        self.recovering = False
        self.coast_enter_frames = 0
        self.output_accel = float(clip(requested_accel, 0.0, PEDAL_FOLLOW_MAX_ACCEL))

    if self.coast_active:
      if self.output_accel is None:
        self.output_accel = float(clip(requested_accel, 0.0, PEDAL_FOLLOW_MAX_ACCEL))
      self.output_accel = max(0.0, self.output_accel - PEDAL_FOLLOW_OUTPUT_FALL_RATE * DT_CTRL)
      # Never exceed a lower request from the main longitudinal planner.
      return min(requested_accel, self.output_accel)

    if not self.recovering:
      self.output_accel = float(clip(requested_accel, 0.0, PEDAL_FOLLOW_MAX_ACCEL))
      return requested_accel

    target_accel = float(clip(requested_accel, 0.0, PEDAL_FOLLOW_MAX_ACCEL))
    if self.output_accel is None:
      self.output_accel = 0.0
    self.output_accel = min(target_accel,
                            self.output_accel + PEDAL_FOLLOW_OUTPUT_RISE_RATE * DT_CTRL)
    if self.output_accel >= target_accel:
      self.recovering = False

    return min(requested_accel, self.output_accel)
