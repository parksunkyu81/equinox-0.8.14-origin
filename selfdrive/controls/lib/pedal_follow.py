from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL


# Pedal-only GM longitudinal control cannot command the brakes. Mild closing
# should therefore taper positive acceleration, while an urgent approach or a
# driver brake must still cut gas immediately.
PEDAL_FOLLOW_MAX_ACCEL = 1.0
PEDAL_FOLLOW_VREL_BP = [-0.80, -0.45, -0.20, 0.05]
PEDAL_FOLLOW_ACCEL_CAP_V = [0.0, 0.18, 0.45, PEDAL_FOLLOW_MAX_ACCEL]
PEDAL_FOLLOW_CAP_FALL_RATE = 1.20  # m/s^3
PEDAL_FOLLOW_CAP_RISE_RATE = 0.55  # m/s^3
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
  return lead.vRel <= PEDAL_FOLLOW_URGENT_VREL or \
         (lead.dRel <= urgent_distance and ttc <= PEDAL_FOLLOW_URGENT_TTC)


class PedalFollowSmoother:
  def __init__(self):
    self.accel_cap = PEDAL_FOLLOW_MAX_ACCEL

  def reset(self):
    self.accel_cap = PEDAL_FOLLOW_MAX_ACCEL

  def update(self, enabled, requested_accel, lead, v_ego):
    requested_accel = float(requested_accel)
    if not enabled or lead is None or not lead.status or lead.dRel <= 0.0:
      self.reset()
      return requested_accel

    if pedal_follow_urgent(lead, v_ego):
      self.accel_cap = 0.0
      return min(requested_accel, 0.0)

    target_cap = interp(float(lead.vRel), PEDAL_FOLLOW_VREL_BP, PEDAL_FOLLOW_ACCEL_CAP_V)

    # When the gap is already short, react to a slower lead a little earlier
    # without requesting braking or stronger acceleration.
    if lead.vRel < 0.05:
      headway = float(lead.dRel) / max(float(v_ego), 1.0)
      headway_mod = interp(headway, [0.6, 1.2], [0.55, 1.0])
      target_cap *= headway_mod

    if target_cap < self.accel_cap:
      self.accel_cap = max(target_cap, self.accel_cap - PEDAL_FOLLOW_CAP_FALL_RATE * DT_CTRL)
    else:
      self.accel_cap = min(target_cap, self.accel_cap + PEDAL_FOLLOW_CAP_RISE_RATE * DT_CTRL)

    self.accel_cap = float(clip(self.accel_cap, 0.0, PEDAL_FOLLOW_MAX_ACCEL))
    return min(requested_accel, self.accel_cap)
