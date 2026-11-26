"""One following gap, tuned for a car openpilot cannot brake.

On this car openpilot drives the comma pedal only; every deceleration beyond
coasting is the driver's brake. A gap is therefore only as good as the speed
it lets the car shed by coasting before the driver has to brake. There are no
selectable profiles: the gap is an elastic band per speed.

* floor_tr(v): the closest the car may run, from the brake-free analysis of a
  lead slowing by 10 km/h over 3 s with the car coasting after 0.8 s. Below
  20 km/h it stays at 0.9 s -- stops there need the brake whatever the gap,
  and a wider gap only invites cut-ins. 30-70 km/h is raised to 1.0 s, where
  the gap decides most brake presses. From 100 km/h the earlier speed floor
  already had margin and is kept.

* The band runs from the floor to floor + width * scale. Inside it the gap is
  allowed to drift. When the lead pulls away the target leans to the middle of
  the band and never rises, so the car follows it. (It used to rise with the
  gap so the car would not chase; on the 11-16/17 drives the driver then
  pressed the gas 291 times in 86 engaged minutes, 41% of them with the lead
  pulling away or the gap already near the target.) When the lead closes or
  slows, the gap target is held or raised, never lowered, so the planner keeps
  easing off instead of redefining the shrinking gap as fine.

* scale comes from FollowLearner (controlsd): gas presses while following
  shrink it, brake presses while closing on a moving lead widen it.

The returned value is the MPC's desired TR. The MPC's own distance law is
TR * v + v^2/(2 b) + STOP_DISTANCE against the lead's v_lead^2/(2 b), so the
"current gap" compared with the band is that same law solved for TR.
"""
import json
import math

from common.numpy_fast import clip, interp

# Where FollowLearner (controlsd) saves what it learned; plannerd reads
# gap_scale back from it. Lives here so neither side imports the other.
FOLLOW_LEARN_PARAM = "CommaFollowLearn"

FLOOR_BP_KPH = (0.0, 20.0, 30.0, 60.0, 70.0, 80.0, 100.0, 120.0, 140.0)
FLOOR_TR_S = (0.9, 0.9, 1.0, 1.0, 1.0, 1.05, 1.1, 1.2, 1.3)
MIN_FOLLOW_TR = FLOOR_TR_S[0]

BAND_BP_KPH = (0.0, 30.0, 60.0, 100.0, 140.0)
BAND_WIDTH_S = (0.4, 0.5, 0.5, 0.4, 0.3)

SCALE_MIN = 0.6   # was 0.8, and the learner sat pinned there (gas presses kept pushing)
SCALE_MAX = 1.3

# Mirrors of long_mpc.py (importing it here would be circular).
MPC_COMFORT_BRAKE = 2.5
MPC_STOP_DISTANCE = 5.5

CLOSING_VREL_MS = -0.3
OPENING_VREL_MS = 0.3
LEAD_SLOWING_MS2 = -0.5
STABLE_DRIFT_S = 0.05          # how far off the current gap the stable target leans toward mid
TR_RISE_PER_S = 0.30
TR_FALL_PER_S = 0.25   # 1.4 -> 1.1 s in ~1.2 s (was 3 s at 0.10)
NEW_LEAD_JUMP_M = 5.0


def read_gap_scale(raw):
  """gap_scale from a saved CommaFollowLearn value, 1.0 if missing or unreadable."""
  try:
    if isinstance(raw, bytes):
      raw = raw.decode('utf8')
    s = float(json.loads(raw).get('s', 1.0))
    return float(clip(s, SCALE_MIN, SCALE_MAX)) if math.isfinite(s) else 1.0
  except Exception:
    return 1.0


def floor_tr(v_ego):
  v = float(v_ego) if math.isfinite(float(v_ego)) else 0.0
  return float(interp(max(0.0, v) * 3.6, FLOOR_BP_KPH, FLOOR_TR_S))


def band(v_ego, scale=1.0):
  """(low, mid, high) time gaps for this speed."""
  low = floor_tr(v_ego)
  width = float(interp(max(0.0, float(v_ego)) * 3.6, BAND_BP_KPH, BAND_WIDTH_S))
  width *= float(clip(scale, SCALE_MIN, SCALE_MAX))
  return low, low + 0.5 * width, low + width


def equivalent_tr(x_lead, v_lead, v_ego):
  """The TR at which the MPC would call the current gap exactly right."""
  d = x_lead + (v_lead ** 2 - v_ego ** 2) / (2.0 * MPC_COMFORT_BRAKE) - MPC_STOP_DISTANCE
  return d / max(v_ego, 1.0)


class FollowGap:
  def __init__(self):
    self.tr = None
    self._prev_x = None
    self.phase = "no_lead"
    self.eq_tr = 0.0

  def reset(self):
    self.tr = None
    self._prev_x = None
    self.phase = "no_lead"

  def update(self, v_ego, lead_status, x_lead, v_lead, a_lead, scale, dt):
    v_ego = max(0.0, float(v_ego))
    low, mid, high = band(v_ego, scale)
    if self.tr is None:
      self.tr = mid

    if not lead_status or x_lead is None or v_lead is None:
      self._prev_x = None
      self.phase = "no_lead"
      target = mid
    else:
      x_lead = float(x_lead)
      v_lead = float(v_lead)
      a_lead = float(a_lead or 0.0)
      v_rel = v_lead - v_ego
      eq = equivalent_tr(x_lead, v_lead, v_ego)
      self.eq_tr = eq
      new_lead = self._prev_x is None or \
        abs(x_lead - self._prev_x - v_rel * dt) > NEW_LEAD_JUMP_M
      self._prev_x = x_lead
      if new_lead:
        # Start from the gap the new lead actually left, inside the band.
        self.tr = float(clip(eq, low, high))

      if v_rel < CLOSING_VREL_MS or a_lead < LEAD_SLOWING_MS2:
        self.phase = "closing"
        target = max(self.tr, mid) if a_lead < LEAD_SLOWING_MS2 else self.tr
      elif v_rel > OPENING_VREL_MS:
        self.phase = "opening"
        # Follow a lead that pulls away: lean to the middle of the band, never
        # widen. The planner then sees the growing gap as too big and closes it.
        target = min(self.tr, mid)
      else:
        self.phase = "stable"
        target = float(clip(eq + clip(mid - eq, -STABLE_DRIFT_S, STABLE_DRIFT_S), low, high))

    step = float(clip(target - self.tr, -TR_FALL_PER_S * dt, TR_RISE_PER_S * dt))
    self.tr = float(clip(self.tr + step, low, high))
    return self.tr
