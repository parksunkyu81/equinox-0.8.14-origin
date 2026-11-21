"""Acceleration ceiling while following a lead in flowing traffic.

On this car openpilot cannot brake: speed it builds toward a lead comes off
only by coasting (0.15-0.4 m/s^2 learned on the road) or by the driver's
brake. The MPC does not know that -- with a lead it plans as if it could brake
at up to 3.5 m/s^2 -- so it accelerates freely and the driver pays for it.
On the 11-16..18 drives, 69 brake presses in flowing traffic (no other pedal
in the 3 s before) followed an accel command above 0.5 m/s^2: 23 with a
steady lead 27 m ahead and the car pulling at ~1.0 m/s^2, 7 where the car
had built up 5 km/h of closing speed on a lead ~49 m ahead, 20 with no lead.

Two ceilings, taken together, from FULL_KPH up (faded in from MIN_KPH, so
the jam launch logic below 25 km/h is untouched):

* Brake-free: the largest accel that, held for REACT_S and then coasted off
  with the learned coast deceleration (scaled down by the learned lift
  margin), still sheds the closing speed before the floor gap. With the lead
  holding its speed. This is what stops the car building closing speed it
  cannot lose.

* Gentle: near the top of the gap band the ceiling is GENTLE_ACCEL, opening
  up to OPEN_ACCEL GENTLE_SPAN_S further out. Not applied while the lead is
  clearly pulling away (PULL_AWAY_VREL_MS), so the car still follows a lead
  that leaves and the gap does not open up for a cut-in.

The ceiling is applied twice: to the planner (so the plan does not run ahead
of what the car will be allowed) and to the positive limit of LongControl's
speed PID in controlsd. The second is the one that matters: before those 69
presses the accel command averaged 1.07 m/s^2 while the plan asked 0.40 --
the car lagged the plan (aEgo 0.25, vPid 1.3 km/h ahead) and the PID
integrator had wound up to 0.45. At the PID limit the integrator also stops
growing. Neither ceiling is lowered faster than EASE_MPS3, so the pedal eases
off instead of dropping.

On the PID the ceiling sits on top of the speed-hold level (SpeedHoldOffset).
The PID's output is a pedal command in accel units, and above 60 km/h holding
speed on the flat already costs 1.0-1.2 of it (2026-11-21 rlogs). An absolute
1.2 ceiling therefore pinned the pedal at 0.205 with a lead 60-110 m ahead
(17.7% of the below-cruise time over 60 km/h, aEgo +0.08) and lost speed on
every climb, with the lead anywhere in sight (past the gentle span the
ceiling stays at OPEN_ACCEL). It is faded in from 40 to 60 km/h
(HOLD_FADE_KPH), so town following keeps the absolute ceiling.
"""
import json
import math

from common.numpy_fast import clip, interp

MIN_KPH = 30.0
FULL_KPH = 35.0
REACT_S = 1.0
STANDSTILL_GAP_M = 4.5        # same as predictive coasting's desired gap
GENTLE_ACCEL = 0.5
OPEN_ACCEL = 1.2
GENTLE_SPAN_S = 2.0
PULL_AWAY_VREL_MS = 1.0
EASE_MPS3 = 1.0
A_SEARCH_MAX = 2.5
# Speed-hold level: low-passed (PID command - aEgo), the part of the command
# that holds speed against drag and grade instead of accelerating the car.
HOLD_TAU_S = 3.0
HOLD_MAX = 2.0
HOLD_MIN_SPEED_MS = 5.0
# Added on top of the ceiling only from 40 km/h, in full from 60 km/h. In town
# the absolute ceiling is what kept the 11-16..18 brake presses down; it is
# the highway, where holding speed alone uses up the 1.2, that it starved.
HOLD_FADE_KPH = (40.0, 60.0)

# Mirror of follow_learner.py: learned coast decel per speed bin, trusted from
# COAST_TRUST_SAMPLES samples, else this default curve.
COAST_BINS_KPH = (10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130)
DEFAULT_COAST_BP_KPH = (0.0, 20.0, 60.0, 100.0, 130.0)
DEFAULT_COAST_MS2 = (0.30, 0.35, 0.50, 0.70, 0.80)
COAST_TRUST_SAMPLES = 40
COAST_FLOOR_MS2 = 0.1


def default_coast_curve():
  return [float(interp(b, DEFAULT_COAST_BP_KPH, DEFAULT_COAST_MS2)) for b in COAST_BINS_KPH]


def read_follow_learn(raw):
  """(coast curve per COAST_BINS_KPH, lift margin) from a saved CommaFollowLearn value."""
  curve, k = default_coast_curve(), 1.0
  try:
    if isinstance(raw, bytes):
      raw = raw.decode('utf8')
    d = json.loads(raw)
    coast = d.get('coast', [])
    if len(coast) == len(COAST_BINS_KPH):
      curve = [float(m) if int(n) >= COAST_TRUST_SAMPLES and math.isfinite(float(m)) else c
               for (m, n), c in zip(coast, curve)]
    k = float(clip(float(d.get('k', 1.0)), 0.8, 1.5))
    if not math.isfinite(k):
      k = 1.0
  except Exception:
    pass
  return curve, k


def coast_decel(curve, v_ego):
  return max(COAST_FLOOR_MS2, float(interp(max(0.0, float(v_ego)) * 3.6, COAST_BINS_KPH, curve)))


def _brake_free_ok(a, v_ego, d_rel, v_rel, coast, floor_tr_s, k):
  t = REACT_S
  v_rel_t = v_rel - a * t                       # lead holds its speed
  if v_rel_t >= 0.0:
    return True
  d_t = d_rel + v_rel * t - 0.5 * a * t * t
  v_t = v_ego + a * t
  margin = d_t - (STANDSTILL_GAP_M + v_t * floor_tr_s)
  if margin <= 0.0:
    return False
  return k * v_rel_t * v_rel_t / (2.0 * margin) <= coast


def brake_free_accel(v_ego, d_rel, v_rel, coast, floor_tr_s, k=1.0):
  """Largest accel in [0, A_SEARCH_MAX] whose closing speed coasting can still shed (0 if none)."""
  if _brake_free_ok(A_SEARCH_MAX, v_ego, d_rel, v_rel, coast, floor_tr_s, k):
    return A_SEARCH_MAX
  if not _brake_free_ok(0.0, v_ego, d_rel, v_rel, coast, floor_tr_s, k):
    return 0.0
  lo, hi = 0.0, A_SEARCH_MAX
  for _ in range(12):
    mid = 0.5 * (lo + hi)
    if _brake_free_ok(mid, v_ego, d_rel, v_rel, coast, floor_tr_s, k):
      lo = mid
    else:
      hi = mid
  return lo


def gentle_accel(headway_s, band_top_s, v_rel):
  if v_rel >= PULL_AWAY_VREL_MS:
    return math.inf
  return float(interp(headway_s, [band_top_s, band_top_s + GENTLE_SPAN_S], [GENTLE_ACCEL, OPEN_ACCEL]))


def hold_weight(v_ego):
  return float(interp(float(v_ego) * 3.6, HOLD_FADE_KPH, [0.0, 1.0]))


class SpeedHoldOffset:
  """How much of the speed PID's command only holds speed (drag + grade).

  Learned while openpilot drives the pedal itself; frozen otherwise, so a
  driver press or a stop does not drag it around."""

  def __init__(self, dt):
    self.alpha = dt / (HOLD_TAU_S + dt)
    self.x = 0.0

  def reset(self):
    self.x = 0.0

  def update(self, learning, command, a_ego, v_ego):
    if learning and v_ego > HOLD_MIN_SPEED_MS and math.isfinite(command) and math.isfinite(a_ego):
      sample = clip(float(command) - float(a_ego), 0.0, HOLD_MAX)
      self.x += self.alpha * (sample - self.x)
    return self.x


def follow_accel_cap(v_ego, d_rel, v_rel, coast, lift_margin, floor_tr_s, band_top_s):
  """Acceleration ceiling for following this lead, math.inf when it does not apply.

  coast: coast deceleration at this speed (positive m/s^2)."""
  v_ego = float(v_ego)
  w = float(interp(v_ego * 3.6, [MIN_KPH, FULL_KPH], [0.0, 1.0]))
  if w <= 0.0 or not (d_rel > 0.0) or not all(map(math.isfinite, (v_ego, d_rel, v_rel, coast))):
    return math.inf
  coast = max(COAST_FLOOR_MS2, float(coast))
  cap = min(brake_free_accel(v_ego, d_rel, v_rel, coast, floor_tr_s, lift_margin),
            gentle_accel(d_rel / max(v_ego, 1.0), band_top_s, v_rel))
  if not math.isfinite(cap):
    return math.inf
  # Faded in over MIN_KPH..FULL_KPH: blend toward "no cap" (OPEN_ACCEL above).
  return cap + (1.0 - w) * max(0.0, A_SEARCH_MAX - cap)
