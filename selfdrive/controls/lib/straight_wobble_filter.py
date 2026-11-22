"""Straight-road wobble filter for the WideOptechLateral steering demand.

The 0.9-style MPC on the pure model path (no lane blend) follows every frame of
model jitter, so on straights the demand moves more than today's lane-blended
plan: replayed on 11-21 corner segments, frame-to-frame step p95 was 0.056 m/s^2
against 0.035 logged. This filter smooths only small moves near zero:

  corner weight  w_a = clip((|a_raw| - A0) / (A1 - A0), 0, 1)
  change weight  w_d = clip((|a_raw - a_prev| - D0) / (D1 - D0), 0, 1)
  tau            = TAU * (1 - max(w_a, w_d))
  a_out          = a_prev + dt / (tau + dt) * (a_raw - a_prev)

with a = curvature * v^2. A corner (|a| above A1) or any real change (more than
D1 away from the filtered value) passes through untouched, so corner entry is
not delayed and, unlike a plain low-pass, corner exit is not held: a magnitude-
only version kept the wheel turned after corners and raised straight |a| p95
from 0.227 to 0.255.

Tuned on the same replay (wobble_filter_tune.py, 15 segments, 8 tight corners):
step p95 0.056 -> 0.018 (logged 0.035), straight |a| p95 0.227 -> 0.229, corner
MAE 0.146 -> 0.148 m/s^2, corner onset unchanged (+0.03 s).
"""
import math

from common.numpy_fast import clip
from common.realtime import DT_MDL

TAU_S = 0.15
CORNER_A0 = 0.25      # m/s^2, full smoothing below
CORNER_A1 = 0.80      # m/s^2, no smoothing above
CHANGE_D0 = 0.04      # m/s^2 away from the filtered value: full smoothing below
CHANGE_D1 = 0.20      # m/s^2: passes at once above
MIN_SPEED = 1.0       # m/s, curvature <-> lateral accel conversion floor


class StraightWobbleFilter:
  """Steps once per lateral plan (20 Hz, the rate it was tuned at) and holds in between.

  controlsd runs at 100 Hz but the demand only changes when a new plan arrives;
  stepping on every control frame would let the change gate close while the
  filtered value is still catching up, which is not the filter that was tuned.
  """

  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.reset()

  def reset(self):
    self.prev_accel = None
    self.last_weight = 1.0

  def update(self, v_ego, curvature, active, new_plan):
    """Return the filtered desired curvature.

    new_plan: True on the control frame a new lateralPlan arrived. Inactive or
    invalid input passes through unchanged and resets the filter.
    """
    try:
      curvature = float(curvature)
      v = max(float(v_ego), MIN_SPEED)
    except (TypeError, ValueError):
      self.reset()
      return curvature
    if not active or not math.isfinite(curvature) or not math.isfinite(v):
      self.reset()
      return curvature

    a_raw = curvature * v * v
    if self.prev_accel is None:
      self.prev_accel = a_raw
      self.last_weight = 1.0
      return curvature
    if not new_plan:
      return self.prev_accel / (v * v)

    w_a = clip((abs(a_raw) - CORNER_A0) / (CORNER_A1 - CORNER_A0), 0.0, 1.0)
    w_d = clip((abs(a_raw - self.prev_accel) - CHANGE_D0) / (CHANGE_D1 - CHANGE_D0), 0.0, 1.0)
    w = max(w_a, w_d)
    tau = TAU_S * (1.0 - w)
    self.prev_accel += self.dt / (tau + self.dt) * (a_raw - self.prev_accel)
    self.last_weight = w
    return self.prev_accel / (v * v)
