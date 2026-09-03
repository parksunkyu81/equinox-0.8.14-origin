"""Official-style lane path blending with EON spatial-model compatibility."""

import numpy as np

from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.params import Params
from common.realtime import DT_MDL
from selfdrive.ntune import ntune_common_get
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lane_probability import (
  LANE_PATH_DISAGREE_M,
  LANE_PATH_DISAGREE_X_M,
  combined_lane_probability,
  enhance_lane_probability,
  lane_head_trust_cap,
  limit_lane_probability_rise,
)
from selfdrive.controls.lib.model_data_validation import as_finite_vector


TRAJECTORY_SIZE = 33
PATH_OFFSET = ntune_common_get('pathOffset')
# Lateral trim. Both lane lines get this added, so it moves the lane centre the
# planner steers to: more negative puts the car further left.
#
# Sized from 2026-08-26--09-33-54, engaged and hands-off, both lane lines above
# 0.5 probability, |curvature| < 0.003, n = 4359: the lane centre sat at
# -0.0148 m, i.e. the car rode 1.5 cm right of centre (3.6 cm at 30-50 km/h,
# 1.3 cm at 50-80). Mean d_prob on those frames was 0.983 and the final path is
#   d_prob * lane_path + (1 - d_prob) * model_path
# so this offset reaches the output at 98.3% while ntune's pathOffset, which
# only enters the model_path term, reaches it at 1.7% -- correcting 1.5 cm that
# way would need pathOffset = -0.86, near its +-1.0 clamp. Hence the trim lives
# here and pathOffset stays 0.
#
# -0.055 -> -0.070 shifts the car 1.5 cm left (-0.0148 / 0.983). Each further
# -0.010 is roughly another 1 cm left.
#
# Caveat: this is measured against the model's OWN lane lines, so it corrects
# where the car sits relative to what the model sees. A bias in the lane-line
# estimate itself (narrow EON FOV) would not show up in that measurement.
CAMERA_OFFSET = -0.070

# Official 0.8.14 uses a soft standard-deviation weight from 0.15 to 0.30.
# The EON model observed on this car reports geometrically continuous lane
# lines around 0.4-0.8, so retain a soft (bounded) contribution in that range.
LANE_STD_FULL_CONFIDENCE = 0.15
LANE_STD_ZERO_CONFIDENCE = 1.20
LANE_WIDTH_CHECK_DISTANCES_M = (5.0, 10.0, 20.0)
LANE_WIDTH_MIN_START_M = 1.8
# Was 2.5, which sat in the middle of this camera's own width distribution
# instead of below it. Measured on 2026-08-25--10-51-17 over 1150 frames where
# both ego lane lines are >=0.5 prob and v_ego >= 5 m/s (i.e. the lane is
# clearly visible and the car is really driving): median width reads 2.41 m at
# 5 m, 2.41 m at 10 m and 2.43 m at 20 m, p10 2.28 m. The consistency across
# all three distances makes this a systematic lateral-scale offset of this
# camera/model pair, not noisy geometry -- a genuinely too-narrow lane would
# read proportionally narrower still. At 2.5 m the ramp discounted 74.5% of
# those frames (median width_mod 0.855, so a ~15% haircut on lane confidence
# on top of already-weak raw probabilities) purely for being this camera.
# 2.2 m sits just under the device's p10, so it still rejects real outliers
# (4.8% of frames) and matches STORE_LANE_WIDTH_MIN_M, whose identical
# "this threshold is mis-placed for this camera" finding is written up in
# get_d_path's store_geometry_plausible comment.
LANE_WIDTH_MIN_END_M = 2.2
# Cache-only width floor; see store_geometry_plausible in get_d_path. Now
# equal to LANE_WIDTH_MIN_END_M above, kept as its own name because it gates
# a different decision (what to remember, not what to trust right now).
STORE_LANE_WIDTH_MIN_M = 2.2
LANE_WIDTH_MOD_START_M = 4.2
LANE_WIDTH_MOD_END_M = 5.0

# Low-speed EON fallback. A single continuous lane line may briefly anchor the
# path, but its influence is capped so a weak prediction cannot pull the car
# far away from the model path.
LOW_SPEED_FALLBACK_MAX_MS = 12.0
CURVE_FALLBACK_MAX_MS = 16.0
# Lane width is only re-measured while both lines are genuinely visible, and is
# then held rather than surrendered to the speed-based default the moment
# confidence dips. Measured on 2026-08-26--12-34-51: through curves at
# |curv| >= 0.008 the two lines actually showed a 3.36 m lane, but l_prob *
# r_prob fell to about 0.07 (0.25 x 0.28), so the published width collapsed onto
# speed_lane_width -- 2.98 m at that speed, a distribution 2.93-3.06 m wide,
# i.e. the measurement was not being used at all.
#
# That 0.38 m shortfall is a centring error, not a cosmetic one: the lane centre
# is derived as (visible line) +/- lane_width / 2, so with one line dominating it
# puts the centre 19 cm too far toward that line. Through those curves the
# dominant line was the OUTER one, and the car sat a median 28.8 cm wide.
LANE_WIDTH_TRUST_PROB = 0.25
LANE_WIDTH_HOLD_S = 8.0
LANE_WIDTH_DECAY_S = 8.0

SINGLE_LANE_MIN_RAW_PROB = 0.20
SINGLE_LANE_MAX_STD = 0.90
SINGLE_LANE_DPROB_FLOOR = 0.30
CURVE_SINGLE_LANE_DPROB_FLOOR = 0.60
LANE_CONFIDENCE_FALL_RATE_PER_S = 1.25
LANE_CENTER_CONTINUITY_MAX_M = 0.35
CURVE_LANE_CENTER_CONTINUITY_MAX_M = 0.55
CURVE_CONFIDENCE_RISE_BONUS_PER_S = 1.00
# Measured on this car: real curves keep bending for several seconds while
# the narrow EON FOV loses lane confidence well before the curve ends. A full
# 1.25/s fall rate collapses d_path back toward the raw (under-curving) model
# path only 0.5-0.8 s after confidence first dips, well before the curve is
# over. Slow the fall while curve_assist is active so partial/weak lane
# signal keeps contributing longer; still fall at at least the floor rate so
# a genuinely straight, lane-lost road recovers to the model path.
CURVE_CONFIDENCE_FALL_SLOWDOWN_PER_S = 0.90
CURVE_CONFIDENCE_MIN_FALL_RATE_PER_S = 0.35
CURVE_ASSIST_FULL_BELOW_MS = 12.5  # Full assist through 45 km/h
CURVE_ASSIST_ZERO_ABOVE_MS = 16.67  # Back to normal by 60 km/h
CURVE_ASSIST_START_CURVATURE = 0.0075
CURVE_ASSIST_FULL_CURVATURE = 0.025
CURVE_ASSIST_START_BEND_M = 0.30
CURVE_ASSIST_FULL_BEND_M = 1.50
LOW_CONFIDENCE_MAX_CORRECTION_NEAR_M = 0.08
LOW_CONFIDENCE_MAX_CORRECTION_M = 0.35
CURVE_LOW_CONFIDENCE_MAX_CORRECTION_NEAR_M = 0.20
CURVE_LOW_CONFIDENCE_MAX_CORRECTION_M = 0.70

# Unconditional bound on ordinary lane blending. The correction clamp above is
# reached only when target_d_prob < 0.50, so a confident-but-wrong lane frame
# could move the planned path by an arbitrary amount. Measured on
# 2026-09-03--11-00-22 at +193.4 s (26 km/h): the path was displaced 10.93 m at
# 20 m ahead, from a single 9.61 m frame-to-frame step, while every documented
# limit in this file is well under 1 m. The lateral MPC absorbed it that time
# (desired lateral accel stayed at 0.12 m/s^2), but the same spike on a
# straight at speed would go through as steering.
#
# The ceiling is deliberately well above normal blending -- over that drive the
# p95 of the applied correction was 0.39 m -- so it changes nothing in ordinary
# driving and only truncates outliers.
LANE_CENTER_MAX_CORRECTION_NEAR_M = 0.25
LANE_CENTER_MAX_CORRECTION_M = 1.00
# Frame-to-frame slew limit on the same correction. parse_model runs at 20 Hz
# and the measured p95 step was 0.062 m (1.24 m/s), so 2.0 m/s leaves normal
# motion untouched while capping a spike at 0.10 m per frame.
LANE_CENTER_MAX_CORRECTION_RATE_MS = 2.0

# Short temporal continuation for tight curves. Cache the last trustworthy
# lane-path shape, compensate it for the car's short ego-motion, hold it
# strongly for 0.10 s, then fade it out completely by 1.20 s. This narrow EON
# FOV loses lane lines well before a real curve (measured up to ~7 s on this
# car) finishes, so the old 0.60 s cap reverted to the raw (under-curving)
# model path mid-corner; doubling it covers more of the gap without letting
# the constant-curvature dead-reckoning run so long that road curvature
# changes within the hold window dominate the error.
CURVE_TEMPORAL_HOLD_FULL_S = 0.10
CURVE_TEMPORAL_HOLD_MAX_S = 1.20
# Speed-aware temporal horizon: keep longer history only at low speed.
CURVE_TEMPORAL_HOLD_BP_MS = (
  0.0, 30.0 / 3.6, 40.0 / 3.6, 45.0 / 3.6,
  50.0 / 3.6, 55.0 / 3.6, 60.0 / 3.6,
)
CURVE_TEMPORAL_HOLD_V_S = (1.20, 1.20, 1.00, 0.90, 0.76, 0.50, 0.0)
CURVE_TEMPORAL_MIN_ASSIST = 0.30
CURVE_TEMPORAL_STORE_DPROB = 0.35
CURVE_TEMPORAL_TRIGGER_DPROB = 0.22
CURVE_TEMPORAL_DPROB_FLOOR = 0.55
CURVE_TEMPORAL_MAX_CORRECTION_NEAR_M = 0.22
CURVE_TEMPORAL_MAX_CORRECTION_M = 0.75
CURVE_TEMPORAL_MAX_YAW_RAD = 0.24

# Below 2.0 m/s, yaw_rate/v_ego is a division by a near-zero denominator and
# too noisy to trust; by 5.0 m/s it is well-conditioned. Matches the vm/llk
# breakpoints latcontrol_torque.py already uses for the same reason.
IMU_CURVATURE_BLEND_BP_MS = (2.0, 5.0)

# Curvature-rate extrapolation for the temporal-hold virtual path. Real curves
# routinely keep tightening rather than holding one radius -- the transition
# spiral into an intersection is the extreme case -- so a frozen constant
# curvature under-turns as the true curve keeps closing after the camera loses
# it. Track the smoothed rate of change of the (IMU-blended) curvature while
# lane confidence is still good, freeze that rate at the moment of storage
# alongside the curvature itself, and extrapolate curvature linearly (capped)
# over the hold's age instead of assuming a fixed radius for the whole bridge.
CURVE_RATE_FILTER_TAU_S = 0.20
CURVE_RATE_MAX_DELTA = 0.020

# Relaxed ceilings used only while CurveVirtualReadinessMonitor currently
# confirms the dead-reckoning assumption: the car's own measured curvature
# (steering-derived, independent of lane visibility) agreeing with the device
# IMU's live yaw rate (liveLocationKalman.angularVelocityCalibrated -- this
# GM's CAN yaw signal reads a constant 0, see curve_virtual_readiness.py).
# That check re-evaluates every frame from live sensors, including through a
# lane dropout, so it can confirm the fallback is still tracking the real
# curve -- or withdraw trust within a few frames if the road straightens out
# faster than the extrapolation assumes. Applied only when
# readiness['eligible'] is True and scaled by readiness['quality']; outside
# that window every fallback behaves exactly as it did before this change.
# The trusted relaxation is switched off for the temporal hold: these now match
# CURVE_TEMPORAL_MAX_CORRECTION_* above, so readiness no longer widens this tier.
#
# It does fire -- modelPathQualityTrusted was true on 3.7% of lateralPlan frames
# of 2026-08-27--02-44-03 -- and the relaxed cap was being spent: of the frames
# where the temporal hold drove the path, 16% pushed past the 0.75 m ordinary
# limit, reaching 1.578 m, half a lane width.
#
# That is the weakest tier spending the largest correction. Its evidence is a
# dead-reckoned lane shape good for about 1.2 s, against dropouts that run 6-15 s
# for 46% of their duration, and it covers only 28.6% of the lost time. Compare
# the model-path tier, which moves the path a median 0.051 m and 0.200 m at the
# 95th, is used more often, and is built from the model's own current output.
#
# Fallback itself stays on: the 2026-08-27 pair of drives on one road (fallback
# on --02-44-03, off --04-22-29) had intervention through collapsed-confidence
# frames rise 36.1% -> 45.6% with it disabled. This narrows the one tier that
# was reaching furthest on the least evidence, it does not remove it.
CURVE_TEMPORAL_TRUSTED_MAX_CORRECTION_NEAR_M = 0.22
CURVE_TEMPORAL_TRUSTED_MAX_CORRECTION_M = 0.75
CURVE_TEMPORAL_TRUSTED_MAX_YAW_RAD = 0.42
CURVE_TEMPORAL_TRUSTED_HOLD_MULT = 2.0

# Road-edge fallback is used only on low/mid-speed tight curves when lane
# confidence is weak. It never blindly centers the whole road: both edges are
# accepted only when the visible road width looks like a single-lane corridor;
# otherwise only a nearby, trustworthy edge may anchor the last lane width.
ROAD_EDGE_STD_FULL_CONFIDENCE = 0.20
ROAD_EDGE_STD_ZERO_CONFIDENCE = 0.90
ROAD_EDGE_MIN_CONFIDENCE = 0.30
ROAD_EDGE_DPROB_FLOOR = 0.55
ROAD_EDGE_SINGLE_ROAD_MIN_M = 2.5
ROAD_EDGE_SINGLE_ROAD_MAX_M = 4.6
ROAD_EDGE_NEAR_MODEL_MIN_M = 1.0
ROAD_EDGE_NEAR_MODEL_MAX_M = 2.9
ROAD_EDGE_MAX_CORRECTION_NEAR_M = 0.20
ROAD_EDGE_MAX_CORRECTION_M = 0.70
# Same readiness-gated relaxation as the temporal-hold ceilings above.
ROAD_EDGE_TRUSTED_MAX_CORRECTION_NEAR_M = 0.35
ROAD_EDGE_TRUSTED_MAX_CORRECTION_M = 1.20
FRESH_LANE_RECOVERY_DPROB = 0.45
FRESH_LANE_RECOVERY_FRAMES = 4  # ~0.20 s at 20 Hz before dropping fallback

# Curve detection can use multiple independent visual sources. Lane geometry
# remains the strongest visual cue; model-path and road-edge bend are slightly
# down-weighted so they can start curve assist early without dominating it.
MODEL_PATH_CURVE_WEIGHT = 0.85
ROAD_EDGE_CURVE_WEIGHT = 0.90

# Third line of defense, used only when neither the lane-derived temporal
# hold nor the road-edge fallback has anything to offer. Continuously caches
# the model's own predicted path (path_xyz) whenever the model itself shows a
# clear bend -- independent of whether lane geometry is trustworthy enough to
# satisfy the temporal-hold's own store gate, since a narrow-FOV width/geometry
# rejection does not mean the model's own path prediction is untrustworthy too.
# Deliberately the most conservative of the three: short fixed hold, no rate
# extrapolation, smallest correction ceiling -- this source carries no lane or
# road-edge confirmation at all, only "the model itself believed this a moment
# ago".
MODEL_PATH_HOLD_MAX_S = 0.50
# Strength profile across the hold. This used to be a bare linear 1.0 -> 0.0
# ramp over MODEL_PATH_HOLD_MAX_S, i.e. it started fading on the very first
# frame, so the mean applied strength across a hold was only ~0.50 and the
# correction was throttled long before any correction ceiling mattered.
# Measured on 2026-08-25--10-51-17 (1522 model-path frames): applied
# |correction| was p50 0.088 m / p90 0.186 m, and on the readiness-trusted
# frames it peaked at 0.201 m -- less than half of the 0.45 m ceiling that was
# in force, so the ceiling was never the binding constraint, this decay was.
# Reshaped to match the temporal-hold profile that already exists above
# (CURVE_TEMPORAL_HOLD_FULL_S plateau, then a 0.55 shoulder, then to zero),
# which lifts mean strength to ~0.62 while still reaching zero at exactly the
# same MODEL_PATH_HOLD_MAX_S -- the hold is not extended, only the shape
# within it changes, so dead-reckoning time is unchanged.
MODEL_PATH_HOLD_FULL_S = 0.10
MODEL_PATH_HOLD_SHOULDER_S = 0.30
MODEL_PATH_HOLD_SHOULDER_STRENGTH = 0.55
# Readiness-gated bridge extension, same idea as CURVE_TEMPORAL_TRUSTED_HOLD_MULT
# and applied through the same interp-on-quality shape. 1.6 takes the 0.50 s
# hold to 0.80 s, but ONLY while readiness is eligible and in proportion to its
# quality -- an unconfirmed model-path hold still expires at exactly 0.50 s, so
# this adds no extra dead-reckoning time to the unverified case.
#
# This is the one change in this file that genuinely spends safety margin
# rather than correcting a mis-placed threshold: it lets the car carry a
# remembered path further without new lane evidence. It is gated on readiness
# precisely because readiness is the check that the remembered curvature still
# agrees with measured IMU yaw. Measured on 2026-08-26--01-04-22 at +1830 s (a
# hands-off right turn where the camera saw essentially nothing, laneLineProbs
# 0.01-0.05): readiness held eligible with quality 1.00 for ~2.8 s straight
# through the buildup, so the extension applies in exactly the situation it was
# added for, and lapses the moment the driver touches the wheel.
MODEL_PATH_TRUSTED_HOLD_MULT = 1.6
MODEL_PATH_HOLD_STORE_STRENGTH = 0.40
MODEL_PATH_MAX_CORRECTION_NEAR_M = 0.12
MODEL_PATH_MAX_CORRECTION_M = 0.45
# Same readiness-gated relaxation as temporal-hold/road-edge (see
# CURVE_TEMPORAL_TRUSTED_* above). Previously model-path had no trusted
# variant at all, so even a fully-confirmed curve stayed capped at the same
# conservative 0.45m as an unverified one -- the tightest cap of any fallback
# tier, right when the sharpest curves (the ones that escalate all the way to
# this last-resort tier) need it least. Kept below temporal-hold/road-edge's
# trusted ceilings since this source still carries no lane or road-edge
# confirmation, only the model's own prior belief.
MODEL_PATH_TRUSTED_MAX_CORRECTION_NEAR_M = 0.22
MODEL_PATH_TRUSTED_MAX_CORRECTION_M = 0.90
MODEL_PATH_DPROB_FLOOR = 0.45

# A/B switch for the whole curve fallback (temporal hold + road edge). Drive
# logs have not settled whether it helps: it ran on only 1.4% of frames and
# added 1-5 deg of equivalent steering, and its data availability and its need
# are anti-correlated -- where it holds a cached shape it is mostly not needed
# (gentle curves), and in sharp curves, where it is, it had a usable cache only
# 12% of the time. Turning it off leaves ordinary lane blending, which is what
# stock openpilot does when lane confidence drops. Drive the same road with it
# on and off to compare.
# Default ON (existing behaviour). Flip from Community -> Curve Lane Fallback.
CURVE_FALLBACK_DEFAULT = True


class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    # Seconds since both lane lines were last clear enough to measure a width.
    # Starts stale so a fresh planner uses the speed default until it has seen
    # a real lane.
    self.lane_width_age_s = LANE_WIDTH_HOLD_S + LANE_WIDTH_DECAY_S
    self.lane_width = 3.7

    self.lll_prob = 0.0
    self.rll_prob = 0.0
    self.d_prob = 0.0
    self.lll_std = 1.0
    self.rll_std = 1.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET
    self.total_camera_offset = self.camera_offset

    # Read once: an A/B switch must not flip mid-drive. Params enforces a key
    # whitelist and raises for keys an older compiled library does not know, so
    # a device running an older build than this file would otherwise take
    # plannerd down on startup. Fall back to the module default instead.
    fallback_disabled = False
    try:
      fallback_disabled = Params().get_bool('CurveFallbackDisabled')
    except Exception:
      cloudlog.warning("CurveFallbackDisabled param unavailable, using default")
    self.curve_fallback_enabled = bool(
      CURVE_FALLBACK_DEFAULT and not fallback_disabled)

    # Set fresh every get_d_path() call from CurveVirtualReadinessMonitor's
    # current (this-frame) report. False/0.0 reproduces the exact pre-change
    # fixed caps.
    self._readiness_eligible = False
    self._readiness_quality = 0.0

    # Compatibility fields retained for the existing lateralPlan schema.
    self.lane_center_correction_m = 0.0
    self.lane_center_correction_active = False
    self._last_lane_center_refs = None
    # Previous frame's applied lane-center offset at 20 m, for the slew limit.
    # Kept in sync with the fallback paths so leaving a fallback does not look
    # like a step change to the rate limiter.
    self._prev_center_delta_m = 0.0

    # Diagnostic-only mirrors of the temporal-hold store gate inputs. They never
    # feed control; they exist so a drive log can show which condition blocks
    # storage on a tight curve.
    self.curve_assist_diag = 0.0
    self.curve_raw_target_d_prob_diag = 0.0
    self.curve_geometry_plausible_diag = False
    self.curve_temporal_stored_diag = False
    # 0 none, 1 ordinary lane blending, 2 temporal hold, 3 road edge
    self.curve_fallback_source_diag = 0
    self.lane_head_disagree = False
    self.lane_head_gap_m = 0.0

    # Last trustworthy tight-curve lane path in the previous ego frame.
    self._curve_hold_x = None
    self._curve_hold_y = None
    self._curve_hold_age_s = CURVE_TEMPORAL_HOLD_MAX_S
    self._curve_hold_strength = 0.0
    self._curve_hold_curvature = 0.0
    self._curve_hold_curvature_rate = 0.0
    self._curve_hold_sign = 0.0
    self._prev_stored_curvature = None
    self._curve_rate_filter = FirstOrderFilter(0.0, CURVE_RATE_FILTER_TAU_S, DT_MDL)

    # Third-line-of-defense cache: a recent snapshot of the model's own
    # predicted path shape, refreshed independently of lane confidence.
    self._model_hold_x = None
    self._model_hold_y = None
    self._model_hold_age_s = MODEL_PATH_HOLD_MAX_S

    # Road-edge geometry from modelV2. Kept separate from lane lines so it can
    # act as a fallback only when lanes are weak.
    self.le_x = np.zeros((TRAJECTORY_SIZE,))
    self.re_x = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_std = 1.0
    self.re_std = 1.0

    # Fallback state. A fresh lane must remain trustworthy for several model
    # frames before stale curve history is discarded, preventing one-frame
    # lane flicker from releasing steering in a deep corner.
    self._fallback_mode_active = False
    self._fresh_lane_recovery_frames = 0

  @staticmethod
  def _finite_sorted_unique(x_values, y_values):
    """Filter to finite points, sort by x, and drop duplicate x.

    Shared prep used everywhere a spatial x/y series (lane path, road edge,
    temporal-hold cache) needs to become valid np.interp input. Returns
    (None, None) if fewer than 2 usable points remain.
    """
    x_values = np.asarray(x_values, dtype=float)
    y_values = np.asarray(y_values, dtype=float)
    if x_values.size < 2 or x_values.size != y_values.size:
      return None, None

    valid = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[valid]
    y_values = y_values[valid]
    if x_values.size < 2:
      return None, None

    order = np.argsort(x_values)
    x_values = x_values[order]
    y_values = y_values[order]
    x_values, unique_indices = np.unique(x_values, return_index=True)
    y_values = y_values[unique_indices]
    if x_values.size < 2:
      return None, None
    return x_values, y_values

  def _edge_bend(self, edge_x, edge_y, edge_std):
    """Prepared (x, y), confidence, check-distance refs, and bend strength for
    one road edge.

    Shared by _road_edge_curve_strength (early curve detector) and
    _road_edge_fallback_path (path builder) so both agree on what counts as a
    trustworthy, curving edge instead of computing it twice via separately
    tuned code. refs/strength are None/0.0 when the edge is unavailable or
    below ROAD_EDGE_MIN_CONFIDENCE -- every caller already re-checks
    confidence before using refs, so this only avoids wasted work.
    """
    x, y = self._finite_sorted_unique(edge_x, edge_y)
    confidence = interp(
      edge_std, [ROAD_EDGE_STD_FULL_CONFIDENCE, ROAD_EDGE_STD_ZERO_CONFIDENCE],
      [1.0, 0.0])
    if x is None or confidence < ROAD_EDGE_MIN_CONFIDENCE:
      return x, y, confidence, None, 0.0

    refs = np.interp(np.asarray(LANE_WIDTH_CHECK_DISTANCES_M, dtype=float), x, y)
    bend_m = float(np.max(np.abs(refs - refs[0])))
    strength = float(np.clip(
      interp(bend_m, [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M], [0.0, 1.0]),
      0.0, 1.0))
    return x, y, confidence, refs, strength

  def _trusted_max_correction(self, x_abs, base_near_m, base_far_m,
                              trusted_near_m, trusted_far_m):
    """Correction-magnitude cap, relaxed while readiness is confirmed.

    Shared by the temporal-hold and road-edge fallbacks -- see the
    CURVE_TEMPORAL_TRUSTED_* comment above for what "confirmed" means.
    """
    near_m, far_m = base_near_m, base_far_m
    if self._readiness_eligible:
      near_m = interp(self._readiness_quality, [0.0, 1.0], [near_m, trusted_near_m])
      far_m = interp(self._readiness_quality, [0.0, 1.0], [far_m, trusted_far_m])
    return np.interp(x_abs, [0.0, 20.0], [near_m, far_m])

  def _blended_curvature(self, measured_curvature, imu_curvature,
                         imu_curvature_valid, v_ego):
    """Curvature used to build the temporal-hold virtual path.

    measured_curvature (steering-angle-derived) is angle-based and stays
    numerically stable at low speed. imu_curvature (yaw_rate / v_ego, from
    liveLocationKalman.angularVelocityCalibrated -- this GM's CAN yaw signal
    reads a constant 0, see curve_virtual_readiness.py) is an independent
    physical measurement, but the division makes it noisy as v_ego shrinks.
    Blend toward IMU only once speed is high enough for that division to be
    well-conditioned -- same shape as the official torque controller's own
    vm/llk blend in latcontrol_torque.py. Scoped to just the two functions
    that construct the dead-reckoned virtual path; curve detection/scoring
    elsewhere keeps using measured_curvature alone.
    """
    if not imu_curvature_valid or imu_curvature is None:
      return float(measured_curvature)
    return float(interp(
      v_ego, IMU_CURVATURE_BLEND_BP_MS,
      [float(measured_curvature), float(imu_curvature)]))

  def _update_d_prob(self, target_d_prob, v_ego, lane_center_refs,
                     lane_change_active, curve_assist):
    """Rate-limit short confidence dropouts, with extra continuity on tight low-speed curves."""
    target_d_prob = float(np.clip(target_d_prob, 0.0, 1.0))
    curve_assist = float(np.clip(curve_assist, 0.0, 1.0))
    continuity_limit = interp(
      curve_assist, [0.0, 1.0],
      [LANE_CENTER_CONTINUITY_MAX_M, CURVE_LANE_CENTER_CONTINUITY_MAX_M])
    refs_continuous = (
      self._last_lane_center_refs is not None and
      np.max(np.abs(lane_center_refs - self._last_lane_center_refs)) <=
      continuity_limit
    )

    if target_d_prob >= self.d_prob:
      # Keep the existing speed-aware rise limiter on straights/high speed,
      # but recover lane confidence faster when a tight low-speed curve
      # temporarily pushes a lane line toward the edge of the camera view.
      next_d_prob = limit_lane_probability_rise(
        self.d_prob, target_d_prob, v_ego, DT_MDL)
      if curve_assist > 0.0:
        curve_max_rise = (
          1.50 + CURVE_CONFIDENCE_RISE_BONUS_PER_S * curve_assist
        ) * DT_MDL
        next_d_prob = max(
          next_d_prob, min(target_d_prob, self.d_prob + curve_max_rise))
    elif lane_change_active or not refs_continuous:
      next_d_prob = target_d_prob
    else:
      fall_rate = LANE_CONFIDENCE_FALL_RATE_PER_S
      if curve_assist > 0.0:
        fall_rate = max(
          CURVE_CONFIDENCE_MIN_FALL_RATE_PER_S,
          fall_rate - CURVE_CONFIDENCE_FALL_SLOWDOWN_PER_S * curve_assist)
      max_fall = fall_rate * DT_MDL
      next_d_prob = max(target_d_prob, self.d_prob - max_fall)

    # Do not replace the continuity reference with a fully untrusted frame.
    if target_d_prob >= 0.10:
      self._last_lane_center_refs = lane_center_refs.copy()
    return next_d_prob

  @property
  def curve_temporal_hold_age_diag(self):
    """Age of the cached tight-curve path, at the max when none is held."""
    if self._curve_hold_x is None:
      return CURVE_TEMPORAL_HOLD_MAX_S
    return float(self._curve_hold_age_s)

  def _clear_curve_temporal_hold(self):
    self._curve_hold_x = None
    self._curve_hold_y = None
    self._curve_hold_age_s = CURVE_TEMPORAL_HOLD_MAX_S
    self._curve_hold_strength = 0.0
    self._curve_hold_curvature = 0.0
    self._curve_hold_curvature_rate = 0.0
    self._curve_hold_sign = 0.0
    # Every call site here means this curve episode's cached state is no
    # longer valid (sign flip, lane change, expired, fresh lane recovered) --
    # the curvature trend from before is equally stale. Forget it so the next
    # _store_curve_temporal_hold treats itself as a fresh first sample rather
    # than diffing against an unrelated prior curve.
    self._prev_stored_curvature = None
    self._curve_rate_filter.x = 0.0

  def _store_curve_temporal_hold(self, path_x, lane_path_y,
                                 curve_assist, measured_curvature, v_ego=0.0,
                                 imu_curvature=None, imu_curvature_valid=False):
    """Cache a finite, monotonic lane path for a possible short dropout."""
    path_x, lane_path_y = self._finite_sorted_unique(path_x, lane_path_y)
    if path_x is None:
      return

    curvature = self._blended_curvature(
      measured_curvature, imu_curvature, imu_curvature_valid, v_ego)

    # Track how fast curvature is trending while lane confidence is still
    # good, so a dropout starting mid-transition can keep closing the curve
    # instead of freezing at whatever radius happened to be visible last.
    if self._prev_stored_curvature is not None:
      raw_rate = (curvature - self._prev_stored_curvature) / DT_MDL
      self._curve_rate_filter.update(raw_rate)
    self._prev_stored_curvature = curvature

    self.curve_temporal_stored_diag = True
    self._curve_hold_x = path_x.copy()
    self._curve_hold_y = lane_path_y.copy()
    self._curve_hold_age_s = 0.0
    self._curve_hold_strength = float(np.clip(curve_assist, 0.0, 1.0))
    if abs(curvature) >= CURVE_ASSIST_START_CURVATURE:
      self._curve_hold_curvature = curvature
      self._curve_hold_curvature_rate = self._curve_rate_filter.x
      self._curve_hold_sign = float(np.sign(curvature))
    else:
      self._curve_hold_curvature = 0.0
      self._curve_hold_curvature_rate = 0.0
      self._curve_hold_sign = 0.0

  def _temporal_hold_max_s(self, v_ego):
    """Maximum stale-curve bridge time for the current vehicle speed."""
    return float(np.clip(
      interp(float(v_ego), CURVE_TEMPORAL_HOLD_BP_MS,
             CURVE_TEMPORAL_HOLD_V_S),
      0.0, CURVE_TEMPORAL_HOLD_MAX_S))

  def _trusted_hold_max_s(self, v_ego):
    """Speed-aware bridge time, extended while readiness is confirmed.

    Multiplies rather than replaces: a speed that already collapses the base
    hold to ~0 (e.g. above the 60 kph top breakpoint) still gets no hold,
    trusted or not.
    """
    hold_max_s = self._temporal_hold_max_s(v_ego)
    if self._readiness_eligible and hold_max_s > 0.0:
      hold_max_s = interp(
        self._readiness_quality, [0.0, 1.0],
        [hold_max_s, hold_max_s * CURVE_TEMPORAL_TRUSTED_HOLD_MULT])
    return hold_max_s

  def _curve_temporal_prediction(self, path_x, v_ego, curve_assist,
                                 measured_curvature, lane_change_active,
                                 imu_curvature=None, imu_curvature_valid=False):
    """Predict the cached lane path with a speed-aware <=0.60 s horizon."""
    if lane_change_active:
      self._clear_curve_temporal_hold()
      return None, 0.0

    if self._curve_hold_x is None or self._curve_hold_y is None:
      return None, 0.0

    if v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
      self._clear_curve_temporal_hold()
      return None, 0.0

    blended_curvature = self._blended_curvature(
      measured_curvature, imu_curvature, imu_curvature_valid, v_ego)

    current_sign = 0.0
    if abs(blended_curvature) >= CURVE_ASSIST_START_CURVATURE:
      current_sign = float(np.sign(blended_curvature))
    if (self._curve_hold_sign != 0.0 and current_sign != 0.0 and
        current_sign != self._curve_hold_sign):
      # Never carry a right-hand curve into an immediate left-hand curve, or
      # vice versa. The fresh model path takes over immediately.
      self._clear_curve_temporal_hold()
      return None, 0.0

    hold_max_s = self._trusted_hold_max_s(v_ego)
    if hold_max_s <= CURVE_TEMPORAL_HOLD_FULL_S:
      self._clear_curve_temporal_hold()
      return None, 0.0

    self._curve_hold_age_s += DT_MDL
    if self._curve_hold_age_s > hold_max_s:
      self._clear_curve_temporal_hold()
      return None, 0.0

    active_assist = max(float(curve_assist), self._curve_hold_strength * 0.70)
    if active_assist < CURVE_TEMPORAL_MIN_ASSIST:
      return None, 0.0

    # Move the previously observed road geometry into the current vehicle
    # frame. The first 0.10 s is held strongly, then stale geometry fades
    # according to speed.
    curvature = blended_curvature
    if abs(curvature) < CURVE_ASSIST_START_CURVATURE:
      # No usable live signal this frame -- extrapolate the frozen curvature
      # using the rate observed just before the dropout began (clothoid-like:
      # real curves keep tightening/opening rather than holding one radius),
      # capped so a noisy rate estimate cannot run away over a long hold.
      delta = float(np.clip(
        self._curve_hold_curvature_rate * self._curve_hold_age_s,
        -CURVE_RATE_MAX_DELTA, CURVE_RATE_MAX_DELTA))
      curvature = self._curve_hold_curvature + delta
      # Trapezoidal average of the curvature at the start of the bridge and
      # now, times arc length, approximates the integral of a linearly
      # changing curvature without a full clothoid (Fresnel-integral) solve.
      yaw_curvature = 0.5 * (self._curve_hold_curvature + curvature)
    else:
      yaw_curvature = curvature
    travel_m = max(float(v_ego), 0.0) * self._curve_hold_age_s
    yaw_cap = CURVE_TEMPORAL_MAX_YAW_RAD
    if self._readiness_eligible:
      yaw_cap = interp(self._readiness_quality, [0.0, 1.0],
                       [CURVE_TEMPORAL_MAX_YAW_RAD, CURVE_TEMPORAL_TRUSTED_MAX_YAW_RAD])
    yaw = float(np.clip(yaw_curvature * travel_m, -yaw_cap, yaw_cap))

    if abs(curvature) > 1e-4:
      dx = np.sin(yaw) / curvature
      dy = (1.0 - np.cos(yaw)) / curvature
    else:
      dx = travel_m
      dy = 0.0

    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    rel_x = self._curve_hold_x - dx
    rel_y = self._curve_hold_y - dy
    predicted_x = cos_yaw * rel_x + sin_yaw * rel_y
    predicted_y = -sin_yaw * rel_x + cos_yaw * rel_y

    predicted_x, predicted_y = self._finite_sorted_unique(predicted_x, predicted_y)
    if predicted_x is None:
      return None, 0.0

    path_x = np.asarray(path_x, dtype=float)
    predicted_lane_y = np.interp(path_x, predicted_x, predicted_y)

    # Preserve the strong first 0.10 s. At lower speed retain the original
    # 0.30 s / 55% shoulder, while higher speed fades stale geometry sooner.
    if hold_max_s > 0.30:
      time_strength = interp(
        self._curve_hold_age_s,
        [0.0, CURVE_TEMPORAL_HOLD_FULL_S, 0.30, hold_max_s],
        [1.0, 1.0, 0.55, 0.0])
    else:
      time_strength = interp(
        self._curve_hold_age_s,
        [0.0, CURVE_TEMPORAL_HOLD_FULL_S, hold_max_s],
        [1.0, 1.0, 0.0])
    assist_strength = interp(
      active_assist, [CURVE_TEMPORAL_MIN_ASSIST, 1.0], [0.55, 1.0])
    strength = float(np.clip(time_strength * assist_strength, 0.0, 1.0))
    return predicted_lane_y, strength

  def _bound_curve_temporal_path(self, path_xyz, predicted_lane_y):
    """Bound stale-path influence relative to the fresh model path."""
    max_correction = self._trusted_max_correction(
      np.abs(path_xyz[:, 0]),
      CURVE_TEMPORAL_MAX_CORRECTION_NEAR_M, CURVE_TEMPORAL_MAX_CORRECTION_M,
      CURVE_TEMPORAL_TRUSTED_MAX_CORRECTION_NEAR_M, CURVE_TEMPORAL_TRUSTED_MAX_CORRECTION_M)
    return path_xyz[:, 1] + np.clip(
      predicted_lane_y - path_xyz[:, 1], -max_correction, max_correction)

  def _spatial_curve_strength(self, x_values, y_values):
    """Estimate visible bend strength from spatial x/y points."""
    x_values, y_values = self._finite_sorted_unique(x_values, y_values)
    if x_values is None:
      return 0.0

    refs = np.interp(
      np.asarray(LANE_WIDTH_CHECK_DISTANCES_M, dtype=float),
      x_values, y_values)
    bend_m = float(np.max(np.abs(refs - refs[0])))
    return float(np.clip(
      interp(
        bend_m,
        [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M],
        [0.0, 1.0]),
      0.0, 1.0))

  def _model_path_curve_strength(self, path_xyz):
    """Use the fresh model path as an early curve cue when lanes are weak."""
    if path_xyz.ndim != 2 or path_xyz.shape[0] < 2 or path_xyz.shape[1] < 2:
      return 0.0
    return self._spatial_curve_strength(path_xyz[:, 0], path_xyz[:, 1])

  def _clear_model_path_hold(self):
    self._model_hold_x = None
    self._model_hold_y = None
    self._model_hold_age_s = MODEL_PATH_HOLD_MAX_S

  def _store_model_path_hold(self, path_x, path_y):
    """Continuously cache the model's own path shape (see the
    MODEL_PATH_HOLD_* comment above the constants for why this is
    independent of the lane-derived temporal hold's own store gate)."""
    x, y = self._finite_sorted_unique(path_x, path_y)
    if x is None:
      return
    self._model_hold_x = x.copy()
    self._model_hold_y = y.copy()
    self._model_hold_age_s = 0.0

  def _model_path_prediction(self, path_x, v_ego, measured_curvature,
                             lane_change_active, imu_curvature=None,
                             imu_curvature_valid=False):
    """Third-line fallback: reposition the cached model-path snapshot.

    No rate extrapolation and a short fixed hold -- deliberately simpler than
    the lane-derived temporal hold, since this source has no lane or
    road-edge confirmation behind it at all.
    """
    if lane_change_active or self._model_hold_x is None:
      return None, 0.0
    if v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
      self._clear_model_path_hold()
      return None, 0.0

    # Bridge time, extended only while readiness confirms the remembered
    # curvature still matches measured yaw (see MODEL_PATH_TRUSTED_HOLD_MULT).
    hold_max_s = MODEL_PATH_HOLD_MAX_S
    if self._readiness_eligible:
      hold_max_s = interp(
        self._readiness_quality, [0.0, 1.0],
        [MODEL_PATH_HOLD_MAX_S,
         MODEL_PATH_HOLD_MAX_S * MODEL_PATH_TRUSTED_HOLD_MULT])

    self._model_hold_age_s += DT_MDL
    if self._model_hold_age_s > hold_max_s:
      self._clear_model_path_hold()
      return None, 0.0

    curvature = self._blended_curvature(
      measured_curvature, imu_curvature, imu_curvature_valid, v_ego)
    travel_m = max(float(v_ego), 0.0) * self._model_hold_age_s
    yaw = float(np.clip(
      curvature * travel_m, -CURVE_TEMPORAL_MAX_YAW_RAD, CURVE_TEMPORAL_MAX_YAW_RAD))

    if abs(curvature) > 1e-4:
      dx = np.sin(yaw) / curvature
      dy = (1.0 - np.cos(yaw)) / curvature
    else:
      dx = travel_m
      dy = 0.0

    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    rel_x = self._model_hold_x - dx
    rel_y = self._model_hold_y - dy
    predicted_x = cos_yaw * rel_x + sin_yaw * rel_y
    predicted_y = -sin_yaw * rel_x + cos_yaw * rel_y
    predicted_x, predicted_y = self._finite_sorted_unique(predicted_x, predicted_y)
    if predicted_x is None:
      return None, 0.0

    path_x = np.asarray(path_x, dtype=float)
    predicted_lane_y = np.interp(path_x, predicted_x, predicted_y)
    # Plateau then shoulder, mirroring the temporal-hold profile; see the
    # MODEL_PATH_HOLD_FULL_S comment for the drive-log measurement behind it.
    # Stretch the same plateau/shoulder shape over whatever bridge time is in
    # force, so an extended hold fades to zero at its own end rather than
    # hitting zero early at the un-extended 0.50 s.
    strength = float(np.clip(
      interp(self._model_hold_age_s,
             [0.0, MODEL_PATH_HOLD_FULL_S, MODEL_PATH_HOLD_SHOULDER_S,
              hold_max_s],
             [1.0, 1.0, MODEL_PATH_HOLD_SHOULDER_STRENGTH, 0.0]),
      0.0, 1.0))
    return predicted_lane_y, strength

  def _road_edge_curve_strength(self):
    """Return confidence-weighted bend from the best visible road edge."""
    best_strength = 0.0
    for edge_x, edge_y, edge_std in (
        (self.le_x, self.le_y, self.le_std),
        (self.re_x, self.re_y, self.re_std)):
      _, _, confidence, refs, strength = self._edge_bend(edge_x, edge_y, edge_std)
      if refs is None:
        continue
      best_strength = max(best_strength, strength * confidence)

    return float(np.clip(best_strength, 0.0, 1.0))

  def _road_edge_fallback_path(self, path_xyz, v_ego, curve_assist,
                               lane_change_active):
    """Build a bounded path from trustworthy road edges on a tight curve."""
    if lane_change_active or v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
      return None, 0.0

    left_x, left_y, left_conf, left_refs, left_strength = self._edge_bend(
      self.le_x, self.le_y, self.le_std)
    right_x, right_y, right_conf, right_refs, right_strength = self._edge_bend(
      self.re_x, self.re_y, self.re_std)
    if left_x is None and right_x is None:
      return None, 0.0

    path_x = np.asarray(path_xyz[:, 0], dtype=float)
    model_y = np.asarray(path_xyz[:, 1], dtype=float)
    check_x = np.asarray(LANE_WIDTH_CHECK_DISTANCES_M, dtype=float)
    model_refs = np.interp(check_x, path_x, model_y)

    left_interp = np.interp(path_x, left_x, left_y) if left_x is not None else None
    right_interp = np.interp(path_x, right_x, right_y) if right_x is not None else None

    speed_weight = interp(
      v_ego, [CURVE_ASSIST_FULL_BELOW_MS, CURVE_ASSIST_ZERO_ABOVE_MS],
      [1.0, 0.0])

    # Road edges can reveal the curve before measured vehicle curvature catches
    # up. Confidence-weighted to match _road_edge_curve_strength's early
    # detector -- a noisy/uncertain edge should not trigger this fallback any
    # more easily than it triggers curve detection upstream.
    edge_curve_strength = max(
      left_strength * left_conf if left_refs is not None else 0.0,
      right_strength * right_conf if right_refs is not None else 0.0)

    active_curve_assist = float(np.clip(
      max(float(curve_assist), edge_curve_strength * speed_weight),
      0.0, 1.0))
    if active_curve_assist < CURVE_TEMPORAL_MIN_ASSIST:
      return None, 0.0

    lane_half_width = float(np.clip(self.lane_width * 0.5, 1.30, 2.00))
    candidates = []
    confidences = []

    # Use the center between both road edges only if it looks like a true
    # single-lane corridor. This deliberately rejects normal two-way/multi-lane
    # road widths so we do not pull the car toward the whole-road center.
    if (left_refs is not None and right_refs is not None and
        left_conf >= ROAD_EDGE_MIN_CONFIDENCE and
        right_conf >= ROAD_EDGE_MIN_CONFIDENCE):
      road_width_refs = np.abs(right_refs - left_refs)
      if (np.all(road_width_refs >= ROAD_EDGE_SINGLE_ROAD_MIN_M) and
          np.all(road_width_refs <= ROAD_EDGE_SINGLE_ROAD_MAX_M)):
        candidates.append(0.5 * (left_interp + right_interp))
        confidences.append(min(left_conf, right_conf))

    # On wider roads, use only an edge that stays approximately one half-lane
    # from the fresh model path. This makes the fallback useful near a curb or
    # shoulder without treating a distant road boundary as our lane boundary.
    if left_refs is not None and left_conf >= ROAD_EDGE_MIN_CONFIDENCE:
      left_sep = np.abs(model_refs - left_refs)
      if (np.all(left_sep >= ROAD_EDGE_NEAR_MODEL_MIN_M) and
          np.all(left_sep <= ROAD_EDGE_NEAR_MODEL_MAX_M)):
        candidates.append(left_interp + lane_half_width)
        confidences.append(left_conf)

    if right_refs is not None and right_conf >= ROAD_EDGE_MIN_CONFIDENCE:
      right_sep = np.abs(right_refs - model_refs)
      if (np.all(right_sep >= ROAD_EDGE_NEAR_MODEL_MIN_M) and
          np.all(right_sep <= ROAD_EDGE_NEAR_MODEL_MAX_M)):
        candidates.append(right_interp - lane_half_width)
        confidences.append(right_conf)

    if not candidates:
      return None, 0.0

    weights = np.asarray(confidences, dtype=float)
    edge_path = np.average(np.vstack(candidates), axis=0, weights=weights)

    max_correction = self._trusted_max_correction(
      np.abs(path_x),
      ROAD_EDGE_MAX_CORRECTION_NEAR_M, ROAD_EDGE_MAX_CORRECTION_M,
      ROAD_EDGE_TRUSTED_MAX_CORRECTION_NEAR_M, ROAD_EDGE_TRUSTED_MAX_CORRECTION_M)
    edge_path = model_y + np.clip(
      edge_path - model_y, -max_correction, max_correction)

    confidence = float(np.clip(
      max(confidences) * active_curve_assist, 0.0, 1.0))
    return edge_path, confidence

  def _age_curve_temporal_hold(self, v_ego):
    if self._curve_hold_x is None:
      return
    hold_max_s = self._trusted_hold_max_s(v_ego)
    if hold_max_s <= CURVE_TEMPORAL_HOLD_FULL_S:
      self._clear_curve_temporal_hold()
      return
    self._curve_hold_age_s += DT_MDL
    if self._curve_hold_age_s > hold_max_s:
      self._clear_curve_temporal_hold()

  def _age_model_path_hold(self, v_ego):
    if self._model_hold_x is None:
      return
    if v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
      self._clear_model_path_hold()
      return
    self._model_hold_age_s += DT_MDL
    if self._model_hold_age_s > MODEL_PATH_HOLD_MAX_S:
      self._clear_model_path_hold()

  def _apply_missing_lane_fallback(self, path_xyz, v_ego, curve_assist,
                                   measured_curvature, lane_change_active,
                                   imu_curvature=None, imu_curvature_valid=False):
    """Fallback order for missing lane geometry: temporal first, then road edge."""
    if not self.curve_fallback_enabled:
      return False

    predicted_lane_y, temporal_strength = self._curve_temporal_prediction(
      path_xyz[:, 0], v_ego, curve_assist,
      measured_curvature, lane_change_active,
      imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid)
    if predicted_lane_y is not None and temporal_strength > 0.0:
      predicted_lane_y = self._bound_curve_temporal_path(
        path_xyz, predicted_lane_y)
      applied_delta = (
        predicted_lane_y - path_xyz[:, 1]
      ) * temporal_strength
      path_xyz[:, 1] += applied_delta
      self.d_prob = max(
        self.d_prob, CURVE_TEMPORAL_DPROB_FLOOR * temporal_strength)
      self.lane_center_correction_m = float(
        np.interp(20.0, path_xyz[:, 0], applied_delta))
      self.lane_center_correction_active = bool(
        abs(self.lane_center_correction_m) > 0.01)
      # This fallback already bounds applied_delta itself; seed the ordinary
      # path's slew limiter so returning to it is not read as a step.
      self._prev_center_delta_m = self.lane_center_correction_m
      self.curve_fallback_source_diag = 2
      self._fallback_mode_active = True
      self._fresh_lane_recovery_frames = 0
      return True

    edge_path, edge_strength = self._road_edge_fallback_path(
      path_xyz, v_ego, curve_assist, lane_change_active)
    if edge_path is not None and edge_strength > 0.0:
      applied_delta = (edge_path - path_xyz[:, 1]) * edge_strength
      path_xyz[:, 1] += applied_delta
      self.d_prob = max(
        self.d_prob, ROAD_EDGE_DPROB_FLOOR * edge_strength)
      self.lane_center_correction_m = float(
        np.interp(20.0, path_xyz[:, 0], applied_delta))
      self.lane_center_correction_active = bool(
        abs(self.lane_center_correction_m) > 0.01)
      # This fallback already bounds applied_delta itself; seed the ordinary
      # path's slew limiter so returning to it is not read as a step.
      self._prev_center_delta_m = self.lane_center_correction_m
      self.curve_fallback_source_diag = 3
      self._fallback_mode_active = True
      self._fresh_lane_recovery_frames = 0
      self._age_curve_temporal_hold(v_ego)
      return True

    model_path_y, model_path_strength = self._model_path_prediction(
      path_xyz[:, 0], v_ego, measured_curvature, lane_change_active,
      imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid)
    if model_path_y is not None and model_path_strength > 0.0:
      max_correction = self._trusted_max_correction(
        np.abs(path_xyz[:, 0]),
        MODEL_PATH_MAX_CORRECTION_NEAR_M, MODEL_PATH_MAX_CORRECTION_M,
        MODEL_PATH_TRUSTED_MAX_CORRECTION_NEAR_M, MODEL_PATH_TRUSTED_MAX_CORRECTION_M)
      model_path_y = path_xyz[:, 1] + np.clip(
        model_path_y - path_xyz[:, 1], -max_correction, max_correction)
      applied_delta = (model_path_y - path_xyz[:, 1]) * model_path_strength
      path_xyz[:, 1] += applied_delta
      self.d_prob = max(
        self.d_prob, MODEL_PATH_DPROB_FLOOR * model_path_strength)
      self.lane_center_correction_m = float(
        np.interp(20.0, path_xyz[:, 0], applied_delta))
      self.lane_center_correction_active = bool(
        abs(self.lane_center_correction_m) > 0.01)
      # This fallback already bounds applied_delta itself; seed the ordinary
      # path's slew limiter so returning to it is not read as a step.
      self._prev_center_delta_m = self.lane_center_correction_m
      self.curve_fallback_source_diag = 4
      self._fallback_mode_active = True
      self._fresh_lane_recovery_frames = 0
      self._age_curve_temporal_hold(v_ego)
      return True

    self._age_curve_temporal_hold(v_ego)
    self._age_model_path_hold(v_ego)
    return False

  def parse_model(self, md):
    # Reset the per-frame store-gate diagnostics here rather than in get_d_path,
    # which is skipped entirely when lane lines are disabled. Without this the
    # published values would go stale instead of reading as "not evaluated".
    self.curve_assist_diag = 0.0
    self.curve_raw_target_d_prob_diag = 0.0
    self.curve_geometry_plausible_diag = False
    self.curve_temporal_stored_diag = False
    self.curve_fallback_source_diag = 0

    lane_lines = md.laneLines
    lane_line_probs = as_finite_vector(md.laneLineProbs, minimum_size=4)
    lane_line_stds = as_finite_vector(md.laneLineStds, minimum_size=4)
    lane_data = None
    if len(lane_lines) == 4 and lane_line_probs is not None and lane_line_stds is not None:
      lane_data = tuple(
        as_finite_vector(values, expected_size=TRAJECTORY_SIZE)
        for values in (lane_lines[1].x, lane_lines[2].x,
                       lane_lines[1].y, lane_lines[2].y)
      )

    if lane_data is not None and all(values is not None for values in lane_data):
      left_x, right_x, left_y, right_y = lane_data
      # This EON model has only three finite lane-line t values. Geometry is
      # spatial, so consume the complete and finite x/y vectors directly.
      self.ll_x = (left_x + right_x) / 2.0
      self.lll_y = left_y + self.total_camera_offset
      self.rll_y = right_y + self.total_camera_offset
      self.lll_prob = float(lane_line_probs[1])
      self.rll_prob = float(lane_line_probs[2])
      self.lll_std = float(lane_line_stds[1])
      self.rll_std = float(lane_line_stds[2])
    else:
      self.lll_prob = 0.0
      self.rll_prob = 0.0
      self.lll_std = 1.0
      self.rll_std = 1.0

    # modelV2 also carries two road edges and their standard deviations. They
    # are parsed independently so lane failure does not erase edge geometry.
    road_edge_data = None
    try:
      road_edges = md.roadEdges
      road_edge_stds = as_finite_vector(md.roadEdgeStds, minimum_size=2)
      if len(road_edges) == 2 and road_edge_stds is not None:
        road_edge_data = tuple(
          as_finite_vector(values, expected_size=TRAJECTORY_SIZE)
          for values in (road_edges[0].x, road_edges[1].x,
                         road_edges[0].y, road_edges[1].y)
        )
    except Exception:
      road_edge_data = None

    if (road_edge_data is not None and
        all(values is not None for values in road_edge_data)):
      left_edge_x, right_edge_x, left_edge_y, right_edge_y = road_edge_data
      self.le_x = left_edge_x
      self.re_x = right_edge_x
      self.le_y = left_edge_y + self.total_camera_offset
      self.re_y = right_edge_y + self.total_camera_offset
      self.le_std = float(road_edge_stds[0])
      self.re_std = float(road_edge_stds[1])
    else:
      self.le_x.fill(0.0)
      self.re_x.fill(0.0)
      self.le_y.fill(0.0)
      self.re_y.fill(0.0)
      self.le_std = 1.0
      self.re_std = 1.0

    left_desire_idx = log.LateralPlan.Desire.laneChangeLeft
    right_desire_idx = log.LateralPlan.Desire.laneChangeRight
    desire_state = as_finite_vector(
      md.meta.desireState,
      minimum_size=max(left_desire_idx, right_desire_idx) + 1)
    if desire_state is not None:
      self.l_lane_change_prob = float(desire_state[left_desire_idx])
      self.r_lane_change_prob = float(desire_state[right_desire_idx])
    else:
      self.l_lane_change_prob = 0.0
      self.r_lane_change_prob = 0.0

  @staticmethod
  def _lane_head_gap(path_xyz, lane_x, lane_path_y):
    """Distance between the two model heads at LANE_PATH_DISAGREE_X_M.

    Returns 0.0 (heads agree, nothing withheld) on any malformed input so a
    parsing problem can never silently reduce lane reliance.
    """
    try:
      px = np.asarray(path_xyz[:, 0], dtype=float)
      py = np.asarray(path_xyz[:, 1], dtype=float)
      lx = np.asarray(lane_x, dtype=float)
      ly = np.asarray(lane_path_y, dtype=float)
    except (TypeError, ValueError, IndexError):
      return 0.0
    for xs in (px, lx):
      if xs.size < 2 or not np.all(np.isfinite(xs)) or not np.all(np.diff(xs) > 0):
        return 0.0
      if xs[0] > LANE_PATH_DISAGREE_X_M or xs[-1] < LANE_PATH_DISAGREE_X_M:
        return 0.0
    if not (np.all(np.isfinite(py)) and np.all(np.isfinite(ly))):
      return 0.0
    model_y = float(np.interp(LANE_PATH_DISAGREE_X_M, px, py))
    lane_y = float(np.interp(LANE_PATH_DISAGREE_X_M, lx, ly))
    return abs(lane_y - model_y)

  def get_d_path(self, v_ego, path_t, path_xyz, measured_curvature=0.0,
                 lane_change_active=False, readiness_eligible=False,
                 readiness_quality=0.0, imu_curvature=None,
                 imu_curvature_valid=False):
    del path_t
    path_xyz[:, 1] += self.path_offset
    self._readiness_eligible = bool(readiness_eligible)
    self._readiness_quality = float(np.clip(readiness_quality, 0.0, 1.0))

    measured_curve_strength = interp(
      abs(float(measured_curvature)),
      [CURVE_ASSIST_START_CURVATURE, CURVE_ASSIST_FULL_CURVATURE],
      [0.0, 1.0])
    speed_curve_weight = interp(
      v_ego, [CURVE_ASSIST_FULL_BELOW_MS, CURVE_ASSIST_ZERO_ABOVE_MS],
      [1.0, 0.0])

    # Detect the curve before the car has fully rotated into it. The model path
    # and road edges can remain informative even when lane confidence is weak.
    # Skipped during a lane change, where the result would be discarded below
    # anyway.
    pre_curve_strength = float(measured_curve_strength)
    if not lane_change_active:
      model_curve_strength = self._model_path_curve_strength(path_xyz)
      pre_curve_strength = max(
        pre_curve_strength,
        MODEL_PATH_CURVE_WEIGHT * model_curve_strength,
        ROAD_EDGE_CURVE_WEIGHT * self._road_edge_curve_strength())
      # Refresh the third-line model-path cache whenever the model itself is
      # confidently showing a bend, regardless of lane confidence -- see the
      # MODEL_PATH_HOLD_* comment above the constants.
      if model_curve_strength >= MODEL_PATH_HOLD_STORE_STRENGTH:
        self._store_model_path_hold(path_xyz[:, 0], path_xyz[:, 1])
    pre_curve_assist = float(np.clip(
      pre_curve_strength * speed_curve_weight, 0.0, 1.0))
    # Publish the pre-curve value now so the fallback early-returns below still
    # report an assist level; the lane-geometry path overwrites it once the
    # richer estimate is available.
    self.curve_assist_diag = pre_curve_assist

    width_pts = self.rll_y - self.lll_y
    geometry_valid = (
      np.isfinite(self.ll_x) & np.isfinite(self.lll_y) &
      np.isfinite(self.rll_y) & np.isfinite(width_pts)
    )
    if np.count_nonzero(geometry_valid) < 2:
      if self._apply_missing_lane_fallback(
          path_xyz, v_ego, pre_curve_assist,
          measured_curvature, lane_change_active,
          imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid):
        return path_xyz

      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
      self._prev_center_delta_m = 0.0
      self._fallback_mode_active = False
      cloudlog.warning("Lateral mpc - incomplete laneline x/y geometry, ignoring")
      return path_xyz

    lane_x = self.ll_x[geometry_valid]
    lane_left_y = self.lll_y[geometry_valid]
    lane_right_y = self.rll_y[geometry_valid]
    lane_width_pts = width_pts[geometry_valid]

    # np.interp requires increasing x. Sorting here is the EON spatial-output
    # compatibility layer and also prevents one malformed point from reversing
    # the inferred lane center.
    lane_order = np.argsort(lane_x)
    lane_x = lane_x[lane_order]
    lane_left_y = lane_left_y[lane_order]
    lane_right_y = lane_right_y[lane_order]
    lane_width_pts = lane_width_pts[lane_order]
    lane_x, unique_indices = np.unique(lane_x, return_index=True)
    lane_left_y = lane_left_y[unique_indices]
    lane_right_y = lane_right_y[unique_indices]
    lane_width_pts = lane_width_pts[unique_indices]
    if lane_x.size < 2:
      if self._apply_missing_lane_fallback(
          path_xyz, v_ego, pre_curve_assist,
          measured_curvature, lane_change_active,
          imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid):
        return path_xyz

      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
      self._prev_center_delta_m = 0.0
      self._fallback_mode_active = False
      return path_xyz

    width_samples = np.abs(np.interp(
      np.asarray(LANE_WIDTH_CHECK_DISTANCES_M, dtype=float), lane_x, lane_width_pts))
    width_mod = min(
      interp(width, [LANE_WIDTH_MIN_START_M, LANE_WIDTH_MIN_END_M], [0.0, 1.0]) *
      interp(width, [LANE_WIDTH_MOD_START_M, LANE_WIDTH_MOD_END_M], [1.0, 0.0])
      for width in width_samples
    )
    l_prob = self.lll_prob * width_mod
    r_prob = self.rll_prob * width_mod
    l_prob *= interp(self.lll_std,
                     [LANE_STD_FULL_CONFIDENCE, LANE_STD_ZERO_CONFIDENCE],
                     [1.0, 0.0])
    r_prob *= interp(self.rll_std,
                     [LANE_STD_FULL_CONFIDENCE, LANE_STD_ZERO_CONFIDENCE],
                     [1.0, 0.0])

    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = float(np.median(width_samples))
    # Only learn from a frame that can actually see both edges of the lane.
    # Updating unconditionally let a curve, where one line is half gone, drag
    # the estimate toward whatever the remaining geometry implied.
    if l_prob * r_prob >= LANE_WIDTH_TRUST_PROB:
      self.lane_width_estimate.update(current_lane_width)
      self.lane_width_age_s = 0.0
    else:
      self.lane_width_age_s = min(
        self.lane_width_age_s + DT_MDL, LANE_WIDTH_HOLD_S + LANE_WIDTH_DECAY_S)
    speed_lane_width = interp(v_ego, [0.0, 31.0], [2.8, 3.5])
    # A lane does not change width in the seconds it takes to round a bend, so
    # hold the last real measurement across the dip and only decay toward the
    # speed default once it is genuinely stale.
    hold = interp(self.lane_width_age_s,
                  [LANE_WIDTH_HOLD_S, LANE_WIDTH_HOLD_S + LANE_WIDTH_DECAY_S],
                  [1.0, 0.0])
    self.lane_width = (
      hold * self.lane_width_estimate.x + (1.0 - hold) * speed_lane_width
    )

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = lane_left_y + clipped_lane_width / 2.0
    path_from_right_lane = lane_right_y - clipped_lane_width / 2.0
    lane_path_y = (
      l_prob * path_from_left_lane + r_prob * path_from_right_lane
    ) / (l_prob + r_prob + 0.0001)

    # d_prob is built only from the model's own lane-line confidence and never
    # checks whether the lane-line head agrees with the end-to-end path head.
    # When they contradict each other, withhold the confidence boost so the
    # blend falls back to the ordinary combined probability. This can only
    # lower reliance to the un-boosted value, never below it.
    self.lane_head_gap_m = self._lane_head_gap(path_xyz, lane_x, lane_path_y)
    self.lane_head_disagree = bool(self.lane_head_gap_m > LANE_PATH_DISAGREE_M)
    raw_target_d_prob = enhance_lane_probability(
      combined_lane_probability(l_prob, r_prob), not self.lane_head_disagree)
    if self.lane_head_disagree:
      # Measured envelope: across 4444 frames with lane confidence >= 0.90 the
      # heads never sat more than 0.996 m apart, so past 1.0 m the lane head is
      # outside its own normal range. Shift weight to the path head in
      # proportion to how far outside, down to the LANE_PATH_MIN_TRUST floor.
      raw_target_d_prob = min(raw_target_d_prob,
                              lane_head_trust_cap(self.lane_head_gap_m))
    geometry_plausible = bool(
      np.all(width_samples >= LANE_WIDTH_MIN_END_M) and
      np.all(width_samples <= LANE_WIDTH_MOD_END_M)
    )
    # Sanity floor for *caching* a lane shape.
    # Drive-log measurement: on straights raw_target_d_prob is healthy (76% of
    # frames pass 0.35) but geometry_plausible only passed 53%, and every
    # rejection is "too narrow" -- the failing frames cluster at a 2.37 m median
    # width, just under the old 2.5 m floor, at 5/10/20 m alike (only 4.8% of
    # rejections are the far sample alone). A threshold that bisects the
    # device's own width distribution is mis-placed for this camera rather than
    # a sign of bad geometry, and it was starving the cache on exactly the
    # straight approach where the lane is still clearly visible.
    #
    # That same finding has since been re-measured and applied to
    # LANE_WIDTH_MIN_END_M itself (see its comment), so this floor is no longer
    # the more permissive of the two -- the two now coincide at 2.2 m. It stays
    # a separate name because it gates a different decision: what we are willing
    # to remember for <=1.2 s, rather than how much lane data is trusted for
    # present steering. The cached path is still applied with strength fading
    # plus the max_correction clamps.
    store_geometry_plausible = bool(
      np.all(width_samples >= STORE_LANE_WIDTH_MIN_M) and
      np.all(width_samples <= LANE_WIDTH_MOD_END_M)
    )
    single_lane_usable = bool(
      (self.lll_prob >= SINGLE_LANE_MIN_RAW_PROB and
       self.lll_std <= SINGLE_LANE_MAX_STD) or
      (self.rll_prob >= SINGLE_LANE_MIN_RAW_PROB and
       self.rll_std <= SINGLE_LANE_MAX_STD)
    )

    # Continuity must use raw geometry, not the probability-weighted path.
    # With both probabilities at zero the weighted path collapses toward zero
    # and could otherwise hide a large one-frame lane-line jump.
    lane_center_refs = 0.5 * (
      np.interp(LANE_WIDTH_CHECK_DISTANCES_M, lane_x, lane_left_y) +
      np.interp(LANE_WIDTH_CHECK_DISTANCES_M, lane_x, lane_right_y)
    )

    # Tight-curve assist is strongest through 45 km/h and fades back to the
    # original planner behavior by 60 km/h. Combine measured curvature, lane
    # bend, road-edge bend and the fresh model-path bend so weak lane paint
    # does not delay curve recognition.
    lane_bend_m = float(np.max(np.abs(lane_center_refs - lane_center_refs[0])))
    geometry_curve_strength = interp(
      lane_bend_m, [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M],
      [0.0, 1.0])
    combined_curve_strength = max(
      pre_curve_strength, geometry_curve_strength)
    curve_assist = float(np.clip(
      combined_curve_strength * speed_curve_weight, 0.0, 1.0))
    self.curve_assist_diag = curve_assist
    self.curve_raw_target_d_prob_diag = float(raw_target_d_prob)
    # Report the floor the store gate actually uses, so drive logs keep
    # measuring the condition that governs caching.
    self.curve_geometry_plausible_diag = store_geometry_plausible

    fallback_max_speed = interp(
      curve_assist, [0.0, 1.0],
      [LOW_SPEED_FALLBACK_MAX_MS, CURVE_FALLBACK_MAX_MS])
    fallback_dprob_floor = interp(
      curve_assist, [0.0, 1.0],
      [SINGLE_LANE_DPROB_FLOOR, CURVE_SINGLE_LANE_DPROB_FLOOR])
    fallback_active = bool(
      not lane_change_active and v_ego <= fallback_max_speed and
      geometry_plausible and single_lane_usable and
      raw_target_d_prob < fallback_dprob_floor
    )
    target_d_prob = max(raw_target_d_prob, fallback_dprob_floor) \
      if fallback_active else raw_target_d_prob

    fresh_lane_candidate = bool(
      self._fallback_mode_active and
      not lane_change_active and
      geometry_plausible and
      raw_target_d_prob >= FRESH_LANE_RECOVERY_DPROB
    )
    if fresh_lane_candidate:
      self._fresh_lane_recovery_frames += 1
    else:
      self._fresh_lane_recovery_frames = 0

    fresh_lane_recovered = bool(
      self._fallback_mode_active and
      self._fresh_lane_recovery_frames >= FRESH_LANE_RECOVERY_FRAMES
    )
    if fresh_lane_recovered:
      # Do not discard the deep-curve history on a single flickering lane frame.
      # Switch back only after ~0.20 s of continuously trustworthy lane data.
      self._clear_curve_temporal_hold()
      self._fallback_mode_active = False
      self._fresh_lane_recovery_frames = 0
      self.d_prob = float(np.clip(target_d_prob, 0.0, 1.0))
      self._last_lane_center_refs = lane_center_refs.copy()
    else:
      self.d_prob = self._update_d_prob(
        target_d_prob, v_ego, lane_center_refs,
        lane_change_active, curve_assist)

    lane_path_y_interp = np.interp(path_xyz[:, 0], lane_x, lane_path_y)
    if target_d_prob < 0.50:
      correction_near = interp(
        curve_assist, [0.0, 1.0],
        [LOW_CONFIDENCE_MAX_CORRECTION_NEAR_M,
         CURVE_LOW_CONFIDENCE_MAX_CORRECTION_NEAR_M])
      correction_far = interp(
        curve_assist, [0.0, 1.0],
        [LOW_CONFIDENCE_MAX_CORRECTION_M,
         CURVE_LOW_CONFIDENCE_MAX_CORRECTION_M])
      max_correction = np.interp(
        np.abs(path_xyz[:, 0]), [0.0, 20.0],
        [correction_near, correction_far])
      lane_path_y_interp = path_xyz[:, 1] + np.clip(
        lane_path_y_interp - path_xyz[:, 1], -max_correction, max_correction)

    center_delta = lane_path_y_interp - path_xyz[:, 1]

    # Cache any trustworthy frame, not only ones already recognized as curving.
    # A drive-log gate analysis (curveAssist/curveRawTargetDProb diagnostics)
    # showed curve_assist is essentially never the blocker on a sharp curve
    # (>=0.30 on 93% of sharp-curvature frames), but raw_target_d_prob is: it
    # never once reached the 0.35 store threshold while |curvature|>=0.025
    # (median 0.018, 0/317 frames passed). That is expected, not a bug -- by
    # the time this narrow EON FOV shows a sharp bend, the lane lines it needs
    # to measure confidently are already leaving the frame. Requiring
    # curve_assist here waits for that same moment before trying to store,
    # which is structurally too late. Storing continuously instead means the
    # most recent confident lane shape (from a moment ago, while still
    # straight/gentle, where raw_target_d_prob passes 80%/43% of the time) is
    # already cached and only a frame or two old once curve_assist ramps up
    # and the dropout below starts consuming it.
    trusted_curve_frame = bool(
      self.curve_fallback_enabled and
      not lane_change_active and
      store_geometry_plausible and
      raw_target_d_prob >= CURVE_TEMPORAL_STORE_DPROB
    )
    # Start fallback gradually as confidence falls below the temporal-store
    # threshold (0.35), reaching full fallback by the trigger threshold (0.22).
    # This removes the previous 0.22-0.35 dead zone.
    visual_dropout = bool(
      self.curve_fallback_enabled and
      not lane_change_active and
      curve_assist >= CURVE_TEMPORAL_MIN_ASSIST and
      raw_target_d_prob < CURVE_TEMPORAL_STORE_DPROB
    )

    if trusted_curve_frame:
      self._store_curve_temporal_hold(
        path_xyz[:, 0], lane_path_y_interp,
        curve_assist, measured_curvature, v_ego=v_ego,
        imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid)
      self._fallback_mode_active = False
    elif visual_dropout:
      dropout_weight = float(np.clip(
        interp(
          raw_target_d_prob,
          [CURVE_TEMPORAL_TRIGGER_DPROB, CURVE_TEMPORAL_STORE_DPROB],
          [1.0, 0.0]),
        0.0, 1.0))

      # Deep-curve priority: keep the last trustworthy lane shape first.
      predicted_lane_y, temporal_strength = self._curve_temporal_prediction(
        path_xyz[:, 0], v_ego, curve_assist,
        measured_curvature, lane_change_active,
        imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid)
      if predicted_lane_y is not None and temporal_strength > 0.0:
        predicted_lane_y = self._bound_curve_temporal_path(
          path_xyz, predicted_lane_y)
        temporal_weight = float(np.clip(
          dropout_weight * temporal_strength, 0.0, 1.0))
        lane_path_y_interp = (
          temporal_weight * predicted_lane_y +
          (1.0 - temporal_weight) * lane_path_y_interp
        )
        self.d_prob = max(
          self.d_prob, CURVE_TEMPORAL_DPROB_FLOOR * temporal_weight)
        center_delta = lane_path_y_interp - path_xyz[:, 1]
        self.curve_fallback_source_diag = 2
        self._fallback_mode_active = True
        self._fresh_lane_recovery_frames = 0
      else:
        edge_path, edge_strength = self._road_edge_fallback_path(
          path_xyz, v_ego, curve_assist, lane_change_active)
        if edge_path is not None and edge_strength > 0.0:
          edge_weight = float(np.clip(
            dropout_weight * edge_strength, 0.0, 1.0))
          lane_path_y_interp = (
            edge_weight * edge_path +
            (1.0 - edge_weight) * lane_path_y_interp
          )
          self.d_prob = max(
            self.d_prob, ROAD_EDGE_DPROB_FLOOR * edge_weight)
          center_delta = lane_path_y_interp - path_xyz[:, 1]
          self.curve_fallback_source_diag = 3
          self._fallback_mode_active = True
          self._fresh_lane_recovery_frames = 0
          self._age_curve_temporal_hold(v_ego)
        else:
          model_path_y, model_path_strength = self._model_path_prediction(
            path_xyz[:, 0], v_ego, measured_curvature, lane_change_active,
            imu_curvature=imu_curvature, imu_curvature_valid=imu_curvature_valid)
          if model_path_y is not None and model_path_strength > 0.0:
            max_correction = self._trusted_max_correction(
              np.abs(path_xyz[:, 0]),
              MODEL_PATH_MAX_CORRECTION_NEAR_M, MODEL_PATH_MAX_CORRECTION_M,
              MODEL_PATH_TRUSTED_MAX_CORRECTION_NEAR_M, MODEL_PATH_TRUSTED_MAX_CORRECTION_M)
            model_path_y = path_xyz[:, 1] + np.clip(
              model_path_y - path_xyz[:, 1], -max_correction, max_correction)
            model_weight = float(np.clip(
              dropout_weight * model_path_strength, 0.0, 1.0))
            lane_path_y_interp = (
              model_weight * model_path_y +
              (1.0 - model_weight) * lane_path_y_interp
            )
            self.d_prob = max(
              self.d_prob, MODEL_PATH_DPROB_FLOOR * model_weight)
            center_delta = lane_path_y_interp - path_xyz[:, 1]
            self.curve_fallback_source_diag = 4
            self._fallback_mode_active = True
            self._fresh_lane_recovery_frames = 0
            self._age_curve_temporal_hold(v_ego)
    else:
      if lane_change_active or v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
        self._clear_curve_temporal_hold()
        self._clear_model_path_hold()
      else:
        self._age_curve_temporal_hold(v_ego)
        self._age_model_path_hold(v_ego)
      if lane_change_active:
        self._fallback_mode_active = False
        self._fresh_lane_recovery_frames = 0

    # Bound ordinary lane blending on every frame, not only the low-confidence
    # ones handled above, then slew-limit what is left. Shape is preserved by
    # scaling the whole profile with the ratio the 20 m sample was limited by,
    # so the path stays smooth instead of developing a kink at the clamp.
    center_max = np.interp(
      np.abs(path_xyz[:, 0]), [0.0, 20.0],
      [LANE_CENTER_MAX_CORRECTION_NEAR_M, LANE_CENTER_MAX_CORRECTION_M])
    center_delta = np.clip(center_delta, -center_max, center_max)

    center_delta_ref = float(np.interp(20.0, path_xyz[:, 0], center_delta))
    max_step = LANE_CENTER_MAX_CORRECTION_RATE_MS * DT_MDL
    limited_ref = float(np.clip(
      center_delta_ref,
      self._prev_center_delta_m - max_step,
      self._prev_center_delta_m + max_step))
    if abs(center_delta_ref) > 1e-6 and limited_ref != center_delta_ref:
      center_delta = center_delta * (limited_ref / center_delta_ref)
    self._prev_center_delta_m = limited_ref
    lane_path_y_interp = path_xyz[:, 1] + center_delta

    self.lane_center_correction_m = float(self.d_prob * limited_ref)
    self.lane_center_correction_active = bool(
      self.d_prob > 0.05 and
      abs(self.lane_center_correction_m) > 0.01)
    # Only ordinary lane blending reached here; a fallback above already
    # claimed the frame and must keep its attribution.
    if self.curve_fallback_source_diag == 0 and self.lane_center_correction_active:
      self.curve_fallback_source_diag = 1
    path_xyz[:, 1] = (
      self.d_prob * lane_path_y_interp +
      (1.0 - self.d_prob) * path_xyz[:, 1]
    )
    return path_xyz
