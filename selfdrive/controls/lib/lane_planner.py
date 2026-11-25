"""Official-style lane path blending with EON spatial-model compatibility."""

import numpy as np

from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
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
# The same distances as an array: built once instead of per call (three a frame).
_LANE_WIDTH_CHECK_X = np.asarray(LANE_WIDTH_CHECK_DISTANCES_M, dtype=float)


def _median(values):
  """np.median for a small finite 1-D array, without its partition machinery
  (~0.3 ms a call on the EON). Same value: the middle element, or the mean of
  the two middle ones computed as (a + b) / 2 as np.median does. Anything
  non-finite goes to np.median for its NaN handling."""
  n = values.size
  if n == 0 or not np.isfinite(values.sum()):
    return float(np.median(values))
  s = np.sort(values)
  h = n // 2
  return float(s[h]) if n % 2 else float((s[h - 1] + s[h]) / 2.0)

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
# (4.8% of frames).
LANE_WIDTH_MIN_END_M = 2.2
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
# Was 0.60: in a curve a single weak line (prob 0.2-0.5) was trusted at 60%.
# Those lines are the flattened far end of a lane the narrow camera is losing,
# so their centre sits outside the bend -- 0.5-0.9 m outside the model path in
# tight corners, where with both lines clearly visible the two agree within
# ~0.16 m. Replaying this planner on the 2026-11-18..21 drives (192 min; live
# setting reproduces the logged dPathPoints to 3 mm median) the floor pulled
# the plan 0.17 m (p10 0.32) toward the outside of the bend in those frames,
# 8% of corner time. At 0.30, the same as on a straight, the pull is 0.11 m;
# straights are untouched and 5-6% of corner/bend frames move by > 5 cm.
CURVE_SINGLE_LANE_DPROB_FLOOR = 0.30
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

# Final output slew limit on how fast the planned path may move sideways
# relative to the model path, applied after every blend stage (see
# _limit_output_rate). 1.0 m/s = 5 cm per 20 Hz frame: a full 1 m correction
# still completes in 1 s, but no single frame can jump the path 0.3-1 m any
# more, which is what produced the sudden left/right swerves. Faster during a
# lane change so the lane-line fade-out is not held back.
OUTPUT_OFFSET_MAX_RATE_MS = 1.0
OUTPUT_OFFSET_MAX_RATE_LC_MS = 2.0

# How far a road edge is trusted, by its reported std. Used by the curve
# detector (_road_edge_curve_strength); there is no road-edge path fallback.
ROAD_EDGE_STD_FULL_CONFIDENCE = 0.20
ROAD_EDGE_STD_ZERO_CONFIDENCE = 0.90
ROAD_EDGE_MIN_CONFIDENCE = 0.30

# Wrong-lane-line guard. Real lane lines cannot move sideways 0.35 m (0.55 m in
# a curve) relative to the car in one 50 ms frame -- the car itself moves a few
# cm per frame sideways at most. A jump that size means the model switched to a
# different line (adjacent lane line, tar seam, worn paint, merge/exit marking,
# road edge), and its probability usually stays high while it does, so the
# confidence-based logic happily steered to it. Once a trusted lane jumps, lane
# reliance is dropped to zero (pure model path, the stock laneless behaviour)
# until the new lane geometry is both:
#   * stable: continuous frame-to-frame for LANE_JUMP_REACCEPT_FRAMES
#     (LANE_JUMP_RETURN_FRAMES if it came back to where the trusted lane was), and
#   * consistent with the model's own path: lane centre within
#     LANE_JUMP_REACCEPT_GAP_M of the model path 5 m ahead
#   (or it simply returned to where the trusted lane was).
# The jump itself is measured relative to the model path at 5/10 m, so curves,
# where lane and model path bend together, do not trigger it.
# A new line that keeps disagreeing with where the model wants to drive is
# never re-trusted, however confident it looks.
LANE_JUMP_TRUSTED_DPROB = 0.35
LANE_JUMP_REACCEPT_FRAMES = 20   # 1.0 s
LANE_JUMP_RETURN_FRAMES = 6      # 0.3 s
LANE_JUMP_REACCEPT_GAP_M = 0.40
# Near distances where the lane-vs-model offset is compared. A real line
# cannot shift this much relative to the model path in 50 ms.
LANE_JUMP_CHECK_X_M = (5.0, 10.0)
LANE_JUMP_MAX_STEP_M = 0.35
LANE_JUMP_CURVE_MAX_STEP_M = 0.55

# Curve detection can use multiple independent visual sources. Lane geometry
# remains the strongest visual cue; model-path and road-edge bend are slightly
# down-weighted so they can start curve assist early without dominating it.
MODEL_PATH_CURVE_WEIGHT = 0.85
ROAD_EDGE_CURVE_WEIGHT = 0.90


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


    # Compatibility fields retained for the existing lateralPlan schema.
    self.lane_center_correction_m = 0.0
    self.lane_center_correction_active = False
    self._last_lane_center_refs = None
    # Previous frame's applied lane-center offset at 20 m, for the slew limit.
    # Reset whenever lane blending is skipped so returning to it does not look
    # like a step change to the rate limiter.
    self._prev_center_delta_m = 0.0
    # Last applied (output - model) lateral offset profile, for the final
    # output slew limiter.
    self._prev_output_x = None
    self._prev_output_delta = None

    # Diagnostic-only curve readings published in lateralPlan. They never feed
    # control; they show in a drive log how strongly a curve was detected and
    # how far lane blending was trusted.
    self.curve_assist_diag = 0.0
    self.curve_raw_target_d_prob_diag = 0.0
    self.curve_geometry_plausible_diag = False
    # 0 none, 1 ordinary lane blending (2-4 were the removed curve fallbacks)
    self.curve_fallback_source_diag = 0
    self.lane_head_disagree = False
    self.lane_head_gap_m = 0.0

    # Wrong-lane-line guard state (see _update_lane_jump_guard).
    self.lane_jump_suspect = False
    self._lane_guard_prev_refs = None
    self._lane_guard_prev_trusted = False
    self._lane_guard_good_refs = None
    self._lane_guard_stable_frames = 0

    # Road-edge geometry from modelV2, read by the curve detector only.
    self.le_x = np.zeros((TRAJECTORY_SIZE,))
    self.re_x = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_std = 1.0
    self.re_std = 1.0

  @staticmethod
  def _finite_sorted_unique(x_values, y_values):
    """Filter to finite points, sort by x, and drop duplicate x.

    Shared prep used everywhere a spatial x/y series (lane path, road edge,
    output offset) needs to become valid np.interp input. Returns
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

    # Model x series are almost always strictly increasing already, and then
    # argsort is the identity and unique keeps every point: return them as
    # they are (~0.25 ms a call less on the EON, three or four calls a frame).
    if np.all(x_values[1:] > x_values[:-1]):
      return x_values, y_values
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

    Used by _road_edge_curve_strength (early curve detector). refs/strength
    are None/0.0 when the edge is unavailable or
    below ROAD_EDGE_MIN_CONFIDENCE -- every caller already re-checks
    confidence before using refs, so this only avoids wasted work.
    """
    x, y = self._finite_sorted_unique(edge_x, edge_y)
    confidence = interp(
      edge_std, [ROAD_EDGE_STD_FULL_CONFIDENCE, ROAD_EDGE_STD_ZERO_CONFIDENCE],
      [1.0, 0.0])
    if x is None or confidence < ROAD_EDGE_MIN_CONFIDENCE:
      return x, y, confidence, None, 0.0

    refs = np.interp(_LANE_WIDTH_CHECK_X, x, y)
    bend_m = float(np.max(np.abs(refs - refs[0])))
    strength = float(min(max(
      interp(bend_m, [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M], [0.0, 1.0]),
      0.0), 1.0))
    return x, y, confidence, refs, strength

  def _update_d_prob(self, target_d_prob, v_ego, lane_center_refs,
                     lane_change_active, curve_assist):
    """Rate-limit short confidence dropouts, with extra continuity on tight low-speed curves."""
    # Scalar clips as min/max: np.clip is ~136 us a call on the EON's numpy
    # against ~4 us, and gives the same result (NaN included).
    target_d_prob = float(min(max(target_d_prob, 0.0), 1.0))
    curve_assist = float(min(max(curve_assist, 0.0), 1.0))
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

  def _spatial_curve_strength(self, x_values, y_values):
    """Estimate visible bend strength from spatial x/y points."""
    x_values, y_values = self._finite_sorted_unique(x_values, y_values)
    if x_values is None:
      return 0.0

    refs = np.interp(
      _LANE_WIDTH_CHECK_X,
      x_values, y_values)
    bend_m = float(np.max(np.abs(refs - refs[0])))
    return float(min(max(
      interp(
        bend_m,
        [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M],
        [0.0, 1.0]),
      0.0), 1.0))

  def _model_path_curve_strength(self, path_xyz):
    """Use the fresh model path as an early curve cue when lanes are weak."""
    if path_xyz.ndim != 2 or path_xyz.shape[0] < 2 or path_xyz.shape[1] < 2:
      return 0.0
    return self._spatial_curve_strength(path_xyz[:, 0], path_xyz[:, 1])

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

    return float(min(max(best_strength, 0.0), 1.0))


  def parse_model(self, md):
    # Reset the per-frame store-gate diagnostics here rather than in get_d_path,
    # which is skipped entirely when lane lines are disabled. Without this the
    # published values would go stale instead of reading as "not evaluated".
    self.curve_assist_diag = 0.0
    self.curve_raw_target_d_prob_diag = 0.0
    self.curve_geometry_plausible_diag = False
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

  def _clear_lane_jump_guard(self):
    self.lane_jump_suspect = False
    self._lane_guard_good_refs = None
    self._lane_guard_stable_frames = 0

  def _update_lane_jump_guard(self, path_xyz, lane_x, lane_left_y,
                              lane_right_y, raw_target_d_prob, curve_assist,
                              lane_change_active):
    """Return True while the current lane lines must not be trusted.

    The jump is measured on the lane centre RELATIVE to the model path, at the
    near distances only (LANE_JUMP_CHECK_X_M). In a curve the lane and the model
    path bend together, so this relative offset barely moves even in a tight
    intersection turn, while the absolute 20 m point moves a lot -- measuring
    the absolute position made the guard fire through ordinary tight curves
    (R25 m at 25 km/h: lane ignored for 5 s, curve error doubled). A switch to
    a different line moves the relative offset by a whole line spacing at once.
    """
    check_x = np.asarray(LANE_JUMP_CHECK_X_M, dtype=float)
    try:
      model_near = np.interp(check_x, path_xyz[:, 0], path_xyz[:, 1])
      lane_near = 0.5 * (np.interp(check_x, lane_x, lane_left_y) +
                         np.interp(check_x, lane_x, lane_right_y))
      rel = lane_near - model_near
    except (TypeError, ValueError, IndexError):
      return self.lane_jump_suspect
    if not np.all(np.isfinite(rel)):
      return self.lane_jump_suspect

    prev = self._lane_guard_prev_refs
    prev_trusted = self._lane_guard_prev_trusted
    self._lane_guard_prev_refs = rel.copy()
    self._lane_guard_prev_trusted = bool(
      raw_target_d_prob >= LANE_JUMP_TRUSTED_DPROB)

    if lane_change_active:
      # A lane change moves the lines by a full lane on purpose.
      self._clear_lane_jump_guard()
      return False

    # Looser in a curve (0.55 m), still well under a line spacing (~2.4 m on
    # this camera, 0.9 m+ for any real line switch seen so far).
    max_step = interp(float(min(max(curve_assist, 0.0), 1.0)), [0.0, 1.0],
                      [LANE_JUMP_MAX_STEP_M, LANE_JUMP_CURVE_MAX_STEP_M])
    jumped = bool(prev is not None and np.max(np.abs(rel - prev)) > max_step)

    if not self.lane_jump_suspect:
      if jumped and prev_trusted:
        self.lane_jump_suspect = True
        self._lane_guard_good_refs = prev.copy()
        self._lane_guard_stable_frames = 0
        cloudlog.warning("lane line jumped %.2f m in one frame; ignoring lane lines"
                         % float(np.max(np.abs(rel - prev))))
      return self.lane_jump_suspect

    self._lane_guard_stable_frames = 0 if jumped else self._lane_guard_stable_frames + 1
    # Back where the trusted lane was (relative to the model path): quick.
    returned = bool(self._lane_guard_good_refs is not None and
                    np.max(np.abs(rel - self._lane_guard_good_refs)) <= max_step)
    # A new line is only taken if it agrees with the model path up close.
    agrees = bool(abs(rel[0]) <= LANE_JUMP_REACCEPT_GAP_M)
    if returned and self._lane_guard_stable_frames >= LANE_JUMP_RETURN_FRAMES:
      self._clear_lane_jump_guard()
    elif agrees and self._lane_guard_stable_frames >= LANE_JUMP_REACCEPT_FRAMES:
      self._clear_lane_jump_guard()
    return self.lane_jump_suspect

  def reset_output_limiter(self):
    """Forget the last applied offset (e.g. after the planner overrode the
    blend with the raw model path), so the limiter ramps from what the car
    was actually given."""
    self._prev_output_x = None
    self._prev_output_delta = None

  def _limit_output_rate(self, path_x, delta, lane_change_active):
    """Final per-point slew limit on (output path - model path).

    The stages above can still change this offset in one frame: d_prob drops
    straight to target on a lane-centre discontinuity, and the lane-jump guard
    zeroes it at once. Each of those is a step of up to ~1 m in the target
    path, which the MPC turns into a sudden swerve. This is the single place
    they converge, so bound it here.
    """
    delta = np.asarray(delta, dtype=float)
    rate = OUTPUT_OFFSET_MAX_RATE_LC_MS if lane_change_active else OUTPUT_OFFSET_MAX_RATE_MS
    max_step = rate * DT_MDL
    limited = delta
    if self._prev_output_x is not None:
      prev = np.interp(np.asarray(path_x, dtype=float),
                       self._prev_output_x, self._prev_output_delta)
      limited = prev + np.minimum(np.maximum(delta - prev, -max_step), max_step)
    limited = np.where(np.isfinite(limited), limited, 0.0)
    sx, sd = self._finite_sorted_unique(path_x, limited)
    if sx is not None:
      self._prev_output_x, self._prev_output_delta = sx, sd
    return limited

  def get_d_path(self, v_ego, path_t, path_xyz, measured_curvature=0.0,
                 lane_change_active=False):
    model_y = path_xyz[:, 1] + self.path_offset
    out = self._get_d_path_unlimited(
      v_ego, path_t, path_xyz, measured_curvature=measured_curvature,
      lane_change_active=lane_change_active)
    delta = self._limit_output_rate(out[:, 0], out[:, 1] - model_y,
                                    lane_change_active)
    out[:, 1] = model_y + delta
    return out

  def _get_d_path_unlimited(self, v_ego, path_t, path_xyz, measured_curvature=0.0,
                            lane_change_active=False):
    del path_t
    path_xyz[:, 1] += self.path_offset

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
    # Scalar clips: min/max, not np.clip (~30 us each on the EON).
    pre_curve_assist = min(max(float(pre_curve_strength * speed_curve_weight), 0.0), 1.0)
    # Publish the pre-curve value now so the early returns below still report
    # an assist level; the lane-geometry path overwrites it once the richer
    # estimate is available.
    self.curve_assist_diag = pre_curve_assist

    width_pts = self.rll_y - self.lll_y
    geometry_valid = (
      np.isfinite(self.ll_x) & np.isfinite(self.lll_y) &
      np.isfinite(self.rll_y) & np.isfinite(width_pts)
    )
    if np.count_nonzero(geometry_valid) < 2:
      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
      self._prev_center_delta_m = 0.0
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
      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
      self._prev_center_delta_m = 0.0
      return path_xyz

    width_samples = np.abs(np.interp(
      _LANE_WIDTH_CHECK_X, lane_x, lane_width_pts))
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
    current_lane_width = _median(width_samples)
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
    curve_assist = min(max(float(combined_curve_strength * speed_curve_weight), 0.0), 1.0)
    self.curve_assist_diag = curve_assist
    self.curve_raw_target_d_prob_diag = float(raw_target_d_prob)
    self.curve_geometry_plausible_diag = geometry_plausible

    # Wrong-lane-line guard: never steer to a lane line that teleported.
    if self._update_lane_jump_guard(
        path_xyz, lane_x, lane_left_y, lane_right_y,
        raw_target_d_prob, curve_assist, lane_change_active):
      raw_target_d_prob = 0.0
      self.curve_raw_target_d_prob_diag = 0.0

    fallback_max_speed = interp(
      curve_assist, [0.0, 1.0],
      [LOW_SPEED_FALLBACK_MAX_MS, CURVE_FALLBACK_MAX_MS])
    fallback_dprob_floor = interp(
      curve_assist, [0.0, 1.0],
      [SINGLE_LANE_DPROB_FLOOR, CURVE_SINGLE_LANE_DPROB_FLOOR])
    fallback_active = bool(
      not lane_change_active and v_ego <= fallback_max_speed and
      not self.lane_jump_suspect and
      geometry_plausible and single_lane_usable and
      raw_target_d_prob < fallback_dprob_floor
    )
    target_d_prob = max(raw_target_d_prob, fallback_dprob_floor) \
      if fallback_active else raw_target_d_prob

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
      lane_path_y_interp = path_xyz[:, 1] + np.minimum(np.maximum(
        lane_path_y_interp - path_xyz[:, 1], -max_correction), max_correction)

    center_delta = lane_path_y_interp - path_xyz[:, 1]

    # Bound ordinary lane blending on every frame, not only the low-confidence
    # ones handled above, then slew-limit what is left. Shape is preserved by
    # scaling the whole profile with the ratio the 20 m sample was limited by,
    # so the path stays smooth instead of developing a kink at the clamp.
    center_max = np.interp(
      np.abs(path_xyz[:, 0]), [0.0, 20.0],
      [LANE_CENTER_MAX_CORRECTION_NEAR_M, LANE_CENTER_MAX_CORRECTION_M])
    center_delta = np.minimum(np.maximum(center_delta, -center_max), center_max)

    center_delta_ref = float(np.interp(20.0, path_xyz[:, 0], center_delta))
    max_step = LANE_CENTER_MAX_CORRECTION_RATE_MS * DT_MDL
    limited_ref = min(max(center_delta_ref, self._prev_center_delta_m - max_step),
                      self._prev_center_delta_m + max_step)
    # That rescale divided by this frame's raw 20 m sample, so its ratio diverged
    # as the sample passed through zero -- which it does routinely, because the
    # slew limiter bites on most frames and the raw correction alternates sign at
    # 20 Hz. On 2026-09-05--09-24-53 at +378.43 s the ratio reached 5.3e4 and
    # multiplied the already-clamped (<=1 m) profile into a 52.7 m path;
    # np.interp then handed the lateral MPC a constant 52.7 m target at every
    # horizon point, the MPC carried the resulting curvature into x0, and the
    # demand ramped to 0.180 1/m (17.0 m/s^2 at 34 km/h) over the next second
    # while the model path itself never left +/-0.5 m. laneCenterCorrectionM read
    # a healthy +0.07 m throughout, which is why no drive log ever showed it.
    # 35 such frames across the big-model drives, worst 192.8 m, against one per
    # stock-model drive at a harmless 5.1 m.
    #
    # A slew limit may only shrink this frame's correction, never grow it, which
    # is exactly the |limited| <= |raw| case: cap the ratio at 1. When the raw
    # sample is numerically zero the ratio carries no information at all, so
    # shift by the (max_step-bounded) difference instead of scaling by it. The
    # magnitude clamp is then reapplied unconditionally, so nothing above
    # LANE_CENTER_MAX_CORRECTION_M can leave this function by construction.
    #
    # 2026-11-15 fix: the ratio rescale above was only one-sided. Capping the
    # ratio at 1 meant that whenever the raw correction SHRANK (|raw| < |prev|)
    # or changed sign, the limiter let it through untouched -- e.g. prev 0.90 m,
    # raw 0.05 m gave 0.05 m, a 0.85 m path jump in a single 50 ms frame, i.e.
    # the car snapping sideways. Shift the profile by the bounded difference
    # instead, which limits rises and falls alike and cannot diverge.
    if limited_ref != center_delta_ref:
      center_delta = center_delta + (limited_ref - center_delta_ref)
    center_delta = np.minimum(np.maximum(center_delta, -center_max), center_max)
    # Re-read the reference from what actually survived, so the next frame's
    # slew limit is anchored on the correction that was applied.
    limited_ref = float(np.interp(20.0, path_xyz[:, 0], center_delta))
    self._prev_center_delta_m = limited_ref
    lane_path_y_interp = path_xyz[:, 1] + center_delta

    self.lane_center_correction_m = float(self.d_prob * limited_ref)
    self.lane_center_correction_active = bool(
      self.d_prob > 0.05 and
      abs(self.lane_center_correction_m) > 0.01)
    if self.lane_center_correction_active:
      self.curve_fallback_source_diag = 1
    path_xyz[:, 1] = (
      self.d_prob * lane_path_y_interp +
      (1.0 - self.d_prob) * path_xyz[:, 1]
    )
    return path_xyz
