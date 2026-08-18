"""Official-style lane path blending with EON spatial-model compatibility."""

import numpy as np

from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from selfdrive.ntune import ntune_common_get
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lane_probability import (
  combined_lane_probability,
  enhance_lane_probability,
  limit_lane_probability_rise,
)
from selfdrive.controls.lib.model_data_validation import as_finite_vector


TRAJECTORY_SIZE = 33
PATH_OFFSET = ntune_common_get('pathOffset')
CAMERA_OFFSET = -0.055

# Official 0.8.14 uses a soft standard-deviation weight from 0.15 to 0.30.
# The EON model observed on this car reports geometrically continuous lane
# lines around 0.4-0.8, so retain a soft (bounded) contribution in that range.
LANE_STD_FULL_CONFIDENCE = 0.15
LANE_STD_ZERO_CONFIDENCE = 1.20
LANE_WIDTH_CHECK_DISTANCES_M = (5.0, 10.0, 20.0)
LANE_WIDTH_MIN_START_M = 1.8
LANE_WIDTH_MIN_END_M = 2.5
LANE_WIDTH_MOD_START_M = 4.2
LANE_WIDTH_MOD_END_M = 5.0

# Low-speed EON fallback. A single continuous lane line may briefly anchor the
# path, but its influence is capped so a weak prediction cannot pull the car
# far away from the model path.
LOW_SPEED_FALLBACK_MAX_MS = 12.0
CURVE_FALLBACK_MAX_MS = 16.0
SINGLE_LANE_MIN_RAW_PROB = 0.20
SINGLE_LANE_MAX_STD = 0.90
SINGLE_LANE_DPROB_FLOOR = 0.30
CURVE_SINGLE_LANE_DPROB_FLOOR = 0.60
LANE_CONFIDENCE_FALL_RATE_PER_S = 1.25
LANE_CENTER_CONTINUITY_MAX_M = 0.35
CURVE_LANE_CENTER_CONTINUITY_MAX_M = 0.55
CURVE_CONFIDENCE_RISE_BONUS_PER_S = 1.00
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

# Short temporal continuation for tight curves. Cache the last trustworthy
# lane-path shape, compensate it for the car's short ego-motion, hold it
# strongly for 0.10 s, then fade it out completely by 0.60 s.
CURVE_TEMPORAL_HOLD_FULL_S = 0.10
CURVE_TEMPORAL_HOLD_MAX_S = 0.60
# Speed-aware temporal horizon: keep longer history only at low speed.
CURVE_TEMPORAL_HOLD_BP_MS = (
  0.0, 30.0 / 3.6, 40.0 / 3.6, 45.0 / 3.6,
  50.0 / 3.6, 55.0 / 3.6, 60.0 / 3.6,
)
CURVE_TEMPORAL_HOLD_V_S = (0.60, 0.60, 0.50, 0.45, 0.38, 0.25, 0.0)
CURVE_TEMPORAL_MIN_ASSIST = 0.30
CURVE_TEMPORAL_STORE_DPROB = 0.35
CURVE_TEMPORAL_TRIGGER_DPROB = 0.22
CURVE_TEMPORAL_DPROB_FLOOR = 0.55
CURVE_TEMPORAL_MAX_CORRECTION_NEAR_M = 0.22
CURVE_TEMPORAL_MAX_CORRECTION_M = 0.75
CURVE_TEMPORAL_MAX_YAW_RAD = 0.24

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
FRESH_LANE_RECOVERY_DPROB = 0.45
FRESH_LANE_RECOVERY_FRAMES = 4  # ~0.20 s at 20 Hz before dropping fallback

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

    # Last trustworthy tight-curve lane path in the previous ego frame.
    self._curve_hold_x = None
    self._curve_hold_y = None
    self._curve_hold_age_s = CURVE_TEMPORAL_HOLD_MAX_S
    self._curve_hold_strength = 0.0
    self._curve_hold_curvature = 0.0
    self._curve_hold_sign = 0.0

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
      max_fall = LANE_CONFIDENCE_FALL_RATE_PER_S * DT_MDL
      next_d_prob = max(target_d_prob, self.d_prob - max_fall)

    # Do not replace the continuity reference with a fully untrusted frame.
    if target_d_prob >= 0.10:
      self._last_lane_center_refs = lane_center_refs.copy()
    return next_d_prob

  def _clear_curve_temporal_hold(self):
    self._curve_hold_x = None
    self._curve_hold_y = None
    self._curve_hold_age_s = CURVE_TEMPORAL_HOLD_MAX_S
    self._curve_hold_strength = 0.0
    self._curve_hold_curvature = 0.0
    self._curve_hold_sign = 0.0

  def _store_curve_temporal_hold(self, path_x, lane_path_y,
                                 curve_assist, measured_curvature):
    """Cache a finite, monotonic lane path for a possible short dropout."""
    path_x = np.asarray(path_x, dtype=float)
    lane_path_y = np.asarray(lane_path_y, dtype=float)
    if path_x.size < 2 or path_x.size != lane_path_y.size:
      return

    finite = np.isfinite(path_x) & np.isfinite(lane_path_y)
    path_x = path_x[finite]
    lane_path_y = lane_path_y[finite]
    if path_x.size < 2:
      return

    order = np.argsort(path_x)
    path_x = path_x[order]
    lane_path_y = lane_path_y[order]
    path_x, unique_indices = np.unique(path_x, return_index=True)
    lane_path_y = lane_path_y[unique_indices]
    if path_x.size < 2:
      return

    self._curve_hold_x = path_x.copy()
    self._curve_hold_y = lane_path_y.copy()
    self._curve_hold_age_s = 0.0
    self._curve_hold_strength = float(np.clip(curve_assist, 0.0, 1.0))
    if abs(float(measured_curvature)) >= CURVE_ASSIST_START_CURVATURE:
      self._curve_hold_curvature = float(measured_curvature)
      self._curve_hold_sign = float(np.sign(measured_curvature))
    else:
      self._curve_hold_curvature = 0.0
      self._curve_hold_sign = 0.0

  def _temporal_hold_max_s(self, v_ego):
    """Maximum stale-curve bridge time for the current vehicle speed."""
    return float(np.clip(
      interp(float(v_ego), CURVE_TEMPORAL_HOLD_BP_MS,
             CURVE_TEMPORAL_HOLD_V_S),
      0.0, CURVE_TEMPORAL_HOLD_MAX_S))

  def _curve_temporal_prediction(self, path_x, v_ego, curve_assist,
                                 measured_curvature, lane_change_active):
    """Predict the cached lane path with a speed-aware <=0.60 s horizon."""
    if lane_change_active:
      self._clear_curve_temporal_hold()
      return None, 0.0

    if self._curve_hold_x is None or self._curve_hold_y is None:
      return None, 0.0

    if v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
      self._clear_curve_temporal_hold()
      return None, 0.0

    current_sign = 0.0
    if abs(float(measured_curvature)) >= CURVE_ASSIST_START_CURVATURE:
      current_sign = float(np.sign(measured_curvature))
    if (self._curve_hold_sign != 0.0 and current_sign != 0.0 and
        current_sign != self._curve_hold_sign):
      # Never carry a right-hand curve into an immediate left-hand curve, or
      # vice versa. The fresh model path takes over immediately.
      self._clear_curve_temporal_hold()
      return None, 0.0

    hold_max_s = self._temporal_hold_max_s(v_ego)
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
    # frame. Constant curvature is used only as a short bridge. The first
    # 0.10 s is held strongly, then stale geometry fades according to speed.
    curvature = float(measured_curvature)
    if abs(curvature) < CURVE_ASSIST_START_CURVATURE:
      curvature = self._curve_hold_curvature
    travel_m = max(float(v_ego), 0.0) * self._curve_hold_age_s
    yaw = float(np.clip(
      curvature * travel_m, -CURVE_TEMPORAL_MAX_YAW_RAD,
      CURVE_TEMPORAL_MAX_YAW_RAD))

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

    finite = np.isfinite(predicted_x) & np.isfinite(predicted_y)
    predicted_x = predicted_x[finite]
    predicted_y = predicted_y[finite]
    if predicted_x.size < 2:
      return None, 0.0

    order = np.argsort(predicted_x)
    predicted_x = predicted_x[order]
    predicted_y = predicted_y[order]
    predicted_x, unique_indices = np.unique(predicted_x, return_index=True)
    predicted_y = predicted_y[unique_indices]
    if predicted_x.size < 2:
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
    max_correction = np.interp(
      np.abs(path_xyz[:, 0]), [0.0, 20.0],
      [CURVE_TEMPORAL_MAX_CORRECTION_NEAR_M,
       CURVE_TEMPORAL_MAX_CORRECTION_M])
    return path_xyz[:, 1] + np.clip(
      predicted_lane_y - path_xyz[:, 1], -max_correction, max_correction)

  def _spatial_curve_strength(self, x_values, y_values):
    """Estimate visible bend strength from spatial x/y points."""
    x_values = np.asarray(x_values, dtype=float)
    y_values = np.asarray(y_values, dtype=float)
    if x_values.size < 2 or x_values.size != y_values.size:
      return 0.0

    valid = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[valid]
    y_values = y_values[valid]
    if x_values.size < 2:
      return 0.0

    order = np.argsort(x_values)
    x_values = x_values[order]
    y_values = y_values[order]
    x_values, unique_indices = np.unique(x_values, return_index=True)
    y_values = y_values[unique_indices]
    if x_values.size < 2:
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

  def _road_edge_curve_strength(self):
    """Return confidence-weighted bend from the best visible road edge."""
    best_strength = 0.0
    for edge_x, edge_y, edge_std in (
        (self.le_x, self.le_y, self.le_std),
        (self.re_x, self.re_y, self.re_std)):
      edge_conf = interp(
        edge_std,
        [ROAD_EDGE_STD_FULL_CONFIDENCE, ROAD_EDGE_STD_ZERO_CONFIDENCE],
        [1.0, 0.0])
      if edge_conf < ROAD_EDGE_MIN_CONFIDENCE:
        continue

      bend_strength = self._spatial_curve_strength(edge_x, edge_y)
      best_strength = max(
        best_strength, float(bend_strength) * float(edge_conf))

    return float(np.clip(best_strength, 0.0, 1.0))

  def _road_edge_fallback_path(self, path_xyz, v_ego, curve_assist,
                               lane_change_active):
    """Build a bounded path from trustworthy road edges on a tight curve."""
    if lane_change_active or v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
      return None, 0.0

    def prepare_edge(edge_x, edge_y):
      valid = np.isfinite(edge_x) & np.isfinite(edge_y)
      edge_x = np.asarray(edge_x[valid], dtype=float)
      edge_y = np.asarray(edge_y[valid], dtype=float)
      if edge_x.size < 2:
        return None, None
      order = np.argsort(edge_x)
      edge_x = edge_x[order]
      edge_y = edge_y[order]
      edge_x, unique_indices = np.unique(edge_x, return_index=True)
      edge_y = edge_y[unique_indices]
      if edge_x.size < 2:
        return None, None
      return edge_x, edge_y

    left_x, left_y = prepare_edge(self.le_x, self.le_y)
    right_x, right_y = prepare_edge(self.re_x, self.re_y)
    if left_x is None and right_x is None:
      return None, 0.0

    left_conf = interp(
      self.le_std,
      [ROAD_EDGE_STD_FULL_CONFIDENCE, ROAD_EDGE_STD_ZERO_CONFIDENCE],
      [1.0, 0.0])
    right_conf = interp(
      self.re_std,
      [ROAD_EDGE_STD_FULL_CONFIDENCE, ROAD_EDGE_STD_ZERO_CONFIDENCE],
      [1.0, 0.0])

    path_x = np.asarray(path_xyz[:, 0], dtype=float)
    model_y = np.asarray(path_xyz[:, 1], dtype=float)
    check_x = np.asarray(LANE_WIDTH_CHECK_DISTANCES_M, dtype=float)
    model_refs = np.interp(check_x, path_x, model_y)

    left_interp = None
    right_interp = None
    left_refs = None
    right_refs = None
    if left_x is not None:
      left_interp = np.interp(path_x, left_x, left_y)
      left_refs = np.interp(check_x, left_x, left_y)
    if right_x is not None:
      right_interp = np.interp(path_x, right_x, right_y)
      right_refs = np.interp(check_x, right_x, right_y)

    speed_weight = interp(
      v_ego, [CURVE_ASSIST_FULL_BELOW_MS, CURVE_ASSIST_ZERO_ABOVE_MS],
      [1.0, 0.0])

    # Road edges can reveal the curve before measured vehicle curvature catches
    # up. Only trustworthy edges may contribute to this curve detector.
    edge_curve_strength = 0.0
    if left_refs is not None and left_conf >= ROAD_EDGE_MIN_CONFIDENCE:
      left_bend = float(np.max(np.abs(left_refs - left_refs[0])))
      edge_curve_strength = max(
        edge_curve_strength,
        interp(left_bend,
               [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M],
               [0.0, 1.0]))
    if right_refs is not None and right_conf >= ROAD_EDGE_MIN_CONFIDENCE:
      right_bend = float(np.max(np.abs(right_refs - right_refs[0])))
      edge_curve_strength = max(
        edge_curve_strength,
        interp(right_bend,
               [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M],
               [0.0, 1.0]))

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

    max_correction = np.interp(
      np.abs(path_x), [0.0, 20.0],
      [ROAD_EDGE_MAX_CORRECTION_NEAR_M, ROAD_EDGE_MAX_CORRECTION_M])
    edge_path = model_y + np.clip(
      edge_path - model_y, -max_correction, max_correction)

    confidence = float(np.clip(
      max(confidences) * active_curve_assist, 0.0, 1.0))
    return edge_path, confidence

  def _age_curve_temporal_hold(self, v_ego):
    if self._curve_hold_x is None:
      return
    hold_max_s = self._temporal_hold_max_s(v_ego)
    if hold_max_s <= CURVE_TEMPORAL_HOLD_FULL_S:
      self._clear_curve_temporal_hold()
      return
    self._curve_hold_age_s += DT_MDL
    if self._curve_hold_age_s > hold_max_s:
      self._clear_curve_temporal_hold()

  def _apply_missing_lane_fallback(self, path_xyz, v_ego, curve_assist,
                                   measured_curvature, lane_change_active):
    """Fallback order for missing lane geometry: temporal first, then road edge."""
    predicted_lane_y, temporal_strength = self._curve_temporal_prediction(
      path_xyz[:, 0], v_ego, curve_assist,
      measured_curvature, lane_change_active)
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
      self._fallback_mode_active = True
      self._fresh_lane_recovery_frames = 0
      self._age_curve_temporal_hold(v_ego)
      return True

    return False

  def parse_model(self, md):
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

  def get_d_path(self, v_ego, path_t, path_xyz, measured_curvature=0.0,
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
    model_curve_strength = self._model_path_curve_strength(path_xyz)
    road_edge_curve_strength = self._road_edge_curve_strength()
    pre_curve_strength = float(measured_curve_strength)
    if not lane_change_active:
      pre_curve_strength = max(
        pre_curve_strength,
        MODEL_PATH_CURVE_WEIGHT * model_curve_strength,
        ROAD_EDGE_CURVE_WEIGHT * road_edge_curve_strength)
    pre_curve_assist = float(np.clip(
      pre_curve_strength * speed_curve_weight, 0.0, 1.0))

    width_pts = self.rll_y - self.lll_y
    geometry_valid = (
      np.isfinite(self.ll_x) & np.isfinite(self.lll_y) &
      np.isfinite(self.rll_y) & np.isfinite(width_pts)
    )
    if np.count_nonzero(geometry_valid) < 2:
      if self._apply_missing_lane_fallback(
          path_xyz, v_ego, pre_curve_assist,
          measured_curvature, lane_change_active):
        return path_xyz

      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
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
          measured_curvature, lane_change_active):
        return path_xyz

      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
      self._fallback_mode_active = False
      return path_xyz

    width_samples = np.array([
      abs(float(np.interp(distance, lane_x, lane_width_pts)))
      for distance in LANE_WIDTH_CHECK_DISTANCES_M
    ])
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
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0.0, 31.0], [2.8, 3.5])
    self.lane_width = (
      self.lane_width_certainty.x * self.lane_width_estimate.x +
      (1.0 - self.lane_width_certainty.x) * speed_lane_width
    )

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = lane_left_y + clipped_lane_width / 2.0
    path_from_right_lane = lane_right_y - clipped_lane_width / 2.0
    lane_path_y = (
      l_prob * path_from_left_lane + r_prob * path_from_right_lane
    ) / (l_prob + r_prob + 0.0001)

    raw_target_d_prob = enhance_lane_probability(
      combined_lane_probability(l_prob, r_prob), True)
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
    curve_assist = float(np.clip(
      combined_curve_strength * speed_curve_weight, 0.0, 1.0))

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

    # Cache only trustworthy tight-curve frames. During a short camera/model
    # confidence dropout, prefer current road edges; if those are unavailable,
    # reuse the ego-motion-compensated lane path with a speed-aware fade.
    trusted_curve_frame = bool(
      not lane_change_active and
      curve_assist >= CURVE_TEMPORAL_MIN_ASSIST and
      geometry_plausible and
      raw_target_d_prob >= CURVE_TEMPORAL_STORE_DPROB
    )
    # Start fallback gradually as confidence falls below the temporal-store
    # threshold (0.35), reaching full fallback by the trigger threshold (0.22).
    # This removes the previous 0.22-0.35 dead zone.
    visual_dropout = bool(
      not lane_change_active and
      curve_assist >= CURVE_TEMPORAL_MIN_ASSIST and
      raw_target_d_prob < CURVE_TEMPORAL_STORE_DPROB
    )

    if trusted_curve_frame:
      self._store_curve_temporal_hold(
        path_xyz[:, 0], lane_path_y_interp,
        curve_assist, measured_curvature)
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
        measured_curvature, lane_change_active)
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
          self._fallback_mode_active = True
          self._fresh_lane_recovery_frames = 0
          self._age_curve_temporal_hold(v_ego)
    else:
      if lane_change_active or v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS:
        self._clear_curve_temporal_hold()
      else:
        self._age_curve_temporal_hold(v_ego)
      if lane_change_active:
        self._fallback_mode_active = False
        self._fresh_lane_recovery_frames = 0

    self.lane_center_correction_m = float(
      self.d_prob * np.interp(20.0, path_xyz[:, 0], center_delta))
    self.lane_center_correction_active = bool(
      self.d_prob > 0.05 and
      abs(self.lane_center_correction_m) > 0.01)
    path_xyz[:, 1] = (
      self.d_prob * lane_path_y_interp +
      (1.0 - self.d_prob) * path_xyz[:, 1]
    )
    return path_xyz
