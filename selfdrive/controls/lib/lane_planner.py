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
CURVE_SINGLE_LANE_DPROB_FLOOR = 0.38
LANE_CONFIDENCE_FALL_RATE_PER_S = 1.25
LANE_CENTER_CONTINUITY_MAX_M = 0.35
CURVE_LANE_CENTER_CONTINUITY_MAX_M = 0.55
CURVE_CONFIDENCE_RISE_BONUS_PER_S = 1.00
CURVE_ASSIST_FULL_BELOW_MS = 12.0
CURVE_ASSIST_ZERO_ABOVE_MS = 18.0
CURVE_ASSIST_START_CURVATURE = 0.0075
CURVE_ASSIST_FULL_CURVATURE = 0.025
CURVE_ASSIST_START_BEND_M = 0.30
CURVE_ASSIST_FULL_BEND_M = 1.50
LOW_CONFIDENCE_MAX_CORRECTION_NEAR_M = 0.08
LOW_CONFIDENCE_MAX_CORRECTION_M = 0.35
CURVE_LOW_CONFIDENCE_MAX_CORRECTION_NEAR_M = 0.12
CURVE_LOW_CONFIDENCE_MAX_CORRECTION_M = 0.45

# Short temporal continuation for tight curves. Cache the last trustworthy
# lane-path shape, compensate it for the car's short ego-motion, hold it
# strongly for 0.10 s, then fade it out completely by 0.30 s.
CURVE_TEMPORAL_HOLD_FULL_S = 0.10
CURVE_TEMPORAL_HOLD_MAX_S = 0.30
CURVE_TEMPORAL_MIN_ASSIST = 0.30
CURVE_TEMPORAL_STORE_DPROB = 0.35
CURVE_TEMPORAL_TRIGGER_DPROB = 0.22
CURVE_TEMPORAL_DPROB_FLOOR = 0.32
CURVE_TEMPORAL_MAX_CORRECTION_NEAR_M = 0.14
CURVE_TEMPORAL_MAX_CORRECTION_M = 0.55
CURVE_TEMPORAL_MAX_YAW_RAD = 0.20


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

  def _curve_temporal_prediction(self, path_x, v_ego, curve_assist,
                                 measured_curvature, lane_change_active):
    """Predict the cached lane path in the current ego frame for <=0.30 s."""
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

    self._curve_hold_age_s += DT_MDL
    if self._curve_hold_age_s > CURVE_TEMPORAL_HOLD_MAX_S:
      self._clear_curve_temporal_hold()
      return None, 0.0

    active_assist = max(float(curve_assist), self._curve_hold_strength * 0.70)
    if active_assist < CURVE_TEMPORAL_MIN_ASSIST:
      return None, 0.0

    # Move the previously observed road geometry into the current vehicle
    # frame. Constant curvature is only assumed for this very short 0.30 s
    # bridge; the result is also bounded again before it reaches the planner.
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

    time_strength = interp(
      self._curve_hold_age_s,
      [0.0, CURVE_TEMPORAL_HOLD_FULL_S, CURVE_TEMPORAL_HOLD_MAX_S],
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
    measured_curve_assist = float(np.clip(
      measured_curve_strength * speed_curve_weight, 0.0, 1.0))

    width_pts = self.rll_y - self.lll_y
    geometry_valid = (
      np.isfinite(self.ll_x) & np.isfinite(self.lll_y) &
      np.isfinite(self.rll_y) & np.isfinite(width_pts)
    )
    if np.count_nonzero(geometry_valid) < 2:
      predicted_lane_y, temporal_strength = self._curve_temporal_prediction(
        path_xyz[:, 0], v_ego, measured_curve_assist,
        measured_curvature, lane_change_active)
      if predicted_lane_y is not None and temporal_strength > 0.0:
        predicted_lane_y = self._bound_curve_temporal_path(
          path_xyz, predicted_lane_y)
        applied_delta = (predicted_lane_y - path_xyz[:, 1]) * temporal_strength
        path_xyz[:, 1] += applied_delta
        self.d_prob = max(
          self.d_prob, CURVE_TEMPORAL_DPROB_FLOOR * temporal_strength)
        self.lane_center_correction_m = float(
          np.interp(20.0, path_xyz[:, 0], applied_delta))
        self.lane_center_correction_active = bool(
          abs(self.lane_center_correction_m) > 0.01)
        return path_xyz

      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
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
      predicted_lane_y, temporal_strength = self._curve_temporal_prediction(
        path_xyz[:, 0], v_ego, measured_curve_assist,
        measured_curvature, lane_change_active)
      if predicted_lane_y is not None and temporal_strength > 0.0:
        predicted_lane_y = self._bound_curve_temporal_path(
          path_xyz, predicted_lane_y)
        applied_delta = (predicted_lane_y - path_xyz[:, 1]) * temporal_strength
        path_xyz[:, 1] += applied_delta
        self.d_prob = max(
          self.d_prob, CURVE_TEMPORAL_DPROB_FLOOR * temporal_strength)
        self.lane_center_correction_m = float(
          np.interp(20.0, path_xyz[:, 0], applied_delta))
        self.lane_center_correction_active = bool(
          abs(self.lane_center_correction_m) > 0.01)
        return path_xyz

      self.d_prob = 0.0
      self.lane_center_correction_m = 0.0
      self.lane_center_correction_active = False
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

    # Tight-curve assist is deliberately limited to low/mid speed. It uses
    # both measured vehicle curvature and visible lane bending so the assist
    # can start before the car has fully rotated into the corner.
    lane_bend_m = float(np.max(np.abs(lane_center_refs - lane_center_refs[0])))
    geometry_curve_strength = interp(
      lane_bend_m, [CURVE_ASSIST_START_BEND_M, CURVE_ASSIST_FULL_BEND_M],
      [0.0, 1.0])
    curve_assist = float(np.clip(
      max(measured_curve_strength, geometry_curve_strength) *
      speed_curve_weight, 0.0, 1.0))

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

    self.d_prob = self._update_d_prob(
      target_d_prob, v_ego, lane_center_refs, lane_change_active, curve_assist)

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
    # confidence dropout, reuse the ego-motion-compensated lane path and fade
    # smoothly back to the fresh model path by 0.30 s.
    trusted_curve_frame = bool(
      not lane_change_active and
      curve_assist >= CURVE_TEMPORAL_MIN_ASSIST and
      geometry_plausible and
      raw_target_d_prob >= CURVE_TEMPORAL_STORE_DPROB
    )
    visual_dropout = bool(
      not lane_change_active and
      curve_assist >= CURVE_TEMPORAL_MIN_ASSIST and
      raw_target_d_prob < CURVE_TEMPORAL_TRIGGER_DPROB
    )

    if trusted_curve_frame:
      self._store_curve_temporal_hold(
        path_xyz[:, 0], lane_path_y_interp,
        curve_assist, measured_curvature)
    elif visual_dropout:
      predicted_lane_y, temporal_strength = self._curve_temporal_prediction(
        path_xyz[:, 0], v_ego, curve_assist,
        measured_curvature, lane_change_active)
      if predicted_lane_y is not None and temporal_strength > 0.0:
        predicted_lane_y = self._bound_curve_temporal_path(
          path_xyz, predicted_lane_y)
        temporal_weight = (
          1.0 - interp(
            raw_target_d_prob,
            [0.0, CURVE_TEMPORAL_TRIGGER_DPROB],
            [0.0, 1.0])
        ) * temporal_strength
        lane_path_y_interp = (
          temporal_weight * predicted_lane_y +
          (1.0 - temporal_weight) * lane_path_y_interp
        )
        self.d_prob = max(
          self.d_prob, CURVE_TEMPORAL_DPROB_FLOOR * temporal_weight)
        center_delta = lane_path_y_interp - path_xyz[:, 1]
    else:
      if self._curve_hold_x is not None:
        self._curve_hold_age_s += DT_MDL
        if (self._curve_hold_age_s > CURVE_TEMPORAL_HOLD_MAX_S or
            lane_change_active or v_ego >= CURVE_ASSIST_ZERO_ABOVE_MS):
          self._clear_curve_temporal_hold()

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
