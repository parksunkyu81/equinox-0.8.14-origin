import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, clip, mean
from common.realtime import DT_MDL
from selfdrive.hardware import EON, TICI
from selfdrive.swaglog import cloudlog
from common.params import Params
from decimal import Decimal
from selfdrive.ntune import ntune_common_get
from selfdrive.car.gm.values import LEFT_EDGE_OFFSET, RIGHT_EDGE_OFFSET
from selfdrive.controls.lib.lane_probability import combined_lane_probability, \
                                                    enhance_lane_probability, \
                                                    limit_lane_probability_rise
from selfdrive.controls.lib.model_data_validation import as_finite_vector
from selfdrive.controls.lib.lateral_path_stability import LaneCenterCorrection

ENABLE_ZORROBYTE = False
ENABLE_INC_LANE_PROB = True

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera. Empirically
# the model knows the difference between TICI and EON
# so a path offset is not needed

# 카메라 오프셋은 중앙 자동차에서 카메라까지 미터입니다.
# 모델 경로가 카메라 프레임에 있습니다. 경험적으로
# 모델은 TICI와 EON의 차이를 알고 있다.
# 따라서 경로 오프셋이 필요하지 않습니다.

PATH_OFFSET = ntune_common_get('pathOffset')
CAMERA_OFFSET = -0.055   # 카메라 오른쪽으로 5.5cm 이동

# Near lane lines are the only lane geometry trusted for lateral-path blending.
# The old 3 second width check reached 50-70 m at normal road speed; one noisy
# far-field lane point then forced dProb to zero even when both lines were clean
# at 20 m.
NEAR_LANE_WIDTH_CHECK_DISTANCES_M = (5.0, 10.0, 20.0)
NEAR_LANE_WIDTH_MOD_START_M = 4.2
NEAR_LANE_WIDTH_MOD_END_M = 5.0

# When both near lane lines are strong, do not allow the model-only path to
# cut materially toward the inside of a curve. Attack is deliberately faster
# than release, and a brief confidence hold prevents one weak frame from
# dropping a correction in the middle of a corner.
CURVE_INSIDE_GUARD_MIN_CURVATURE = 0.00045
CURVE_INSIDE_GUARD_MAX_OFFSET_M = 0.10
CURVE_INSIDE_GUARD_MIN_LANE_PROB = 0.50
CURVE_INSIDE_GUARD_MAX_LANE_STD = 0.20
CURVE_INSIDE_GUARD_MIN_LANE_WIDTH_M = 2.5
CURVE_INSIDE_GUARD_MAX_LANE_WIDTH_M = 4.2
CURVE_INSIDE_GUARD_MAX_CORRECTION_M = 1.00
CURVE_INSIDE_GUARD_ATTACK_MPS = 1.50
CURVE_INSIDE_GUARD_RELEASE_MPS = 0.35
CURVE_INSIDE_GUARD_CONFIDENCE_HOLD_S = 0.80


def curve_inside_guard_target(path_y_20m, lane_center_y_20m, measured_curvature,
                              left_prob, right_prob, left_std, right_std,
                              lane_width_20m, lane_change_active=False):
  """Return a bounded outward correction for a reliable curved-road sample."""
  curvature = float(measured_curvature)
  eligible = bool(
    not lane_change_active and
    abs(curvature) >= CURVE_INSIDE_GUARD_MIN_CURVATURE and
    float(left_prob) >= CURVE_INSIDE_GUARD_MIN_LANE_PROB and
    float(right_prob) >= CURVE_INSIDE_GUARD_MIN_LANE_PROB and
    float(left_std) <= CURVE_INSIDE_GUARD_MAX_LANE_STD and
    float(right_std) <= CURVE_INSIDE_GUARD_MAX_LANE_STD and
    CURVE_INSIDE_GUARD_MIN_LANE_WIDTH_M <= float(lane_width_20m) <= CURVE_INSIDE_GUARD_MAX_LANE_WIDTH_M
  )
  if not eligible:
    return 0.0, False

  curve_direction = 1.0 if curvature > 0.0 else -1.0
  inside_offset = (float(path_y_20m) - float(lane_center_y_20m)) * curve_direction
  excess_inside = max(0.0, inside_offset - CURVE_INSIDE_GUARD_MAX_OFFSET_M)
  target = -curve_direction * min(excess_inside, CURVE_INSIDE_GUARD_MAX_CORRECTION_M)
  return float(target), bool(excess_inside > 0.0)

class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = CAMERA_OFFSET
    self.path_offset = PATH_OFFSET

    self.readings = []
    self.frame = 0

    self.wide_camera = wide_camera

    #opkr
    self.params = Params()
    self.drive_close_to_edge = self.params.get_bool("closeToRoadEdge")
    self.left_edge_offset = float(
      Decimal(LEFT_EDGE_OFFSET) * Decimal('0.01'))  # 0.15 move to right
    self.right_edge_offset = float(
      Decimal(RIGHT_EDGE_OFFSET) * Decimal('0.01'))  # -0.15 move to left

    self.road_edge_offset = 0.0
    self.total_camera_offset = self.camera_offset
    self.lp_timer = 0
    self.lp_timer2 = 0
    self.lp_timer3 = 0
    self._lane_center = LaneCenterCorrection()
    self.lane_center_correction_m = 0.0
    self.lane_center_correction_active = False
    self.curve_inside_correction_m = 0.0
    self.curve_inside_correction_active = False
    self.curve_inside_target_m = 0.0
    self.curve_inside_hold_s = 0.0
    self.curve_inside_direction = 0
    self.near_lane_pair_reliable = False
    self.near_lane_center_y_20m = 0.0

  def parse_model(self, md):

    #opkr
    self.lp_timer += DT_MDL
    if self.lp_timer > 1.0:
      self.lp_timer = 0.0
      self.camera_offset = CAMERA_OFFSET  # m from center car to camera
      self.drive_close_to_edge = self.params.get_bool("closeToRoadEdge")

    #opkr
    if self.drive_close_to_edge:
      road_edge_stds = as_finite_vector(md.roadEdgeStds, minimum_size=2)
      lane_line_probs = as_finite_vector(md.laneLineProbs, minimum_size=4)
      if road_edge_stds is not None and lane_line_probs is not None:
        left_edge_prob = np.clip(1.0 - road_edge_stds[0], 0.0, 1.0)
        left_nearside_prob, left_close_prob, right_close_prob, right_nearside_prob = lane_line_probs[:4]
        right_edge_prob = np.clip(1.0 - road_edge_stds[1], 0.0, 1.0)

        self.lp_timer3 += DT_MDL
        if self.lp_timer3 > 3.0:
          self.lp_timer3 = 0.0
          if right_nearside_prob < 0.1 and left_nearside_prob < 0.1:
            self.road_edge_offset = 0.0
          elif right_edge_prob > 0.35 and right_nearside_prob < 0.2 and right_close_prob > 0.5 and left_nearside_prob >= right_nearside_prob:
            self.road_edge_offset = -self.right_edge_offset
          elif left_edge_prob > 0.35 and left_nearside_prob < 0.2 and left_close_prob > 0.5 and right_nearside_prob >= left_nearside_prob:
            self.road_edge_offset = -self.left_edge_offset
          else:
            self.road_edge_offset = 0.0
      else:
        self.road_edge_offset = 0.0
    else:
      self.road_edge_offset = 0.0

    self.total_camera_offset = self.camera_offset + self.road_edge_offset


    lane_lines = md.laneLines
    lane_line_probs = as_finite_vector(md.laneLineProbs, minimum_size=4)
    lane_line_stds = as_finite_vector(md.laneLineStds, minimum_size=4)
    lane_data = None
    if len(lane_lines) == 4 and lane_line_probs is not None and lane_line_stds is not None:
      lane_data = tuple(
        as_finite_vector(values, expected_size=TRAJECTORY_SIZE)
        for values in (lane_lines[1].t, lane_lines[2].t, lane_lines[1].x,
                       lane_lines[1].y, lane_lines[2].y)
      )
    if lane_data is not None and all(values is not None for values in lane_data):
      left_t, right_t, left_x, left_y, right_y = lane_data
      self.ll_t = (left_t + right_t) / 2.0
      # left and right ll x is the same
      self.ll_x = left_x
      # only offset left and right lane lines; offsetting path does not make sense

      self.lll_y = left_y + self.total_camera_offset
      self.rll_y = right_y + self.total_camera_offset
      self.lll_prob = lane_line_probs[1]
      self.rll_prob = lane_line_probs[2]
      self.lll_std = lane_line_stds[1]
      self.rll_std = lane_line_stds[2]
    else:
      # Fall back to the model path instead of reusing stale lane confidence.
      self.lll_prob = 0.0
      self.rll_prob = 0.0
      self.lll_std = 1.0
      self.rll_std = 1.0

    left_desire_idx = log.LateralPlan.Desire.laneChangeLeft
    right_desire_idx = log.LateralPlan.Desire.laneChangeRight
    desire_state = as_finite_vector(md.meta.desireState, minimum_size=max(left_desire_idx, right_desire_idx) + 1)
    if desire_state is not None:
      self.l_lane_change_prob = desire_state[left_desire_idx]
      self.r_lane_change_prob = desire_state[right_desire_idx]
    else:
      self.l_lane_change_prob = 0.0
      self.r_lane_change_prob = 0.0

  def get_d_path(self, v_ego, path_t, path_xyz, measured_curvature=0.0,
                 lane_change_active=False):
    # Reduce reliance only from near-field lane geometry. Far model lane-line
    # divergence is common on curves and must not erase a clean 5-20 m pair.
    path_xyz[:, 1] += self.path_offset
    raw_l_prob, raw_r_prob = self.lll_prob, self.rll_prob
    l_prob, r_prob = raw_l_prob, raw_r_prob
    width_pts = self.rll_y - self.lll_y
    lane_center_y = (self.lll_y + self.rll_y) / 2.0
    lane_geometry_idxs = np.isfinite(self.ll_x) & np.isfinite(width_pts) & np.isfinite(lane_center_y)
    near_widths = []
    if np.count_nonzero(lane_geometry_idxs) >= 2:
      lane_x_safe = self.ll_x[lane_geometry_idxs]
      width_safe = width_pts[lane_geometry_idxs]
      center_safe = lane_center_y[lane_geometry_idxs]
      near_widths = [abs(float(np.interp(distance, lane_x_safe, width_safe)))
                     for distance in NEAR_LANE_WIDTH_CHECK_DISTANCES_M]
      self.near_lane_center_y_20m = float(np.interp(20.0, lane_x_safe, center_safe))
    else:
      self.near_lane_center_y_20m = 0.0

    near_width_valid = bool(
      len(near_widths) == len(NEAR_LANE_WIDTH_CHECK_DISTANCES_M) and
      all(CURVE_INSIDE_GUARD_MIN_LANE_WIDTH_M <= width <= CURVE_INSIDE_GUARD_MAX_LANE_WIDTH_M
          for width in near_widths))
    self.near_lane_pair_reliable = bool(
      not lane_change_active and near_width_valid and
      raw_l_prob >= CURVE_INSIDE_GUARD_MIN_LANE_PROB and
      raw_r_prob >= CURVE_INSIDE_GUARD_MIN_LANE_PROB and
      self.lll_std <= CURVE_INSIDE_GUARD_MAX_LANE_STD and
      self.rll_std <= CURVE_INSIDE_GUARD_MAX_LANE_STD)

    prob_mods = [interp(width, [NEAR_LANE_WIDTH_MOD_START_M, NEAR_LANE_WIDTH_MOD_END_M], [1.0, 0.0])
                 for width in near_widths]
    mod = min(prob_mods) if prob_mods else 0.0
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    if ENABLE_ZORROBYTE:
      # zorrobyte code
      if l_prob > 0.5 and r_prob > 0.5:
        self.frame += 1
        if self.frame > 20:
          self.frame = 0
          current_lane_width = clip(abs(self.rll_y[0] - self.lll_y[0]), 2.5, 3.5)
          self.readings.append(current_lane_width)
          self.lane_width = mean(self.readings)
          if len(self.readings) >= 30:
            self.readings.pop(0)

      # zorrobyte
      # Don't exit dive
      if abs(self.rll_y[0] - self.lll_y[0]) > self.lane_width:
        r_prob = r_prob / interp(l_prob, [0, 1], [1, 3])

    else:
      # Find current lanewidth
      self.lane_width_certainty.update(l_prob * r_prob)
      current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
      self.lane_width_estimate.update(current_lane_width)
      speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
      self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                        (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # Build lane-path reliance continuously. The previous >0.65 then x1.3
    # rule caused a ~21 percentage-point jump for a 0.01 change in each lane
    # probability. Smooth enhancement removes that discontinuity. Only rising
    # confidence is rate-limited (more strongly at highway speed); falling
    # confidence passes immediately so the model path remains the safe fallback.
    raw_d_prob = combined_lane_probability(l_prob, r_prob)
    target_d_prob = enhance_lane_probability(raw_d_prob, ENABLE_INC_LANE_PROB)
    self.d_prob = limit_lane_probability_rise(self.d_prob, target_d_prob, v_ego, DT_MDL)

    if self.near_lane_pair_reliable:
      # With two good lines, their geometric center is safer than weighting
      # two independently shifted boundaries by slightly different probs.
      lane_path_y = lane_center_y
    else:
      lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]

      center_safe_idxs = safe_idxs & np.isfinite(self.ll_x) & np.isfinite(lane_center_y) & np.isfinite(width_pts)
      path_safe_idxs = np.isfinite(path_xyz[:, 0]) & np.isfinite(path_xyz[:, 1])
      curve_geometry_valid = False
      curve_geometry_inside = 0.0
      if np.count_nonzero(center_safe_idxs) >= 2 and np.count_nonzero(path_safe_idxs) >= 2:
        lane_center_y_20m = float(np.interp(
          20.0, self.ll_x[center_safe_idxs], lane_center_y[center_safe_idxs]))
        lane_width_20m = abs(float(np.interp(
          20.0, self.ll_x[center_safe_idxs], width_pts[center_safe_idxs])))
        path_y_20m_before_guard = float(np.interp(
          20.0, path_xyz[path_safe_idxs, 0], path_xyz[path_safe_idxs, 1]))
        curve_geometry_valid = bool(
          CURVE_INSIDE_GUARD_MIN_LANE_WIDTH_M <= lane_width_20m <=
          CURVE_INSIDE_GUARD_MAX_LANE_WIDTH_M)
        curve_target, curve_target_active = curve_inside_guard_target(
          path_y_20m_before_guard, lane_center_y_20m, measured_curvature,
          raw_l_prob, raw_r_prob, self.lll_std, self.rll_std, lane_width_20m,
          lane_change_active=lane_change_active)
      else:
        curve_target, curve_target_active = 0.0, False
      curve_direction = (1 if float(measured_curvature) > 0.0 else -1) \
        if abs(float(measured_curvature)) >= CURVE_INSIDE_GUARD_MIN_CURVATURE else 0
      if curve_geometry_valid and curve_direction != 0:
        curve_geometry_inside = (
          path_y_20m_before_guard - lane_center_y_20m) * curve_direction
      previous_curve_direction = self.curve_inside_direction
      curve_direction_changed = bool(
        previous_curve_direction != 0 and curve_direction != previous_curve_direction)
      if curve_target_active:
        self.curve_inside_target_m = float(curve_target)
        self.curve_inside_hold_s = CURVE_INSIDE_GUARD_CONFIDENCE_HOLD_S
        self.curve_inside_direction = curve_direction
      elif (not self.near_lane_pair_reliable and self.curve_inside_hold_s > 0.0 and
            curve_direction != 0 and curve_direction == self.curve_inside_direction and
            not lane_change_active):
        # Keep the last outward target across a brief confidence dip.
        self.curve_inside_hold_s = max(0.0, self.curve_inside_hold_s - DT_MDL)
        if curve_geometry_valid:
          held_required = max(
            0.0, curve_geometry_inside - CURVE_INSIDE_GUARD_MAX_OFFSET_M)
          held_target = -curve_direction * min(
            held_required, CURVE_INSIDE_GUARD_MAX_CORRECTION_M)
          if held_target * self.curve_inside_target_m > 0.0:
            curve_target = np.sign(self.curve_inside_target_m) * min(
              abs(self.curve_inside_target_m), abs(held_target))
          else:
            curve_target = 0.0
        else:
          curve_target = self.curve_inside_target_m
      else:
        self.curve_inside_target_m = 0.0
        self.curve_inside_hold_s = 0.0
        if curve_direction != self.curve_inside_direction:
          self.curve_inside_direction = curve_direction
        curve_target = 0.0

      same_direction = self.curve_inside_correction_m * curve_target >= 0.0
      increasing = same_direction and abs(curve_target) > abs(self.curve_inside_correction_m)
      curve_slew_mps = CURVE_INSIDE_GUARD_ATTACK_MPS if curve_direction_changed or increasing or not same_direction \
        else CURVE_INSIDE_GUARD_RELEASE_MPS
      curve_step = curve_slew_mps * DT_MDL
      self.curve_inside_correction_m += float(np.clip(
        curve_target - self.curve_inside_correction_m, -curve_step, curve_step))
      if abs(self.curve_inside_correction_m) < 1e-6:
        self.curve_inside_correction_m = 0.0
      curve_distance_weight = np.clip(path_xyz[:, 0] / 20.0, 0.0, 1.0)
      path_xyz[:, 1] += self.curve_inside_correction_m * curve_distance_weight
      self.curve_inside_correction_active = bool(
        curve_target_active or self.curve_inside_hold_s > 0.0 or
        abs(self.curve_inside_correction_m) > 0.002)

      lane_y_20m = float(np.interp(20.0, self.ll_x, lane_path_y))
      path_y_20m = float(np.interp(20.0, path_xyz[:, 0], path_xyz[:, 1]))
      residual_20m = lane_y_20m - path_y_20m
      center_eligible = bool(
        float(v_ego) * 3.6 >= 30.0 and
        abs(float(measured_curvature)) < CURVE_INSIDE_GUARD_MIN_CURVATURE and
        not bool(lane_change_active) and
        l_prob >= 0.50 and r_prob >= 0.50 and
        self.lll_std <= 0.20 and self.rll_std <= 0.20 and
        2.5 <= float(self.lane_width) <= 4.0
      )
      self.lane_center_correction_m = self._lane_center.update(
        residual_20m, center_eligible, DT_MDL)
      distance_weight = np.clip(path_xyz[:, 0] / 20.0, 0.0, 1.0)
      path_xyz[:, 1] += self.lane_center_correction_m * distance_weight
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
      self.lane_center_correction_m = self._lane_center.update(0.0, False, DT_MDL)
      self.near_lane_pair_reliable = False
      self.curve_inside_target_m = 0.0
      self.curve_inside_hold_s = 0.0
      curve_step = CURVE_INSIDE_GUARD_RELEASE_MPS * DT_MDL
      self.curve_inside_correction_m += float(np.clip(
        -self.curve_inside_correction_m, -curve_step, curve_step))
      self.curve_inside_correction_active = bool(abs(self.curve_inside_correction_m) > 0.002)
    self.lane_center_correction_active = bool(self._lane_center.active)
    return path_xyz
