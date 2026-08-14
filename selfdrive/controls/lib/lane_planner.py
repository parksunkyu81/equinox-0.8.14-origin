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
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
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

    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]

      lane_y_20m = float(np.interp(20.0, self.ll_x, lane_path_y))
      path_y_20m = float(np.interp(20.0, path_xyz[:, 0], path_xyz[:, 1]))
      residual_20m = lane_y_20m - path_y_20m
      center_eligible = bool(
        float(v_ego) * 3.6 >= 30.0 and
        abs(float(measured_curvature)) <= 0.0012 and
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
    self.lane_center_correction_active = bool(self._lane_center.active)
    return path_xyz
