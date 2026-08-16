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
# This EON model reports useful geometry above 0.30, so extend only the soft
# tail. Values at 0.50 and above remain fully rejected.
LANE_STD_FULL_CONFIDENCE = 0.15
LANE_STD_ZERO_CONFIDENCE = 0.50
LANE_WIDTH_CHECK_DISTANCES_M = (5.0, 10.0, 20.0)
LANE_WIDTH_MOD_START_M = 4.2
LANE_WIDTH_MOD_END_M = 5.0


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
    del path_t, measured_curvature, lane_change_active
    path_xyz[:, 1] += self.path_offset

    width_pts = self.rll_y - self.lll_y
    geometry_valid = (
      np.isfinite(self.ll_x) & np.isfinite(self.lll_y) &
      np.isfinite(self.rll_y) & np.isfinite(width_pts)
    )
    if np.count_nonzero(geometry_valid) < 2:
      self.d_prob = 0.0
      cloudlog.warning("Lateral mpc - incomplete laneline x/y geometry, ignoring")
      return path_xyz

    lane_x = self.ll_x[geometry_valid]
    lane_left_y = self.lll_y[geometry_valid]
    lane_right_y = self.rll_y[geometry_valid]
    lane_width_pts = width_pts[geometry_valid]

    width_mod = min(
      interp(abs(float(np.interp(distance, lane_x, lane_width_pts))),
             [LANE_WIDTH_MOD_START_M, LANE_WIDTH_MOD_END_M], [1.0, 0.0])
      for distance in LANE_WIDTH_CHECK_DISTANCES_M
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
    current_lane_width = abs(float(lane_right_y[0] - lane_left_y[0]))
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

    target_d_prob = enhance_lane_probability(
      combined_lane_probability(l_prob, r_prob), True)
    self.d_prob = limit_lane_probability_rise(
      self.d_prob, target_d_prob, v_ego, DT_MDL)

    lane_path_y_interp = np.interp(path_xyz[:, 0], lane_x, lane_path_y)
    path_xyz[:, 1] = (
      self.d_prob * lane_path_y_interp +
      (1.0 - self.d_prob) * path_xyz[:, 1]
    )
    return path_xyz
