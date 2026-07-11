import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from selfdrive.swaglog import cloudlog

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera. Empirically
# the model knows the difference between TICI and EON
# so a path offset is not needed

# 카메라 오프셋은 중앙 자동차에서 카메라까지 미터입니다.
# 모델 경로가 카메라 프레임에 있습니다. 경험적으로
# 모델은 TICI와 EON의 차이를 알고 있다.
# 따라서 경로 오프셋이 필요하지 않습니다.

PATH_OFFSET = 0.00
CAMERA_OFFSET = -0.06
CENTER_LANE_CONF_BP = [0.35, 0.60]
CENTER_LANE_CONF_V = [0.0, 1.0]
LANE_WIDTH_MIN_BP = [2.5, 2.8]
LANE_WIDTH_MAX_BP = [4.0, 5.0]
LANE_PATH_AGREEMENT_BP = [0.08, 0.20]
LANE_CENTER_CONTINUITY_BP = [0.10, 0.30]

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

    self.wide_camera = wide_camera

    self.total_camera_offset = self.camera_offset
    self.last_reliable_lane_center_y = None

  def parse_model(self, md):

    self.camera_offset = CAMERA_OFFSET
    self.total_camera_offset = self.camera_offset


    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      # only offset left and right lane lines; offsetting path does not make sense

      self.lll_y = np.array(lane_lines[1].y) + self.total_camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.total_camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 2.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      min_width_mod = interp(width_at_t, LANE_WIDTH_MIN_BP, [0.0, 1.0])
      max_width_mod = interp(width_at_t, LANE_WIDTH_MAX_BP, [1.0, 0.0])
      prob_mods.append(min_width_mod * max_width_mod)
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])

    # Use the last trusted width to evaluate the current lines. Updating the
    # width estimate before the safety checks would let a persistent misread
    # gradually teach the planner the wrong lane width.
    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    safe_idxs = np.isfinite(self.ll_t) & np.isfinite(self.lll_y) & np.isfinite(self.rll_y)
    lane_lines_valid = np.count_nonzero(safe_idxs) >= 2
    if lane_lines_valid:
      # A high lane-line probability is not sufficient: a confidently
      # misdetected line can still point well away from the model path. Check
      # each line-derived center independently so the reliable side can remain
      # useful while the inconsistent side is rejected.
      left_path_interp = np.interp(path_t, self.ll_t[safe_idxs], path_from_left_lane[safe_idxs])
      right_path_interp = np.interp(path_t, self.ll_t[safe_idxs], path_from_right_lane[safe_idxs])
      near_path_idxs = np.isfinite(path_t) & np.isfinite(path_xyz[:, 1]) & (path_t <= 1.5)
      if not np.any(near_path_idxs):
        near_path_idxs = np.isfinite(path_t) & np.isfinite(path_xyz[:, 1])

      if np.any(near_path_idxs):
        left_path_error = np.median(np.abs(left_path_interp[near_path_idxs] - path_xyz[near_path_idxs, 1]))
        right_path_error = np.median(np.abs(right_path_interp[near_path_idxs] - path_xyz[near_path_idxs, 1]))
        l_prob *= interp(left_path_error, LANE_PATH_AGREEMENT_BP, [1.0, 0.0])
        r_prob *= interp(right_path_error, LANE_PATH_AGREEMENT_BP, [1.0, 0.0])
      else:
        l_prob = 0.0
        r_prob = 0.0
    else:
      l_prob = 0.0
      r_prob = 0.0

    self.lane_width_certainty.update(l_prob * r_prob)
    if min(l_prob, r_prob) >= 0.5 and np.isfinite(current_lane_width):
      self.lane_width_estimate.update(current_lane_width)
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    raw_d_prob = l_prob + r_prob - l_prob * r_prob
    both_lane_conf = interp(min(l_prob, r_prob), CENTER_LANE_CONF_BP, CENTER_LANE_CONF_V)

    # When both lane lines are reliable, prefer their geometric midpoint.
    # This prevents model-path bias, lane-width error, or unequal line
    # probabilities from pulling the target away from the lane center.
    direct_lane_center_y = (self.lll_y + self.rll_y) / 2.0
    current_lane_center_y = direct_lane_center_y[0]
    if self.last_reliable_lane_center_y is None:
      center_continuity_mod = 1.0
    else:
      center_jump = abs(current_lane_center_y - self.last_reliable_lane_center_y)
      center_continuity_mod = interp(center_jump, LANE_CENTER_CONTINUITY_BP, [1.0, 0.0])
    both_lane_conf *= center_continuity_mod

    # Only a center supported by two independently consistent lines may update
    # the continuity reference. A rejected misread must not become the new
    # baseline merely because it persists for several frames.
    if both_lane_conf >= 0.5:
      if self.last_reliable_lane_center_y is None:
        self.last_reliable_lane_center_y = current_lane_center_y
      else:
        self.last_reliable_lane_center_y = 0.9 * self.last_reliable_lane_center_y + 0.1 * current_lane_center_y

    # Squared weights prevent a weak or uncertain line from pulling strongly
    # against the line that agrees with the model path.
    l_weight = l_prob * l_prob
    r_weight = r_prob * r_prob
    inferred_lane_center_y = \
      (l_weight * path_from_left_lane + r_weight * path_from_right_lane) / (l_weight + r_weight + 0.0001)
    lane_path_y = both_lane_conf * direct_lane_center_y + \
                  (1.0 - both_lane_conf) * inferred_lane_center_y

    # Smoothly remove the model path only when both lane lines agree.
    # Keep the original model fallback for weak or single-line detection.
    self.d_prob = raw_d_prob + (1.0 - raw_d_prob) * both_lane_conf
    if lane_lines_valid:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz
