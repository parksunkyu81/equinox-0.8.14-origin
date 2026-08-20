import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MPC_COST_LAT, LAT_MPC_N
from selfdrive.controls.lib.desire_helper import DesireHelper, AUTO_LCA_START_TIME
import cereal.messaging as messaging
from cereal import log
from common.params import Params
from selfdrive.controls.lib.model_data_validation import as_finite_vector, validated_model_trajectory

TRAJECTORY_SIZE = 33

# A short virtual path is permitted only after a trusted curve estimate loses
# perception confidence. It is intentionally a decaying re-projection of the
# previous path, not a new curvature or a torque-authority increase.
CURVE_HOLD_MIN_SPEED_MS = 5.0
CURVE_HOLD_MAX_DURATION_S = 0.35
CURVE_HOLD_REFERENCE_MAX_AGE_S = 0.45
CURVE_HOLD_MIN_REFERENCE_CURVATURE = 0.0035
CURVE_HOLD_MIN_ACTUAL_CURVATURE = 0.0015
CURVE_HOLD_MODEL_CONFIDENCE_MIN = 0.55
CURVE_HOLD_MODEL_CONFIDENCE_TRUSTED = 0.75
CURVE_HOLD_LANE_RAW_DPROB_TRUSTED = 0.45
CURVE_HOLD_LANE_CONTINUITY_MAX_AGE_S = 0.45
CURVE_HOLD_BLEND = 0.65
CURVE_HOLD_MAX_CORRECTION_NEAR_M = 0.12
CURVE_HOLD_MAX_CORRECTION_FAR_M = 0.75

class LateralPlanner:
  def __init__(self, CP):
    self.use_lanelines = not Params().get_bool('EndToEndToggle')
    self.LP = LanePlanner()
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.path_xyz_stds = np.ones((TRAJECTORY_SIZE, 3))
    self.speed_forward = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.plan_curv = np.zeros((TRAJECTORY_SIZE,))
    self.plan_curv_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros(TRAJECTORY_SIZE)
    self.model_data_valid = False
    self.model_position_stds_valid = False
    self.model_confidence = 0.0

    self._curve_hold_path = None
    self._curve_hold_curvature = 0.0
    self._curve_hold_reference_t = -np.inf
    self._curve_hold_start_t = None
    self.virtual_curve_hold_active = False
    self.virtual_curve_hold_weight = 0.0

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def _update_model_confidence(self, position_stds_valid):
    """Return a bounded confidence for the near/mid model trajectory."""
    self.model_position_stds_valid = bool(position_stds_valid)
    if not self.model_data_valid or not self.model_position_stds_valid:
      self.model_confidence = 0.0
      return

    # y uncertainty over roughly the first second is the relevant confidence
    # for lateral control. Do not retain an old standard-deviation frame.
    lateral_std = float(np.median(self.path_xyz_stds[1:13, 1]))
    self.model_confidence = float(np.clip(
      interp(lateral_std, [0.30, 1.50], [1.0, 0.0]), 0.0, 1.0))

  @staticmethod
  def _same_curve_direction(reference_curvature, measured_curvature,
                            yaw_curvature):
    return bool(
      reference_curvature * measured_curvature > 0.0 and
      reference_curvature * yaw_curvature > 0.0
    )

  def _vehicle_matches_curve_hold(self, car_state, measured_curvature):
    """Require independent steering-model and yaw evidence of the held turn."""
    v_ego = float(car_state.vEgo)
    yaw_rate = float(car_state.yawRate)
    if (not np.isfinite(measured_curvature) or not np.isfinite(yaw_rate) or
        v_ego < CURVE_HOLD_MIN_SPEED_MS):
      return False

    yaw_curvature = yaw_rate / max(v_ego, 1.0)
    reference = self._curve_hold_curvature
    if (abs(reference) < CURVE_HOLD_MIN_REFERENCE_CURVATURE or
        abs(measured_curvature) < CURVE_HOLD_MIN_ACTUAL_CURVATURE or
        abs(yaw_curvature) < CURVE_HOLD_MIN_ACTUAL_CURVATURE or
        not self._same_curve_direction(reference, measured_curvature,
                                       yaw_curvature)):
      return False

    # The car may still be catching up to the reference, so permit lag but
    # reject a material disagreement from either independent measurement.
    max_error = max(0.004, 1.25 * abs(reference))
    return bool(
      abs(measured_curvature - reference) <= max_error and
      abs(yaw_curvature - reference) <= max_error
    )

  def _curve_hold_conditions_met(self, car_state, controls_active,
                                 measured_curvature, lane_change_active, t):
    if (not self.use_lanelines or not controls_active or
        car_state.steeringPressed or lane_change_active or
        not self.model_data_valid or
        self.model_confidence < CURVE_HOLD_MODEL_CONFIDENCE_MIN or
        t - self.LP.lane_center_last_continuous_t >
        CURVE_HOLD_LANE_CONTINUITY_MAX_AGE_S or
        self._curve_hold_path is None or
        t - self._curve_hold_reference_t > CURVE_HOLD_REFERENCE_MAX_AGE_S):
      return False

    # Hold is only a bridge for a confidence transition. A normally trusted
    # frame must continue to use its current model/lane path directly.
    perception_degraded = bool(
      self.model_confidence < CURVE_HOLD_MODEL_CONFIDENCE_TRUSTED or
      self.LP.raw_lane_d_prob < CURVE_HOLD_LANE_RAW_DPROB_TRUSTED)
    return bool(perception_degraded and
                self._vehicle_matches_curve_hold(car_state,
                                                  measured_curvature))

  def _apply_virtual_curve_hold(self, d_path_xyz, car_state,
                                controls_active, measured_curvature,
                                lane_change_active):
    """Blend a re-projected trusted path for a strictly bounded dropout hold."""
    self.virtual_curve_hold_active = False
    self.virtual_curve_hold_weight = 0.0
    t = sec_since_boot()
    if not self._curve_hold_conditions_met(
        car_state, controls_active, measured_curvature, lane_change_active, t):
      return d_path_xyz

    if self._curve_hold_start_t is None:
      self._curve_hold_start_t = t
    elapsed = t - self._curve_hold_start_t
    if elapsed >= CURVE_HOLD_MAX_DURATION_S:
      self._curve_hold_start_t = None
      self._curve_hold_path = None
      return d_path_xyz

    # Transform the previous ego-relative path into the current ego frame.
    # The hold window is short enough that the present speed/yaw-rate provide
    # a conservative re-projection without inventing a future road shape.
    reference_age = max(0.0, t - self._curve_hold_reference_t)
    travel = float(car_state.vEgo) * reference_age
    yaw_change = float(car_state.yawRate) * reference_age
    old_x = self._curve_hold_path[:, 0] - travel
    old_y = self._curve_hold_path[:, 1]
    cos_yaw = np.cos(yaw_change)
    sin_yaw = np.sin(yaw_change)
    held_x = cos_yaw * old_x + sin_yaw * old_y
    held_y = -sin_yaw * old_x + cos_yaw * old_y

    order = np.argsort(held_x)
    held_x, unique_indices = np.unique(held_x[order], return_index=True)
    held_y = held_y[order][unique_indices]
    if held_x.size < 2 or not np.isfinite(held_y).all():
      self._curve_hold_start_t = None
      self._curve_hold_path = None
      return d_path_xyz

    in_reference_range = ((d_path_xyz[:, 0] >= held_x[0]) &
                          (d_path_xyz[:, 0] <= held_x[-1]))
    if not np.any(in_reference_range):
      self._curve_hold_start_t = None
      self._curve_hold_path = None
      return d_path_xyz

    held_path_y = np.interp(d_path_xyz[:, 0], held_x, held_y)
    decay = 1.0 - elapsed / CURVE_HOLD_MAX_DURATION_S
    weight = CURVE_HOLD_BLEND * decay
    max_correction = np.interp(
      np.abs(d_path_xyz[:, 0]), [0.0, 25.0],
      [CURVE_HOLD_MAX_CORRECTION_NEAR_M,
       CURVE_HOLD_MAX_CORRECTION_FAR_M])
    correction = np.clip(
      held_path_y - d_path_xyz[:, 1], -max_correction, max_correction)
    d_path_xyz[in_reference_range, 1] += (
      weight * correction[in_reference_range])
    self.virtual_curve_hold_active = True
    self.virtual_curve_hold_weight = float(weight)
    return d_path_xyz

  def _refresh_curve_hold_reference(self, d_path_xyz, car_state,
                                    controls_active, measured_curvature,
                                    lane_change_active, mpc_valid):
    """Arm the next dropout only from a current, independently confirmed turn."""
    if (not self.use_lanelines or not controls_active or
        car_state.steeringPressed or lane_change_active or not mpc_valid or
        not self.model_data_valid or
        self.model_confidence < CURVE_HOLD_MODEL_CONFIDENCE_TRUSTED or
        self.LP.raw_lane_d_prob < CURVE_HOLD_LANE_RAW_DPROB_TRUSTED or
        not self.LP.lane_center_continuous):
      return

    reference_curvature = float(self.lat_mpc.x_sol[1, 3])
    if not np.isfinite(reference_curvature):
      return

    previous_curvature = self._curve_hold_curvature
    self._curve_hold_curvature = reference_curvature
    if not self._vehicle_matches_curve_hold(car_state, measured_curvature):
      self._curve_hold_curvature = previous_curvature
      return

    reference_path = d_path_xyz[:, :2]
    if not np.isfinite(reference_path).all():
      self._curve_hold_curvature = previous_curvature
      return

    self._curve_hold_path = reference_path.copy()
    self._curve_hold_reference_t = sec_since_boot()
    self._curve_hold_start_t = None

  def update(self, sm):
    car_state = sm['carState']
    v_ego = car_state.vEgo
    measured_curvature = sm['controlsState'].curvature
    controls_active = bool(sm['controlsState'].active)

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md)
    trajectory = validated_model_trajectory(md, TRAJECTORY_SIZE)
    self.model_data_valid = trajectory is not None
    if trajectory is not None:
      self.path_xyz, self.speed_forward, self.t_idxs, self.plan_yaw, self.plan_yaw_rate = trajectory
      self.plan_curv = self.plan_yaw_rate / np.maximum(self.speed_forward, np.ones_like(self.speed_forward))
      # Curvature rate is currently not passed to the MPC. Keep it finite and
      # avoid np.gradient failures on malformed/non-monotonic model timestamps.
      self.plan_curv_rate.fill(0.0)
    elif sec_since_boot() > self.last_cloudlog_t + 5.0:
      self.last_cloudlog_t = sec_since_boot()
      cloudlog.warning("Lateral planner - incomplete or non-finite model trajectory")

    position_stds = (
      as_finite_vector(md.position.xStd, expected_size=TRAJECTORY_SIZE),
      as_finite_vector(md.position.yStd, expected_size=TRAJECTORY_SIZE),
      as_finite_vector(md.position.zStd, expected_size=TRAJECTORY_SIZE),
    )
    if all(position_std is not None for position_std in position_stds):
      self.path_xyz_stds = np.column_stack(position_stds)
    self._update_model_confidence(
      all(position_std is not None for position_std in position_stds))

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], sm['controlsState'].active, lane_change_prob)

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    lane_change_active = self.DH.lane_change_state != log.LateralPlan.LaneChangeState.off
    if self.use_lanelines:
      # LanePlanner applies its offset in-place; a copy prevents an invalid
      # model frame from accumulating the offset on the last valid trajectory.
      d_path_xyz = self.LP.get_d_path(
        v_ego, self.t_idxs, self.path_xyz.copy(),
        measured_curvature=measured_curvature,
        lane_change_active=lane_change_active)
      self.lat_mpc.set_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, MPC_COST_LAT.STEER_RATE)
    else:
      d_path_xyz = self.path_xyz
      # Heading cost is useful at low speed, otherwise end of plan can be off-heading
      heading_cost = interp(v_ego, [5.0, 10.0], [MPC_COST_LAT.HEADING, 0.15])
      self.lat_mpc.set_weights(MPC_COST_LAT.PATH, heading_cost, MPC_COST_LAT.STEER_RATE)

    d_path_xyz = self._apply_virtual_curve_hold(
      d_path_xyz, car_state, controls_active, measured_curvature,
      lane_change_active)

    # The current model/lane blend normally goes directly to MPC. The only
    # exception is the bounded virtual hold above, which is disabled as soon
    # as model confidence, lane continuity, yaw, or curvature disagree.
    d_path_distance = np.linalg.norm(d_path_xyz, axis=1)
    y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], d_path_distance, d_path_xyz[:, 1])
    heading_pts = np.interp(
      v_ego * self.t_idxs[:LAT_MPC_N + 1],
      np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    if (self.virtual_curve_hold_active and
        np.all(np.diff(d_path_distance) > 1e-3)):
      # While the held path is active, give MPC a heading reference derived
      # from that same blended path. Keeping the current-model heading here
      # would counteract the intentionally short curvature continuation.
      held_heading = np.arctan(np.gradient(
        d_path_xyz[:, 1], d_path_distance))
      if np.isfinite(held_heading).all():
        heading_pts = np.interp(
          v_ego * self.t_idxs[:LAT_MPC_N + 1],
          d_path_distance, held_heading)
    curv_rate_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_curv_rate)
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(curv_rate_pts) == LAT_MPC_N + 1
    lateral_factor = max(0, self.factor1 - (self.factor2 * v_ego**2))
    p = np.array([v_ego, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     np.zeros_like(curv_rate_pts))
    # init state for next
    # mpc.u_sol is the desired curvature rate given x0 curv state.
    # with x0[3] = measured_curvature, this would be the actual desired rate.
    # instead, interpolate x_sol so that x0[3] is the desired curvature for lat_control.
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    mpc_solution_valid = not mpc_nans and self.lat_mpc.solution_status == 0
    t = sec_since_boot()
    if not mpc_solution_valid:
      self.reset_mpc()
      self.x0[3] = measured_curvature
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    self._refresh_curve_hold_reference(
      d_path_xyz, car_state, controls_active, measured_curvature,
      lane_change_active,
      mpc_valid=mpc_solution_valid)

    if self.lat_mpc.cost > 20000. or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    required_services = ['carState', 'controlsState', 'modelV2']
    # Message validity describes whether this plan was built from present,
    # valid inputs. Average-rate health is monitored independently by
    # controlsd; folding its long rolling window into this flag can keep every
    # lateralPlan invalid long after a brief EON scheduling delay has cleared.
    plan_send.valid = (
      self.model_data_valid and
      sm.all_alive(service_list=required_services) and
      sm.all_valid(service_list=required_services)
    )

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
    lateralPlan.curvatures = self.lat_mpc.x_sol[0:CONTROL_N, 3].tolist()
    lateralPlan.curvatureRates = [float(x) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.use_lanelines
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    lateralPlan.autoLaneChangeEnabled = self.DH.auto_lane_change_enabled
    lateralPlan.autoLaneChangeTimer = int(AUTO_LCA_START_TIME) - int(self.DH.auto_lane_change_timer)

    lateralPlan.totalCameraOffset = float(self.LP.total_camera_offset)
    # Compatibility diagnostics. The official-style planner no longer blocks
    # or filters the current path with a custom instability state machine.
    lateralPlan.pathStabilityActive = False
    lateralPlan.pathWobbleRangeM = 0.0
    lateralPlan.pathWobbleFlips = 0
    lateralPlan.laneCenterCorrectionM = float(self.LP.lane_center_correction_m)
    lateralPlan.laneCenterCorrectionActive = bool(self.LP.lane_center_correction_active)

    pm.send('lateralPlan', plan_send)
