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

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    measured_curvature = sm['controlsState'].curvature

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

    # Match the official planner: the current model/lane blend goes directly
    # to MPC. Do not retain or blend a previous path across real curve changes.
    d_path_distance = np.linalg.norm(d_path_xyz, axis=1)
    y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], d_path_distance, d_path_xyz[:, 1])
    heading_pts = np.interp(
      v_ego * self.t_idxs[:LAT_MPC_N + 1],
      np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
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
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

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
