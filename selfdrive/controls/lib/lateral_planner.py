import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MPC_COST_LAT, LAT_MPC_N
from selfdrive.controls.lib.desire_helper import DesireHelper, AUTO_LCA_START_TIME
from selfdrive.controls.lib.curve_virtual_readiness import CurveVirtualReadinessMonitor
import cereal.messaging as messaging
from cereal import log
from common.params import Params
from selfdrive.controls.lib.model_data_validation import as_finite_vector, validated_model_trajectory

TRAJECTORY_SIZE = 33

# How far the lane blend is allowed to move the path away from the model path.
# Every correction ceiling inside lane_planner is at or below this: ordinary
# blending 1.00 m, temporal hold 0.75 m, road edge 0.70 m (1.20 m readiness
# trusted), model-path hold 0.45 m (0.90 m trusted). Anything past it is a bug
# in the blend, not a wide correction, so the model path is used for that frame.
MAX_LANE_BLEND_DEVIATION_M = 1.50

# Geometric sanity envelope on the target handed to the MPC:
#
#   |y(t)| <= LATERAL_ENVELOPE_MARGIN_M + v_ego * t
#
# A point on the path cannot be further sideways than the distance travelled to
# reach it, so this is a hard geometric bound rather than a tuning choice, and
# it cannot reject a real plan: a first pass at 0.5 * v * t (a 30-degree heading
# cap) clipped a genuine low-speed turn on 2026-09-05--09-24-53 at 8.9 m, so the
# full arc length is used instead. The margin covers the path offset, the camera
# offset, and the interpolation onto the horizon grid. What it does catch is a
# path that is not a path: the 52.7 m target at +378.43 s is rejected on the
# very first sample, where the envelope is only the margin.
LATERAL_ENVELOPE_MARGIN_M = 1.50

class LateralPlanner:
  def __init__(self, CP):
    self.LP = LanePlanner()
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0
    self.last_plan_cloudlog_t = 0
    self.plan_implausible_cnt = 0
    self.plan_implausible = False

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
    self.curve_virtual_readiness = CurveVirtualReadinessMonitor()

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=None):
    # Callers write into x0 straight after this (x0[3] = measured curvature), so
    # a shared default array would carry the last reset's curvature into the
    # next one instead of starting from zero. Build a fresh one every time.
    self.x0 = np.zeros(4) if x0 is None else np.array(x0, dtype=float)
    self.lat_mpc.reset(x0=self.x0)

  def _log_implausible_plan(self, detail):
    """Rate-limited report of a lateral plan that was rejected or clipped."""
    t = sec_since_boot()
    if t > self.last_plan_cloudlog_t + 1.0:
      self.last_plan_cloudlog_t = t
      cloudlog.error("lateral plan implausible (%d so far): %s"
                     % (self.plan_implausible_cnt + 1, detail))

  def update(self, sm):
    car_state = sm['carState']
    v_ego = car_state.vEgo
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
    lane_confidence = max(self.LP.lll_prob, self.LP.rll_prob)
    # This GM's EBCM does not transmit a usable yaw-rate signal on the CAN bus
    # (car_state.yawRate's DBC-mapped bits read a constant 0 in every real
    # drive log checked). Use the device's own IMU-derived yaw instead. Gate
    # on the measurement's own validity, not liveLocationKalman.status: status
    # stays 'uninitialized' on this device (it never gets a GPS lock) even
    # while angularVelocityCalibrated itself is fine.
    llk = sm['liveLocationKalman']
    angular_velocity = llk.angularVelocityCalibrated
    yaw_rate_valid = bool(
      sm.valid['liveLocationKalman'] and angular_velocity.valid and
      llk.inputsOK and llk.sensorsOK and llk.deviceStable)
    yaw_rate_imu = (
      float(angular_velocity.value[2])
      if yaw_rate_valid and len(angular_velocity.value) > 2 else 0.0)
    # Independent physical curvature estimate for the curve-fallback virtual
    # path (see lane_planner.py's _blended_curvature): yaw_rate / v_ego, floored
    # so the division stays finite near a stop.
    imu_curvature = yaw_rate_imu / max(v_ego, 1.0)
    _, completed_curve_readiness = self.curve_virtual_readiness.update(
      v_ego, measured_curvature, yaw_rate_imu, car_state.steeringPressed,
      lane_change_active, lane_confidence, yaw_valid=yaw_rate_valid)
    if (completed_curve_readiness is not None and
        completed_curve_readiness['laneLossRatio'] > 0.0):
      cloudlog.event('virtualCurveReadiness', **completed_curve_readiness)

    # LanePlanner applies its offset in-place; a copy prevents an invalid
    # model frame from accumulating the offset on the last valid trajectory.
    d_path_xyz = self.LP.get_d_path(
      v_ego, self.t_idxs, self.path_xyz.copy(),
      measured_curvature=measured_curvature,
      lane_change_active=lane_change_active,
      readiness_eligible=self.curve_virtual_readiness.current['eligible'],
      readiness_quality=self.curve_virtual_readiness.current['quality'],
      imu_curvature=imu_curvature, imu_curvature_valid=yaw_rate_valid)
    self.lat_mpc.set_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, MPC_COST_LAT.STEER_RATE)

    # Match the official planner: the current model/lane blend goes directly
    # to MPC. Do not retain or blend a previous path across real curve changes.
    #
    # Everything get_d_path may legitimately do to the model path is bounded by
    # its own correction ceilings, the largest of which is 1.20 m. Check that
    # here rather than trusting it: a planner bug on the far side of that call
    # reaches the steering with nothing else in the way, and one did. On
    # 2026-09-05--09-24-53 at +378.43 s the lane-centre slew rescale drove the
    # blended path to 52.7 m while the model path stayed inside +/-0.5 m, and the
    # MPC turned that single frame into a 17.0 m/s^2 demand that took a second to
    # unwind because x0 carries the solution forward. Fall back to the model's
    # own path for the frame instead; it is the input the blend started from.
    self.plan_implausible = False
    lane_deviation = float(np.max(np.abs(d_path_xyz[:, 1] - self.path_xyz[:, 1])))
    if lane_deviation > MAX_LANE_BLEND_DEVIATION_M:
      d_path_xyz = self.path_xyz.copy()
      self.plan_implausible = True
      self._log_implausible_plan(
        "lane blend moved the path %.1f m from the model path (limit %.2f m)"
        % (lane_deviation, MAX_LANE_BLEND_DEVIATION_M))

    d_path_distance = np.linalg.norm(d_path_xyz, axis=1)
    y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], d_path_distance, d_path_xyz[:, 1])
    heading_pts = np.interp(
      v_ego * self.t_idxs[:LAT_MPC_N + 1],
      np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    curv_rate_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_curv_rate)

    # Second, independent check on the target the MPC actually receives, so a
    # bad model path is caught too and not only a bad blend. The interp above
    # can also read the far end of the path when d_path_distance is not
    # monotonic, which it is not whenever the model's position.x starts negative.
    # Clip rather than reject: the near samples of an otherwise sane path stay
    # usable, and the shape is preserved wherever it is inside the envelope.
    horizon_t = np.asarray(self.t_idxs[:LAT_MPC_N + 1], dtype=float)
    y_envelope = LATERAL_ENVELOPE_MARGIN_M + max(v_ego, 0.0) * horizon_t
    if np.any(np.abs(y_pts) > y_envelope):
      worst = int(np.argmax(np.abs(y_pts) - y_envelope))
      self.plan_implausible = True
      self._log_implausible_plan(
        "target %.1f m at %.2f s exceeds the achievable envelope %.1f m"
        % (float(y_pts[worst]), float(horizon_t[worst]), float(y_envelope[worst])))
      y_pts = np.clip(y_pts, -y_envelope, y_envelope)

    if self.plan_implausible:
      self.plan_implausible_cnt += 1

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

    # x0 carries the solution into the next frame, so a frame built on a
    # rejected plan would keep steering the car after the plan itself is sane
    # again -- that carry-over, not the bad frame, is what turned +378.43 s of
    # 2026-09-05--09-24-53 into a full second of rising demand. Re-seed from the
    # curvature the car is actually holding instead.
    #
    # After the cost check above, not before it: LateralMpc.reset() zeroes
    # self.cost, so resetting first would report a genuinely expensive solve on
    # this frame as a free one and clear solution_invalid_cnt with it. Only the
    # next frame's x0 depends on this, so the later position is equivalent.
    if self.plan_implausible:
      self.reset_mpc()
      self.x0[3] = measured_curvature

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
    # Lane lines are always used; the laneless mode was removed 2026-08-27.
    # Kept in the message so existing log tooling keeps reading a valid field.
    lateralPlan.useLaneLines = True
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
    # Curve-tracking readiness. Published for drive-log analysis, but the same
    # two values were passed to get_d_path above, where they widen the
    # curve-fallback limits -- this is not a diagnostic-only signal.
    #
    # The field names predate that wiring and mislead: they measure agreement
    # between steering-derived curvature and IMU yaw rate, not model path
    # quality, and the model's output is not an input. They also read low
    # because the monitor only samples inside a curve -- roughly 13% of frames
    # on a mixed drive -- so a low rate here is mostly "not asked".
    lateralPlan.modelPathQuality = float(self.curve_virtual_readiness.current['quality'])
    lateralPlan.modelPathQualityTrusted = bool(self.curve_virtual_readiness.current['eligible'])
    lateralPlan.modelNearCurvature = float(self.curve_virtual_readiness.current['curvatureMean'])
    # Diagnostic-only view of the tight-curve temporal-hold store gate, so a
    # drive log can show which condition prevents a fallback path from being
    # cached before the lane lines leave this narrow camera's view.
    lateralPlan.curveAssist = float(self.LP.curve_assist_diag)
    lateralPlan.curveRawTargetDProb = float(self.LP.curve_raw_target_d_prob_diag)
    lateralPlan.curveGeometryPlausible = bool(self.LP.curve_geometry_plausible_diag)
    lateralPlan.curveTemporalStored = bool(self.LP.curve_temporal_stored_diag)
    lateralPlan.curveTemporalHoldAgeS = float(self.LP.curve_temporal_hold_age_diag)
    lateralPlan.curveFallbackSource = int(self.LP.curve_fallback_source_diag)

    pm.send('lateralPlan', plan_send)
