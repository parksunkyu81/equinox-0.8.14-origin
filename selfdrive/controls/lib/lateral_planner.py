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
from selfdrive.controls.lib.model_data_validation import validated_model_trajectory
from selfdrive.controls.lib.curve_speed_limiter import (
  build_model_curve_profile_from_model, build_v0813_model_curve_profile, model_curve_tuning_band)

TRAJECTORY_SIZE = 33

# How far the lane blend is allowed to move the path away from the model path.
# Every correction ceiling inside lane_planner is at or below this (ordinary
# blending tops out at 1.00 m). Anything past it is a bug in the blend, not a
# wide correction, so the model path is used for that frame.
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

# WideOptechLateral: the OPTECH-KR (optech-spst-dev) lateral planner, i.e.
# comma's 0.9 one, driven by the 59 deg wide model input (modeld, same param).
# Ported as it is there with its default dp_lat_lane_priority_mode off: the
# MPC follows the model's own path (no lane blend, no camera offset) with yaw-
# rate references, MPC speed floored at 1 m/s, and its 1e6 cost validity limit.
# Replayed on 11-21 corner segments this was the closest of the four input x
# planner pairings to the curvature the car drove; the straight-road jitter it
# adds is handled by StraightWobbleFilter in get_lag_adjusted_curvature. The
# sanity checks below (model validation, target envelope, reset on an
# implausible plan) apply to both modes.
V09_MIN_SPEED = 1.0
V09_COST_INVALID = 1e6
# v09 (OPTECH) follows the model's own path, which carries no lane centring:
# on 2026-11-23--01-36-34 the car sat 0.29 m inside left curves and 0.41 m
# inside right ones (29% of curve time > 0.4 m off), and when it was 0.5 m off
# the path at 10 m still pointed 0.23 m to the same side, so it took 1.6-4 s
# (median 2.3 s) to come back. This share of LanePlanner's lane-centre
# correction is added to the model path: replayed on that drive the residual
# at 10 m falls to 0.12 m; on straights the correction averages -0.03 m. The
# correction keeps LanePlanner's confidence weighting (d_prob), clamps (0.25 m
# near, 1.0 m at 20 m) and 2 m/s slew limit; none during a turn or lane change.
# 0.5 -> 0.8 on 2026-11-24: four drives at 0.5 (6.8 min with both lines sure)
# against four without it -- straights over 0.4 m off 4.2% -> 1.7%, offset std
# 0.19 -> 0.145 m, recovery median 1.9 -> 1.4 s -- but with the car 0.46 m off
# it pulled only 0.11 m toward the centre at 20 m (0.06 straight, 0.18 curves).
V09_LANE_CENTER_GAIN = 0.8

# road_speed_limiter publishes roadLimitSpeed (with atcType) at 20 Hz.
ATC_MSG_TIMEOUT_S = 0.5

# Turn opposite-side limit. With a turn desire out, the path may not go further
# toward the other side than the ego lane's own centre (and never further than
# straight ahead when the lane bends toward the turn). 2026-11-25--06-53-47:
# before the slow left turns the model's path bent right while the left-turn
# desire was out -- 774 s +1.0 m at 20 m with the lane centred at ~0 (the
# desire's own wobble), 834 s +3.4 m where the lane itself curved right to
# +2.4 m -- and at 834 s the car steered right until the driver pushed back.
# Zeroing the right side outright would have put the 834 s target 1 m left of
# the left lane line, so the bound follows the lane. Where the model's two ego
# lines are not a plausible lane (width outside TURN_OPPOSITE_LANE_WIDTH_M) the
# path is left alone. Model frame: y + is to the right.
TURN_OPPOSITE_LANE_WIDTH_M = (2.5, 4.5)


def limit_turn_opposite(path_xyz, turn_left, ll_x, lll_y, rll_y):
  """Path y with the opposite-side excursion cut at the lane centre, or None if nothing changed."""
  x = path_xyz[:, 0]
  y = path_xyz[:, 1]
  try:
    centre = np.interp(x, ll_x, (lll_y + rll_y) * 0.5)
    width = np.interp(x, ll_x, rll_y - lll_y)
  except (ValueError, TypeError):
    return None
  lo, hi = TURN_OPPOSITE_LANE_WIDTH_M
  valid = (width >= lo) & (width <= hi) & np.isfinite(centre)
  if turn_left:
    bound = np.where(valid, np.maximum(centre, 0.0), np.inf)
    new_y = np.minimum(y, bound)
  else:
    bound = np.where(valid, np.minimum(centre, 0.0), -np.inf)
    new_y = np.maximum(y, bound)
  if not np.any(np.abs(new_y - y) > 0.01):
    return None
  return new_y


# The model's orientation.z has the same sign as the heading of its own path
# but ~0.55 of its size (11-25 seg 12, 204 curve frames: corr 0.99, sign 100%).
TURN_OPPOSITE_YAW_SCALE = 0.55


def limit_turn_opposite_yaw(x, new_y, t, plan_yaw, plan_yaw_rate, turn_left):
  """The model's heading and yaw rate, cut where they point further to the other
  side than the limited path does (scaled to the model's own size)."""
  geo = np.arctan2(np.gradient(new_y), np.maximum(np.gradient(x), 1e-3))
  geo_rate = np.gradient(geo) / np.maximum(np.gradient(t), 1e-3)
  yaw_bound = TURN_OPPOSITE_YAW_SCALE * geo
  rate_bound = TURN_OPPOSITE_YAW_SCALE * geo_rate
  if turn_left:
    return (np.minimum(plan_yaw, np.maximum(yaw_bound, 0.0)),
            np.minimum(plan_yaw_rate, np.maximum(rate_bound, 0.0)))
  return (np.maximum(plan_yaw, np.minimum(yaw_bound, 0.0)),
          np.maximum(plan_yaw_rate, np.minimum(rate_bound, 0.0)))


class LateralPlanner:
  def __init__(self, CP, use_v09=None):
    self.LP = LanePlanner()
    self.v09_center_corr_m = 0.0   # lane centring added to the v09 path at 20 m [m]
    self.DH = DesireHelper()
    # Read once, like modeld and controlsd: all three must agree for a drive.
    self.use_v09 = Params().get_bool("WideOptechLateral") if use_v09 is None else bool(use_v09)

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0
    self.last_plan_cloudlog_t = 0
    self.plan_implausible_cnt = 0
    self.plan_implausible = False

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.speed_forward = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.plan_curv_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros(TRAJECTORY_SIZE)
    self.model_data_valid = False
    # controlsd's model curve profile, built here (see log.capnp curveProfile*).
    self.curve_profile = None
    self.curve_profile_band = 0

    self.v_mpc = 0.0
    if self.use_v09:
      # Imported only here: the 0.8.13 mode must not depend on the second solver.
      from selfdrive.controls.lib.lateral_mpc_lib_v09 import lat_mpc as lat_mpc_v09
      self.lat_mpc = lat_mpc_v09.LateralMpc()
      self.lat_mpc.set_weights(lat_mpc_v09.PATH_COST, lat_mpc_v09.LATERAL_MOTION_COST,
                               lat_mpc_v09.LATERAL_ACCEL_COST, lat_mpc_v09.LATERAL_JERK_COST,
                               lat_mpc_v09.STEERING_RATE_COST)
    else:
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

  @staticmethod
  def _atc_type(sm):
    """AutoTurnControl type from road_speed_limiter (20 Hz). A stale message
    (process down, or a SubMaster without the service) means no request: a
    frozen "turn left" must never keep steering the model."""
    try:
      if 'roadLimitSpeed' not in sm.data or sm.rcv_time['roadLimitSpeed'] <= 0.:
        return ""
      if sec_since_boot() - sm.rcv_time['roadLimitSpeed'] > ATC_MSG_TIMEOUT_S:
        return ""
      return str(sm['roadLimitSpeed'].atcType)
    except (KeyError, AttributeError, TypeError):
      return ""

  @staticmethod
  def _atc_dist(sm):
    """Navigation distance to the AutoTurnControl point [m]; 0 when stale or absent."""
    try:
      if 'roadLimitSpeed' not in sm.data or sm.rcv_time['roadLimitSpeed'] <= 0.:
        return 0.
      if sec_since_boot() - sm.rcv_time['roadLimitSpeed'] > ATC_MSG_TIMEOUT_S:
        return 0.
      return float(sm['roadLimitSpeed'].xDistToTurn)
    except (KeyError, AttributeError, TypeError, ValueError):
      return 0.

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
      # (plan_curv and path_xyz_stds, three more model fields read off capnp
      # each frame, went: nothing read them. plan_curv_rate stays all zeros.)
    elif sec_since_boot() > self.last_cloudlog_t + 5.0:
      self.last_cloudlog_t = sec_since_boot()
      cloudlog.warning("Lateral planner - incomplete or non-finite model trajectory")

    # controlsd's curve slowdown profile, once per model frame. Index 0 is a
    # placeholder that controlsd overwrites with its live measured curvature.
    # Reading a capnp list costs ~40 us a field on the EON, so the five fields
    # validated_model_trajectory already converted are handed over as lists
    # (the same float values); only velocity is still read off the message.
    try:
      self.curve_profile_band = model_curve_tuning_band(v_ego)
      if trajectory is not None:
        vel = md.velocity
        path_cols = self.path_xyz.T
        self.curve_profile = build_v0813_model_curve_profile(
          self.t_idxs.tolist(), self.plan_yaw_rate.tolist(),
          vel.x, vel.y, vel.z,
          path_cols[0].tolist(), path_cols[1].tolist(), path_cols[2].tolist(),
          0.0, v_ego=v_ego)
      else:
        self.curve_profile = build_model_curve_profile_from_model(md, 0.0, v_ego)
    except Exception:
      self.curve_profile = None

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], sm['controlsState'].active, lane_change_prob,
                   modeldata=md, atc_type=self._atc_type(sm), atc_dist=self._atc_dist(sm))

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    lane_change_active = self.DH.lane_change_state != log.LateralPlan.LaneChangeState.off

    if self.use_v09:
      # OPTECH default (lane priority mode off): the model's own path. The
      # weights were set once in __init__ (OPTECH sets the same ones each frame).
      d_path_xyz = self.path_xyz.copy()
      # The lane blend also keeps LP.d_prob current for the onroad confidence
      # gauge (dProb). A share of its centring is added to the model path
      # (V09_LANE_CENTER_GAIN), measured from the model path plus LanePlanner's
      # own path offset so that offset is not brought in.
      lane_path = self.LP.get_d_path(
        v_ego, self.t_idxs, self.path_xyz.copy(),
        measured_curvature=measured_curvature,
        lane_change_active=lane_change_active)
      self.v09_center_corr_m = 0.0
      # turn_direction too: the turn desire re-pulse blinks DH.desire to none
      # for a frame or two in the middle of a turn.
      if (self.DH.desire == log.LateralPlan.Desire.none and not lane_change_active and
          self.DH.turn_direction == log.LateralPlan.Desire.none):
        centre = lane_path[:, 1] - (self.path_xyz[:, 1] + self.LP.path_offset)
        if np.all(np.isfinite(centre)):
          d_path_xyz[:, 1] += V09_LANE_CENTER_GAIN * centre
          self.v09_center_corr_m = float(V09_LANE_CENTER_GAIN * np.interp(20.0, self.path_xyz[:, 0], centre))
    else:
      # LanePlanner applies its offset in-place; a copy prevents an invalid
      # model frame from accumulating the offset on the last valid trajectory.
      d_path_xyz = self.LP.get_d_path(
        v_ego, self.t_idxs, self.path_xyz.copy(),
        measured_curvature=measured_curvature,
        lane_change_active=lane_change_active)
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
      # The car is given the model path this frame, so the lane planner's
      # output limiter must ramp from zero offset, not from the rejected one.
      self.LP.reset_output_limiter()
      self.plan_implausible = True
      self._log_implausible_plan(
        "lane blend moved the path %.1f m from the model path (limit %.2f m)"
        % (lane_deviation, MAX_LANE_BLEND_DEVIATION_M))

    # Turn opposite-side limit (TURN_OPPOSITE_*): heading and yaw rate follow
    # the limited path, so the MPC is not handed a straight path that still
    # points the other way.
    plan_yaw, plan_yaw_rate = self.plan_yaw, self.plan_yaw_rate
    self.turn_opposite_cut_m = 0.0
    turn = self.DH.turn_direction
    if turn != log.LateralPlan.Desire.none and not lane_change_active and self.model_data_valid:
      turn_left = turn == log.LateralPlan.Desire.turnLeft
      new_y = limit_turn_opposite(d_path_xyz, turn_left, self.LP.ll_x, self.LP.lll_y, self.LP.rll_y)
      if new_y is not None:
        self.turn_opposite_cut_m = float(np.interp(20.0, d_path_xyz[:, 0], d_path_xyz[:, 1] - new_y))
        d_path_xyz[:, 1] = new_y
        plan_yaw, plan_yaw_rate = limit_turn_opposite_yaw(
          d_path_xyz[:, 0], new_y, self.t_idxs, plan_yaw, plan_yaw_rate, turn_left)

    # OPTECH floors the MPC speed at 1 m/s (lateral planning is not possible at 0).
    v_mpc = max(V09_MIN_SPEED, v_ego) if self.use_v09 else v_ego
    self.v_mpc = v_mpc
    d_path_distance = np.linalg.norm(d_path_xyz, axis=1)
    path_distance = np.linalg.norm(self.path_xyz, axis=1)
    y_pts = np.interp(v_mpc * self.t_idxs[:LAT_MPC_N + 1], d_path_distance, d_path_xyz[:, 1])
    heading_pts = np.interp(v_mpc * self.t_idxs[:LAT_MPC_N + 1], path_distance, plan_yaw)
    if self.use_v09:
      yaw_rate_pts = np.interp(v_mpc * self.t_idxs[:LAT_MPC_N + 1], path_distance, plan_yaw_rate)
    else:
      curv_rate_pts = np.interp(v_mpc * self.t_idxs[:LAT_MPC_N + 1], path_distance, self.plan_curv_rate)

    # Second, independent check on the target the MPC actually receives, so a
    # bad model path is caught too and not only a bad blend. The interp above
    # can also read the far end of the path when d_path_distance is not
    # monotonic, which it is not whenever the model's position.x starts negative.
    # Clip rather than reject: the near samples of an otherwise sane path stay
    # usable, and the shape is preserved wherever it is inside the envelope.
    horizon_t = np.asarray(self.t_idxs[:LAT_MPC_N + 1], dtype=float)
    y_envelope = LATERAL_ENVELOPE_MARGIN_M + max(v_mpc, 0.0) * horizon_t
    if np.any(np.abs(y_pts) > y_envelope):
      worst = int(np.argmax(np.abs(y_pts) - y_envelope))
      self.plan_implausible = True
      self._log_implausible_plan(
        "target %.1f m at %.2f s exceeds the achievable envelope %.1f m"
        % (float(y_pts[worst]), float(horizon_t[worst]), float(y_envelope[worst])))
      y_pts = np.minimum(np.maximum(y_pts, -y_envelope), y_envelope)

    if self.plan_implausible:
      self.plan_implausible_cnt += 1

    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    lateral_factor = max(0, self.factor1 - (self.factor2 * v_mpc**2))
    p = np.array([v_mpc, lateral_factor])
    if self.use_v09:
      assert len(yaw_rate_pts) == LAT_MPC_N + 1
      self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts)
    else:
      assert len(curv_rate_pts) == LAT_MPC_N + 1
      self.lat_mpc.run(self.x0,
                       p,
                       y_pts,
                       heading_pts,
                       np.zeros_like(curv_rate_pts))
    # init state for next
    # mpc.u_sol is the desired curvature rate given x0 curv state.
    # with x0[3] = measured_curvature, this would be the actual desired rate.
    # instead, interpolate x_sol so that x0[3] is the desired curvature for lat_control.
    # (v09: state 3 is the yaw rate; the same interpolation, as OPTECH does it.)
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = self._x0_from_curvature(measured_curvature)
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    cost_invalid = V09_COST_INVALID if self.use_v09 else 20000.
    if self.lat_mpc.cost > cost_invalid or mpc_nans:
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
      self.x0[3] = self._x0_from_curvature(measured_curvature)

  def _x0_from_curvature(self, curvature):
    """MPC state 3 for a measured curvature: the curvature, or the yaw rate for v09."""
    return curvature * self.v_mpc if self.use_v09 else curvature

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
    if self.use_v09:
      # The v09 MPC state is yaw rate and its control yaw acceleration; the
      # message carries curvature, divided by speed exactly as OPTECH does.
      v_mpc = max(self.v_mpc, V09_MIN_SPEED)
      lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3] / v_mpc).tolist()
      lateralPlan.curvatureRates = [float(x) / v_mpc for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    else:
      lateralPlan.curvatures = self.lat_mpc.x_sol[0:CONTROL_N, 3].tolist()
      lateralPlan.curvatureRates = [float(x) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    # The 0.8.13 planner always blends lane lines; the v09 one (WideOptechLateral)
    # follows the model path alone.
    lateralPlan.useLaneLines = not self.use_v09
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    lateralPlan.autoLaneChangeEnabled = self.DH.auto_lane_change_enabled
    # The countdown belongs to the driver-blinker path only. The ATC path never
    # advances auto_lane_change_timer and never starts on a timer, so it used to
    # publish a stuck "1 s" for the whole ATC wait.
    lateralPlan.autoLaneChangeTimer = 0 if self.DH.atc_owner else \
      int(AUTO_LCA_START_TIME) - int(self.DH.auto_lane_change_timer)

    lateralPlan.totalCameraOffset = float(self.LP.total_camera_offset)
    # Compatibility diagnostics. The official-style planner no longer blocks
    # or filters the current path with a custom instability state machine.
    lateralPlan.pathStabilityActive = False
    lateralPlan.pathWobbleRangeM = 0.0
    lateralPlan.pathWobbleFlips = 0
    # The v09 planner computes the lane blend only for dProb and never steers
    # to it, so it applies no lane-centre correction.
    if self.use_v09:
      lateralPlan.laneCenterCorrectionM = float(self.v09_center_corr_m)
      lateralPlan.laneCenterCorrectionActive = abs(self.v09_center_corr_m) > 0.01
    else:
      lateralPlan.laneCenterCorrectionM = float(self.LP.lane_center_correction_m)
      lateralPlan.laneCenterCorrectionActive = bool(self.LP.lane_center_correction_active)
    # modelPathQuality*, modelNearCurvature, curveTemporalStored and
    # curveTemporalHoldAgeS went with the curve lane fallback and its readiness
    # monitor; the schema keeps the fields, which now read as their defaults.
    # Diagnostic-only curve readings: how strongly a curve was detected and how
    # far lane blending was trusted.
    lateralPlan.curveAssist = float(self.LP.curve_assist_diag)
    lateralPlan.curveRawTargetDProb = float(self.LP.curve_raw_target_d_prob_diag)
    lateralPlan.curveGeometryPlausible = bool(self.LP.curve_geometry_plausible_diag)
    lateralPlan.curveFallbackSource = int(self.LP.curve_fallback_source_diag)

    if self.curve_profile is not None:
      curvatures, times, distances, valid, _diag = self.curve_profile
      lateralPlan.curveProfileBuilt = True
      lateralPlan.curveProfileBand = int(self.curve_profile_band)
      lateralPlan.curveProfileValid = bool(valid)
      lateralPlan.curveProfileCurvatures = curvatures
      lateralPlan.curveProfileTimes = times
      lateralPlan.curveProfileDistances = distances

    pm.send('lateralPlan', plan_send)
