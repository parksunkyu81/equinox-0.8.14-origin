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
from selfdrive.process_diagnostics import append_process_diagnostic

TRAJECTORY_SIZE = 33

# Camera-edge curve extension. A trusted curvature is used to draw a new,
# short virtual arc from the current vehicle pose; no previous path is reused.
# Tight urban corners often happen below 18 km/h, precisely where an inside
# lane line can leave the camera view. Keep this aid available down to
# 12.6 km/h, but only with the independent steering/yaw checks below.
CURVE_EXTENSION_MIN_SPEED_MS = 3.5
CURVE_EXTENSION_MAX_DURATION_S = 1.0
CURVE_EXTENSION_MAX_DISTANCE_M = 6.0
CURVE_EXTENSION_REFERENCE_MAX_AGE_S = 1.20
CURVE_EXTENSION_MIN_REFERENCE_CURVATURE = 0.0035
CURVE_EXTENSION_MIN_ACTUAL_CURVATURE = 0.0015
CURVE_EXTENSION_LANE_RAW_DPROB_TRUSTED = 0.35
CURVE_EXTENSION_LANE_CONTINUITY_MAX_AGE_S = 1.20
CURVE_EXTENSION_BLEND = 0.75
CURVE_EXTENSION_MAX_CORRECTION_NEAR_M = 0.15
CURVE_EXTENSION_MAX_CORRECTION_FAR_M = 1.00
CURVE_EXTENSION_LOG_INTERVAL_S = 0.10
CURVE_EXTENSION_LLK_MAX_AGE_S = 0.25
CURVE_EXTENSION_LLK_MAX_YAW_RATE_RAD_S = 1.0
CURVE_EXTENSION_MIN_YAW_RATE_RAD_S = 1e-4
MODEL_PATH_QUALITY_TRUSTED = 0.75
MODEL_PATH_QUALITY_EDGE_STD_TRUSTED = 1.0
MODEL_PATH_QUALITY_EDGE_STD_LIMIT = 3.0
MODEL_PATH_QUALITY_CURVATURE_ERROR_MIN = 0.010
MODEL_PATH_QUALITY_CURVATURE_JUMP = 0.025
CURVE_EXTENSION_FUSED_MODEL_WEIGHT = 0.65

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

    self._curve_extension_curvature = 0.0
    self._curve_extension_reference_t = -np.inf
    self._curve_extension_start_t = None
    self._curve_extension_last_t = None
    self._curve_extension_distance_m = 0.0
    self.virtual_curve_extension_active = False
    self.virtual_curve_extension_weight = 0.0
    self._curve_extension_log_active = False
    self._curve_extension_last_log_t = -np.inf
    self._curve_extension_virtual_y_5m = 0.0
    self._curve_extension_virtual_y_10m = 0.0
    self._curve_extension_correction_5m = 0.0
    self._curve_extension_correction_10m = 0.0
    self._curve_extension_last_mpc_target_curvature = None
    self._curve_extension_last_block_reason = None
    self._curve_extension_yaw_rate = None
    self._curve_extension_yaw_source = "unavailable"
    self._curve_extension_yaw_age_s = None
    self._curve_extension_fused_curvature = 0.0
    self.model_path_quality = 0.0
    self.model_path_quality_reason = "unavailable"
    self.model_near_curvature = 0.0
    self._previous_model_near_curvature = None
    self._previous_vehicle_curvature = None
    self._previous_model_quality_t = None

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

  def _update_curve_extension_yaw_rate(self, sm, car_state, t):
    """Select a fresh vehicle-frame yaw rate without trusting a silent zero."""
    car_yaw_rate = self._diagnostic_float(getattr(car_state, 'yawRate', None))
    self._curve_extension_yaw_rate = None
    self._curve_extension_yaw_source = "unavailable"
    self._curve_extension_yaw_age_s = None

    # carState is synchronized with the control loop. A real non-zero value is
    # preferred, but several vehicle interfaces leave this optional field at 0.
    if (car_yaw_rate is not None and
        CURVE_EXTENSION_MIN_YAW_RATE_RAD_S <= abs(car_yaw_rate) <=
        CURVE_EXTENSION_LLK_MAX_YAW_RATE_RAD_S):
      self._curve_extension_yaw_rate = car_yaw_rate
      self._curve_extension_yaw_source = "carState"
      self._curve_extension_yaw_age_s = 0.0
      return

    # locationd publishes calibrated angular velocity in the vehicle frame and
    # is already used by the torque controller. Require its service validity,
    # measurement validity, finite uncertainty, and a recent timestamp before
    # using it as the independent turn confirmation.
    try:
      llk = sm['liveLocationKalman']
      llk_age_s = t - float(sm.logMonoTime['liveLocationKalman']) * 1e-9
      angular_velocity = llk.angularVelocityCalibrated
      yaw_rate = float(angular_velocity.value[2])
      yaw_rate_std = float(angular_velocity.std[2])
      llk_valid = bool(sm.valid['liveLocationKalman'])
      measurement_valid = bool(angular_velocity.valid)
      if (llk_valid and measurement_valid and
          0.0 <= llk_age_s <= CURVE_EXTENSION_LLK_MAX_AGE_S and
          np.isfinite(yaw_rate) and np.isfinite(yaw_rate_std) and
          0.0 < yaw_rate_std < 10.0 and
          abs(yaw_rate) <= CURVE_EXTENSION_LLK_MAX_YAW_RATE_RAD_S):
        self._curve_extension_yaw_rate = yaw_rate
        self._curve_extension_yaw_source = "liveLocationKalman"
        self._curve_extension_yaw_age_s = llk_age_s
    except (AttributeError, IndexError, KeyError, TypeError, ValueError):
      # No IMU fallback is safer than accepting a malformed or stale value.
      pass

  def _yaw_curvature(self, car_state):
    if self._curve_extension_yaw_rate is None:
      return None
    v_ego = self._diagnostic_float(car_state.vEgo)
    if v_ego is None:
      return None
    return self._diagnostic_float(
      self._curve_extension_yaw_rate / max(v_ego, 1.0))

  def _vehicle_curve_estimate(self, car_state, measured_curvature):
    """Use only agreeing physical sources as a curvature confirmation."""
    measured = self._diagnostic_float(measured_curvature)
    yaw_curvature = self._yaw_curvature(car_state)
    if measured is None:
      return yaw_curvature
    if yaw_curvature is None:
      return measured
    if measured * yaw_curvature <= 0.0:
      return None
    max_error = max(0.004, 1.25 * max(abs(measured), abs(yaw_curvature)))
    if abs(measured - yaw_curvature) > max_error:
      return None
    return 0.5 * (measured + yaw_curvature)

  def _update_model_path_quality(self, md, car_state, measured_curvature, t):
    """Score whether the model trajectory agrees with independent vehicle motion."""
    reasons = []
    if not self.model_data_valid or self.plan_curv.size < 4:
      self.model_path_quality = 0.0
      self.model_path_quality_reason = "model_path_invalid"
      return

    model_curvature = self._diagnostic_float(np.median(self.plan_curv[1:4]))
    vehicle_curvature = self._vehicle_curve_estimate(car_state, measured_curvature)
    if model_curvature is None:
      self.model_path_quality = 0.0
      self.model_path_quality_reason = "model_curvature_invalid"
      return
    self.model_near_curvature = model_curvature

    lane_score = float(np.clip(
      self.LP.raw_lane_d_prob / CURVE_EXTENSION_LANE_RAW_DPROB_TRUSTED, 0.0, 1.0))
    if lane_score < 1.0:
      reasons.append("lane_low")

    edge_score = 1.0
    road_edge_stds = as_finite_vector(getattr(md, 'roadEdgeStds', []), minimum_size=2)
    if road_edge_stds is not None:
      edge_score = float(np.clip(
        (MODEL_PATH_QUALITY_EDGE_STD_LIMIT - np.max(road_edge_stds[:2])) /
        (MODEL_PATH_QUALITY_EDGE_STD_LIMIT - MODEL_PATH_QUALITY_EDGE_STD_TRUSTED),
        0.0, 1.0))
      if edge_score < 0.5:
        reasons.append("road_edge_uncertain")

    agreement_score = 1.0
    if vehicle_curvature is None:
      agreement_score = 0.0
      reasons.append("vehicle_curve_unconfirmed")
    else:
      max_error = max(
        MODEL_PATH_QUALITY_CURVATURE_ERROR_MIN,
        1.25 * max(abs(model_curvature), abs(vehicle_curvature)))
      if (model_curvature * vehicle_curvature < 0.0 or
          abs(model_curvature - vehicle_curvature) > max_error):
        agreement_score = 0.0
        reasons.append("model_vehicle_disagree")

    temporal_score = 1.0
    if (self._previous_model_quality_t is not None and
        0.0 < t - self._previous_model_quality_t <= 0.30 and
        self._previous_model_near_curvature is not None and
        self._previous_vehicle_curvature is not None and
        vehicle_curvature is not None):
      model_jump = abs(model_curvature - self._previous_model_near_curvature)
      vehicle_jump = abs(vehicle_curvature - self._previous_vehicle_curvature)
      if (model_jump > MODEL_PATH_QUALITY_CURVATURE_JUMP and
          model_jump > vehicle_jump + MODEL_PATH_QUALITY_CURVATURE_ERROR_MIN):
        temporal_score = 0.0
        reasons.append("model_curvature_jump")

    # Road-edge uncertainty measures confidence in the *edge locations*, not
    # confidence in the model trajectory. On tight or bounded curves an edge
    # is routinely occluded or outside the camera view, which made the old
    # min(lane_score, edge_score) gate withdraw torque authority exactly when
    # the independently confirmed path needed it most. Keep edge_score as a
    # diagnostic, but do not let it veto path confidence. Model uncertainty,
    # lane confidence, vehicle agreement, and temporal stability remain hard
    # gates.
    visual_score = lane_score
    self.model_path_quality = float(np.clip(
      min(self.model_confidence, visual_score, agreement_score, temporal_score), 0.0, 1.0))
    self.model_path_quality_reason = "trusted" if not reasons else ",".join(reasons)
    self._previous_model_near_curvature = model_curvature
    self._previous_vehicle_curvature = vehicle_curvature
    self._previous_model_quality_t = t

  def _fused_curve_extension_curvature(self, car_state, measured_curvature):
    """Blend a trusted model curve with agreeing IMU/steering motion only."""
    reference = self._curve_extension_curvature
    vehicle_curvature = self._vehicle_curve_estimate(car_state, measured_curvature)
    if vehicle_curvature is None:
      return reference

    max_error = max(0.004, 1.25 * abs(reference))
    vehicle_curvature = float(np.clip(
      vehicle_curvature, reference - max_error, reference + max_error))
    return float(
      CURVE_EXTENSION_FUSED_MODEL_WEIGHT * reference +
      (1.0 - CURVE_EXTENSION_FUSED_MODEL_WEIGHT) * vehicle_curvature)

  def _vehicle_matches_curve_extension(self, car_state, measured_curvature):
    """Require independent steering-model and yaw evidence of the saved turn."""
    v_ego = float(car_state.vEgo)
    yaw_rate = self._curve_extension_yaw_rate
    if (not np.isfinite(measured_curvature) or yaw_rate is None or
        not np.isfinite(yaw_rate) or
        v_ego < CURVE_EXTENSION_MIN_SPEED_MS):
      return False

    yaw_curvature = yaw_rate / max(v_ego, 1.0)
    reference = self._curve_extension_curvature
    if (abs(reference) < CURVE_EXTENSION_MIN_REFERENCE_CURVATURE or
        abs(measured_curvature) < CURVE_EXTENSION_MIN_ACTUAL_CURVATURE or
        abs(yaw_curvature) < CURVE_EXTENSION_MIN_ACTUAL_CURVATURE or
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

  def _model_curve_exit_reason(self):
    """Trust a confident model immediately when it says the saved turn ended."""
    if self.model_path_quality < MODEL_PATH_QUALITY_TRUSTED:
      return None

    model_curvature = float(np.median(self.plan_curv[1:4]))
    if not np.isfinite(model_curvature):
      return None
    if abs(model_curvature) < CURVE_EXTENSION_MIN_REFERENCE_CURVATURE * 0.5:
      return "model_curve_ended"
    if model_curvature * self._curve_extension_curvature < 0.0:
      return "model_curve_reversed"
    return None

  @staticmethod
  def _virtual_arc_y(path_x, curvature):
    """Return a constant-curvature arc starting at the current vehicle pose."""
    forward_x = np.maximum(np.asarray(path_x, dtype=float), 0.0)
    curvature_x = np.clip(curvature * forward_x, -0.95, 0.95)
    return (1.0 - np.sqrt(1.0 - curvature_x ** 2)) / curvature

  @staticmethod
  def _diagnostic_float(value):
    try:
      value = float(value)
      return value if np.isfinite(value) else None
    except (TypeError, ValueError):
      return None

  def _curve_extension_log_fields(self, car_state, measured_curvature, t,
                                  reason=None, mpc_target_curvature=None):
    v_ego = self._diagnostic_float(car_state.vEgo)
    car_yaw_rate = self._diagnostic_float(getattr(car_state, 'yawRate', None))
    yaw_rate = self._diagnostic_float(self._curve_extension_yaw_rate)
    yaw_curvature = None
    if v_ego is not None and yaw_rate is not None:
      yaw_curvature = yaw_rate / max(v_ego, 1.0)
    model_curvature = self._diagnostic_float(np.median(self.plan_curv[1:4]))
    reference_age = t - self._curve_extension_reference_t
    lane_continuity_age = t - self.LP.curve_extension_lane_center_last_continuous_t
    return {
      "reason": reason,
      "v_ego": v_ego,
      "steering_angle_deg": self._diagnostic_float(car_state.steeringAngleDeg),
      "steering_rate_deg": self._diagnostic_float(car_state.steeringRateDeg),
      "steering_torque": self._diagnostic_float(car_state.steeringTorque),
      "measured_curvature": self._diagnostic_float(measured_curvature),
      "yaw_rate": yaw_rate,
      "car_state_yaw_rate": car_yaw_rate,
      "yaw_rate_source": self._curve_extension_yaw_source,
      "yaw_rate_age_s": self._diagnostic_float(self._curve_extension_yaw_age_s),
      "yaw_curvature": self._diagnostic_float(yaw_curvature),
      "saved_curvature": self._diagnostic_float(self._curve_extension_curvature),
      "model_curvature": model_curvature,
      "mpc_target_curvature": self._diagnostic_float(mpc_target_curvature),
      "model_data_valid": bool(self.model_data_valid),
      "model_confidence": self._diagnostic_float(self.model_confidence),
      "model_path_quality": self._diagnostic_float(self.model_path_quality),
      "model_path_quality_reason": self.model_path_quality_reason,
      "model_near_curvature": self._diagnostic_float(self.model_near_curvature),
      "raw_lane_d_prob": self._diagnostic_float(self.LP.raw_lane_d_prob),
      "lane_continuity_age_s": self._diagnostic_float(lane_continuity_age),
      "reference_age_s": self._diagnostic_float(reference_age),
      "extension_elapsed_s": self._diagnostic_float(
        0.0 if self._curve_extension_start_t is None else t - self._curve_extension_start_t),
      "extension_distance_m": self._diagnostic_float(self._curve_extension_distance_m),
      "extension_weight": self._diagnostic_float(self.virtual_curve_extension_weight),
      "fused_curvature": self._diagnostic_float(self._curve_extension_fused_curvature),
      "virtual_arc_y_5m": self._diagnostic_float(self._curve_extension_virtual_y_5m),
      "virtual_arc_y_10m": self._diagnostic_float(self._curve_extension_virtual_y_10m),
      "path_correction_5m": self._diagnostic_float(self._curve_extension_correction_5m),
      "path_correction_10m": self._diagnostic_float(self._curve_extension_correction_10m),
    }

  def _write_curve_extension_log(self, event_type, car_state,
                                 measured_curvature, t, reason=None,
                                 mpc_target_curvature=None):
    # Diagnostics must never affect planning if storage or serialization fails.
    try:
      append_process_diagnostic(
        event_type,
        **self._curve_extension_log_fields(
          car_state, measured_curvature, t, reason,
          mpc_target_curvature))
    except Exception:
      pass

  def _stop_curve_extension(self, car_state, measured_curvature, t, reason):
    if self._curve_extension_log_active:
      self._write_curve_extension_log(
        "virtual_curve_extension_stopped", car_state, measured_curvature,
        t, reason, self._curve_extension_last_mpc_target_curvature)
    self._curve_extension_log_active = False
    self._curve_extension_start_t = None
    self._curve_extension_last_t = None
    self._curve_extension_distance_m = 0.0
    self._curve_extension_curvature = 0.0
    self._curve_extension_fused_curvature = 0.0

  def _log_curve_extension_block(self, car_state, measured_curvature, t, reason):
    """Record block-reason transitions without writing once per model frame."""
    if reason != self._curve_extension_last_block_reason:
      self._write_curve_extension_log(
        "virtual_curve_extension_blocked", car_state, measured_curvature, t, reason)
      self._curve_extension_last_block_reason = reason

  def _curve_extension_block_reason(self, car_state, controls_active,
                                    measured_curvature, lane_change_active, t):
    if not self.use_lanelines:
      return "lane_lines_disabled"
    if not controls_active:
      return "controls_inactive"
    if car_state.steeringPressed:
      return "driver_steering"
    if lane_change_active:
      return "lane_change_active"
    if not self.model_data_valid:
      return "model_path_invalid"
    if self._curve_extension_yaw_rate is None:
      return "yaw_rate_unavailable"
    if (t - self.LP.curve_extension_lane_center_last_continuous_t >
        CURVE_EXTENSION_LANE_CONTINUITY_MAX_AGE_S):
      return "lane_continuity_expired"
    if t - self._curve_extension_reference_t > CURVE_EXTENSION_REFERENCE_MAX_AGE_S:
      return "saved_curve_expired"
    model_exit_reason = self._model_curve_exit_reason()
    if model_exit_reason is not None:
      return model_exit_reason

    # Extension begins only as confidence is lost. The current trajectory is
    # still finite, but the saved curvature came from a trusted model frame.
    perception_degraded = bool(
      self.model_path_quality < MODEL_PATH_QUALITY_TRUSTED)
    if not perception_degraded:
      return "perception_recovered"
    if not self._vehicle_matches_curve_extension(car_state, measured_curvature):
      return "vehicle_curve_disagrees"
    return None

  def _apply_virtual_curve_extension(self, d_path_xyz, car_state,
                                     controls_active, measured_curvature,
                                     lane_change_active):
    """Draw a bounded virtual arc from the verified entry curvature."""
    self.virtual_curve_extension_active = False
    self.virtual_curve_extension_weight = 0.0
    t = sec_since_boot()
    block_reason = self._curve_extension_block_reason(
      car_state, controls_active, measured_curvature, lane_change_active, t)
    if block_reason is not None:
      self._log_curve_extension_block(
        car_state, measured_curvature, t, block_reason)
      self._stop_curve_extension(
        car_state, measured_curvature, t, block_reason)
      return d_path_xyz

    self._curve_extension_last_block_reason = None

    if self._curve_extension_start_t is None:
      self._curve_extension_start_t = t
      self._curve_extension_last_t = t
      self._curve_extension_distance_m = 0.0
    else:
      frame_dt = max(0.0, t - self._curve_extension_last_t)
      self._curve_extension_distance_m += float(car_state.vEgo) * frame_dt
      self._curve_extension_last_t = t
    elapsed = t - self._curve_extension_start_t
    if (elapsed >= CURVE_EXTENSION_MAX_DURATION_S or
        self._curve_extension_distance_m >= CURVE_EXTENSION_MAX_DISTANCE_M):
      limit_reason = ("distance_limit" if
                      self._curve_extension_distance_m >= CURVE_EXTENSION_MAX_DISTANCE_M
                      else "time_limit")
      self._stop_curve_extension(
        car_state, measured_curvature, t, limit_reason)
      return d_path_xyz

    fused_curvature = self._fused_curve_extension_curvature(
      car_state, measured_curvature)
    if abs(fused_curvature) < CURVE_EXTENSION_MIN_REFERENCE_CURVATURE:
      self._stop_curve_extension(
        car_state, measured_curvature, t, "fused_curve_invalid")
      return d_path_xyz
    self._curve_extension_fused_curvature = fused_curvature
    virtual_path_y = self._virtual_arc_y(d_path_xyz[:, 0], fused_curvature)
    if not np.isfinite(virtual_path_y).all():
      self._stop_curve_extension(
        car_state, measured_curvature, t, "virtual_arc_invalid")
      return d_path_xyz

    time_decay = 1.0 - elapsed / CURVE_EXTENSION_MAX_DURATION_S
    distance_decay = 1.0 - self._curve_extension_distance_m / CURVE_EXTENSION_MAX_DISTANCE_M
    decay = max(0.0, min(time_decay, distance_decay))
    weight = CURVE_EXTENSION_BLEND * decay
    max_correction = np.interp(
      np.abs(d_path_xyz[:, 0]), [0.0, 25.0],
      [CURVE_EXTENSION_MAX_CORRECTION_NEAR_M,
       CURVE_EXTENSION_MAX_CORRECTION_FAR_M])
    correction = np.clip(
      virtual_path_y - d_path_xyz[:, 1], -max_correction, max_correction)
    d_path_xyz[:, 1] += weight * correction
    self._curve_extension_virtual_y_5m = float(np.interp(
      5.0, d_path_xyz[:, 0], virtual_path_y))
    self._curve_extension_virtual_y_10m = float(np.interp(
      10.0, d_path_xyz[:, 0], virtual_path_y))
    self._curve_extension_correction_5m = float(np.interp(
      5.0, d_path_xyz[:, 0], correction))
    self._curve_extension_correction_10m = float(np.interp(
      10.0, d_path_xyz[:, 0], correction))
    self.virtual_curve_extension_active = True
    self.virtual_curve_extension_weight = float(weight)
    if not self._curve_extension_log_active:
      self._curve_extension_log_active = True
      self._curve_extension_last_log_t = t
      self._curve_extension_last_mpc_target_curvature = None
      self._write_curve_extension_log(
        "virtual_curve_extension_started", car_state, measured_curvature, t)
    return d_path_xyz

  def _log_curve_extension_sample(self, car_state, measured_curvature):
    if not self._curve_extension_log_active:
      return
    t = sec_since_boot()
    if t - self._curve_extension_last_log_t < CURVE_EXTENSION_LOG_INTERVAL_S:
      return
    mpc_target_curvature = None
    if self.lat_mpc.x_sol.shape[0] > 1:
      mpc_target_curvature = self.lat_mpc.x_sol[1, 3]
    self._curve_extension_last_mpc_target_curvature = mpc_target_curvature
    self._curve_extension_last_log_t = t
    self._write_curve_extension_log(
      "virtual_curve_extension_sample", car_state, measured_curvature, t,
      mpc_target_curvature=mpc_target_curvature)

  def _refresh_curve_extension_reference(self, car_state, controls_active,
                                         measured_curvature, lane_change_active,
                                         mpc_valid):
    """Save curvature only from a current, independently confirmed turn."""
    if (not self.use_lanelines or not controls_active or
        car_state.steeringPressed or lane_change_active or not mpc_valid or
        not self.model_data_valid or
        self.model_path_quality < MODEL_PATH_QUALITY_TRUSTED or
        not self.LP.curve_extension_lane_center_continuous):
      return

    reference_curvature = float(self.lat_mpc.x_sol[1, 3])
    if not np.isfinite(reference_curvature):
      return

    previous_curvature = self._curve_extension_curvature
    self._curve_extension_curvature = reference_curvature
    if not self._vehicle_matches_curve_extension(car_state, measured_curvature):
      self._curve_extension_curvature = previous_curvature
      return

    self._curve_extension_reference_t = sec_since_boot()
    self._curve_extension_start_t = None
    self._curve_extension_last_t = None
    self._curve_extension_distance_m = 0.0

  def update(self, sm):
    car_state = sm['carState']
    v_ego = car_state.vEgo
    measured_curvature = sm['controlsState'].curvature
    controls_active = bool(sm['controlsState'].active)
    self._update_curve_extension_yaw_rate(sm, car_state, sec_since_boot())

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
    self._update_model_path_quality(
      md, car_state, measured_curvature, sec_since_boot())

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

    d_path_xyz = self._apply_virtual_curve_extension(
      d_path_xyz, car_state, controls_active, measured_curvature,
      lane_change_active)

    # The current model/lane blend normally goes directly to MPC. The only
    # exception is the bounded virtual curve extension above, which is disabled as soon
    # as model confidence, lane continuity, yaw, or curvature disagree.
    d_path_distance = np.linalg.norm(d_path_xyz, axis=1)
    y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], d_path_distance, d_path_xyz[:, 1])
    heading_pts = np.interp(
      v_ego * self.t_idxs[:LAT_MPC_N + 1],
      np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    if (self.virtual_curve_extension_active and
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
    self._log_curve_extension_sample(car_state, measured_curvature)
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

    self._refresh_curve_extension_reference(
      car_state, controls_active, measured_curvature, lane_change_active,
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
    lateralPlan.modelPathQuality = float(self.model_path_quality)
    lateralPlan.modelPathQualityTrusted = bool(
      self.model_path_quality >= MODEL_PATH_QUALITY_TRUSTED)
    lateralPlan.modelNearCurvature = float(self.model_near_curvature)

    pm.send('lateralPlan', plan_send)
