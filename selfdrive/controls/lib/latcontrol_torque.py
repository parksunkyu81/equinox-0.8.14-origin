"""Fixed-parameter torque lateral control based on official openpilot 0.8.14."""

import math

from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


# Keep the official low-speed curvature-error idea, with a gradual reduction
# that avoids the full constant 200 gain through the Equinox's low-speed band.
LOW_SPEED_FACTOR_BP_MS = [0.0, 10.0 / 3.6, 20.0 / 3.6,
                          30.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6]
LOW_SPEED_FACTOR_V = [200.0, 170.0, 120.0, 60.0, 20.0, 0.0]
LAT_ACCEL_FACTOR_MIN = 0.50
LAT_ACCEL_FACTOR_MAX = 5.00
FRICTION_MIN = 0.0
FRICTION_MAX = 0.50
LAT_ACCEL_OFFSET_MAX = 0.03
CONFIDENT_CORNER_BOOST_MAX = 0.18
CONFIDENT_CORNER_CURVATURE_BP = [0.003, 0.030]
CONFIDENT_CORNER_LAT_ACCEL_BP = [0.08, 1.20]
# The GM command path is enabled at MIN_STEER_SPEED (10 km/h).  Previously
# this independent boost gate stayed at zero until 3.5 m/s (12.6 km/h), which
# made a confirmed tight corner receive no additional authority for the first
# 2.6 km/h of valid LKAS operation.  Start modestly at the same threshold and
# ramp smoothly; this does not alter the normal actuator limit.
CONFIDENT_CORNER_SPEED_BP = [0.0, MIN_STEER_SPEED, 4.0, 5.0, 7.0, 22.0, 28.0]
CONFIDENT_CORNER_SPEED_V = [0.0, 0.35, 0.55, 0.70, 0.80, 1.00, 0.0]
CONFIDENT_CORNER_BOOST_RISE = 0.006
CONFIDENT_CORNER_BOOST_FALL = 0.020
CONFIDENT_CORNER_HOLD_FRAMES = 100  # one second at the control rate


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController(
      CP.lateralTuning.torque.kp,
      CP.lateralTuning.torque.ki,
      k_f=CP.lateralTuning.torque.kf,
      pos_limit=self.steer_max,
      neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.steering_angle_deadzone_deg = CP.lateralTuning.torque.steeringAngleDeadzoneDeg
    self.fixed_torque_params = {
      'latAccelFactor': float(clip(
        CP.lateralTuning.torque.latAccelFactor,
        LAT_ACCEL_FACTOR_MIN, LAT_ACCEL_FACTOR_MAX)),
      'latAccelOffset': float(clip(
        CP.lateralTuning.torque.latAccelOffset,
        -LAT_ACCEL_OFFSET_MAX, LAT_ACCEL_OFFSET_MAX)),
      'friction': float(clip(
        CP.lateralTuning.torque.friction,
        FRICTION_MIN, FRICTION_MAX)),
      'totalBucketPoints': 0,
    }
    self.live_torque_params = dict(self.fixed_torque_params)
    self._last_requested_steer = 0.0
    self._last_applied_steer = 0.0
    self.model_path_quality = 0.0
    self.model_path_quality_trusted = False
    self.model_near_curvature = 0.0
    self.confident_corner_boost = 0.0
    self.confident_corner_strength = 0.0
    self.confident_corner_hold_frames = 0
    self.confident_corner_direction = 0

  def reset(self):
    super().reset()
    self.pid.reset()
    self._last_requested_steer = 0.0
    self._last_applied_steer = 0.0
    self.confident_corner_boost = 0.0
    self.confident_corner_strength = 0.0
    self.confident_corner_hold_frames = 0
    self.confident_corner_direction = 0

  def update_live_torque_params(self, latAccelFactor, latAccelOffset,
                                friction, totalBucketPoints=0):
    # The live learner does not override the ntune-controlled torque settings.
    del latAccelFactor, latAccelOffset, friction, totalBucketPoints
    self.live_torque_params = dict(self.fixed_torque_params)

  def update_ntune_torque_params(self, latAccelFactor, friction):
    """Apply the current ntune request to the parameters used for torque control."""
    self.fixed_torque_params['latAccelFactor'] = float(clip(
      float(latAccelFactor), LAT_ACCEL_FACTOR_MIN, LAT_ACCEL_FACTOR_MAX))
    self.fixed_torque_params['friction'] = float(clip(
      float(friction), FRICTION_MIN, FRICTION_MAX))
    self.live_torque_params = dict(self.fixed_torque_params)

  def get_fixed_torque_params(self):
    return dict(self.fixed_torque_params)

  def set_path_stability(self, active, range_m=0.0, flips=0):
    # Compatibility hook. Official-style torque control is not modified by a
    # custom path-state machine.
    del active, range_m, flips

  def set_model_path_quality(self, quality, trusted, model_near_curvature):
    """Receive the planner's camera/IMU/vehicle-motion quality decision."""
    try:
      quality = float(quality)
      model_near_curvature = float(model_near_curvature)
    except (TypeError, ValueError):
      quality = 0.0
      model_near_curvature = 0.0
    self.model_path_quality = float(clip(quality, 0.0, 1.0))
    self.model_path_quality_trusted = bool(trusted and self.model_path_quality >= 0.75)
    self.model_near_curvature = model_near_curvature if math.isfinite(
      model_near_curvature) else 0.0

  def _update_confident_corner_boost(self, active, CS, desired_curvature,
                                     actual_curvature, steer_limited):
    """Add torque authority only for a camera/IMU-confirmed same-direction turn."""
    desired_curvature = float(desired_curvature)
    actual_curvature = float(actual_curvature)
    direction = 1 if desired_curvature > 0.0 else (-1 if desired_curvature < 0.0 else 0)
    model_direction_ok = (
      abs(self.model_near_curvature) < 0.003 or
      self.model_near_curvature * desired_curvature > 0.0)
    vehicle_direction_ok = (
      abs(actual_curvature) < 0.003 or
      actual_curvature * desired_curvature > 0.0)
    context_safe = bool(
      active and self.model_path_quality_trusted and model_direction_ok and
      vehicle_direction_ok and not CS.steeringPressed and not steer_limited and
      CS.vEgo >= MIN_STEER_SPEED)

    if not context_safe:
      # Any perception, driver, or actuator-limit issue returns immediately to
      # the learned base torque. Do not carry extra authority across a fault.
      self.confident_corner_boost = 0.0
      self.confident_corner_strength = 0.0
      self.confident_corner_hold_frames = 0
      self.confident_corner_direction = 0
      return 0.0

    desired_lat_accel = abs(desired_curvature) * CS.vEgo ** 2
    curvature_strength = interp(
      abs(desired_curvature), CONFIDENT_CORNER_CURVATURE_BP, [0.0, 1.0])
    lateral_accel_strength = interp(
      desired_lat_accel, CONFIDENT_CORNER_LAT_ACCEL_BP, [0.0, 1.0])
    self.confident_corner_strength = float(clip(
      max(curvature_strength, lateral_accel_strength), 0.0, 1.0))
    speed_gate = interp(
      CS.vEgo, CONFIDENT_CORNER_SPEED_BP, CONFIDENT_CORNER_SPEED_V)
    target = CONFIDENT_CORNER_BOOST_MAX * self.confident_corner_strength * speed_gate

    if direction and self.confident_corner_direction and direction != self.confident_corner_direction:
      self.confident_corner_hold_frames = 0
      self.confident_corner_boost = 0.0
    self.confident_corner_direction = direction
    if target >= 0.02:
      self.confident_corner_hold_frames = CONFIDENT_CORNER_HOLD_FRAMES
    elif self.confident_corner_hold_frames > 0:
      self.confident_corner_hold_frames -= 1
      target = max(target, self.confident_corner_boost)

    if target > self.confident_corner_boost:
      self.confident_corner_boost = min(
        target, self.confident_corner_boost + CONFIDENT_CORNER_BOOST_RISE)
    else:
      self.confident_corner_boost = max(
        target, self.confident_corner_boost - CONFIDENT_CORNER_BOOST_FALL)
    return float(clip(self.confident_corner_boost, 0.0, CONFIDENT_CORNER_BOOST_MAX))

  def get_dynamic_debug_torque_params(self):
    params = self.fixed_torque_params
    effective_lat_accel_factor = float(clip(
      params['latAccelFactor'] / (1.0 + self.confident_corner_boost),
      LAT_ACCEL_FACTOR_MIN, LAT_ACCEL_FACTOR_MAX))
    return {
      'active': bool(self.confident_corner_boost > 1e-4),
      'latAccelFactor': effective_lat_accel_factor,
      'friction': float(params['friction']),
      'blend': float(self.confident_corner_boost / CONFIDENT_CORNER_BOOST_MAX),
      'authorityCeiling': CONFIDENT_CORNER_BOOST_MAX,
      'corner_strength': float(self.confident_corner_strength),
      'directionDamping': False,
      'responseScale': 1.0,
      'responseRatio': 1.0,
      'responseBin': 0,
      'responseStable': False,
      'responseFrozen': not self.model_path_quality_trusted,
      'responseUpdateCount': 0,
      'pathStabilityActive': False,
      'pathWobbleRangeM': 0.0,
      'pathWobbleFlips': 0,
      'modelCurvatureGuardActive': False,
      'modelCurvatureRaw': 0.0,
      'modelCurvatureFiltered': 0.0,
      'modelCurvatureFilterAlpha': 1.0,
      'modelCurvatureDirectionReversal': False,
      'modelSteerDelayCompensation': 0.0,
      'lowSpeedTorqueGuardActive': False,
      'lowSpeedTorqueGuardState': 0,
      'lowSpeedTorqueRawSteer': float(self._last_requested_steer),
      'lowSpeedTorqueGuardedSteer': float(self._last_requested_steer),
      'lowSpeedTorqueAppliedSteer': float(self._last_applied_steer),
      'lowSpeedTorqueConfirmMs': 0,
      'lowSpeedTorqueReversalCount': 0,
      'lowSpeedTorqueBoostSuppressed': False,
    }

  def update(self, active, CS, VM, params, last_actuators, steer_limited,
             desired_curvature, desired_curvature_rate, llk):
    del desired_curvature_rate
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      angle_steers_des = 0.0
      pid_log.active = False
      self._last_requested_steer = 0.0
      self._last_applied_steer = 0.0
      self.confident_corner_boost = 0.0
      self.confident_corner_strength = 0.0
      self.confident_corner_hold_frames = 0
      self.confident_corner_direction = 0
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(
          math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
          CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(
          math.radians(self.steering_angle_deadzone_deg),
          CS.vEgo, 0.0))
      else:
        actual_curvature_vm = -VM.calc_curvature(
          math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
          CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(
          CS.vEgo, [2.0, 5.0],
          [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0

      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2
      low_speed_factor = interp(
        CS.vEgo, LOW_SPEED_FACTOR_BP_MS, LOW_SPEED_FACTOR_V)
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement

      corner_boost = self._update_confident_corner_boost(
        active, CS, desired_curvature, actual_curvature, steer_limited)
      torque_params = dict(self.fixed_torque_params)
      # Lower latAccelFactor increases the requested steering torque for the
      # same lateral-acceleration error. The actuator's normal steer_max is
      # unchanged, so this cannot exceed the vehicle safety command limit.
      torque_params['latAccelFactor'] = float(clip(
        torque_params['latAccelFactor'] / (1.0 + corner_boost),
        LAT_ACCEL_FACTOR_MIN, LAT_ACCEL_FACTOR_MAX))
      pid_log.error = self.torque_from_lateral_accel(
        lateral_accel_value=error,
        torque_params=torque_params)
      feedforward = self.torque_from_lateral_accel(
        lateral_accel_value=(
          desired_lateral_accel -
          params.roll * ACCELERATION_DUE_TO_GRAVITY),
        torque_params=torque_params,
        lateral_accel_error=error,
        lateral_accel_deadzone=lateral_accel_deadzone,
        friction_compensation=True)
      # Keep the integrator frozen only at very low speed. Steering is already
      # active from about 10 km/h on this GM setup, so freezing I all the way to
      # 18 km/h can leave persistent curvature error in tight low-speed turns.
      freeze_integrator = bool(
        steer_limited or CS.steeringPressed or CS.vEgo < 3.5)
      output_torque = self.pid.update(
        pid_log.error,
        feedforward=feedforward,
        speed=CS.vEgo,
        freeze_integrator=freeze_integrator)

      angle_steers_des = math.degrees(
        VM.get_steer_from_curvature(
          -desired_curvature, CS.vEgo, params.roll)
      ) + params.angleOffsetDeg
      self._last_requested_steer = float(-output_torque)
      try:
        self._last_applied_steer = float(last_actuators.steer)
      except Exception:
        self._last_applied_steer = 0.0

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(
        self.steer_max - abs(output_torque) < 1e-3,
        CS, steer_limited)

    return -output_torque, angle_steers_des, pid_log
