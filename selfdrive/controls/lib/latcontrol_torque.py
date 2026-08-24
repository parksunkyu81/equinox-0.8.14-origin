"""Fixed-parameter torque lateral control based on official openpilot 0.8.14."""

import math

from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


# Official 0.8.14 value: a fixed low-speed curvature-error gain.
LOW_SPEED_FACTOR = 200
LAT_ACCEL_FACTOR_MIN = 0.50
LAT_ACCEL_FACTOR_MAX = 5.00
FRICTION_MIN = 0.0
FRICTION_MAX = 0.50
LAT_ACCEL_OFFSET_MAX = 0.03


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

  def reset(self):
    super().reset()
    self.pid.reset()
    self._last_requested_steer = 0.0
    self._last_applied_steer = 0.0

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

  def get_dynamic_debug_torque_params(self):
    params = self.fixed_torque_params
    return {
      'active': False,
      'latAccelFactor': float(params['latAccelFactor']),
      'friction': float(params['friction']),
      'blend': 0.0,
      'authorityCeiling': 0.0,
      'corner_strength': 0.0,
      'directionDamping': False,
      'responseScale': 1.0,
      'responseRatio': 1.0,
      'responseBin': 0,
      'responseStable': False,
      'responseFrozen': True,
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
      setpoint = desired_lateral_accel + LOW_SPEED_FACTOR * desired_curvature
      measurement = actual_lateral_accel + LOW_SPEED_FACTOR * actual_curvature
      error = setpoint - measurement

      torque_params = self.fixed_torque_params
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
      # Official 0.8.14 also freezes below vEgo 5, but this branch is only
      # reached above MIN_STEER_SPEED, so a speed term here can never fire.
      # Freeze only for reasons that still apply while steering is live.
      freeze_integrator = bool(steer_limited or CS.steeringPressed)
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
