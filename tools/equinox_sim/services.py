#!/usr/bin/env python3
import os
import time

from cereal import log, messaging
from common.realtime import Priority, config_realtime_process, sec_since_boot
from selfdrive.hardware import TICI
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.swaglog import cloudlog
from tools.equinox_sim.command_state import CommandStateReader


SIM_ENV = "EQUINOX_SIMULATOR"
SERVICE_DT = 0.05
MODEL_POINT_COUNT = 33


class EquinoxBenchServices:
  """20 Hz synthetic perception/planning services for the bench simulator.

  Keeping this separate from virtual_panda.py prevents modelV2 and Cap'n Proto
  construction from delaying the timing-critical 100 Hz virtual CAN stream.
  """

  def __init__(self):
    if os.getenv(SIM_ENV) != "1" or os.getenv("SIMULATION") is None or os.getenv("NOBOARD") is None:
      raise RuntimeError(
        "Equinox bench services require EQUINOX_SIMULATOR=1, SIMULATION=1 and NOBOARD=1"
      )

    self.command_reader = CommandStateReader()
    self.pm = messaging.PubMaster([
      "longitudinalPlan", "lateralPlan", "driverMonitoringState", "dynamicFollowData",
      "modelV2", "liveCalibration", "liveLocationKalman", "liveParameters",
      "radarState", "liveTorqueParameters",
    ])
    self.sm = messaging.SubMaster(["carState"], poll=["carState"])
    self.started_at = sec_since_boot()
    self.frame = 0
    self.last_speed_mps = float(os.getenv("EQUINOX_SIM_INITIAL_SPEED_KPH", "80")) / 3.6
    self.last_accel_mps2 = 0.0

    self.model_xs = [float(i) * 4.0 for i in range(MODEL_POINT_COUNT)]
    self.model_zeros = [0.0] * MODEL_POINT_COUNT
    self.model_lane_ys = [[offset] * MODEL_POINT_COUNT
                          for offset in (5.55, 1.85, -1.85, -5.55)]
    self.model_edge_ys = [[offset] * MODEL_POINT_COUNT for offset in (5.8, -5.8)]

  def _target_speed_kph(self):
    return float(self.command_reader.read()["targetSpeedKph"])

  def _vehicle_state(self):
    if self.sm.rcv_frame["carState"] > 0:
      self.last_speed_mps = max(0.0, float(self.sm["carState"].vEgo))
      self.last_accel_mps2 = float(self.sm["carState"].aEgo)
    return self.last_speed_mps, self.last_accel_mps2

  def _publish_plans(self, target_speed_kph, speed_mps):
    target = target_speed_kph / 3.6
    if target > speed_mps:
      first_speed = min(target, speed_mps + 1.0)
      plan_accel = 0.5
    else:
      first_speed = target
      plan_accel = -0.3 if target < speed_mps else 0.0

    speeds = [first_speed + (target - first_speed) * i / (CONTROL_N - 1)
              for i in range(CONTROL_N)]

    long_msg = messaging.new_message("longitudinalPlan")
    long_plan = long_msg.longitudinalPlan
    long_plan.speeds = speeds
    long_plan.accels = [plan_accel] * CONTROL_N
    long_plan.jerks = [0.0] * CONTROL_N
    long_plan.hasLead = False
    long_plan.fcw = False
    long_plan.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.cruise
    long_plan.accelLimitMax = 0.8
    self.pm.send("longitudinalPlan", long_msg)

    lateral_msg = messaging.new_message("lateralPlan")
    lateral = lateral_msg.lateralPlan
    lateral.laneWidth = 3.7
    lateral.lProb = 1.0
    lateral.rProb = 1.0
    lateral.dProb = 1.0
    lateral.dPathPoints = [0.0] * CONTROL_N
    lateral.psis = [0.0] * CONTROL_N
    lateral.curvatures = [0.0] * CONTROL_N
    lateral.curvatureRates = [0.0] * CONTROL_N
    lateral.mpcSolutionValid = True
    lateral.useLaneLines = True
    lateral.laneChangeState = log.LateralPlan.LaneChangeState.off
    lateral.laneChangeDirection = log.LateralPlan.LaneChangeDirection.none
    self.pm.send("lateralPlan", lateral_msg)

    monitoring_msg = messaging.new_message("driverMonitoringState")
    monitoring = monitoring_msg.driverMonitoringState
    monitoring.faceDetected = True
    monitoring.isDistracted = False
    monitoring.awarenessStatus = 1.0
    monitoring.awarenessActive = 1.0
    monitoring.awarenessPassive = 1.0
    monitoring.isActiveMode = True
    self.pm.send("driverMonitoringState", monitoring_msg)

    follow_msg = messaging.new_message("dynamicFollowData")
    follow = follow_msg.dynamicFollowData
    follow.mpcTR = 1.3
    follow.profilePred = 1
    follow.leadCatchupActive = False
    follow.catchupFactor = 0.0
    follow.targetFollowDistance = 0.0
    follow.predictedFollowDistance = 0.0
    follow.baseTR = 1.3
    self.pm.send("dynamicFollowData", follow_msg)

  @staticmethod
  def _set_measurement(measurement, value, std):
    measurement.value = value
    measurement.std = std
    measurement.valid = True

  def _publish_model(self, speed_mps):
    xs = self.model_xs
    zeros = self.model_zeros

    model_msg = messaging.new_message("modelV2")
    model = model_msg.modelV2
    model.frameId = self.frame
    model.frameIdExtra = self.frame
    model.frameAge = 0
    model.frameDropPerc = 0.0
    model.timestampEof = int(sec_since_boot() * 1e9)
    model.modelExecutionTime = 0.0
    model.gpuExecutionTime = 0.0
    model.position.x = xs
    model.position.y = zeros
    model.position.z = zeros
    model.orientation.z = zeros
    model.acceleration.x = zeros

    model.init("laneLines", 4)
    for lane, y_values in zip(model.laneLines, self.model_lane_ys):
      lane.x = xs
      lane.y = y_values
      lane.z = zeros
    model.laneLineProbs = [0.05, 0.95, 0.95, 0.05]
    model.laneLineStds = [0.3, 0.1, 0.1, 0.3]

    model.init("roadEdges", 2)
    for edge, y_values in zip(model.roadEdges, self.model_edge_ys):
      edge.x = xs
      edge.y = y_values
      edge.z = zeros
    model.roadEdgeStds = [1.0, 1.0]

    model.init("leadsV3", 2)
    for lead in model.leadsV3:
      lead.prob = 0.0
      lead.probTime = 0.0
      lead.t = [0.0]
      lead.x = [100.0]
      lead.xStd = [1.0]
      lead.y = [0.0]
      lead.yStd = [1.0]
      lead.v = [speed_mps]
      lead.vStd = [1.0]
      lead.a = [0.0]
      lead.aStd = [1.0]
    model.meta.engagedProb = 1.0
    model.meta.desirePrediction = []
    model.meta.desireState = []
    model.meta.hardBrakePredicted = False
    self.pm.send("modelV2", model_msg)

  def _publish_localization(self, speed_mps, accel_mps2):
    location_msg = messaging.new_message("liveLocationKalman")
    location = location_msg.liveLocationKalman
    location.status = log.LiveLocationKalman.Status.valid
    location.inputsOK = True
    location.posenetOK = True
    location.gpsOK = True
    location.sensorsOK = True
    location.deviceStable = True
    location.excessiveResets = False
    location.timeSinceReset = max(0.0, sec_since_boot() - self.started_at)
    self._set_measurement(location.orientationNED, [0.0, 0.0, 0.0], [0.01, 0.01, 0.01])
    self._set_measurement(location.calibratedOrientationNED, [0.0, 0.0, 0.0], [0.01, 0.01, 0.01])
    self._set_measurement(location.angularVelocityCalibrated, [0.0, 0.0, 0.0], [0.01, 0.01, 0.01])
    self._set_measurement(location.velocityCalibrated,
                          [speed_mps, 0.0, 0.0], [0.1, 0.1, 0.1])
    self._set_measurement(location.accelerationCalibrated,
                          [accel_mps2, 0.0, 0.0], [0.1, 0.1, 0.1])
    self.pm.send("liveLocationKalman", location_msg)

    params_msg = messaging.new_message("liveParameters")
    live_params = params_msg.liveParameters
    live_params.valid = True
    live_params.sensorValid = True
    live_params.posenetValid = True
    live_params.stiffnessFactor = 1.0
    live_params.steerRatio = 16.8
    live_params.angleOffsetDeg = 0.0
    live_params.angleOffsetAverageDeg = 0.0
    live_params.angleOffsetFastStd = 0.0
    live_params.angleOffsetAverageStd = 0.0
    live_params.stiffnessFactorStd = 0.0
    live_params.steerRatioStd = 0.0
    live_params.yawRate = 0.0
    live_params.roll = 0.0
    live_params.posenetSpeed = speed_mps
    self.pm.send("liveParameters", params_msg)

    radar_msg = messaging.new_message("radarState")
    radar_msg.radarState.leadOne.status = False
    radar_msg.radarState.leadTwo.status = False
    radar_msg.radarState.radarErrors = []
    radar_msg.radarState.cumLagMs = 0.0
    self.pm.send("radarState", radar_msg)

  def _publish_slow_state(self):
    calibration_msg = messaging.new_message("liveCalibration")
    calibration = calibration_msg.liveCalibration
    calibration.calStatus = 1
    calibration.calPerc = 100
    calibration.validBlocks = 5
    calibration.rpyCalib = [0.0, 0.0, 0.0]
    calibration.rpyCalibSpread = [0.0, 0.0, 0.0]
    self.pm.send("liveCalibration", calibration_msg)

    torque_msg = messaging.new_message("liveTorqueParameters")
    torque = torque_msg.liveTorqueParameters
    torque.liveValid = False
    torque.useParams = False
    torque.version = 0
    torque.latAccelFactorFiltered = 0.0
    torque.latAccelOffsetFiltered = 0.0
    torque.frictionCoefficientFiltered = 0.0
    torque.totalBucketPoints = 0.0
    torque.points = []
    torque.bucketPoints = "Equinox bench simulator"
    self.pm.send("liveTorqueParameters", torque_msg)

  def run(self):
    config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)
    cloudlog.warning("Starting Equinox synthetic bench services at 20 Hz")
    next_frame_time = sec_since_boot()
    while True:
      self.sm.update(0)
      speed_mps, accel_mps2 = self._vehicle_state()
      self._publish_plans(self._target_speed_kph(), speed_mps)
      self._publish_model(speed_mps)
      self._publish_localization(speed_mps, accel_mps2)
      if self.frame % 5 == 0:
        self._publish_slow_state()
      self.frame += 1
      next_frame_time += SERVICE_DT
      now = sec_since_boot()
      if now < next_frame_time:
        time.sleep(next_frame_time - now)
      elif now - next_frame_time > SERVICE_DT:
        next_frame_time = now


def main():
  EquinoxBenchServices().run()


if __name__ == "__main__":
  main()
