#!/usr/bin/env python3
import json
import os
import time

from cereal import car, log, messaging
from common.params import Params
from common.realtime import DT_CTRL, sec_since_boot
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car import crc8_pedal
from selfdrive.car.gm.values import CAR, DBC, CanBus, CruiseButtons
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.swaglog import cloudlog
from tools.equinox_sim.vehicle import EquinoxVehicle


SIM_ENV = "EQUINOX_SIMULATOR"
STATUS_DIR = "/tmp/equinox_sim"
STATUS_FILE = os.path.join(STATUS_DIR, "status.json")

PARAM_TARGET_SPEED = "EquinoxSimTargetSpeedKph"
PARAM_IGNITION = "EquinoxSimIgnition"
PARAM_ACCEL_ZERO = "EquinoxSimAccelZero"
PARAM_BRAKE = "EquinoxSimBrakePressed"
PARAM_GAS = "EquinoxSimGasPressed"
PARAM_RESET = "EquinoxSimReset"
PARAM_ENGAGE = "EquinoxSimEngage"


class EquinoxVirtualPanda:
  def __init__(self):
    if os.getenv(SIM_ENV) != "1" or os.getenv("SIMULATION") is None or os.getenv("NOBOARD") is None:
      raise RuntimeError(
        "Equinox simulator requires EQUINOX_SIMULATOR=1, SIMULATION=1 and NOBOARD=1"
      )

    self.params = Params()
    self._put_default(PARAM_TARGET_SPEED, "100")
    self._put_default(PARAM_IGNITION, "1")
    self._put_default(PARAM_ACCEL_ZERO, "0")
    self._put_default(PARAM_BRAKE, "0")
    self._put_default(PARAM_GAS, "0")
    self._put_default(PARAM_RESET, "0")
    self._put_default(PARAM_ENGAGE, "1")

    initial_speed_kph = float(os.getenv("EQUINOX_SIM_INITIAL_SPEED_KPH", "80"))
    self.vehicle = EquinoxVehicle(initial_speed_kph=initial_speed_kph)

    self.pm = messaging.PubMaster([
      "can", "pandaStates", "peripheralState", "longitudinalPlan", "lateralPlan",
      "driverMonitoringState", "dynamicFollowData", "modelV2", "liveCalibration",
      "liveLocationKalman", "liveParameters", "radarState", "liveTorqueParameters",
    ])
    self.sm = messaging.SubMaster(["carControl", "controlsState"])
    self.sendcan_sock = messaging.sub_sock("sendcan")

    dbc_name = DBC[CAR.EQUINOX_NR]["pt"]
    self.packer = CANPacker(dbc_name)
    self.output_parser = CANParser(
      dbc_name,
      [
        ("GAS_COMMAND", "GAS_COMMAND"),
        ("ENABLE", "GAS_COMMAND"),
        ("RollingCounter", "ASCMLKASteeringCmd"),
        ("LKASteeringCmd", "ASCMLKASteeringCmd"),
      ],
      [("GAS_COMMAND", 0), ("ASCMLKASteeringCmd", 0)],
      CanBus.POWERTRAIN,
    )

    self.frame = 0
    self.started_at = sec_since_boot()
    self.pedal_command = 0.0
    self.steer_command = 0.0
    self.loopback_counter = 0
    self.gas_sensor_counter = 0
    self.gas_can_frames = 0
    self.steer_can_frames = 0
    self.button_queue = []
    self.last_engage_attempt_frame = -1000
    self.last_resume_frame = -1000
    self.last_status_frame = -100
    self.last_status_time = self.started_at
    self.measured_loop_hz = 0.0
    self.last_vehicle_update_time = sec_since_boot()
    self.model_point_count = 33
    self.model_xs = [float(i) * 4.0 for i in range(self.model_point_count)]
    self.model_zeros = [0.0] * self.model_point_count
    self.model_lane_ys = [[offset] * self.model_point_count
                          for offset in (5.55, 1.85, -1.85, -5.55)]
    self.model_edge_ys = [[offset] * self.model_point_count for offset in (5.8, -5.8)]

  def _put_default(self, key, value):
    if self.params.get(key) is None:
      self.params.put(key, value)

  def _get_bool(self, key):
    return self.params.get_bool(key)

  def _get_float(self, key, default):
    raw = self.params.get(key, encoding="utf8")
    try:
      return float(raw) if raw is not None else float(default)
    except ValueError:
      return float(default)

  def _get_int(self, key, default):
    raw = self.params.get(key, encoding="utf8")
    try:
      return int(raw) if raw is not None else int(default)
    except ValueError:
      return int(default)

  def _read_commands(self):
    if self._get_bool(PARAM_RESET):
      self.vehicle.reset()
      self.pedal_command = 0.0
      self.steer_command = 0.0
      self.gas_sensor_counter = 0
      self.button_queue.clear()
      self.params.put_bool(PARAM_RESET, False)
      self.params.put_bool(PARAM_ENGAGE, True)

    fault_mode = min(2, max(0, self._get_int(PARAM_ACCEL_ZERO, 0)))
    return {
      "target_speed_kph": min(145.0, max(20.0, self._get_float(PARAM_TARGET_SPEED, 100.0))),
      "ignition": self._get_bool(PARAM_IGNITION),
      "fault_accel_zero": fault_mode in (1, 2),
      "fault_recovery_enabled": fault_mode == 2,
      "brake_pressed": self._get_bool(PARAM_BRAKE),
      "gas_pressed": self._get_bool(PARAM_GAS),
    }

  def _consume_sendcan(self):
    for raw in messaging.drain_sock_raw(self.sendcan_sock):
      updated = self.output_parser.update_string(raw, sendcan=True)
      if 512 in updated:
        gas_values = self.output_parser.vl["GAS_COMMAND"]
        self.pedal_command = max(0.0, float(gas_values["GAS_COMMAND"]) / 255.0) \
          if gas_values["ENABLE"] else 0.0
        self.gas_can_frames += 1
      if 384 in updated:
        steer_values = self.output_parser.vl["ASCMLKASteeringCmd"]
        self.loopback_counter = int(steer_values["RollingCounter"])
        self.steer_command = max(-1.0, min(1.0, float(steer_values["LKASteeringCmd"]) / 300.0))
        self.steer_can_frames += 1

  def _controls_enabled(self):
    return bool(self.sm["controlsState"].enabled) if self.sm.rcv_frame["controlsState"] > 0 else False

  def _update_buttons(self, target_speed_kph, brake_pressed):
    controls_enabled = self._controls_enabled()

    if self._get_bool(PARAM_ENGAGE):
      self.params.put_bool(PARAM_ENGAGE, False)
      self.last_engage_attempt_frame = -1000

    if not controls_enabled and not brake_pressed and not self.button_queue and \
       self.frame - self.last_engage_attempt_frame >= 100:
      self.button_queue.extend([CruiseButtons.DECEL_SET, CruiseButtons.DECEL_SET,
                                CruiseButtons.UNPRESS, CruiseButtons.UNPRESS])
      self.last_engage_attempt_frame = self.frame

    current_v_cruise = float(self.sm["controlsState"].vCruise) \
      if self.sm.rcv_frame["controlsState"] > 0 else 0.0
    if controls_enabled and current_v_cruise < target_speed_kph - 1.0 and \
       not self.button_queue and self.frame - self.last_resume_frame >= 30:
      self.button_queue.extend([CruiseButtons.RES_ACCEL, CruiseButtons.RES_ACCEL,
                                CruiseButtons.UNPRESS, CruiseButtons.UNPRESS])
      self.last_resume_frame = self.frame

    return self.button_queue.pop(0) if self.button_queue else CruiseButtons.UNPRESS

  def _build_can(self, button, brake_pressed, gas_pressed):
    speed_kph = self.vehicle.speed_mps * 3.6
    driver_gas = 0.12 if gas_pressed else 0.0
    steering_torque = self.steer_command * 2.0

    # Keep one CAN packet arriving every 10 ms so controlsd continues to run at
    # 100 Hz. Individual vehicle messages are distributed across their required
    # parser timeouts; packing all 15 messages at 100 Hz overloads EON hardware.
    frames = [
      self.packer.make_can_msg("ECMEngineStatus", CanBus.POWERTRAIN, {
        "CruiseMainOn": 0, "Brake_Pressed": int(brake_pressed),
        "Standstill": int(self.vehicle.speed_mps < 0.01), "EngineRPM": 1500,
      }),
      self.packer.make_can_msg("ASCMSteeringButton", CanBus.POWERTRAIN, {
        "ACCButtons": button, "DistanceButton": 0, "LKAButton": 0,
      }),
    ]

    if self.frame % 2 == 0:  # 50 Hz
      frames.extend([
        self.packer.make_can_msg("PSCMSteeringAngle", CanBus.POWERTRAIN, {
          "SteeringWheelAngle": self.vehicle.steering_angle_deg,
          "SteeringWheelRate": self.vehicle.steering_rate_deg_s,
        }),
        self.packer.make_can_msg("EBCMBrakePedalPosition", CanBus.POWERTRAIN, {
          "BrakePedalPosition": 80 if brake_pressed else 0,
        }),
        self._make_gas_sensor(driver_gas),
        self.packer.make_can_msg("ASCMLKASteeringCmd", CanBus.LOOPBACK, {
          "RollingCounter": self.loopback_counter,
          "LKASteeringCmd": int(round(self.steer_command * 300.0)),
          "LKASteeringCmdActive": int(self._controls_enabled()),
          "LKASteeringCmdChecksum": 0,
        }),
      ])

    if self.frame % 3 == 0:  # about 33 Hz
      frames.append(self.packer.make_can_msg("AcceleratorPedal2", CanBus.POWERTRAIN, {
        "CruiseState": 1, "AcceleratorPedal2": driver_gas * 254.0,
      }))

    if self.frame % 5 == 0:  # 20 Hz
      frames.extend([
        self.packer.make_can_msg("EBCMWheelSpdFront", CanBus.POWERTRAIN, {
          "FLWheelSpd": speed_kph, "FRWheelSpd": speed_kph,
        }),
        self.packer.make_can_msg("EBCMWheelSpdRear", CanBus.POWERTRAIN, {
          "RLWheelSpd": speed_kph, "RRWheelSpd": speed_kph,
        }),
        self.packer.make_can_msg("EPBStatus", CanBus.POWERTRAIN, {"EPBClosed": 0}),
      ])

    if self.frame % 10 == 0:  # 10 Hz
      frames.extend([
        self.packer.make_can_msg("ECMPRDNL2", CanBus.POWERTRAIN, {
          "PRNDL2": 4, "ManualMode": 0, "TransmissionState": 9,
        }),
        self.packer.make_can_msg("PSCMStatus", CanBus.POWERTRAIN, {
          "LKADriverAppldTrq": 0.0,
          "LKATorqueDelivered": steering_torque,
          "LKATorqueDeliveredStatus": 1,
        }),
        self.packer.make_can_msg("BCMDoorBeltStatus", CanBus.POWERTRAIN, {
          "FrontLeftDoor": 0, "FrontRightDoor": 0,
          "RearLeftDoor": 0, "RearRightDoor": 0,
          "LeftSeatBelt": 1, "RightSeatBelt": 1,
        }),
        self.packer.make_can_msg("ESPStatus", CanBus.POWERTRAIN, {"TractionControlOn": 1}),
      ])

    if self.frame % 100 == 0:  # 1 Hz
      frames.append(self.packer.make_can_msg(
        "BCMTurnSignals", CanBus.POWERTRAIN, {"TurnSignals": 0}
      ))

    return frames

  def _make_gas_sensor(self, driver_gas):
    values = {
      "INTERCEPTOR_GAS": driver_gas * 255.0,
      "INTERCEPTOR_GAS2": driver_gas * 255.0,
      "STATE": 0,
      "COUNTER_PEDAL": self.gas_sensor_counter,
      "CHECKSUM_PEDAL": 0,
    }

    # CHECKSUM_PEDAL is not auto-filled by this openpilot version's packer.
    # Match pedal_checksum() in opendbc: CRC-8 (poly 0xD5, init 0xFF)
    # over bytes 0..4, in reverse byte order.
    unsigned_message = self.packer.make_can_msg("GAS_SENSOR", CanBus.POWERTRAIN, values)
    values["CHECKSUM_PEDAL"] = crc8_pedal(unsigned_message[2][:-1])
    message = self.packer.make_can_msg("GAS_SENSOR", CanBus.POWERTRAIN, values)
    self.gas_sensor_counter = (self.gas_sensor_counter + 1) & 0x0F
    return message

  def _publish_panda(self, ignition):
    panda_msg = messaging.new_message("pandaStates", 1)
    panda = panda_msg.pandaStates[0]
    panda.ignitionLine = bool(ignition)
    panda.ignitionCan = bool(ignition)
    panda.controlsAllowed = True
    panda.gasInterceptorDetected = True
    panda.pandaType = log.PandaState.PandaType.uno
    panda.safetyModel = car.CarParams.SafetyModel.gm
    panda.safetyParam = 0
    panda.alternativeExperience = 1
    panda.harnessStatus = log.PandaState.HarnessStatus.normal
    panda.uptime = int(max(0.0, sec_since_boot() - self.started_at))
    self.pm.send("pandaStates", panda_msg)

    peripheral_msg = messaging.new_message("peripheralState")
    peripheral = peripheral_msg.peripheralState
    peripheral.pandaType = log.PandaState.PandaType.uno
    peripheral.voltage = 12500
    peripheral.current = 0
    peripheral.fanSpeedRpm = 3000
    peripheral.usbPowerMode = log.PeripheralState.UsbPowerMode.cdp
    self.pm.send("peripheralState", peripheral_msg)

  def _publish_plans(self, target_speed_kph):
    speed = self.vehicle.speed_mps
    target = target_speed_kph / 3.6
    if target > speed:
      first_speed = min(target, speed + 1.0)
      plan_accel = 0.5
    else:
      first_speed = target
      plan_accel = -0.3 if target < speed else 0.0

    speeds = [first_speed + (target - first_speed) * i / (CONTROL_N - 1)
              for i in range(CONTROL_N)]
    accels = [plan_accel] * CONTROL_N

    long_msg = messaging.new_message("longitudinalPlan")
    long_plan = long_msg.longitudinalPlan
    long_plan.speeds = speeds
    long_plan.accels = accels
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

    # plannerd is disabled in bench mode. Its DynamicFollow instance normally
    # publishes this service, while controlsd still consumes it for the UI and
    # communication-health checks.
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

  def _publish_bench_state(self):
    """Publish light-weight substitutes for the heavy perception/localization stack.

    The real road camera remains owned by camerad and is still drawn by the UI.
    These messages describe a deterministic straight, clear road so the EON can
    exercise controls without running modeld, locationd, radard, or learners.
    """
    xs = self.model_xs
    zeros = self.model_zeros

    model_msg = messaging.new_message("modelV2")
    model = model_msg.modelV2
    model.frameId = self.frame // 5
    model.frameIdExtra = self.frame // 5
    model.frameAge = 0
    model.frameDropPerc = 0.0
    model.timestampEof = int(sec_since_boot() * 1e9)
    model.modelExecutionTime = 0.0
    model.gpuExecutionTime = 0.0

    # Only populate fields consumed by this fork's UI. Keeping unused model
    # tensors empty materially reduces Python/Cap'n Proto work on EON.
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

    # This fork's onroad.cc indexes both leads unconditionally, even when no
    # lead is present. Supply two zero-probability records to keep that safe.
    model.init("leadsV3", 2)
    for lead in model.leadsV3:
      lead.prob = 0.0
      lead.probTime = 0.0
      lead.t = [0.0]
      lead.x = [100.0]
      lead.xStd = [1.0]
      lead.y = [0.0]
      lead.yStd = [1.0]
      lead.v = [self.vehicle.speed_mps]
      lead.vStd = [1.0]
      lead.a = [0.0]
      lead.aStd = [1.0]
    model.meta.engagedProb = 1.0
    model.meta.desirePrediction = []
    model.meta.desireState = []
    model.meta.hardBrakePredicted = False
    self.pm.send("modelV2", model_msg)

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
                          [self.vehicle.speed_mps, 0.0, 0.0], [0.1, 0.1, 0.1])
    self._set_measurement(location.accelerationCalibrated,
                          [self.vehicle.accel_mps2, 0.0, 0.0], [0.1, 0.1, 0.1])
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
    live_params.posenetSpeed = self.vehicle.speed_mps
    self.pm.send("liveParameters", params_msg)

    radar_msg = messaging.new_message("radarState")
    radar = radar_msg.radarState
    radar.leadOne.status = False
    radar.leadTwo.status = False
    radar.radarErrors = []
    radar.cumLagMs = 0.0
    self.pm.send("radarState", radar_msg)

    # Both of these services are specified at 4 Hz (one message per 25 loops).
    if self.frame % 25 == 0:
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

  def _write_status(self, commands):
    controls = self.sm["controlsState"]
    controls_seen = self.sm.rcv_frame["controlsState"] > 0
    status = {
      "mode": "EQUINOX VIRTUAL PANDA",
      "speedKph": round(self.vehicle.speed_mps * 3.6, 2),
      "accelMps2": round(self.vehicle.accel_mps2, 3),
      "targetSpeedKph": round(commands["target_speed_kph"], 1),
      "pedalCommand": round(self.pedal_command, 4),
      "steerCommand": round(self.steer_command, 4),
      "controlsEnabled": bool(controls.enabled) if controls_seen else False,
      "vCruiseKph": round(float(controls.vCruise), 1) if controls_seen else 0.0,
      "faultAccelZero": commands["fault_accel_zero"],
      "faultRecoveryEnabled": commands["fault_recovery_enabled"],
      "brakePressed": commands["brake_pressed"],
      "ignition": commands["ignition"],
      "recoveryActive": bool(controls.pedalForceRecoveryActive) if controls_seen else False,
      "recoveryDuration": round(float(controls.pedalForceRecoveryDuration), 2) if controls_seen else 0.0,
      "recoveryCount": int(controls.pedalForceRecoveryCount) if controls_seen else 0,
      "recoveryRawAccel": round(float(controls.pedalForceRecoveryRawAccel), 3) if controls_seen else 0.0,
      "recoveryForcedAccel": round(float(controls.pedalForceRecoveryAccel), 3) if controls_seen else 0.0,
      "targetErrorMps": round(commands["target_speed_kph"] / 3.6 - self.vehicle.speed_mps, 3),
      "gasCanFrames": self.gas_can_frames,
      "steerCanFrames": self.steer_can_frames,
      "distanceM": round(self.vehicle.distance_m, 1),
      "loopHz": round(self.measured_loop_hz, 1),
    }
    os.makedirs(STATUS_DIR, exist_ok=True)
    temp_file = STATUS_FILE + ".tmp"
    with open(temp_file, "w", encoding="utf8") as status_file:
      json.dump(status, status_file, ensure_ascii=False, indent=2)
    os.replace(temp_file, STATUS_FILE)
    print(
      "EquinoxSim "
      f"v={status['speedKph']:6.2f}km/h target={status['targetSpeedKph']:5.1f} "
      f"pedal={status['pedalCommand']:.4f} enabled={status['controlsEnabled']} "
      f"fault={status['faultAccelZero']} recovery={status['recoveryActive']} "
      f"loop={status['loopHz']:.1f}Hz"
    )

  def run(self):
    cloudlog.warning("Starting Equinox virtual Panda bench simulator (NOBOARD)")

    while True:
      loop_started_at = sec_since_boot()
      self.sm.update(0)
      commands = self._read_commands()
      self._consume_sendcan()

      if commands["ignition"]:
        now = sec_since_boot()
        vehicle_dt = max(0.001, min(0.05, now - self.last_vehicle_update_time))
        self.last_vehicle_update_time = now
        self.vehicle.step(
          vehicle_dt,
          self.pedal_command,
          self.steer_command,
          brake_pressed=commands["brake_pressed"],
          driver_gas_pressed=commands["gas_pressed"],
        )

      button = self._update_buttons(commands["target_speed_kph"], commands["brake_pressed"])
      frames = self._build_can(button, commands["brake_pressed"], commands["gas_pressed"])
      self.pm.send("can", can_list_to_can_capnp(frames))

      if self.frame % 5 == 0:
        self._publish_panda(commands["ignition"])
        self._publish_plans(commands["target_speed_kph"])
        self._publish_bench_state()

      if self.frame - self.last_status_frame >= 100:
        status_time = sec_since_boot()
        status_frames = self.frame - max(0, self.last_status_frame)
        status_elapsed = status_time - self.last_status_time
        if self.last_status_frame >= 0 and status_elapsed > 0.0:
          self.measured_loop_hz = status_frames / status_elapsed
        self._write_status(commands)
        self.last_status_frame = self.frame
        self.last_status_time = status_time

      self.frame += 1
      # Do not run missed frames faster than real time. Ratekeeper's cumulative
      # schedule catches up after startup stalls and can flood EON at 200-300
      # Hz, evicting other messaging readers. This limiter drops missed ticks.
      loop_elapsed = sec_since_boot() - loop_started_at
      if loop_elapsed < DT_CTRL:
        time.sleep(DT_CTRL - loop_elapsed)


def main():
  EquinoxVirtualPanda().run()


if __name__ == "__main__":
  main()
