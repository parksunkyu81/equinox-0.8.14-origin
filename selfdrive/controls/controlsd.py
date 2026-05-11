 #!/usr/bin/env python3
import os
import math
import numpy as np
from numbers import Number
from math import sqrt

from cereal import car, log
from common.numpy_fast import clip, interp, mean
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from common.conversions import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from selfdrive.controls.lib.longcontrol import LongControl
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.hardware import HARDWARE, TICI, EON
from selfdrive.manager.process_config import managed_processes

from selfdrive.ntune import ntune_common_get, ntune_common_enabled, ntune_scc_get, ntune_torque_get
from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, road_speed_limiter_get_active, \
  get_road_speed_limiter
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, CONTROL_N
from selfdrive.car.gm.values import SLOW_ON_CURVES, MIN_CURVE_SPEED
#from decimal import Decimal
from selfdrive.controls.lib.dynamic_follow.df_manager import dfManager

MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

SOFT_DISABLE_TIME = 3  # seconds
# controlsAllowed mismatch는 CAN/pandaState 수신 타이밍 차이로 순간 발생할 수 있으므로
# 연속 mismatch만 controlsMismatch로 처리한다. 100Hz 기준 10프레임 = 약 100ms.
CONTROLS_ALLOWED_MISMATCH_FRAMES = 10
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = {"rtshield", "uploader", "deleter", "loggerd", "logmessaged", "tombstoned",
                    "logcatd", "proclogd", "clocksd", "updated", "timezoned", "manage_athenad",
                    "statsd", "shutdownd"} | \
                   {k for k, v in managed_processes.items() if not v.enabled}

ACTUATOR_FIELDS = set(car.CarControl.Actuators.schema.fields.keys())

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonEvent = car.CarState.ButtonEvent
SafetyModel = car.CarParams.SafetyModel

IGNORED_SAFETY_MODES = [SafetyModel.silent, SafetyModel.noOutput]
CSID_MAP = {"0": EventName.roadCameraError, "1": EventName.wideRoadCameraError, "2": EventName.driverCameraError}


class Controls:

    def kph_to_clu(self, kph):
        speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
        return int(kph * CV.KPH_TO_MS * speed_conv_to_clu)

    def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
        config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)

        # Setup sockets
        self.pm = pm
        if self.pm is None:
            self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                           'carControl', 'carEvents', 'carParams'])

        self.camera_packets = ["roadCameraState", "driverCameraState"]
        if TICI:
            self.camera_packets.append("wideRoadCameraState")

        params = Params()
        self.joystick_mode = params.get_bool("JoystickDebugMode")
        joystick_packet = ['testJoystick'] if self.joystick_mode else []

        self.sm = sm
        if self.sm is None:
            ignore = ['driverCameraState', 'managerState'] if SIMULATION else None
            self.sm = messaging.SubMaster(
                ['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                 'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman', 'dynamicFollowData',
                 'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters'] + self.camera_packets + joystick_packet,
                ignore_alive=ignore, ignore_avg_freq=['radarState', 'longitudinalPlan'])

        self.df_manager = dfManager()

        self.can_sock = can_sock
        if can_sock is None:
            can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
            self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

        if TICI:
            self.log_sock = messaging.sub_sock('androidLog')

        if CI is None:
            # wait for one pandaState and one CAN packet
            print("Waiting for CAN messages...")
            get_one_can(self.can_sock)

            self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'])
        else:
            self.CI, self.CP = CI, CI.CP


        # read params
        self.is_live_torque = params.get_bool('IsLiveTorque')
        self.is_metric = params.get_bool("IsMetric")
        self.is_ldw_enabled = params.get_bool("IsLdwEnabled")
        openpilot_enabled_toggle = params.get_bool("OpenpilotEnabledToggle")
        passive = params.get_bool("Passive") or not openpilot_enabled_toggle

        # detect sound card presence and ensure successful init
        sounds_available = HARDWARE.get_sound_card_online()

        car_recognized = self.CP.carName != 'mock'

        controller_available = self.CI.CC is not None and not passive and not self.CP.dashcamOnly
        self.read_only = not car_recognized or not controller_available or self.CP.dashcamOnly
        if self.read_only:
            safety_config = car.CarParams.SafetyConfig.new_message()
            safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
            self.CP.safetyConfigs = [safety_config]

        # Write CarParams for radard
        cp_bytes = self.CP.to_bytes()
        params.put("CarParams", cp_bytes)
        put_nonblocking("CarParamsCache", cp_bytes)

        self.CC = car.CarControl.new_message()
        self.AM = AlertManager()
        self.events = Events()

        self.LoC = LongControl(self.CP)
        self.VM = VehicleModel(self.CP)

        if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            self.LaC = LatControlAngle(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'pid':
            self.LaC = LatControlPID(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'indi':
            self.LaC = LatControlINDI(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'lqr':
            self.LaC = LatControlLQR(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'torque':
            self.LaC = LatControlTorque(self.CP, self.CI)


        self.initialized = False
        self.state = State.disabled
        self.enabled = False
        self.active = False
        self.can_rcv_error = False
        self.soft_disable_timer = 0
        self.v_cruise_kph = 255
        self.v_cruise_kph_last = 0
        self.max_speed_clu = 0.
        self.curve_speed_ms = 0.
        self.v_cruise_kph_limit = 0
        self.applyMaxSpeed = 0
        self.roadLimitSpeedActive = 0
        self.roadLimitSpeed = 0
        self.roadLimitSpeedLeftDist = 0

        self.slow_on_curves = Params().get_bool('SccSmootherSlowOnCurves')
        self.stop_accel_boost = Params().get_bool('StopAccelBoost')

        self.min_set_speed_clu = self.kph_to_clu(MIN_SET_SPEED_KPH)
        self.max_set_speed_clu = self.kph_to_clu(MAX_SET_SPEED_KPH)

        self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
        self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

        self.slowing_down = False
        self.slowing_down_alert = False
        self.slowing_down_sound_alert = False
        self.active_cam = False
        self.over_speed_limit = False

        # scc smoother
        self.is_cruise_enabled = False
        self.applyMaxSpeed = 0

        self.mismatch_counter = 0
        self.last_safety_mismatch_log_frame = -1000000
        self.last_controls_allowed_mismatch_log_frame = -1000000
        self.cruise_mismatch_counter = 0
        self.can_rcv_error_counter = 0
        self.last_blinker_frame = 0
        self.distance_traveled = 0
        self.last_functional_fan_frame = 0
        self.events_prev = []
        self.current_alert_types = [ET.PERMANENT]
        self.logged_comm_issue = False
        self.button_timers = {ButtonEvent.Type.decelCruise: 0, ButtonEvent.Type.accelCruise: 0}
        self.last_actuators = car.CarControl.Actuators.new_message()

        self.steer_limited = False
        self.desired_curvature = 0.0
        self.desired_curvature_rate = 0.0

        # scc smoother
        self.is_cruise_enabled = False
        self.applyMaxSpeed = 0
        self.fused_accel = 0.
        self.lead_drel = 0.

        self.left_lane_visible = False
        self.right_lane_visible = False

        self.wide_camera = TICI and params.get_bool('EnableWideCamera')
        self.disable_op_fcw = params.get_bool('DisableOpFcw')

        self.limited_lead = False

        # TODO: no longer necessary, aside from process replay
        self.sm['liveParameters'].valid = True

        # Live torque
        self.torque_latAccelFactor = 0.
        self.torque_latAccelOffset = 0.
        self.torque_friction = 0.
        self.totalBucketPoints = 0.

        self.startup_event = get_startup_event(car_recognized, controller_available, len(self.CP.carFw) > 0)

        if not sounds_available:
            self.events.add(EventName.soundsUnavailable, static=True)
        if not car_recognized:
            self.events.add(EventName.carUnrecognized, static=True)
            if len(self.CP.carFw) > 0:
                set_offroad_alert("Offroad_CarUnrecognized", True)
            else:
                set_offroad_alert("Offroad_NoFirmware", True)
        elif self.read_only:
            self.events.add(EventName.dashcamMode, static=True)
        elif self.joystick_mode:
            self.events.add(EventName.joystickDebug, static=True)
            self.startup_event = None

        # NDA
        #if self.slowing_down_sound_alert:
        #    self.slowing_down_sound_alert = False
        #    self.events.add(EventName.slowingDownSpeedSound, static=True)
        #elif self.slowing_down_alert:
        #    self.events.add(EventName.slowingDownSpeed, static=True)

        # controlsd is driven by can recv, expected at 100Hz
        self.rk = Ratekeeper(100, print_delay_threshold=None)
        self.prof = Profiler(False)  # off by default

    def reset(self):
        self.max_speed_clu = 0.
        self.curve_speed_ms = 0.
        self.slowing_down = False
        self.slowing_down_alert = False
        self.slowing_down_sound_alert = False

    def get_lead(self, sm):
        radar = sm['radarState']
        if radar.leadOne.status:
            return radar.leadOne
        return None

    def get_long_lead_safe_speed(self, sm, CS, vEgo):
        if CS.adaptiveCruise:
            lead = self.get_lead(sm)
            if lead is not None:
                # d : 비전 거리
                d = lead.dRel
                if 0. < d < -lead.vRel * 20. and lead.vRel < -1.:
                    t = d / lead.vRel
                    accel = -(lead.vRel / t) * self.speed_conv_to_clu
                    accel *= 1.2

                    if accel < 0.:
                        target_speed = vEgo + accel
                        target_speed = max(target_speed, self.kph_to_clu(10))
                        return target_speed

                elif 0. < d < -lead.vRel * 25. and lead.vRel < -1.:
                    t = d / lead.vRel
                    accel = -(lead.vRel / t) * self.speed_conv_to_clu
                    accel *= 1.2

                    if accel < 0.:
                        target_speed = vEgo + accel
                        target_speed = max(target_speed, self.kph_to_clu(20))
                        return target_speed

                elif 0. < d < -lead.vRel * 30. and lead.vRel < -1.:
                    t = d / lead.vRel
                    accel = -(lead.vRel / t) * self.speed_conv_to_clu
                    accel *= 1.2

                    if accel < 0.:
                        target_speed = vEgo + accel
                        target_speed = max(target_speed, self.kph_to_clu(30))
                        return target_speed

        return 0

    def cal_curve_speed(self, sm, v_ego, frame):

        lateralPlan = sm['lateralPlan']
        if len(lateralPlan.curvatures) == CONTROL_N:
            curv = lateralPlan.curvatures[-1]
            a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
            v_curvature = sqrt(a_y_max / max(abs(curv), 1e-4))
            model_speed = v_curvature * 0.85 * ntune_scc_get("sccCurvatureFactor")

            if model_speed < v_ego:
                self.curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
            else:
                self.curve_speed_ms = 255.

            if np.isnan(self.curve_speed_ms):
                self.curve_speed_ms = 255.
        else:
            self.curve_speed_ms = 255.


    # [크루즈 MAX 속도 설정] #
    def cal_max_speed(self, frame: int, vEgo, sm, CS):

        road_speed_limiter = get_road_speed_limiter()

        apply_limit_speed, road_limit_speed, left_dist, first_started, max_speed_log = \
            road_speed_limiter_get_max_speed(vEgo, self.is_metric)

        # print("apply_limit_speed : ", apply_limit_speed)
        # print("road_limit_speed : ", road_limit_speed)
        # print("left_dist : ", left_dist)
        # print("first_started : ", first_started)
        # print("max_speed_log : ", max_speed_log)

        curv_limit = 0
        self.cal_curve_speed(sm, vEgo, frame)
        if self.slow_on_curves and SLOW_ON_CURVES and self.curve_speed_ms >= MIN_CURVE_SPEED:
            max_speed_clu = min(self.v_cruise_kph * CV.KPH_TO_MS, self.curve_speed_ms) * self.speed_conv_to_clu
            curv_limit = int(max_speed_clu)
        else:
            max_speed_clu = self.kph_to_clu(self.v_cruise_kph)

        if road_speed_limiter.roadLimitSpeed is not None:
            camSpeedFactor = clip(road_speed_limiter.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)
            self.over_speed_limit = road_speed_limiter.roadLimitSpeed.camLimitSpeedLeftDist > 0 and \
                                    0 < road_limit_speed * camSpeedFactor < vEgo + 2
        else:
            self.over_speed_limit = False

        max_speed_log = ""

        if apply_limit_speed >= self.kph_to_clu(V_CRUISE_MIN):       # 크루즈 최저 속도보다 큰 경우 설정

            # 크루즈 초기 설정 속도 (PSK)
            # controls.v_cruise_kph : 크루즈 설정 속도
            if first_started:
                self.max_speed_clu = vEgo
                # self.max_speed_clu = self.v_cruise_kph

            max_speed_clu = min(max_speed_clu, apply_limit_speed)

            # if self.v_cruise_kph > apply_limit_speed:
            if vEgo > apply_limit_speed:
                if not self.slowing_down_alert and not self.slowing_down:
                    self.slowing_down_sound_alert = True
                    self.slowing_down = True
                self.slowing_down_alert = True
            else:
                self.slowing_down_alert = False
        else:
            self.slowing_down_alert = False
            self.slowing_down = False

        lead_speed = self.get_long_lead_safe_speed(sm, CS, vEgo)
        if self.safe_distance_speed and lead_speed >= self.min_set_speed_clu:
            if lead_speed < max_speed_clu:
                max_speed_clu = min(max_speed_clu, lead_speed)
                if not self.limited_lead:
                    self.max_speed_clu = vEgo + 3.
                    self.limited_lead = True
        else:
          self.limited_lead = False


        self.update_max_speed(int(max_speed_clu + 0.5), CS,
                              curv_limit != 0 and curv_limit == int(max_speed_clu))
        # print("update_max_speed() value : ", self.max_speed_clu)

        return road_limit_speed, left_dist, max_speed_log

    def update_max_speed(self, max_speed, CS, limited_curv):
        if not CS.cruiseState.enabled or self.max_speed_clu <= 0:
            self.max_speed_clu = max_speed
        else:
            kp = 0.02 if limited_curv else 0.01
            error = max_speed - self.max_speed_clu
            self.max_speed_clu = self.max_speed_clu + error * kp

    def update_events(self, CS):
        """Compute carEvents from carState"""

        self.events.clear()

        # Add startup event
        if self.startup_event is not None:
            self.events.add(self.startup_event)
            self.startup_event = None

        # Don't add any more events if not initialized
        if not self.initialized:
            self.events.add(EventName.controlsInitializing)
            return

        self.events.add_from_msg(CS.events)
        self.events.add_from_msg(self.sm['driverMonitoringState'].events)

        # Create events for battery, temperature, disk space, and memory
        #if EON and (self.sm['peripheralState'].pandaType != PandaType.uno) and \
        #        self.sm['deviceState'].batteryPercent < 1 and self.sm['deviceState'].chargingError:
            # at zero percent battery, while discharging, OP should not allowed
        #    self.events.add(EventName.lowBattery)
        #if EON and (self.sm['peripheralState'].pandaType != PandaType.uno) and \
        #        self.sm['deviceState'].batteryPercent < 30:
            # at zero percent battery, while discharging, OP should not allowed
        #    self.events.add(EventName.lowBattery)
        if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
            self.events.add(EventName.overheat)
        if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
            # under 7% of space free no enable allowed
            self.events.add(EventName.outOfSpace)
        # TODO: make tici threshold the same
        if self.sm['deviceState'].memoryUsagePercent > (90 if TICI else 65) and not SIMULATION:
            self.events.add(EventName.lowMemory)

        # TODO: enable this once loggerd CPU usage is more reasonable
        cpus = list(self.sm['deviceState'].cpuUsagePercent)[:(-1 if EON else None)]
        if max(cpus, default=0) > 95 and not SIMULATION:
          self.events.add(EventName.highCpuUsage)

        # Alert if fan isn't spinning for 5 seconds
        if self.sm['peripheralState'].pandaType in (PandaType.uno, PandaType.dos):
            if self.sm['peripheralState'].fanSpeedRpm == 0 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
                if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 5.0:
                    self.events.add(EventName.fanMalfunction)
            else:
                self.last_functional_fan_frame = self.sm.frame

        # Handle calibration status
        cal_status = self.sm['liveCalibration'].calStatus
        if cal_status != Calibration.CALIBRATED:
            if cal_status == Calibration.UNCALIBRATED:
                self.events.add(EventName.calibrationIncomplete)
            else:
                self.events.add(EventName.calibrationInvalid)

        # Handle lane change
        if self.sm['lateralPlan'].laneChangeState == LaneChangeState.preLaneChange:
            direction = self.sm['lateralPlan'].laneChangeDirection
            if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
                    (CS.rightBlindspot and direction == LaneChangeDirection.right):
                self.events.add(EventName.laneChangeBlocked)
            elif self.sm['lateralPlan'].autoLaneChangeEnabled and self.sm['lateralPlan'].autoLaneChangeTimer > 0:
                self.events.add(EventName.autoLaneChange)
            else:
                if direction == LaneChangeDirection.left:
                    self.events.add(EventName.preLaneChangeLeft)
                else:
                    self.events.add(EventName.preLaneChangeRight)
        elif self.sm['lateralPlan'].laneChangeState in (LaneChangeState.laneChangeStarting,
                                                        LaneChangeState.laneChangeFinishing):
            self.events.add(EventName.laneChange)

        #if not CS.canValid:
        #    self.events.add(EventName.canError)

        # Panda safety 설정 불일치는 즉시 controlsMismatch로 처리한다.
        # 단, pandaStates 자체가 invalid/stale이면 아래 usbError/commIssue 경로에서 처리한다.
        if self.sm.valid["pandaStates"]:
            for i, pandaState in enumerate(self.sm['pandaStates']):
                # All pandas must match the list of safetyConfigs,
                # and if outside this list, must be silent or noOutput.
                if i < len(self.CP.safetyConfigs):
                    expected_safety = self.CP.safetyConfigs[i]
                    safety_mismatch = pandaState.safetyModel != expected_safety.safetyModel or \
                                      pandaState.safetyParam != expected_safety.safetyParam or \
                                      pandaState.alternativeExperience != self.CP.alternativeExperience

                    if safety_mismatch and (self.sm.frame - self.last_safety_mismatch_log_frame) > int(1. / DT_CTRL):
                        cloudlog.warning(
                            "controlsMismatch safety mismatch: "
                            f"idx={i} "
                            f"pandaModel={pandaState.safetyModel} expectedModel={expected_safety.safetyModel} "
                            f"pandaParam={pandaState.safetyParam} expectedParam={expected_safety.safetyParam} "
                            f"pandaAltExp={pandaState.alternativeExperience} expectedAltExp={self.CP.alternativeExperience}"
                        )
                        self.last_safety_mismatch_log_frame = self.sm.frame
                else:
                    safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

                    if safety_mismatch and (self.sm.frame - self.last_safety_mismatch_log_frame) > int(1. / DT_CTRL):
                        cloudlog.warning(
                            "controlsMismatch extra panda not ignored: "
                            f"idx={i} pandaModel={pandaState.safetyModel}"
                        )
                        self.last_safety_mismatch_log_frame = self.sm.frame

                if safety_mismatch:
                    self.events.add(EventName.controlsMismatch)

                if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
                    self.events.add(EventName.relayMalfunction)

        # controlsAllowed mismatch는 순간값 누적이 아니라 연속 프레임만 카운트한다.
        if self.mismatch_counter >= CONTROLS_ALLOWED_MISMATCH_FRAMES:
            if (self.mismatch_counter == CONTROLS_ALLOWED_MISMATCH_FRAMES or
                    (self.sm.frame - self.last_controls_allowed_mismatch_log_frame) > int(1. / DT_CTRL)):
                cloudlog.warning(
                    "controlsMismatch controlsAllowed mismatch: "
                    f"mismatch_counter={self.mismatch_counter} enabled={self.enabled}"
                )
                self.last_controls_allowed_mismatch_log_frame = self.sm.frame
            self.events.add(EventName.controlsMismatch)

        # Check for HW or system issues
        if len(self.sm['radarState'].radarErrors):
            self.events.add(EventName.radarFault)
        elif not self.sm.valid["pandaStates"]:
            self.events.add(EventName.usbError)
        # self.sm.all_checks()
        # self.sm.all_alive_and_valid()
        elif not self.sm.all_checks() or self.can_rcv_error:
            self.events.add(EventName.commIssue)
            if not self.logged_comm_issue:
                invalid = [s for s, valid in self.sm.valid.items() if not valid]
                not_alive = [s for s, alive in self.sm.alive.items() if not alive]
                cloudlog.event("commIssue", invalid=invalid, not_alive=not_alive, can_error=self.can_rcv_error,
                               error=True)
                self.logged_comm_issue = True
        else:
            self.logged_comm_issue = False

        if not self.sm['liveParameters'].valid:
            self.events.add(EventName.vehicleModelInvalid)
        if not self.sm['lateralPlan'].mpcSolutionValid and not (EventName.turningIndicatorOn in self.events.names):
            self.events.add(EventName.plannerError)
        if not self.sm['liveLocationKalman'].sensorsOK and not NOSENSOR:
            if self.sm.frame > 5 / DT_CTRL:  # Give locationd some time to receive all the inputs
                self.events.add(EventName.sensorDataInvalid)
        if not self.sm['liveLocationKalman'].posenetOK:
            self.events.add(EventName.posenetInvalid)
        if not self.sm['liveLocationKalman'].deviceStable:
            self.events.add(EventName.deviceFalling)

        if not REPLAY:
            # Check for mismatch between openpilot and car's PCM
            cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
            self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
            if self.cruise_mismatch_counter > int(3. / DT_CTRL):
                self.events.add(EventName.cruiseMismatch)

        # Check for FCW (브레이크! 추돌위험)
        stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
        model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
        planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
        if not self.disable_op_fcw and (planner_fcw or model_fcw):
            self.events.add(EventName.fcw)

        if TICI:
            for m in messaging.drain_sock(self.log_sock, wait_for_one=False):
                try:
                    msg = m.androidLog.message
                    if any(err in msg for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED")):
                        csid = msg.split("CSID:")[-1].split(" ")[0]
                        evt = CSID_MAP.get(csid, None)
                        if evt is not None:
                            self.events.add(evt)
                except UnicodeDecodeError:
                    pass

        # TODO: fix simulator
        if not SIMULATION:
            # if not NOSENSOR:
            #  if not self.sm['liveLocationKalman'].gpsOK and (self.distance_traveled > 1000):
            #    # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
            #    self.events.add(EventName.noGps)
            if not self.sm.all_alive(self.camera_packets):
                self.events.add(EventName.cameraMalfunction)
            if self.sm['modelV2'].frameDropPerc > 20:
                self.events.add(EventName.modeldLagging)
            if self.sm['liveLocationKalman'].excessiveResets:
                self.events.add(EventName.localizerMalfunction)

            # Check if all manager processes are running
            not_running = {p.name for p in self.sm['managerState'].processes if not p.running}
            if self.sm.rcv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
                self.events.add(EventName.processNotRunning)

        # Only allow engagement with brake pressed when stopped behind another stopped car
        speeds = self.sm['longitudinalPlan'].speeds
        if len(speeds) > 1:
            v_future = speeds[-1]
        else:
            v_future = 100.0
        # if CS.brakePressed and v_future >= self.CP.vEgoStarting \
        #  and self.CP.openpilotLongitudinalControl and CS.vEgo < 0.3:
        #  self.events.add(EventName.noTarget)

        self.df_manager.update()

    def data_sample(self):
        """Receive data from sockets and update carState"""

        # Update carState from CAN
        can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
        CS = self.CI.update(self.CC, can_strs)

        self.sm.update(0)

        if not self.initialized:
            all_valid = CS.canValid and self.sm.all_checks()
            if all_valid or self.sm.frame * DT_CTRL > 3.5 or SIMULATION:
                if not self.read_only:
                    self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])
                self.initialized = True

                if REPLAY and self.sm['pandaStates'][0].controlsAllowed:
                    self.state = State.enabled

                Params().put_bool("ControlsReady", True)

        # Check for CAN timeout
        if not can_strs:
            self.can_rcv_error_counter += 1
            self.can_rcv_error = True
        else:
            self.can_rcv_error = False

        # When the panda and controlsd do not agree on controls_allowed,
        # disengage openpilot after consecutive mismatch frames.
        # 중요: mismatch가 사라지면 반드시 counter를 0으로 되돌려야 한다.
        controls_allowed_mismatch = False
        if self.enabled and self.sm.valid["pandaStates"]:
            controls_allowed_mismatch = any(
                not ps.controlsAllowed
                for ps in self.sm['pandaStates']
                if ps.safetyModel not in IGNORED_SAFETY_MODES
            )

        if not self.enabled or not controls_allowed_mismatch:
            self.mismatch_counter = 0
        else:
            self.mismatch_counter += 1

        self.distance_traveled += CS.vEgo * DT_CTRL

        return CS

    def state_transition(self, CS):
        """Compute conditional state transitions and execute actions on state transitions"""

        self.v_cruise_kph_last = self.v_cruise_kph

        self.CP.pcmCruise = self.CI.CP.pcmCruise

        # if stock cruise is completely disabled, then we can use our own set speed logic
        # if CS.adaptiveCruise:
        # update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled, metric):
        if not self.CP.pcmCruise:
          if CS.adaptiveCruise:
            self.v_cruise_kph = update_v_cruise(self.v_cruise_kph, CS.buttonEvents, self.button_timers, self.enabled, self.is_metric)
        elif CS.cruiseState.enabled:
            self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

        # decrement the soft disable timer at every step, as it's reset on
        # entrance in SOFT_DISABLING state
        self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

        self.current_alert_types = [ET.PERMANENT]

        # ENABLED, PRE ENABLING, SOFT DISABLING
        if self.state != State.disabled:
            # user and immediate disable always have priority in a non-disabled state
            if self.events.any(ET.USER_DISABLE):
                self.state = State.disabled
                self.current_alert_types.append(ET.USER_DISABLE)

            elif self.events.any(ET.IMMEDIATE_DISABLE):
                self.state = State.disabled
                self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

            else:
                # ENABLED
                if self.state == State.enabled:
                    if self.events.any(ET.SOFT_DISABLE):
                        self.state = State.softDisabling
                        self.soft_disable_timer = int(0.5 / DT_CTRL)
                        self.current_alert_types.append(ET.SOFT_DISABLE)

                # SOFT DISABLING
                elif self.state == State.softDisabling:
                    if not self.events.any(ET.SOFT_DISABLE):
                        # no more soft disabling condition, so go back to ENABLED
                        self.state = State.enabled

                    elif self.soft_disable_timer > 0:
                        self.current_alert_types.append(ET.SOFT_DISABLE)

                    elif self.soft_disable_timer <= 0:
                        self.state = State.disabled

                # PRE ENABLING
                elif self.state == State.preEnabled:
                    if not self.events.any(ET.PRE_ENABLE):
                        self.state = State.enabled
                    else:
                        self.current_alert_types.append(ET.PRE_ENABLE)

        # DISABLED
        elif self.state == State.disabled:
            if self.events.any(ET.ENABLE):
                if self.events.any(ET.NO_ENTRY):
                    self.current_alert_types.append(ET.NO_ENTRY)

                else:
                    if self.events.any(ET.PRE_ENABLE):
                        self.state = State.preEnabled
                    else:
                        self.state = State.enabled
                    self.current_alert_types.append(ET.ENABLE)
                    if not self.CP.pcmCruise:
                        self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)

        # Check if actuators are enabled
        self.active = self.state == State.enabled or self.state == State.softDisabling
        if self.active:
            self.current_alert_types.append(ET.WARNING)

        # Check if openpilot is engaged
        self.enabled = self.active or self.state == State.preEnabled

    def state_control(self, CS):
        """Given the state, this function returns an actuators packet"""

        # Update VehicleModel
        params = self.sm['liveParameters']
        x = max(params.stiffnessFactor, 0.1)
        # sr = max(params.steerRatio, 0.1)

        if ntune_common_enabled('useLiveSteerRatio'):
            sr = max(params.steerRatio, 0.1)
        else:
            sr = max(ntune_common_get('steerRatio'), 0.1)

        self.VM.update_params(x, sr)

        # Update Torque Params
        if self.CP.lateralTuning.which() == 'torque':
            if self.is_live_torque:
                torque_params = self.sm['liveTorqueParameters']

                if (torque_params.latAccelFactorFiltered > 0) and (self.sm.valid['liveTorqueParameters']):
                    self.torque_latAccelFactor = torque_params.latAccelFactorFiltered
                    self.torque_latAccelOffset = torque_params.latAccelOffsetFiltered
                    self.torque_friction = torque_params.frictionCoefficientFiltered
                    self.totalBucketPoints = torque_params.totalBucketPoints

                    self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered,
                                                       torque_params.latAccelOffsetFiltered,
                                                       torque_params.frictionCoefficientFiltered)

            else:
                self.torque_latAccelFactor = ntune_torque_get('latAccelFactor')  # LAT_ACCEL_FACTOR
                self.torque_friction = ntune_torque_get('friction')  # FRICTION
                self.torque_latAccelOffset = 1
                self.LaC.update_live_torque_params(self.torque_latAccelFactor, self.torque_latAccelOffset,
                                                   self.torque_friction)


        lat_plan = self.sm['lateralPlan']
        long_plan = self.sm['longitudinalPlan']

        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        # Check which actuators can be enabled
        CC.latActive = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                       CS.vEgo > self.CP.minSteerSpeed and not CS.standstill \
                       and abs(CS.steeringAngleDeg) < self.CP.maxSteeringAngleDeg
        CC.longActive = self.active and not self.events.any(ET.OVERRIDE) and self.CP.openpilotLongitudinalControl

        actuators = CC.actuators
        actuators.longControlState = self.LoC.long_control_state

        #actuators = car.CarControl.Actuators.new_message()
        #actuators.longControlState = self.LoC.long_control_state

        if CS.leftBlinker or CS.rightBlinker:
            self.last_blinker_frame = self.sm.frame

        # State specific actions

        if not self.active:
            self.LaC.reset()
            self.LoC.reset(v_pid=CS.vEgo)

        if not CS.cruiseState.enabled:
            self.LoC.reset(v_pid=CS.vEgo)

        if not self.joystick_mode:
            # accel PID loop
            pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_kph * CV.KPH_TO_MS)
            t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL

            actuators.accel = self.LoC.update(self.active, CS, long_plan, pid_accel_limits, t_since_plan)

            # Steering PID loop and lateral MPC
            # lat_active = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
            #             CS.vEgo > self.CP.minSteerSpeed and not CS.standstill \
            #             and abs(CS.steeringAngleDeg) < self.CP.maxSteeringAngleDeg

            self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,
                                                                                   lat_plan.psis,
                                                                                   lat_plan.curvatures,
                                                                                   lat_plan.curvatureRates)
            actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, params,
                                                                                   self.last_actuators,
                                                                                   self.steer_limited,
                                                                                   self.desired_curvature,
                                                                                   self.desired_curvature_rate,
                                                                                   self.sm['liveLocationKalman'])
        else:
            lac_log = log.ControlsState.LateralDebugState.new_message()
            if self.sm.rcv_frame['testJoystick'] > 0 and self.active:
                actuators.accel = 4.0 * clip(self.sm['testJoystick'].axes[0], -1, 1)

                steer = clip(self.sm['testJoystick'].axes[1], -1, 1)
                # max angle is 45 for angle-based cars (최대 각도 45도)
                actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.

                lac_log.active = True
                lac_log.steeringAngleDeg = CS.steeringAngleDeg
                lac_log.output = steer
                lac_log.saturated = abs(steer) >= 0.9

        # Send a "steering required alert" if saturation count has reached the limit (조향 제어 초과)
        if lac_log.active and lac_log.saturated and not CS.steeringPressed:
            dpath_points = lat_plan.dPathPoints
            if len(dpath_points):
                # Check if we deviated from the path
                # TODO use desired vs actual curvature
                left_deviation = actuators.steer > 0 and dpath_points[0] < -0.20
                right_deviation = actuators.steer < 0 and dpath_points[0] > 0.20

                if left_deviation or right_deviation:
                    self.events.add(EventName.steerSaturated)

        # Ensure no NaNs/Infs
        for p in ACTUATOR_FIELDS:
            attr = getattr(actuators, p)
            if not isinstance(attr, Number):
                continue

            if not math.isfinite(attr):
                cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
                setattr(actuators, p, 0.0)

        return actuators, lac_log

    def update_button_timers(self, buttonEvents):
        # increment timer for buttons still pressed
        for k in self.button_timers:
            if self.button_timers[k] > 0:
                self.button_timers[k] += 1

        for b in buttonEvents:
            if b.type.raw in self.button_timers:
                self.button_timers[b.type.raw] = 1 if b.pressed else 0

    def publish_logs(self, CS, start_time, actuators, lac_log):
        """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        CC.active = self.active
        CC.actuators = actuators

        orientation_value = self.sm['liveLocationKalman'].orientationNED.value
        if len(orientation_value) > 2:
            CC.roll = orientation_value[0]
            CC.pitch = orientation_value[1]

        CC.cruiseControl.cancel = self.CP.pcmCruise and not self.enabled and CS.cruiseState.enabled
        if self.joystick_mode and self.sm.rcv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
            CC.cruiseControl.cancel = True

        hudControl = CC.hudControl
        hudControl.setSpeed = float(self.v_cruise_kph * CV.KPH_TO_MS)
        hudControl.speedVisible = self.enabled
        hudControl.lanesVisible = self.enabled
        hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead

        right_lane_visible = self.sm['lateralPlan'].rProb > 0.5
        left_lane_visible = self.sm['lateralPlan'].lProb > 0.5

        totalCameraOffset = self.sm['lateralPlan'].totalCameraOffset

        if self.sm.frame % 100 == 0:
            self.right_lane_visible = right_lane_visible
            self.left_lane_visible = left_lane_visible

        hudControl.rightLaneVisible = self.right_lane_visible
        hudControl.leftLaneVisible = self.left_lane_visible

        recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
        ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                      and not self.active and self.sm['liveCalibration'].calStatus == Calibration.CALIBRATED

        model_v2 = self.sm['modelV2']
        desire_prediction = model_v2.meta.desirePrediction
        if len(desire_prediction) and ldw_allowed:
            right_lane_visible = self.sm['lateralPlan'].rProb > 0.5
            left_lane_visible = self.sm['lateralPlan'].lProb > 0.5
            l_lane_change_prob = desire_prediction[Desire.laneChangeLeft - 1]
            r_lane_change_prob = desire_prediction[Desire.laneChangeRight - 1]

            lane_lines = model_v2.laneLines
            l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
            r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

            hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
            hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

        if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
            self.events.add(EventName.ldw)

        clear_event_types = set()
        if ET.WARNING not in self.current_alert_types:
            clear_event_types.add(ET.WARNING)
        if self.enabled:
            clear_event_types.add(ET.NO_ENTRY)

        alerts = self.events.create_alerts(self.current_alert_types,
                                           [self.CP, self.sm, self.is_metric, self.soft_disable_timer])
        self.AM.add_many(self.sm.frame, alerts)
        current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)
        if current_alert:
            hudControl.visualAlert = current_alert.visual_alert

        if not self.read_only and self.initialized:
            # send car controls over can
            self.last_actuators, can_sends = self.CI.apply(CC, self)
            self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
            CC.actuatorsOutput = self.last_actuators
            self.steer_limited = abs(CC.actuators.steer - CC.actuatorsOutput.steer) > 1e-2

        force_decel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                      (self.state == State.softDisabling)

        # Curvature & Steering angle
        params = self.sm['liveParameters']

        steer_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
        curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, params.roll)

        # NDA Add.. (PSK)
        road_limit_speed, left_dist, max_speed_log = self.cal_max_speed(self.sm.frame, CS.vEgo, self.sm, CS)

        # controlsState
        dat = messaging.new_message('controlsState')
        dat.valid = CS.canValid
        controlsState = dat.controlsState
        if current_alert:
            controlsState.alertText1 = current_alert.alert_text_1
            controlsState.alertText2 = current_alert.alert_text_2
            controlsState.alertSize = current_alert.alert_size
            controlsState.alertStatus = current_alert.alert_status
            controlsState.alertBlinkingRate = current_alert.alert_rate
            controlsState.alertType = current_alert.alert_type
            controlsState.alertSound = current_alert.audible_alert

        controlsState.canMonoTimes = list(CS.canMonoTimes)
        controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
        controlsState.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan']
        controlsState.enabled = self.enabled
        controlsState.active = self.active
        controlsState.curvature = curvature
        controlsState.state = self.state
        controlsState.engageable = not self.events.any(ET.NO_ENTRY)
        controlsState.longControlState = self.LoC.long_control_state
        controlsState.vPid = float(self.LoC.v_pid)

        # Cruise SET
        # kph [applyMaxSpeed, cruiseMaxSpeed]
        controlsState.applyMaxSpeed = float(
            clip(self.v_cruise_kph, MIN_SET_SPEED_KPH, self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))
        controlsState.cruiseMaxSpeed = self.v_cruise_kph

        if controlsState.applyMaxSpeed == controlsState.cruiseMaxSpeed:
            controlsState.vCruise = float(controlsState.cruiseMaxSpeed)
        elif controlsState.applyMaxSpeed < controlsState.cruiseMaxSpeed:
            controlsState.vCruise = float(controlsState.applyMaxSpeed)

        controlsState.upAccelCmd = float(self.LoC.pid.p)
        controlsState.uiAccelCmd = float(self.LoC.pid.i)
        controlsState.ufAccelCmd = float(self.LoC.pid.f)
        controlsState.cumLagMs = -self.rk.remaining * 1000.
        controlsState.startMonoTime = int(start_time * 1e9)
        controlsState.forceDecel = bool(force_decel)
        controlsState.canErrorCounter = self.can_rcv_error_counter
        controlsState.angleSteers = steer_angle_without_offset * CV.RAD_TO_DEG

        # NDA
        controlsState.roadLimitSpeedActive = road_speed_limiter_get_active()
        controlsState.roadLimitSpeed = road_limit_speed
        controlsState.roadLimitSpeedLeftDist = left_dist

        controlsState.steerRatio = self.VM.sR
        controlsState.steerActuatorDelay = ntune_common_get('steerActuatorDelay')

        controlsState.sccGasFactor = ntune_scc_get('sccGasFactor')
        controlsState.sccBrakeFactor = ntune_scc_get('sccBrakeFactor')
        controlsState.sccCurvatureFactor = ntune_scc_get('sccCurvatureFactor')

        # Live Torque
        controlsState.latAccelFactor = self.torque_latAccelFactor
        controlsState.latAccelOffset = self.torque_latAccelOffset
        controlsState.friction = self.torque_friction
        controlsState.totalBucketPoints = self.totalBucketPoints

        # Dynamic TR
        #controlsState.cruiseGap = int(Params().get("cruiseGap", encoding="utf8"))
        controlsState.minTR = float(Params().get("minTR", encoding="utf8"))
        #controlsState.dynamicTRMode = int(self.sm['longitudinalPlan'].dynamicTRMode)
        controlsState.dynamicTRMode = Params().get("DynamicTRGap", encoding="utf8")
        controlsState.globalDfMod = float(Params().get("globalDfMod", encoding="utf8"))
        # self.sm['liveTorqueParameters']
        controlsState.dynamicTRValue = float(self.sm['dynamicFollowData'].mpcTR)

        controlsState.totalCameraOffset = totalCameraOffset

        lat_tuning = self.CP.lateralTuning.which()
        if self.joystick_mode:
          controlsState.lateralControlState.debugState = lac_log
        elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
          controlsState.lateralControlState.angleState = lac_log
        elif lat_tuning == 'pid':
          controlsState.lateralControlState.pidState = lac_log
        elif lat_tuning == 'lqr':
          controlsState.lateralControlState.lqrState = lac_log
        elif lat_tuning == 'indi':
          controlsState.lateralControlState.indiState = lac_log
        elif lat_tuning == 'torque':
          controlsState.lateralControlState.torqueState = lac_log

        self.pm.send('controlsState', dat)

        # carState
        car_events = self.events.to_msg()
        cs_send = messaging.new_message('carState')
        cs_send.valid = CS.canValid
        cs_send.carState = CS
        cs_send.carState.events = car_events
        self.pm.send('carState', cs_send)

        # carEvents - logged every second or on change
        if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
            ce_send = messaging.new_message('carEvents', len(self.events))
            ce_send.carEvents = car_events
            self.pm.send('carEvents', ce_send)
        self.events_prev = self.events.names.copy()

        # carParams - logged every 50 seconds (> 1 per segment)
        if (self.sm.frame % int(50. / DT_CTRL) == 0):
            cp_send = messaging.new_message('carParams')
            cp_send.carParams = self.CP
            self.pm.send('carParams', cp_send)

        # carControl
        cc_send = messaging.new_message('carControl')
        cc_send.valid = CS.canValid
        cc_send.carControl = CC
        self.pm.send('carControl', cc_send)

        # copy CarControl to pass to CarInterface on the next iteration
        self.CC = CC

    def step(self):
        start_time = sec_since_boot()
        self.prof.checkpoint("Ratekeeper", ignore=True)

        # Sample data from sockets and get a carState
        CS = self.data_sample()
        self.prof.checkpoint("Sample")

        self.update_events(CS)

        if not self.read_only and self.initialized:
            # Update control state
            self.state_transition(CS)
            self.prof.checkpoint("State transition")

        # Compute actuators (runs PID loops and lateral MPC)
        actuators, lac_log = self.state_control(CS)

        self.prof.checkpoint("State Control")

        # Publish data
        self.publish_logs(CS, start_time, actuators, lac_log)
        self.prof.checkpoint("Sent")

        self.update_button_timers(CS.buttonEvents)

    def controlsd_thread(self):
        while True:
            self.step()
            self.rk.monitor_time()
            self.prof.display()


def main(sm=None, pm=None, logcan=None):
    controls = Controls(sm, pm, logcan)
    controls.controlsd_thread()


if __name__ == "__main__":
    main()
