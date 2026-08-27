 #!/usr/bin/env python3
import os
import math
import numpy as np
from collections import deque
from numbers import Number

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
from selfdrive.controls.lib.control_activation import apply_control_activation
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.hardware import HARDWARE, TICI, EON
from selfdrive.manager.process_config import managed_processes

from selfdrive.ntune import ntune_common_get, ntune_common_enabled, ntune_scc_get, ntune_torque_get
from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, road_speed_limiter_get_active, \
  get_road_speed_limiter
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, V_CRUISE_ENABLE_MIN, CONTROL_N
from selfdrive.car.gm.values import MIN_CURVE_SPEED
#from decimal import Decimal
from selfdrive.controls.lib.dynamic_follow.df_manager import dfManager
from selfdrive.controls.lib.stop_accel_boost import (
  STOP_ACCEL_BOOST_FACTOR, StopAccelBoostLatch,
  boost_floor_context_allowed, speed_limit_decel_requested,
)
from selfdrive.controls.lib.curve_speed_limiter import (
  CurveSpeedLimiter, CURVE_SPEED_DISABLED, build_v0813_model_curve_profile, calculate_curve_speed,
)
from selfdrive.controls.lib.curve_pedal_coordinator import CurvePedalCoordinator
from selfdrive.controls.lib.predictive_coasting import PredictiveCoastingCoordinator
from selfdrive.controls.lib.natural_decel_learner import NaturalDecelLearner, select_road_pitch
from selfdrive.controls.lib.panda_safety import panda_safety_config_matches, update_panda_safety_readiness
from selfdrive.controls.lib.process_health import (
  controlsd_communication_ok, expected_not_running_processes,
  panda_power_down_in_progress, update_process_not_running_state,
)
from selfdrive.controls.lib.driving_style_learner import DrivingStyleLearner
from selfdrive.controls.lib.comma_pedal_profile import (
  CommaPedalProfileController, combine_comma_pedal_gain,
  normalize_comma_pedal_profile,
)
from selfdrive.controls.lib.pedal_force_recovery import (
  PEDAL_FORCE_RECOVERY_PEDAL_FLOOR, RECOVERY_MODE_HARD_ZERO,
  RECOVERY_MODE_LEAD_COAST_ASSIST, RECOVERY_MODE_LEAD_LOSS_CRUISE,
  RECOVERY_MODE_MOVING_GAP_CATCHUP, RECOVERY_MODE_NONE, LeadCoastAssist,
  LeadLossCruiseAssist, MovingGapCatchupAssist, PedalForceRecovery,
  recovery_speed_demand,
)
from selfdrive.process_diagnostics import append_controls_mismatch_diagnostic, append_process_diagnostic

MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

SOFT_DISABLE_TIME = 3  # seconds
# Gate engagement until the final Panda safety configuration is stable. After engagement,
# debounce only the short controlsAllowed message skew; safety configuration changes stay immediate.
PANDA_SAFETY_MATCH_FRAMES = 10  # 100 ms at 100 Hz
CONTROLS_ALLOWED_MISMATCH_FRAMES = 25  # 250 ms at 100 Hz
CONTROLS_MISMATCH_HISTORY_SECONDS = 5
CONTROLS_MISMATCH_SAMPLE_FRAMES = max(1, int(0.05 / DT_CTRL))  # 20 Hz
PROCESS_NOT_RUNNING_CONSECUTIVE_UPDATES = 3
COMM_ISSUE_CONSECUTIVE_FRAMES = max(1, int(0.30 / DT_CTRL))
COMMA_PEDAL_PARAM_REFRESH_FRAMES = max(1, int(0.2 / DT_CTRL))
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1

# Lane-confidence watchdog (EventName.laneConfidenceLow). Tuned on
# 2026-08-26--12-34-51, 11.5 min engaged, 48 driver interventions:
#   dProb < 0.25, 8 s sustained, 30 s cooldown -> 2 alerts, both immediately
#   before the driver took over. Loosening the sustain is what makes it noisy,
#   not the threshold: at 3 s the same 0.25 fires 24 times (~37/hour) because
#   brief dropouts are normal -- 33 of 72 last under 0.5 s, median 0.99 s.
# Sample is one night city route, so expect to retune on daytime/highway data.
LANE_CONF_DPROB = 0.25
LANE_CONF_SUSTAIN_S = 8.0
LANE_CONF_COOLDOWN_S = 30.0
LANE_CONF_MIN_SPEED = 5 * CV.KPH_TO_MS

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = {"rtshield", "uploader", "deleter", "loggerd", "logmessaged", "tombstoned",
                    "logcatd", "proclogd", "clocksd", "updated", "timezoned", "manage_athenad",
                    "statsd", "shutdownd", "recoverylogger"} | \
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
        self.params = params
        self.joystick_mode = params.get_bool("JoystickDebugMode")
        joystick_packet = ['testJoystick'] if self.joystick_mode else []

        self.sm = sm
        if self.sm is None:
            ignore = ['driverCameraState', 'managerState'] if SIMULATION else None
            # These derived EON services can briefly miss their nominal average
            # rate under load while remaining alive and valid. Their liveness and
            # validity are still checked; only the noisy average-rate gate is
            # relaxed to avoid false "device process" alerts.
            ignore_avg_freq = ['radarState', 'longitudinalPlan']
            if EON:
                ignore_avg_freq += ['driverMonitoringState', 'lateralPlan',
                                    'dynamicFollowData']
            self.sm = messaging.SubMaster(
                ['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                 'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman', 'dynamicFollowData',
                 'managerState', 'liveParameters', 'radarState'] + self.camera_packets + joystick_packet,
                ignore_alive=ignore, ignore_avg_freq=ignore_avg_freq)

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
        # Use fixed CarParams torque tuning. The live learner may continue to
        # publish diagnostics, but it must not modify steering authority.
        self.is_live_torque = False
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
        self.pedal_force_recovery = PedalForceRecovery(DT_CTRL)
        self.lead_coast_assist = LeadCoastAssist(DT_CTRL)
        self.lead_loss_cruise_assist = LeadLossCruiseAssist(DT_CTRL)
        self.moving_gap_catchup_assist = MovingGapCatchupAssist(DT_CTRL)
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
        self.curve_speed_limiter = CurveSpeedLimiter()
        self._model_curve_control_enabled = False
        self.curve_pedal_coordinator = CurvePedalCoordinator(DT_CTRL)
        self.predictive_coasting = PredictiveCoastingCoordinator(DT_CTRL)
        self.predictive_coast_pedal_scale = 1.0
        self.natural_decel_learner = NaturalDecelLearner(params=params)
        self.natural_decel_status = self.natural_decel_learner.status(0.0)
        self.natural_decel_pitch_deg = 0.0
        self.natural_decel_pitch_valid = False
        self.natural_decel_pitch_fallback = False
        self.natural_decel_pitch_source = "invalid"
        self.predictive_brake_alert_enabled = params.get_bool("PredictiveBrakeAlert")
        self.speed_limit_coast_active = False
        self.speed_limit_coast_target_ms = 0.0
        self.speed_limit_coast_distance_m = math.inf
        self.curve_plan_speed_ms = CURVE_SPEED_DISABLED
        self.curve_pedal_raw_accel = 0.0
        self.curve_pedal_final_accel = 0.0
        self.is_curv_driving = False
        self.curv_speed = 0.0
        self.v_cruise_kph_limit = 0
        self.applyMaxSpeed = 0
        self.roadLimitSpeedActive = 0
        self.roadLimitSpeed = 0
        self.roadLimitSpeedLeftDist = 0

        self.slow_on_curves = Params().get_bool('SccSmootherSlowOnCurves')
        self.min_set_speed_clu = self.kph_to_clu(MIN_SET_SPEED_KPH)
        self.max_set_speed_clu = self.kph_to_clu(MAX_SET_SPEED_KPH)

        self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
        self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

        self.slowing_down = False
        self.slowing_down_alert = False
        self.slowing_down_sound_alert = False
        # Lane-confidence watchdog, see EventName.laneConfidenceLow.
        self.lane_conf_low_s = 0.0
        self.lane_conf_alert_t = -LANE_CONF_COOLDOWN_S
        self.active_cam = False
        self.over_speed_limit = False

        # scc smoother
        self.is_cruise_enabled = False
        self.applyMaxSpeed = 0

        self.mismatch_counter = 0
        self.panda_safety_ready = False
        self.panda_safety_match_counter = 0
        self.last_safety_mismatch_log_frame = -1000000
        self.last_controls_allowed_mismatch_log_frame = -1000000
        self.controls_mismatch_history = deque(maxlen=int(
            CONTROLS_MISMATCH_HISTORY_SECONDS / (CONTROLS_MISMATCH_SAMPLE_FRAMES * DT_CTRL)))
        self.controls_mismatch_last_sample_frame = -CONTROLS_MISMATCH_SAMPLE_FRAMES
        self.controls_mismatch_active = False
        self.process_not_running_counter = 0
        self.process_not_running_candidates = set()
        self.process_not_running_active = False
        self.process_not_running_logged_names = ()
        self.cruise_mismatch_counter = 0
        self.can_rcv_error_counter = 0
        self.last_blinker_frame = 0
        self.distance_traveled = 0
        self.last_functional_fan_frame = 0
        self.events_prev = []

        self.current_alert_types = [ET.PERMANENT]
        self.logged_comm_issue = False
        self.comm_issue_counter = 0
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
        self.stop_accel_boost_latch = StopAccelBoostLatch(DT_CTRL)
        self.stop_accel_boost_active = False
        self.driving_style_learner = DrivingStyleLearner(params=params)
        self.driving_style_status = self.driving_style_learner.status(0.0)
        self.driving_style_gain = 1.0
        self.comma_pedal_profile = normalize_comma_pedal_profile(
          params.get("CommaPedalResistance", encoding="utf8") or 'mid')
        self.comma_pedal_profile_controller = CommaPedalProfileController(
          self.comma_pedal_profile)
        self.comma_pedal_profile_gain = 1.0
        self.comma_pedal_learned_gain = 1.0
        self.comma_pedal_effective_gain = 1.0
        self.comma_pedal_profile_changing = False
        self.comma_pedal_raw_command = 0.0
        self.comma_pedal_styled_command = 0.0
        self.comma_pedal_final_command = 0.0

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

    @staticmethod
    def _diagnostic_enum_value(value):
        """Serialize Python and pycapnp enum values without affecting control."""
        try:
            return int(value)
        except (TypeError, ValueError):
            # pycapnp's _DynamicEnum is not necessarily int-convertible.
            # Keep diagnostics best-effort: a snapshot must never stop
            # controlsd merely because an enum representation differs.
            raw_value = getattr(value, "raw", None)
            if raw_value is not None:
                try:
                    return int(raw_value)
                except (TypeError, ValueError):
                    pass
            return str(value)

    def _controls_mismatch_panda_snapshot(self, panda_state, index):
        return {
            "index": index,
            "safety_model": self._diagnostic_enum_value(panda_state.safetyModel),
            "safety_param": int(panda_state.safetyParam),
            "alternative_experience": int(panda_state.alternativeExperience),
            "controls_allowed": bool(panda_state.controlsAllowed),
            "heartbeat_lost": bool(panda_state.heartbeatLost),
            "uptime": int(panda_state.uptime),
            "fault_status": self._diagnostic_enum_value(panda_state.faultStatus),
            "faults": [self._diagnostic_enum_value(fault) for fault in panda_state.faults],
            "can_rx_errs": int(panda_state.canRxErrs),
            "can_send_errs": int(panda_state.canSendErrs),
            "can_fwd_errs": int(panda_state.canFwdErrs),
            "blocked_cnt": int(panda_state.blockedCnt),
            "power_save_enabled": bool(panda_state.powerSaveEnabled),
            "interrupt_load": float(panda_state.interruptLoad),
        }

    def _controls_mismatch_service_snapshot(self, service):
        rcv_time = self.sm.rcv_time[service]
        return {
            "valid": bool(self.sm.valid[service]),
            "alive": bool(self.sm.alive[service]),
            "frequency_ok": bool(self.sm.freq_ok[service]),
            "updated": bool(self.sm.updated[service]),
            "receive_age_s": max(0.0, sec_since_boot() - rcv_time) if rcv_time else None,
            "receive_frame": int(self.sm.rcv_frame[service]),
            "log_mono_time": int(self.sm.logMonoTime[service]),
        }

    def _controls_mismatch_snapshot(self, panda_safety_matches, include_manager_processes=False):
        panda_states = self.sm['pandaStates']
        snapshot = {
            "frame": int(self.sm.frame),
            "started": bool(self.sm['deviceState'].started),
            "enabled": bool(self.enabled),
            "active": bool(self.active),
            "controls_ready": bool(self.params.get_bool("ControlsReady")),
            "panda_safety_ready": bool(self.panda_safety_ready),
            "panda_safety_match_counter": int(self.panda_safety_match_counter),
            "panda_safety_matches": bool(panda_safety_matches),
            "controls_allowed_mismatch_counter": int(self.mismatch_counter),
            "can_receive_error": bool(self.can_rcv_error),
            "can_receive_error_counter": int(self.can_rcv_error_counter),
            "charging_disabled": bool(self.sm['deviceState'].chargingDisabled),
            "panda_count": len(panda_states),
            "peripheral_usb_power_mode": self._diagnostic_enum_value(
                self.sm['peripheralState'].usbPowerMode),
            "car": {
                "fingerprint": str(self.CP.carFingerprint),
                "name": str(self.CP.carName),
                "alternative_experience": int(self.CP.alternativeExperience),
                "safety_configs": [{
                    "index": i,
                    "safety_model": self._diagnostic_enum_value(config.safetyModel),
                    "safety_param": int(config.safetyParam),
                } for i, config in enumerate(self.CP.safetyConfigs)],
            },
            "pandas": [self._controls_mismatch_panda_snapshot(panda_state, i)
                       for i, panda_state in enumerate(panda_states)],
            "services": {service: self._controls_mismatch_service_snapshot(service)
                         for service in ("pandaStates", "deviceState", "managerState")},
        }
        if include_manager_processes:
            snapshot["manager_processes"] = [{
                "name": process.name,
                "running": bool(process.running),
                "should_be_running": bool(process.shouldBeRunning),
                "pid": int(process.pid),
                "exit_code": int(process.exitCode),
            } for process in self.sm['managerState'].processes]
        return snapshot

    def _record_controls_mismatch_snapshot(self, snapshot):
        if self.sm.frame - self.controls_mismatch_last_sample_frame < CONTROLS_MISMATCH_SAMPLE_FRAMES:
            return
        self.controls_mismatch_history.append(snapshot)
        self.controls_mismatch_last_sample_frame = self.sm.frame

    def _record_controls_mismatch_episode(self, reasons, snapshot):
        if not reasons:
            self.controls_mismatch_active = False
            return
        if self.controls_mismatch_active:
            return

        self.controls_mismatch_active = True
        append_controls_mismatch_diagnostic(
            "controls_mismatch",
            reasons=list(reasons),
            trigger=snapshot,
            history=list(self.controls_mismatch_history),
        )

    def reset(self):
        self.max_speed_clu = 0.
        self.curve_speed_ms = 0.
        self.curve_speed_limiter.reset()
        self._model_curve_control_enabled = False
        self.curve_pedal_coordinator.reset()
        self.predictive_coasting.reset()
        self.pedal_force_recovery.reset()
        self.lead_coast_assist.reset()
        self.lead_loss_cruise_assist.reset()
        self.moving_gap_catchup_assist.reset()
        self.predictive_coast_pedal_scale = 1.0
        self.speed_limit_coast_active = False
        self.speed_limit_coast_target_ms = 0.0
        self.speed_limit_coast_distance_m = math.inf
        self.curve_plan_speed_ms = CURVE_SPEED_DISABLED
        self.curve_pedal_raw_accel = 0.0
        self.curve_pedal_final_accel = 0.0
        self.slowing_down = False
        self.slowing_down_alert = False
        self.slowing_down_sound_alert = False

    def get_lead(self, sm):
        radar = sm['radarState']
        if radar.leadOne.status:
            return radar.leadOne
        return None

    def pedal_force_recovery_eligible(self, CS, long_plan, t_since_plan):
        """True only when a zero accel request contradicts a fresh speed demand.

        This gate intentionally permits both cruise and lead plan sources: the
        planner's present and future speed trajectories must independently ask
        for acceleration. Driver input and every explicit safety/curve state
        cancel the recovery immediately in PedalForceRecovery.update().
        """
        speeds = long_plan.speeds
        full_plan = len(speeds) == CONTROL_N
        speed_error = float(self.LoC.v_pid - CS.vEgo)
        future_speed_error = float(speeds[-1] - CS.vEgo) if full_plan else 0.0
        driver_aware = float(self.sm['driverMonitoringState'].awarenessStatus) >= 0.0
        plan_valid = bool(self.sm.valid['longitudinalPlan'])
        can_valid = bool(getattr(CS, 'canValid', True))
        speed_limit_decel = speed_limit_decel_requested(
          bool(getattr(self, 'speed_limit_coast_active', False)),
          float(getattr(self, 'speed_limit_coast_target_ms', 0.0)),
          CS.vEgo)

        return bool(
          self.CP.enableGasInterceptor and self.active and self.state == State.enabled and
          CS.adaptiveCruise and
          self.LoC.long_control_state == car.CarControl.Actuators.LongControlState.pid and
          not CS.brakePressed and not CS.gasPressed and not CS.standstill and
          CS.vEgo > V_CRUISE_ENABLE_MIN * CV.KPH_TO_MS and
          driver_aware and not self.is_curv_driving and
          not bool(getattr(getattr(self, 'curve_pedal_coordinator', None),
                           'engaged', False)) and
          not speed_limit_decel and
          not long_plan.fcw and
          can_valid and plan_valid and 0.0 <= t_since_plan <= 0.25 and full_plan and
          recovery_speed_demand(speed_error, future_speed_error))

    def lead_coast_assist_base_safe(self, CS, long_plan, t_since_plan, radar_valid):
        """Common safety gate for low-demand lead-follow pedal assistance."""
        speed_limit_decel = speed_limit_decel_requested(
          self.speed_limit_coast_active, self.speed_limit_coast_target_ms,
          CS.vEgo)
        return bool(
          self.CP.enableGasInterceptor and self.active and self.state == State.enabled and
          CS.adaptiveCruise and
          self.LoC.long_control_state == car.CarControl.Actuators.LongControlState.pid and
          not CS.brakePressed and not CS.gasPressed and not CS.standstill and
          CS.vEgo > V_CRUISE_ENABLE_MIN * CV.KPH_TO_MS and
          float(self.sm['driverMonitoringState'].awarenessStatus) >= 0.0 and
          not self.is_curv_driving and not self.curve_pedal_coordinator.engaged and
          not speed_limit_decel and not long_plan.fcw and
          not self.stop_accel_boost_active and bool(getattr(CS, 'canValid', True)) and
          radar_valid and self.sm.valid['longitudinalPlan'] and
          0.0 <= t_since_plan <= 0.25 and len(long_plan.speeds) == CONTROL_N and
          str(long_plan.longitudinalPlanSource) == 'lead0')

    def lead_loss_cruise_assist_base_safe(self, CS, long_plan, t_since_plan,
                                          radar_valid):
        """Safety gate for a confirmed lead0 -> cruise transition ramp."""
        speed_limit_decel = speed_limit_decel_requested(
          self.speed_limit_coast_active, self.speed_limit_coast_target_ms,
          CS.vEgo)
        return bool(
          self.CP.enableGasInterceptor and self.active and self.state == State.enabled and
          CS.adaptiveCruise and
          self.LoC.long_control_state == car.CarControl.Actuators.LongControlState.pid and
          not CS.brakePressed and not CS.gasPressed and not CS.standstill and
          CS.vEgo > V_CRUISE_ENABLE_MIN * CV.KPH_TO_MS and
          float(self.sm['driverMonitoringState'].awarenessStatus) >= 0.0 and
          not self.is_curv_driving and not self.curve_pedal_coordinator.engaged and
          not speed_limit_decel and not long_plan.fcw and
          bool(getattr(CS, 'canValid', True)) and radar_valid and
          self.sm.valid['longitudinalPlan'] and 0.0 <= t_since_plan <= 0.25 and
          len(long_plan.speeds) == CONTROL_N)

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

    def cal_curve_speed(self, sm, v_ego, frame, measured_curvature):
        lateralPlan = sm['lateralPlan']
        if not self.slow_on_curves:
            self.curve_speed_limiter.reset()
            self._model_curve_control_enabled = False
            self.curve_pedal_coordinator.reset()
            self.curve_speed_ms = CURVE_SPEED_DISABLED
            self.curve_plan_speed_ms = CURVE_SPEED_DISABLED
            return

        # modelV2 and lateralPlan are produced at 20 Hz while controlsd runs at
        # 100 Hz. Prefer each fresh model frame and do not count the same model
        # again when its derived lateralPlan arrives on a later control tick.
        model_updated = bool(sm.updated['modelV2'])
        lateral_plan_updated = bool(sm.updated['lateralPlan'])
        if not (model_updated or lateral_plan_updated):
            return

        cruise_speed_ms = self.v_cruise_kph * CV.KPH_TO_MS
        curvature_factor = 0.85 * ntune_scc_get("sccCurvatureFactor")

        model = sm['modelV2']
        model_curve_control_min_kph = float(self.CP.minSteerSpeed) * CV.MS_TO_KPH
        model_curve_control_release_kph = max(0.0, model_curve_control_min_kph - 1.0)
        (model_curvatures, model_times, model_distances,
         model_profile_valid, model_profile_diag) = build_v0813_model_curve_profile(
          model.position.t,
          model.orientationRate.z,
          model.velocity.x, model.velocity.y, model.velocity.z,
          model.position.x, model.position.y, model.position.z,
          measured_curvature, v_ego=v_ego,
          control_min_speed_kph=model_curve_control_min_kph)

        # The Equinox torque/LKAS path starts at 10 km/h. Below that speed the
        # v0.8.13 adapter still runs for diagnostics, but it cannot acquire CURV
        # authority. Retain state down to 9 km/h on deceleration so speed noise
        # around the hardware gate does not repeatedly reset confirmation.
        v_ego_kph = float(v_ego) * CV.MS_TO_KPH
        if v_ego_kph >= model_curve_control_min_kph:
            self._model_curve_control_enabled = True
        elif v_ego_kph < model_curve_control_release_kph:
            self._model_curve_control_enabled = False
            self.curve_speed_limiter.reset()

        if v_ego_kph < model_curve_control_min_kph:
            self.curve_speed_ms = CURVE_SPEED_DISABLED
            shadow_diag = {
              "source": "modelV2_v0813_shadow",
              "values_valid": False,
              "plan_valid": False,
              "raw_speed_ms": CURVE_SPEED_DISABLED,
              "filtered_speed_ms": CURVE_SPEED_DISABLED,
              "confirmed": False,
              "invalid_hold": False,
              "model_valid": bool(sm.valid['modelV2'] and model_profile_valid),
              "model_profile_points": int(len(model_curvatures)),
              "mpc_valid": bool(sm.valid['lateralPlan'] and lateralPlan.mpcSolutionValid and
                                len(lateralPlan.curvatures) == CONTROL_N),
              "measured_curvature": (
                float(measured_curvature) if np.isfinite(measured_curvature) else None),
              "curvature_factor": float(curvature_factor),
            }
            shadow_diag.update(model_profile_diag)
            self.curve_speed_limiter.last_diag = shadow_diag
            return

        model_valid = bool(sm.valid['modelV2'] and model_profile_valid)
        mpc_valid = bool(sm.valid['lateralPlan'] and lateralPlan.mpcSolutionValid and
                         len(lateralPlan.curvatures) == CONTROL_N)

        # A healthy model is the primary 20 Hz clock. lateralPlan is used only
        # while the model itself is invalid, avoiding duplicate confirmation and
        # filter updates from one piece of model evidence.
        if not model_updated and model_valid:
            return

        if model_updated and model_valid:
            curvatures = model_curvatures
            time_idxs = model_times
            distances = model_distances
            source = "modelV2_v0813_adaptive"
            input_valid = True
        elif lateral_plan_updated and mpc_valid:
            curvatures = list(lateralPlan.curvatures)
            if len(curvatures) > 0 and np.isfinite(measured_curvature):
                curvatures[0] = math.copysign(
                  max(abs(float(curvatures[0])), abs(float(measured_curvature))),
                  float(curvatures[0]) if abs(float(curvatures[0])) > 1e-9 else float(measured_curvature))
            time_idxs = None
            distances = None
            source = "lateralPlan_17_fallback"
            input_valid = True
        elif np.isfinite(measured_curvature) and calculate_curve_speed(
              [measured_curvature], v_ego, cruise_speed_ms, MIN_CURVE_SPEED,
              curvature_factor, time_idxs=[0.0])[0] < CURVE_SPEED_DISABLED:
            # Keep an already-entered, physically measured curve recognized
            # through a model/MPC dropout. A straight measurement is not treated
            # as new evidence, so the previous safe limit is held briefly.
            curvatures = [float(measured_curvature)]
            time_idxs = [0.0]
            distances = [0.0]
            source = "measured_fallback"
            input_valid = True
        else:
            curvatures = []
            time_idxs = []
            distances = []
            source = "invalid"
            input_valid = False

        update_kwargs = {
          "plan_valid": input_valid,
          "distances": distances,
          "source": source,
        }
        if time_idxs is not None:
            update_kwargs["time_idxs"] = time_idxs
        self.curve_speed_ms = self.curve_speed_limiter.update(
          curvatures, v_ego, cruise_speed_ms, MIN_CURVE_SPEED,
          curvature_factor,
          confirm_frames=model_profile_diag.get("model_confirm_frames"),
          invalid_hold_frames=model_profile_diag.get("model_invalid_hold_frames"),
          **update_kwargs)
        self.curve_speed_limiter.last_diag.update({
          "model_valid": bool(model_valid),
          "model_profile_points": int(len(model_curvatures)),
          "mpc_valid": bool(mpc_valid),
          "measured_curvature": float(measured_curvature) if np.isfinite(measured_curvature) else None,
          "curvature_factor": float(curvature_factor),
        })
        self.curve_speed_limiter.last_diag.update(model_profile_diag)

    # [크루즈 MAX 속도 설정] #
    def cal_max_speed(self, frame: int, vEgo, sm, CS, measured_curvature):

        road_speed_limiter = get_road_speed_limiter()
        # CS.vEgo is m/s; RoadSpeedLimiter and max_speed_clu use the active
        # cluster unit (km/h or mph).
        v_ego_clu = vEgo * self.speed_conv_to_clu

        apply_limit_speed, road_limit_speed, left_dist, first_started, max_speed_log = \
            road_speed_limiter_get_max_speed(v_ego_clu, self.is_metric)

        # print("apply_limit_speed : ", apply_limit_speed)
        # print("road_limit_speed : ", road_limit_speed)
        # print("left_dist : ", left_dist)
        # print("first_started : ", first_started)
        # print("max_speed_log : ", max_speed_log)

        curv_limit = 0
        self.cal_curve_speed(sm, vEgo, frame, measured_curvature)
        cruise_speed_ms = self.v_cruise_kph * CV.KPH_TO_MS
        if self.CP.enableGasInterceptor:
            curve_diag = dict(getattr(self.curve_speed_limiter, "last_diag", {}) or {})
            try:
                raw_curve_speed_ms = float(curve_diag.get("raw_speed_ms", CURVE_SPEED_DISABLED))
            except (TypeError, ValueError):
                raw_curve_speed_ms = CURVE_SPEED_DISABLED
            if not np.isfinite(raw_curve_speed_ms):
                raw_curve_speed_ms = CURVE_SPEED_DISABLED
            curve_detected = bool(curve_diag.get("confirmed", False) and
                                  MIN_CURVE_SPEED <= raw_curve_speed_ms < CURVE_SPEED_DISABLED)
            curve_plan_speed_ms = self.curve_pedal_coordinator.update_curve(
              curve_detected,
              vEgo,
              self.v_cruise_kph,
              raw_curve_speed_ms,
              selected_time_s=curve_diag.get("selected_time_s", None))
            self.curve_plan_speed_ms = (CURVE_SPEED_DISABLED if curve_plan_speed_ms is None
                                        else float(curve_plan_speed_ms))
        else:
            self.curve_plan_speed_ms = self.curve_speed_ms

        if (self.slow_on_curves and MIN_CURVE_SPEED <= self.curve_plan_speed_ms <
                min(CURVE_SPEED_DISABLED, cruise_speed_ms)):
            max_speed_clu = min(cruise_speed_ms, self.curve_plan_speed_ms) * self.speed_conv_to_clu
            curv_limit = int(max_speed_clu)
        else:
            max_speed_clu = self.kph_to_clu(self.v_cruise_kph)

        # onroad CURV indicator: show the configured curve target while curve
        # slowdown is actively available to the engaged cruise controller.
        curve_state_engaged = (self.curve_pedal_coordinator.engaged
                               if self.CP.enableGasInterceptor else curv_limit > 0)
        self.is_curv_driving = bool(curve_state_engaged and curv_limit > 0 and CS.cruiseState.enabled)
        # The configured curve target is fixed at MIN_CURVE_SPEED. The applied
        # speed still approaches it through the safety filters above.
        self.curv_speed = (float(MIN_CURVE_SPEED) * CV.MS_TO_KPH
                           if self.is_curv_driving else 0.0)

        if road_speed_limiter.roadLimitSpeed is not None:
            camSpeedFactor = clip(road_speed_limiter.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)
            self.over_speed_limit = road_speed_limiter.roadLimitSpeed.camLimitSpeedLeftDist > 0 and \
                                    0 < road_limit_speed * camSpeedFactor < v_ego_clu + 2
        else:
            self.over_speed_limit = False

        max_speed_log = ""

        if apply_limit_speed >= self.kph_to_clu(V_CRUISE_MIN):       # 크루즈 최저 속도보다 큰 경우 설정

            # 크루즈 초기 설정 속도 (PSK)
            # controls.v_cruise_kph : 크루즈 설정 속도
            if first_started:
                self.max_speed_clu = v_ego_clu
                # self.max_speed_clu = self.v_cruise_kph

            max_speed_clu = min(max_speed_clu, apply_limit_speed)
            self.speed_limit_coast_active = bool(left_dist > 0.0)
            self.speed_limit_coast_target_ms = max(
              0.0, float(apply_limit_speed) * self.speed_conv_to_ms)
            self.speed_limit_coast_distance_m = (
              max(0.0, float(left_dist)) if left_dist > 0.0 else math.inf)

            # if self.v_cruise_kph > apply_limit_speed:
            if v_ego_clu > apply_limit_speed:
                if not self.slowing_down_alert and not self.slowing_down:
                    self.slowing_down_sound_alert = True
                    self.slowing_down = True
                self.slowing_down_alert = True
            else:
                self.slowing_down_alert = False
        else:
            self.slowing_down_alert = False
            self.slowing_down = False
            self.speed_limit_coast_active = False
            self.speed_limit_coast_target_ms = 0.0
            self.speed_limit_coast_distance_m = math.inf

        '''lead_speed = self.get_long_lead_safe_speed(sm, CS, vEgo)
        if self.safe_distance_speed and lead_speed >= self.min_set_speed_clu:
            if lead_speed < max_speed_clu:
                max_speed_clu = min(max_speed_clu, lead_speed)
                if not self.limited_lead:
                    self.max_speed_clu = vEgo + 3.
                    self.limited_lead = True
        else:
          self.limited_lead = False'''


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

        panda_states_valid = self.sm.valid["pandaStates"]
        panda_safety_matches = panda_states_valid and panda_safety_config_matches(
            self.sm['pandaStates'], self.CP.safetyConfigs, self.CP.alternativeExperience,
            IGNORED_SAFETY_MODES)
        controls_mismatch_reasons = []

        # A Panda can temporarily enter noOutput while it is being reset or
        # reconfigured offroad. A previously latched ready state must not turn
        # that expected transition into controlsMismatch; require the normal
        # consecutive-frame handshake again before a subsequent engagement.
        # A mismatch while controls are enabled remains an immediate safety
        # event below.
        if not self.enabled and not panda_safety_matches:
            self.panda_safety_ready = False
            self.panda_safety_match_counter = 0

        # ControlsReady lets boardd apply CarParams. Do not allow engagement until the
        # resulting Panda safety configuration has remained correct for a short period.
        if not self.panda_safety_ready:
            self.panda_safety_ready, self.panda_safety_match_counter = update_panda_safety_readiness(
                self.panda_safety_ready, self.panda_safety_match_counter,
                panda_safety_matches, PANDA_SAFETY_MATCH_FRAMES)
            if self.panda_safety_ready:
                cloudlog.info("Panda safety configuration ready")
            else:
                self.events.add(EventName.controlsInitializing)

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

                    if self.panda_safety_ready and safety_mismatch and \
                            (self.sm.frame - self.last_safety_mismatch_log_frame) > int(1. / DT_CTRL):
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

                    if self.panda_safety_ready and safety_mismatch and \
                            (self.sm.frame - self.last_safety_mismatch_log_frame) > int(1. / DT_CTRL):
                        cloudlog.warning(
                            "controlsMismatch extra panda not ignored: "
                            f"idx={i} pandaModel={pandaState.safetyModel}"
                        )
                        self.last_safety_mismatch_log_frame = self.sm.frame

                if self.panda_safety_ready and safety_mismatch:
                    self.events.add(EventName.controlsMismatch)
                    controls_mismatch_reasons.append(
                        "safety_config" if i < len(self.CP.safetyConfigs) else "unexpected_extra_panda")

                if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
                    self.events.add(EventName.relayMalfunction)

            # Catch a missing expected Panda, which cannot be represented by the per-item loop.
            if self.panda_safety_ready and not panda_safety_matches:
                if len(self.sm['pandaStates']) < len(self.CP.safetyConfigs) and \
                        (self.sm.frame - self.last_safety_mismatch_log_frame) > int(1. / DT_CTRL):
                    cloudlog.warning(
                        "controlsMismatch missing panda: "
                        f"actualCount={len(self.sm['pandaStates'])} expectedCount={len(self.CP.safetyConfigs)}"
                    )
                    self.last_safety_mismatch_log_frame = self.sm.frame
                self.events.add(EventName.controlsMismatch)
                controls_mismatch_reasons.append("missing_expected_panda")

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
            controls_mismatch_reasons.append("controls_allowed")

        controls_mismatch_snapshot = self._controls_mismatch_snapshot(
            panda_safety_matches, include_manager_processes=bool(controls_mismatch_reasons))
        self._record_controls_mismatch_snapshot(controls_mismatch_snapshot)
        self._record_controls_mismatch_episode(controls_mismatch_reasons, controls_mismatch_snapshot)

        # Check for HW or system issues
        panda_powering_down = panda_power_down_in_progress(
            self.sm['deviceState'].chargingDisabled, self.enabled)
        if len(self.sm['radarState'].radarErrors):
            self.events.add(EventName.radarFault)
        elif not self.sm.valid["pandaStates"] and not panda_powering_down:
            self.events.add(EventName.usbError)
        # self.sm.all_checks()
        # self.sm.all_alive_and_valid()
        else:
            # torqued is disabled (see process_config.py) -- its output was
            # never consumed, so there is no longer a liveTorqueParameters
            # service to special-case here.
            communication_bad = not controlsd_communication_ok(self.sm)
            if panda_powering_down:
                self.comm_issue_counter = 0
            elif communication_bad:
                self.comm_issue_counter = min(
                    self.comm_issue_counter + 1, COMM_ISSUE_CONSECUTIVE_FRAMES)
            else:
                self.comm_issue_counter = 0

            # CAN receive failures remain immediate. A service-health failure
            # must persist for 300 ms, which rejects scheduler jitter without
            # hiding an actually stopped or invalid process.
            comm_issue_active = bool(
                not panda_powering_down and
                (self.can_rcv_error or
                 self.comm_issue_counter >= COMM_ISSUE_CONSECUTIVE_FRAMES))
            if comm_issue_active:
                self.events.add(EventName.commIssue)
                if not self.logged_comm_issue:
                    invalid = [s for s, valid in self.sm.valid.items() if not valid]
                    not_alive = [s for s, alive in self.sm.alive.items() if not alive]
                    bad_frequency = [s for s, freq_ok in self.sm.freq_ok.items()
                                     if not freq_ok and s not in self.sm.ignore_average_freq]
                    cloudlog.event("commIssue", invalid=invalid, not_alive=not_alive,
                                   can_error=self.can_rcv_error,
                                   consecutive_frames=self.comm_issue_counter,
                                   bad_frequency=bad_frequency, error=True)
                    self.logged_comm_issue = True
            elif self.comm_issue_counter == 0:
                self.logged_comm_issue = False

        if not self.sm['liveParameters'].valid:
            self.events.add(EventName.vehicleModelInvalid)
        if not self.sm['lateralPlan'].mpcSolutionValid and not (EventName.turningIndicatorOn in self.events.names):
            self.events.add(EventName.plannerError)

        # Lane-confidence watchdog. Thresholds measured on 2026-08-26--12-34-51
        # (11.5 min, 48 driver interventions): dProb < 0.25 sustained 8 s with a
        # 30 s cooldown fired twice, both immediately before the driver had to
        # take over. Shorter sustains do not get rarer as the threshold drops --
        # brief dropouts are normal and frequent, so duration is what separates
        # trouble from noise. Re-check these against a daytime/highway route.
        if (self.active and CS.vEgo > LANE_CONF_MIN_SPEED and
                self.sm['lateralPlan'].dProb < LANE_CONF_DPROB):
            self.lane_conf_low_s += DT_CTRL
        else:
            self.lane_conf_low_s = 0.0
        if self.lane_conf_low_s >= LANE_CONF_SUSTAIN_S:
            now = sec_since_boot()
            if now - self.lane_conf_alert_t >= LANE_CONF_COOLDOWN_S:
                self.events.add(EventName.laneConfidenceLow)
                self.lane_conf_alert_t = now
        if not self.sm['liveLocationKalman'].sensorsOK and not NOSENSOR:
            if self.sm.frame > 5 / DT_CTRL:  # Give locationd some time to receive all the inputs
                self.events.add(EventName.sensorDataInvalid)
        if not self.sm['liveLocationKalman'].posenetOK:
            self.events.add(EventName.posenetInvalid)
        if not self.sm['liveLocationKalman'].deviceStable:
            self.events.add(EventName.deviceFalling)

        if not REPLAY:
            # Check for mismatch between openpilot and car's PCM.
            #
            # The upstream condition also fires on `not self.CP.pcmCruise`, which
            # asks "is openpilot doing longitudinal itself?" -- on this GM the
            # answer is always yes (gm/interface.py sets pcmCruise = False for the
            # pedal interceptor), so that term is constantly true and the check
            # collapses to "the car's cruise is on". It then fires continuously
            # for the whole drive: 817 events in 13 min on 2026-08-27--02-44-03,
            # 301 in 11.5 min the day before, tracking cruise-on time and nothing
            # else. The event's own handlers are already commented out in
            # events.py, so it was pure log noise.
            #
            # What the check is actually for -- openpilot failing to cancel the
            # car's cruise while disengaged -- still works via the first term.
            cruise_mismatch = CS.cruiseState.enabled and not self.enabled
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
            if self.sm.updated['managerState']:
                manager_processes = self.sm['managerState'].processes
                not_running = expected_not_running_processes(manager_processes, IGNORE_PROCESSES)
                was_active = self.process_not_running_active
                previous_names = tuple(sorted(self.process_not_running_candidates))
                self.process_not_running_counter, self.process_not_running_candidates, \
                    self.process_not_running_active = update_process_not_running_state(
                        self.process_not_running_counter,
                        self.process_not_running_candidates,
                        not_running,
                        PROCESS_NOT_RUNNING_CONSECUTIVE_UPDATES)

                names = tuple(sorted(self.process_not_running_candidates))
                if self.process_not_running_active and names != self.process_not_running_logged_names:
                    cloudlog.error(
                        "processNotRunning persistent: "
                        f"names={list(names)} consecutiveUpdates={self.process_not_running_counter}"
                    )
                    append_process_diagnostic(
                        "controlsd_process_not_running",
                        processes=list(names),
                        consecutive_updates=self.process_not_running_counter,
                        manager_processes=[{
                            "name": p.name,
                            "running": bool(p.running),
                            "should_be_running": bool(p.shouldBeRunning),
                            "pid": int(p.pid),
                            "exit_code": int(p.exitCode),
                        } for p in manager_processes],
                    )
                    self.process_not_running_logged_names = names
                elif was_active and not self.process_not_running_active:
                    cloudlog.info(f"processNotRunning recovered: names={list(previous_names)}")
                    append_process_diagnostic(
                        "controlsd_process_recovered",
                        processes=list(previous_names),
                    )
                    self.process_not_running_logged_names = ()

            if self.process_not_running_active:
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
            if hasattr(self.LaC, 'update_ntune_torque_params'):
                try:
                    self.LaC.update_ntune_torque_params(
                        ntune_torque_get('latAccelFactor'),
                        ntune_torque_get('friction'))
                except Exception:
                    # Keep the last valid torque parameters if ntune is
                    # temporarily unavailable or contains an invalid value.
                    pass

            if hasattr(self.LaC, 'get_fixed_torque_params'):
                fixed_torque = self.LaC.get_fixed_torque_params()
                self.torque_latAccelFactor = fixed_torque['latAccelFactor']
                self.torque_friction = fixed_torque['friction']
                self.torque_latAccelOffset = fixed_torque['latAccelOffset']
                self.totalBucketPoints = 0
            else:
                self.torque_latAccelFactor = ntune_torque_get('latAccelFactor')
                self.torque_friction = ntune_torque_get('friction')
                self.torque_latAccelOffset = 0.0
                self.totalBucketPoints = 0


        lat_plan = self.sm['lateralPlan']
        long_plan = self.sm['longitudinalPlan']
        if hasattr(self.LaC, 'set_path_stability'):
            self.LaC.set_path_stability(
              bool(getattr(lat_plan, 'pathStabilityActive', False)),
              float(getattr(lat_plan, 'pathWobbleRangeM', 0.0)),
              int(getattr(lat_plan, 'pathWobbleFlips', 0)))
        if hasattr(self.LaC, 'set_model_path_quality'):
            self.LaC.set_model_path_quality(
              float(getattr(lat_plan, 'modelPathQuality', 0.0)),
              bool(getattr(lat_plan, 'modelPathQualityTrusted', False)),
              float(getattr(lat_plan, 'modelNearCurvature', 0.0)))

        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        # CarControl is reconstructed again in publish_logs. Use the same
        # helper in both places so activation flags cannot be dropped between
        # lateral control and the vehicle CarController.
        apply_control_activation(CC, self.active, CS, self.CP, self.events.any(ET.OVERRIDE))

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

        # Remember one confirmed lead launch so BOOST does not disappear when
        # dynamic-follow finishes its short launch phase. The latch is output-
        # gated below 1 km/h and released at 25 km/h, on brake, lead re-stop, or
        # an unsafe closing rate. Normal longitudinal limits still apply.
        dynamic_follow_valid = self.sm.valid['dynamicFollowData']
        dynamic_follow = self.sm['dynamicFollowData']
        boost_system_ready = bool(not self.joystick_mode and
                                  self.CP.carName == 'gm' and self.CP.enableGasInterceptor and
                                  self.active and self.state == State.enabled and
                                  CS.canValid and self.sm.valid['longitudinalPlan'] and
                                  self.sm.valid['radarState'] and
                                  len(self.sm['radarState'].radarErrors) == 0 and
                                  not long_plan.fcw and dynamic_follow_valid and
                                  dynamic_follow.stopAccelBoostEnabled)
        self.stop_accel_boost_active = self.stop_accel_boost_latch.update(
          boost_system_ready,
          dynamic_follow_valid and dynamic_follow.leadCatchupActive,
          CS.vEgo,
          brake_pressed=CS.brakePressed,
          gas_pressed=CS.gasPressed,
          lead_speed=dynamic_follow.leadSpeed if dynamic_follow_valid else 0.0,
          lead_relative_speed=dynamic_follow.leadRelativeSpeed if dynamic_follow_valid else 0.0,
          lead_distance=dynamic_follow.leadDistance if dynamic_follow_valid else 0.0)

        if not self.joystick_mode:
            # accel PID loop
            pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_kph * CV.KPH_TO_MS)
            t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL

            boost_floor_context_safe = boost_floor_context_allowed(
              self.stop_accel_boost_latch.floor_allowed,
              can_valid=CS.canValid,
              radar_valid=self.sm.valid['radarState'],
              radar_error=len(self.sm['radarState'].radarErrors) > 0,
              driver_aware=(
                float(self.sm['driverMonitoringState'].awarenessStatus) >= 0.0),
              curv_driving=self.is_curv_driving,
              curve_active=(
                self.curve_pedal_coordinator.engaged or
                self.curve_pedal_coordinator.pedal_intervening),
              speed_limit_active=self.speed_limit_coast_active,
              speed_limit_target=self.speed_limit_coast_target_ms,
              v_ego=CS.vEgo,
              fcw=long_plan.fcw,
              plan_valid=self.sm.valid['longitudinalPlan'],
              plan_age=t_since_plan,
              plan_full=len(long_plan.speeds) == CONTROL_N,
              plan_source_lead=(
                str(long_plan.longitudinalPlanSource) == 'lead0'))
            boost_floor_accel = self.stop_accel_boost_latch.update_hill_response(
              boost_floor_context_safe, CS.aEgo)
            raw_long_accel = self.LoC.update(
              self.active, CS, long_plan, pid_accel_limits, t_since_plan,
              self.stop_accel_boost_active, boost_floor_accel,
              self.stop_accel_boost_latch.driver_launch_handoff)
            lead_one = self.sm['radarState'].leadOne
            radar_valid = bool(self.sm.valid['radarState'] and
                               len(self.sm['radarState'].radarErrors) == 0)
            effective_tr = (dynamic_follow.mpcTR if dynamic_follow_valid else 1.3)
            speeds = long_plan.speeds
            speed_error = float(self.LoC.v_pid - CS.vEgo)
            future_speed_error = float(speeds[-1] - CS.vEgo) if len(speeds) == CONTROL_N else 0.0
            cruise_speed_error = float(
              min(self.v_cruise_kph, V_CRUISE_MAX) * CV.KPH_TO_MS - CS.vEgo)
            lead_loss_base_safe = self.lead_loss_cruise_assist_base_safe(
              CS, long_plan, t_since_plan, radar_valid)
            lead_loss_accel = self.lead_loss_cruise_assist.update(
              base_safe=lead_loss_base_safe,
              lead_valid=bool(lead_one.status),
              cruise_speed_error=cruise_speed_error,
              requested_accel=raw_long_accel)
            recovery_eligible = (
              self.pedal_force_recovery_eligible(CS, long_plan, t_since_plan) and
              not self.lead_loss_cruise_assist.active and
              not self.lead_loss_cruise_assist.armed and
              not self.lead_coast_assist.active and
              not self.moving_gap_catchup_assist.active)
            hard_recovery_accel = self.pedal_force_recovery.update(
              recovery_eligible, raw_long_accel)
            lead_assist_base_safe = self.lead_coast_assist_base_safe(
              CS, long_plan, t_since_plan, radar_valid) and \
              not self.pedal_force_recovery.active and \
              not self.lead_loss_cruise_assist.active and \
              not self.moving_gap_catchup_assist.active
            lead_assist_accel = self.lead_coast_assist.update(
              base_safe=lead_assist_base_safe,
              lead_valid=bool(lead_one.status),
              lead_v_rel=lead_one.vRel if lead_one.status else 0.0,
              lead_distance=lead_one.dRel if lead_one.status else 0.0,
              v_ego=CS.vEgo,
              desired_tr=effective_tr,
              speed_error=speed_error,
              future_speed_error=future_speed_error,
              a_ego=CS.aEgo,
              requested_accel=raw_long_accel,
              lead_measurement_updated=bool(self.sm.updated['radarState']))
            moving_gap_base_safe = self.lead_coast_assist_base_safe(
              CS, long_plan, t_since_plan, radar_valid) and \
              not self.pedal_force_recovery.active and \
              not self.lead_loss_cruise_assist.active and \
              not self.lead_coast_assist.active
            moving_gap_accel = self.moving_gap_catchup_assist.update(
              base_safe=moving_gap_base_safe,
              lead_valid=bool(lead_one.status),
              lead_v_rel=lead_one.vRel if lead_one.status else 0.0,
              lead_distance=lead_one.dRel if lead_one.status else 0.0,
              lead_model_prob=lead_one.modelProb if lead_one.status else 0.0,
              v_ego=CS.vEgo,
              desired_tr=effective_tr,
              cruise_speed_error=cruise_speed_error,
              requested_accel=raw_long_accel,
              lead_measurement_updated=bool(self.sm.updated['radarState']))
            actuators.accel = (hard_recovery_accel if self.pedal_force_recovery.active
                               else lead_loss_accel if self.lead_loss_cruise_assist.active
                               else lead_assist_accel if self.lead_coast_assist.active
                               else moving_gap_accel)
            self.curve_pedal_raw_accel = float(actuators.accel)
            # Curve target shaping remains in the longitudinal plan. Lead,
            # curve, and speed-limit pedal lift are arbitrated once below so
            # independent smoothers cannot multiply each other.
            self.curve_pedal_final_accel = float(actuators.accel)

            # Driving-style gain is applied later in CarController. Predictive
            # coasting supplies a final 0..1 pedal ceiling so learned response
            # cannot add back pedal while a lead is consuming the desired gap.
            predictive_enabled = bool(self.CP.enableGasInterceptor and
                                      self.driving_style_status.enabled)
            curve_diag = dict(getattr(self.curve_speed_limiter, "last_diag", {}) or {})
            curve_target_ms = (self.curve_pedal_coordinator.plan_speed_kph * CV.KPH_TO_MS
                               if self.curve_pedal_coordinator.plan_speed_kph > 0.0 else CS.vEgo)
            self.predictive_coast_pedal_scale = self.predictive_coasting.update(
              enabled=predictive_enabled,
              control_active=self.active,
              requested_accel=actuators.accel,
              v_ego=CS.vEgo,
              a_ego=CS.aEgo,
              brake_pressed=CS.brakePressed,
              gas_pressed=CS.gasPressed,
              lead_valid=lead_one.status,
              lead_distance=lead_one.dRel if lead_one.status else 0.0,
              lead_rel_speed=lead_one.vRel if lead_one.status else 0.0,
              lead_accel=lead_one.aLeadK if lead_one.status else 0.0,
              lead_model_prob=lead_one.modelProb if lead_one.status else 0.0,
              effective_tr=effective_tr,
              fcw=long_plan.fcw,
              radar_valid=radar_valid,
              can_valid=CS.canValid,
              curve_active=self.curve_pedal_coordinator.curve_active,
              curve_target_speed=curve_target_ms,
              curve_time_s=curve_diag.get("selected_time_s", math.inf),
              curve_distance_m=curve_diag.get("selected_distance_m", math.inf),
              speed_limit_active=self.speed_limit_coast_active,
              speed_limit_target=self.speed_limit_coast_target_ms,
              speed_limit_distance_m=self.speed_limit_coast_distance_m,
              natural_decel_ms2=self.natural_decel_status.decel_ms2,
              natural_decel_confidence=self.natural_decel_status.confidence,
              brake_alert_enabled=self.predictive_brake_alert_enabled,
              lead_loss_recovery_active=self.lead_loss_cruise_assist.active,
              launch_boost_floor_active=bool(
                boost_floor_context_safe and boost_floor_accel > 0.0),
              positive_recovery_active=bool(
                self.pedal_force_recovery.active or
                self.lead_coast_assist.active or
                self.lead_loss_cruise_assist.active or
                self.moving_gap_catchup_assist.active),
              learned_low_speed_coast_offset_s=(
                self.driving_style_status.low_speed_coast_offset_s))

            coast_lane_change = lat_plan.laneChangeState != LaneChangeState.off
            coast_orientation = self.sm['liveLocationKalman'].calibratedOrientationNED
            coast_orientation_values = coast_orientation.value
            coast_pitch_rad = (float(coast_orientation_values[1])
                               if len(coast_orientation_values) > 1 else math.nan)
            llk = self.sm['liveLocationKalman']
            calibration_ok = bool(
              self.sm.valid['liveCalibration'] and
              self.sm['liveCalibration'].calStatus == Calibration.CALIBRATED)
            (self.natural_decel_pitch_deg,
             self.natural_decel_pitch_valid,
             self.natural_decel_pitch_fallback,
             self.natural_decel_pitch_source) = select_road_pitch(
               coast_pitch_rad,
               llk_valid=self.sm.valid['liveLocationKalman'],
               orientation_valid=coast_orientation.valid,
               inputs_ok=llk.inputsOK,
               sensors_ok=llk.sensorsOK,
               calibration_ok=calibration_ok)
            natural_context_ok = bool(
              predictive_enabled and self.active and
              not self.curve_pedal_coordinator.engaged and
              not CS.leftBlinker and not CS.rightBlinker and not coast_lane_change and
              CS.canValid)
            self.natural_decel_status = self.natural_decel_learner.update(
              v_ego=CS.vEgo,
              a_ego=CS.aEgo,
              pedal_output=self.last_actuators.gas,
              brake_pressed=CS.brakePressed,
              gas_pressed=CS.gasPressed,
              context_ok=natural_context_ok,
              pitch_deg=self.natural_decel_pitch_deg,
              pitch_valid=self.natural_decel_pitch_valid,
              pitch_fallback=self.natural_decel_pitch_fallback,
              dt=DT_CTRL)

            if self.is_curv_driving:
                self.events.add(EventName.curveEntry)
            elif self.predictive_coasting.brake_advisory:
                self.events.add(EventName.predictiveBrakeNeeded)

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
            self.predictive_coasting.reset()
            self.pedal_force_recovery.update(False, actuators.accel)
            self.lead_coast_assist.update(False, False, 0.0, 0.0, CS.vEgo, 1.3,
                                          0.0, 0.0, CS.aEgo, actuators.accel)
            self.lead_loss_cruise_assist.update(
              False, False, 0.0, actuators.accel)
            self.moving_gap_catchup_assist.update(
              base_safe=False, lead_valid=False, lead_v_rel=0.0,
              lead_distance=0.0, lead_model_prob=0.0, v_ego=CS.vEgo,
              desired_tr=1.3, cruise_speed_error=0.0,
              requested_accel=actuators.accel)
            self.predictive_coast_pedal_scale = 1.0
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

        if self.sm.frame % COMMA_PEDAL_PARAM_REFRESH_FRAMES == 0:
            self.comma_pedal_profile = normalize_comma_pedal_profile(
              self.params.get("CommaPedalResistance", encoding="utf8") or 'mid')
        pedal_profile_active = bool(
          self.active and self.CP.enableGasInterceptor and
          self.last_actuators.gas > 0.001 and
          not CS.gasPressed and not CS.brakePressed)
        self.comma_pedal_profile_gain = self.comma_pedal_profile_controller.update(
          self.comma_pedal_profile, CS.vEgo, pedal_profile_active, DT_CTRL)
        self.comma_pedal_profile_changing = bool(
          self.comma_pedal_profile_controller.changing)

        # Event-based driver-style learning. Inputs are evaluated only in clean,
        # straight, stable-control context. Curve slowdown, lane changes, FCW,
        # and stop-launch boost are excluded so those safety/context responses
        # are not mistaken for driver preference.
        lane_change_active = lat_plan.laneChangeState != LaneChangeState.off
        style_unsafe_context = bool(self.joystick_mode or not self.CP.enableGasInterceptor or
                                    CS.leftBlinker or CS.rightBlinker or lane_change_active or
                                    self.is_curv_driving or long_plan.fcw or
                                    self.stop_accel_boost_active or
                                    self.moving_gap_catchup_assist.active or
                                    self.comma_pedal_profile_changing or
                                    self.predictive_coasting.learning_blocked)
        low_speed_brake_context_ok = bool(
          not self.joystick_mode and self.CP.enableGasInterceptor and
          not CS.leftBlinker and not CS.rightBlinker and not lane_change_active and
          not self.is_curv_driving and not self.curve_pedal_coordinator.engaged and
          not self.speed_limit_coast_active and not long_plan.fcw and CS.canValid)
        style_lead_valid = bool(dynamic_follow_valid and dynamic_follow.leadDistance > 0.0)
        self.driving_style_status = self.driving_style_learner.update(
          v_ego=CS.vEgo,
          a_ego=CS.aEgo,
          gas=CS.gas,
          gas_pressed=CS.gasPressed,
          brake=CS.brake,
          brake_pressed=CS.brakePressed,
          cruise_enabled=CS.cruiseState.enabled,
          control_active=self.active and self.CP.enableGasInterceptor,
          requested_accel=actuators.accel,
          lead_valid=style_lead_valid,
          lead_distance=dynamic_follow.leadDistance if style_lead_valid else 0.0,
          lead_rel_speed=dynamic_follow.leadRelativeSpeed if style_lead_valid else 0.0,
          base_tr=dynamic_follow.mpcTR if dynamic_follow_valid else 1.3,
          pedal_output=self.last_actuators.gas,
          unsafe_context=style_unsafe_context,
          low_speed_brake_context_ok=low_speed_brake_context_ok,
          can_valid=CS.canValid,
          dt=DT_CTRL)
        # Never stack learned pedal gain on top of the dedicated 40% lead-launch
        # boost. The learner remains bounded, but the two features serve
        # different purposes and must not compound each other.
        self.driving_style_gain = 1.0 if self.stop_accel_boost_active else self.driving_style_status.gain
        self.comma_pedal_learned_gain = float(self.driving_style_status.gain)
        self.comma_pedal_effective_gain = combine_comma_pedal_gain(
          self.comma_pedal_profile_gain, self.comma_pedal_learned_gain,
          self.stop_accel_boost_active)

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
        # state_control computes these flags on a different temporary
        # CarControl message. Re-populate them on the message passed to CI.apply
        # and published to carControl; otherwise both fields default to false.
        apply_control_activation(CC, self.active, CS, self.CP, self.events.any(ET.OVERRIDE))

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
        road_limit_speed, left_dist, max_speed_log = self.cal_max_speed(
            self.sm.frame, CS.vEgo, self.sm, CS, curvature)

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
        # Report the effective CarParams value after the GM minimum clamp, not
        # the raw ntune request. v0.8.13's additional 0.2 s is logged below.
        controlsState.steerActuatorDelay = float(self.CP.steerActuatorDelay)

        controlsState.sccGasFactor = ntune_scc_get('sccGasFactor')
        controlsState.sccBrakeFactor = ntune_scc_get('sccBrakeFactor')
        controlsState.sccCurvatureFactor = ntune_scc_get('sccCurvatureFactor')

        # Curve slowdown state consumed by the onroad CURV indicator.
        controlsState.curvDriving = bool(self.is_curv_driving)
        controlsState.curvSpeed = float(self.curv_speed)

        # Live Torque
        controlsState.latAccelFactor = self.torque_latAccelFactor
        controlsState.latAccelOffset = self.torque_latAccelOffset
        controlsState.friction = self.torque_friction
        controlsState.totalBucketPoints = self.totalBucketPoints
        if hasattr(self.LaC, 'get_dynamic_debug_torque_params'):
            dyn_torque = self.LaC.get_dynamic_debug_torque_params()
            controlsState.dynamicTorqueActive = bool(dyn_torque['active'])
            controlsState.dynamicTorqueLatAccelFactor = float(dyn_torque['latAccelFactor'])
            controlsState.dynamicTorqueFriction = float(dyn_torque['friction'])
            controlsState.dynamicTorqueBlend = float(dyn_torque['blend'])
            controlsState.dynamicTorqueAuthorityCeiling = float(dyn_torque['authorityCeiling'])
            controlsState.dynamicTorqueCornerStrength = float(dyn_torque['corner_strength'])
            controlsState.dynamicTorqueDirectionDamping = bool(dyn_torque['directionDamping'])
            controlsState.dynamicTorqueResponseScale = float(dyn_torque['responseScale'])
            controlsState.dynamicTorqueResponseRatio = float(dyn_torque['responseRatio'])
            controlsState.dynamicTorqueResponseBin = int(dyn_torque['responseBin'])
            controlsState.dynamicTorqueResponseStable = bool(dyn_torque['responseStable'])
            controlsState.dynamicTorqueResponseFrozen = bool(dyn_torque['responseFrozen'])
            controlsState.dynamicTorqueResponseUpdateCount = int(dyn_torque['responseUpdateCount'])
            controlsState.dynamicTorquePathStabilityActive = bool(dyn_torque['pathStabilityActive'])
            controlsState.dynamicTorquePathWobbleRange = float(dyn_torque['pathWobbleRangeM'])
            controlsState.dynamicTorquePathWobbleFlips = int(dyn_torque['pathWobbleFlips'])
            controlsState.laneCenterCorrectionM = float(
              getattr(self.sm['lateralPlan'], 'laneCenterCorrectionM', 0.0))
            controlsState.laneCenterCorrectionActive = bool(
              getattr(self.sm['lateralPlan'], 'laneCenterCorrectionActive', False))
            controlsState.modelCurvatureGuardActive = bool(dyn_torque['modelCurvatureGuardActive'])
            controlsState.modelCurvatureRaw = float(dyn_torque['modelCurvatureRaw'])
            controlsState.modelCurvatureFiltered = float(dyn_torque['modelCurvatureFiltered'])
            controlsState.modelCurvatureFilterAlpha = float(dyn_torque['modelCurvatureFilterAlpha'])
            controlsState.modelCurvatureDirectionReversal = bool(
              dyn_torque['modelCurvatureDirectionReversal'])
            controlsState.modelSteerDelayCompensation = float(
              dyn_torque['modelSteerDelayCompensation'])
            controlsState.lowSpeedTorqueGuardActive = bool(
              dyn_torque['lowSpeedTorqueGuardActive'])
            controlsState.lowSpeedTorqueGuardState = int(
              dyn_torque['lowSpeedTorqueGuardState'])
            controlsState.lowSpeedTorqueRawSteer = float(
              dyn_torque['lowSpeedTorqueRawSteer'])
            controlsState.lowSpeedTorqueGuardedSteer = float(
              dyn_torque['lowSpeedTorqueGuardedSteer'])
            controlsState.lowSpeedTorqueAppliedSteer = float(
              dyn_torque['lowSpeedTorqueAppliedSteer'])
            controlsState.lowSpeedTorqueConfirmMs = int(
              dyn_torque['lowSpeedTorqueConfirmMs'])
            controlsState.lowSpeedTorqueReversalCount = int(
              dyn_torque['lowSpeedTorqueReversalCount'])
            controlsState.lowSpeedTorqueBoostSuppressed = bool(
              dyn_torque['lowSpeedTorqueBoostSuppressed'])

        # Dynamic TR
        #controlsState.cruiseGap = int(Params().get("cruiseGap", encoding="utf8"))
        controlsState.minTR = float(Params().get("minTR", encoding="utf8"))
        #controlsState.dynamicTRMode = int(self.sm['longitudinalPlan'].dynamicTRMode)
        controlsState.dynamicTRMode = Params().get("DynamicTRGap", encoding="utf8")
        controlsState.globalDfMod = float(Params().get("globalDfMod", encoding="utf8"))
        controlsState.dynamicTRValue = float(self.sm['dynamicFollowData'].mpcTR)
        controlsState.followingDistanceRawTR = float(
          getattr(self.sm['dynamicFollowData'], 'rawTR', 1.3))
        controlsState.followingDistanceLearnedTROffset = float(
          getattr(self.sm['dynamicFollowData'], 'learnedTROffset', 0.0))

        # Stop-and-go launch diagnostics. These report the controller-accepted
        # request and whether it actually raised the acceleration command.
        controlsState.stopAccelBoostActive = bool(self.stop_accel_boost_active)
        controlsState.stopAccelBoostApplied = bool(self.LoC.stop_accel_boost_applied)
        controlsState.stopAccelBoostRawAccel = float(self.LoC.stop_accel_boost_raw_accel)
        controlsState.stopAccelBoostFinalAccel = float(self.LoC.stop_accel_boost_final_accel)
        controlsState.stopAccelBoostFactor = float(STOP_ACCEL_BOOST_FACTOR)
        controlsState.driverLaunchHandoffActive = bool(
          self.LoC.driver_launch_handoff_active)
        controlsState.driverLaunchHandoffShadowAccel = float(
          self.LoC.driver_launch_handoff_shadow_accel)
        controlsState.stopAccelBoostFloorAccel = float(
          self.stop_accel_boost_latch.floor_accel)
        controlsState.stopAccelBoostHillExtraAccel = float(
          self.stop_accel_boost_latch.hill_extra_accel)
        recovery_mode = (RECOVERY_MODE_HARD_ZERO if self.pedal_force_recovery.active else
                         RECOVERY_MODE_LEAD_LOSS_CRUISE if self.lead_loss_cruise_assist.active else
                         RECOVERY_MODE_LEAD_COAST_ASSIST if self.lead_coast_assist.active else
                         RECOVERY_MODE_MOVING_GAP_CATCHUP if self.moving_gap_catchup_assist.active else
                         RECOVERY_MODE_NONE)
        recovery = (self.pedal_force_recovery if self.pedal_force_recovery.active else
                    self.lead_loss_cruise_assist if self.lead_loss_cruise_assist.active else
                    self.lead_coast_assist if self.lead_coast_assist.active else
                    self.moving_gap_catchup_assist)
        controlsState.pedalForceRecoveryActive = recovery_mode != RECOVERY_MODE_NONE
        controlsState.pedalForceRecoveryDuration = float(recovery.duration)
        controlsState.pedalForceRecoveryCount = int(
          self.pedal_force_recovery.activation_count +
          self.lead_loss_cruise_assist.activation_count +
          self.lead_coast_assist.activation_count +
          self.moving_gap_catchup_assist.activation_count)
        controlsState.pedalForceRecoveryRawAccel = float(recovery.raw_accel)
        controlsState.pedalForceRecoveryAccel = float(recovery.forced_accel)
        controlsState.pedalForceRecoveryPedalFloor = float(
          PEDAL_FORCE_RECOVERY_PEDAL_FLOOR if self.pedal_force_recovery.active
          else recovery.pedal_target)
        controlsState.pedalForceRecoveryMode = int(recovery_mode)
        controlsState.pedalLeadAssistActive = bool(self.lead_coast_assist.active)
        controlsState.pedalLeadAssistCandidateDuration = float(self.lead_coast_assist.candidate_duration)
        controlsState.pedalLeadAssistFilteredVRel = float(self.lead_coast_assist.filtered_v_rel)
        controlsState.pedalLeadAssistActualTR = float(self.lead_coast_assist.actual_tr)
        controlsState.pedalLeadAssistDesiredTR = float(self.lead_coast_assist.desired_tr)
        controlsState.pedalLeadAssistTrMargin = float(self.lead_coast_assist.tr_margin)
        controlsState.pedalLeadAssistCancelReason = int(self.lead_coast_assist.cancel_reason)
        controlsState.pedalLeadAssistCount = int(self.lead_coast_assist.activation_count)
        controlsState.pedalLeadAssistPedalTarget = float(self.lead_coast_assist.pedal_target)
        controlsState.movingGapCatchupActive = bool(self.moving_gap_catchup_assist.active)
        controlsState.movingGapCatchupCandidateDuration = float(
          self.moving_gap_catchup_assist.candidate_duration)
        controlsState.movingGapCatchupLeadStableDuration = float(
          self.moving_gap_catchup_assist.lead_stable_duration)
        controlsState.movingGapCatchupFilteredVRel = float(
          self.moving_gap_catchup_assist.filtered_v_rel)
        controlsState.movingGapCatchupDesiredGap = float(
          self.moving_gap_catchup_assist.desired_gap_m)
        controlsState.movingGapCatchupDistanceMargin = float(
          self.moving_gap_catchup_assist.distance_margin_m)
        controlsState.movingGapCatchupEnterMargin = float(
          self.moving_gap_catchup_assist.enter_margin_m)
        controlsState.movingGapCatchupExitMargin = float(
          self.moving_gap_catchup_assist.exit_margin_m)
        controlsState.movingGapCatchupTargetAccel = float(
          self.moving_gap_catchup_assist.target_accel)
        controlsState.movingGapCatchupFinalAccel = float(
          self.moving_gap_catchup_assist.forced_accel)
        controlsState.movingGapCatchupPedalTarget = float(
          self.moving_gap_catchup_assist.pedal_target)
        controlsState.movingGapCatchupCancelReason = int(
          self.moving_gap_catchup_assist.cancel_reason)
        controlsState.movingGapCatchupCount = int(
          self.moving_gap_catchup_assist.activation_count)
        controlsState.movingGapCatchupLeadJump = bool(
          self.moving_gap_catchup_assist.lead_jump_detected)
        controlsState.drivingStyleAIActive = bool(self.driving_style_status.enabled)
        controlsState.drivingStyleAIGain = float(self.driving_style_gain)
        controlsState.drivingStyleAITrOffset = float(self.driving_style_status.tr_offset)
        controlsState.drivingStyleAIConfidence = float(self.driving_style_status.confidence)
        controlsState.drivingStyleAIGasEvents = int(self.driving_style_status.gas_events)
        controlsState.drivingStyleAIBrakeEvents = int(self.driving_style_status.brake_events)
        controlsState.drivingStyleAIStableFollowSec = float(self.driving_style_status.stable_follow_s)
        controlsState.commaPedalResistanceProfile = str(self.comma_pedal_profile)
        controlsState.commaPedalProfileGain = float(self.comma_pedal_profile_gain)
        controlsState.commaPedalLearnedGain = float(self.comma_pedal_learned_gain)
        controlsState.commaPedalEffectiveGain = float(self.comma_pedal_effective_gain)
        controlsState.commaPedalProfileChanging = bool(self.comma_pedal_profile_changing)
        controlsState.commaPedalRawCommand = float(self.comma_pedal_raw_command)
        controlsState.commaPedalStyledCommand = float(self.comma_pedal_styled_command)
        controlsState.commaPedalFinalCommand = float(self.comma_pedal_final_command)

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
