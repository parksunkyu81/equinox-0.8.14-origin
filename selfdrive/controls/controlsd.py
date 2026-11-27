 #!/usr/bin/env python3
import os
import math
import numpy as np
from collections import deque
from numbers import Number

from cereal import car, log
from common.numpy_fast import clip
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from common.conversions import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.atc_desire import ATC_LEFT_TYPES, ATC_RIGHT_TYPES, AtcLaneGuide, \
  BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT, TURN_STATE
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from selfdrive.controls.lib.straight_wobble_filter import StraightWobbleFilter
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
from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, \
  get_road_speed_limiter, ATC_LOW_FLOOR_KPH
from selfdrive.speed_bump_planner import SpeedBumpPlanner
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, CONTROL_N
from selfdrive.car.gm.values import MIN_CURVE_SPEED
#from decimal import Decimal
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.stop_accel_boost import (
  MODEL_GO_PLAN_T_S,
  StopAccelBoostLatch,
  boost_floor_context_block_reasons,
)
from selfdrive.controls.lib.launch_floor_diag import LaunchFloorDiag
from selfdrive.controls.lib.curve_speed_limiter import (
  CurveSpeedLimiter, CURVE_SPEED_DISABLED, build_v0813_model_curve_profile, calculate_curve_speed,
  CornerAlert, model_curve_tuning_band, prepare_profile, CORNER_ALERT_MIN_SPEED_KPH,
  model_curve_band_diag,
)
from selfdrive.controls.lib.curve_pedal_coordinator import CurvePedalCoordinator
from selfdrive.controls.lib.predictive_coasting import PredictiveCoastingCoordinator
from selfdrive.controls.lib.pre_corner_coast import PreCornerCoast
from selfdrive.controls.lib.corner_lift_speed_hold import CornerLiftSpeedHold
from selfdrive.controls.lib.panda_safety import panda_safety_config_matches, update_panda_safety_readiness
from selfdrive.controls.lib.process_health import (
  controlsd_communication_ok, expected_not_running_processes,
  panda_power_down_in_progress, update_process_not_running_state,
)
from selfdrive.controls.lib.comma_pedal_rise_limiter import (
  PEDAL_FALL_HARD_DECEL, PEDAL_RISE_DEFAULT_TR_S)
from selfdrive.controls.lib.comma_pedal_profile import (
  CommaPedalProfileController, comma_pedal_profile_rise_scale,
  normalize_comma_pedal_profile,
)
from selfdrive.controls.lib.comma_pedal_auto import CommaPedalAutoLearner
from selfdrive.controls.lib.follow_learner import FollowLearner
from selfdrive.controls.lib.traffic_state import JamDetector
from selfdrive.controls.lib.traffic_light import TrafficLight
from selfdrive.controls.lib.follow_accel_cap import (
  follow_accel_cap, hold_weight, SpeedHoldOffset, EASE_MPS3 as FOLLOW_CAP_EASE_MPS3)
from selfdrive.controls.lib.pedal_hold_diag import PedalHoldDiag
from selfdrive.controls.lib.dynamic_follow.follow_gap import FOLLOW_LEARN_PARAM, floor_tr
from selfdrive.controls.lib.dynamic_follow.follow_gap import band as follow_band
from selfdrive.process_diagnostics import (append_controls_mismatch_diagnostic,
                                          append_process_diagnostic)

MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

# The camera's hardBrakePredicted only raises FCW above ~18 km/h and after it has
# held for 0.3 s, so slow traffic and single-frame model flickers stay quiet.
# The planner's lead-car FCW (longitudinalPlan.fcw) is not gated.
MODEL_FCW_MIN_SPEED = 18. * CV.KPH_TO_MS
MODEL_FCW_HOLD_FRAMES = int(0.3 / DT_CTRL)  # 30 frames at 100 Hz

# Gate engagement until the final Panda safety configuration is stable. After engagement,
# debounce only the short controlsAllowed message skew; safety configuration changes stay immediate.
# "좌/우회전 중" second line: under this speed LKAS is off on this car.
TURN_SLOW_KPH = 10.
# How long "핸들을 잡아 주세요" stays under the turn message after the steering
# saturation prompt would have fired (it replaces steerSaturated in a turn).
TURN_SAT_HOLD_S = 1.5
# The turn desire as controlsd sees it is held this long over a gap.
TURN_DESIRE_HOLD_S = 0.3
# Under TURN_SLOW_KPH the message shows only while the car is moving and the
# wheel is turned that way, i.e. the turn itself, not the wait before it.
TURN_MOVING_KPH = 2.
TURN_SHOW_WHEEL_DEG = 30.
PANDA_SAFETY_MATCH_FRAMES = 10  # 100 ms at 100 Hz
# This debounce has to outlast the loop that re-arms controlsAllowed, not just message skew.
# Panda clears controlsAllowed when boardd reports engaged=0 for three of its 1 Hz ticks
# (main.c heartbeat_engaged_mismatches). boardd can only report engaged=1 again from
# panda_state_thread, which runs at 2 Hz -- so after the driver re-engages, telling Panda
# about it takes up to 500 ms. At 25 frames openpilot disengaged itself 250 ms in, before
# that heartbeat could ever go out, and Panda re-cleared controlsAllowed on its next tick:
# on 2026-09-06--09-46-21 at 849.4 s that livelocked for 6.0 s across repeated re-engagements
# (boardd_safety_diagnostics controls_allowed_event_detail counted 3 then 4). Back to the
# upstream 200 frames, which covers the 500 ms heartbeat and the 1 s Panda tick with margin.
CONTROLS_ALLOWED_MISMATCH_FRAMES = 200  # 2 s at 100 Hz
CONTROLS_MISMATCH_HISTORY_SECONDS = 5
CONTROLS_MISMATCH_SAMPLE_FRAMES = max(1, int(0.05 / DT_CTRL))  # 20 Hz
PROCESS_NOT_RUNNING_CONSECUTIVE_UPDATES = 3
COMM_ISSUE_CONSECUTIVE_FRAMES = max(1, int(0.30 / DT_CTRL))
# paramsd sets liveParameters.valid from its own SubMaster.all_checks(), so its 90%-of-nominal
# average-rate gate on liveLocationKalman -- 55.6 ms against a measured 55.6-58.2 ms -- turns
# every locationd hiccup into an invalid message. Every commIssue logged on the EON names this
# one service and nothing else (invalid=["liveParameters"], not_alive=[], can_error=false),
# and 0cdbe516 already relaxed the same gate one process downstream. What liveParameters
# carries is steer ratio and angle offset: slowly-varying estimates that consumers hold the
# last value of, and whose staleness is separately and correctly reported as
# vehicleModelInvalid below. It still has to be alive; only its payload validity stops
# raising commIssue.
COMM_ISSUE_OPTIONAL_VALIDITY = ('liveParameters',)
COMMA_PEDAL_PARAM_REFRESH_FRAMES = max(1, int(0.2 / DT_CTRL))
# The AUTO pedal-profile learner runs at 20 Hz: driver acceleration episodes
# last seconds, and this keeps it to a few comparisons every fifth frame.
COMMA_PEDAL_AUTO_FRAMES = 5

# Warn while the lateral controller is still out of authority, instead of after
# the car has already left the path. latcontrol's sat_count filter already
# requires steerLimitTimer (0.4 s on GM) of continuous full-command steering;
# this is the additional hold applied on top of it before prompting, so the
# total is about 0.7 s. Lower it toward 0.0 for the earliest possible prompt.
STEER_SAT_WARN_HOLD_S = 0.3
# Saturation must stay clear this long before the hold is re-armed, so a corner
# that briefly unloads does not restart the countdown from zero.
STEER_SAT_CLEAR_S = 0.5
# Path deviation that still warrants an immediate prompt with no extra hold.
STEER_SAT_DEVIATION_M = 0.20
# The sustained-saturation prompt also needs the car to be turning less than
# asked by this much (m/s^2). Full command is not lack of authority by itself:
# when a driver starts a turn from a stop with the wheel at 130-360 deg, the
# controller hits 100% unwinding it and tracks the plan within 0.03-0.13 m/s^2.
# On 2026-09-19--08-58-29 six episodes met the hold rule; the three real corners
# were 0.21, 0.27 and 0.30 m/s^2 short at the moment the prompt fired, the three
# that were not (+791 s, +1273 s, +1447 s) were 0.13, 0.03 and 0.06. A measured
# path deviation still raises the prompt without this.
STEER_SAT_MIN_SHORTFALL = 0.15
# While the steering is pinned at full command the pedal is taken off, so the
# car does not add speed to a corner it is already under-turning. Speed raises
# the demand (lat accel = v^2 * curvature) while the command has nothing left to
# answer it with. On 2026-09-19--08-58-29 at +1263 s the steering sat at 86 deg
# for 8 s under a 2.2-3.4 m/s^2 demand, and openpilot itself commanded +2.0 m/s^2
# through it, taking the car from 28 to 35 km/h: the curve slowdown is off below
# CURVE_SLOWDOWN_MIN_SPEED_KPH, so nothing else was holding the speed. Replaying
# that corner with the pedal released for the saturated stretch cut the peak
# under-turn from 0.50 to 0.28 m/s^2 (0.48 to 0.38 if full torque holds the same
# curvature at every speed, 0.77 to 0.30 if it holds more at lower speed).
#
# This only removes pedal, the way predictive coasting does; it sets no target
# speed and asks for no deceleration, which this car could not deliver anyway.
# The saturation flag is latcontrol's own, so it already needs 0.4 s of full
# command, hands off, above 10 km/h. The hold outlasts it by this long so a
# corner that briefly unloads does not bring the pedal back and drop it again.
STEER_SAT_ACCEL_HOLD_S = 1.5
# One step-timing line per window. Ten seconds is long enough to average out a
# single slow frame and rare enough that the write cannot matter.
STEP_TIMING_WINDOW_FRAMES = max(1, int(10.0 / DT_CTRL))
# The control loop targets 100 Hz. Ratekeeper.lagging only trips below 90 Hz,
# which is far past the point where the loop has started shedding rate -- a
# drive that averaged 96.1 Hz never tripped it. Log below this instead.
CONTROL_LOOP_MIN_HZ = 99.0
# The control core is only sampled at 1 Hz and a line is written at most this
# often: when the loop is off-rate it stays off-rate, and one line per episode
# is what makes the log readable afterwards.
CONTROL_CORE_CHECK_FRAMES = max(1, int(1.0 / DT_CTRL))
CONTROL_CORE_DIAG_PERIOD_S = 10.0
# Ticks per second behind /proc/self/stat's utime and stime. 100 here, so a
# one second window resolves this process's own CPU share to 1%.
try:
    CLK_TCK = os.sysconf("SC_CLK_TCK") or 100
except (ValueError, OSError, AttributeError):
    CLK_TCK = 100
# Curve slowdown commands this fraction of the speed the curve was entered at.
# The curvature profile still decides whether a bend is worth acting on; it no
# longer decides how much speed comes off.
CURVE_ENTRY_SPEED_FACTOR = 0.9
# Speed the curve slowdown engages from. Deliberately not CP.minSteerSpeed:
# that gates LKAS torque at 10 km/h, and a curve slowdown that engages there
# spent 37% of its active time under 30 km/h on the 2026-09-08--05-34-00 drive,
# doing nothing.
#
# Derived rather than written down, because the only speeds worth engaging at
# are the ones where the target clears the floor: below MIN_CURVE_SPEED /
# CURVE_ENTRY_SPEED_FACTOR the entry-speed target lands under MIN_CURVE_SPEED
# and is clamped back up to it, so the slowdown engages and then asks for no
# reduction at all. At the current 30 km/h floor and 0.9 factor that is
# 33.3 km/h; keeping it derived means changing either one cannot reopen that
# dead band.
CURVE_SLOWDOWN_MIN_SPEED_KPH = MIN_CURVE_SPEED * CV.MS_TO_KPH / CURVE_ENTRY_SPEED_FACTOR
# Confirmation state survives to here, so speed noise at the gate cannot keep
# resetting it while the car sits just below.
CURVE_SLOWDOWN_RELEASE_KPH = CURVE_SLOWDOWN_MIN_SPEED_KPH - 1.0
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


def _read_own_cpu_seconds():
    """utime + stime for this process, in seconds, or None if /proc will not read.

    Costs about 87us on the EON, which is why it is affordable once a second
    inside the control loop.
    """
    try:
        with open("/proc/self/stat", "rb") as f:
            # Everything after the ")" that closes comm is positional, so the
            # process name cannot shift the fields no matter what it contains.
            fields = f.read().rsplit(b")", 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / CLK_TCK
    except (OSError, ValueError, IndexError):
        return None


def _read_own_preemptions():
    """Involuntary context switches for this process, or None if unreadable.

    Only /proc/self/status carries this, and it costs about 220us against the
    87us of /proc/self/stat, with spikes past 4ms -- the kernel formats some
    fifty lines to answer. Read it sparingly.
    """
    try:
        with open("/proc/self/status", "rb") as f:
            for line in f:
                if line.startswith(b"nonvoluntary_ctxt_switches:"):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return None


class Controls:
    # The step's phases in the order it runs them, so the emitted dicts read
    # the way the loop does.
    STEP_NAMES = ("wait", "events", "transition", "control", "publish", "buttons")
    # publish_logs is the largest phase of the step, so it carries a breakdown
    # of its own: the CarControl and HUD fill, the CI.apply that puts the
    # frame on CAN, the 75-field controlsState build, its send, and the
    # remaining message copies -- carState above all, which deep-copies the
    # whole struct into a fresh message every frame.
    PUBLISH_NAMES = ("cc", "apply", "cs_fill", "cs_send", "msgs")

    # data_sample ("wait"), split. Only drain is slack: the blocking receive
    # of boardd's CAN frame. carstate (CI.update: CAN parse and CarState
    # build) and sm (SubMaster poll) are CPU work the "wait" label hid -- on
    # 2026-11-26 the main thread used 7.9 ms a frame against a work_ms of 6.1.
    SAMPLE_NAMES = ("drain", "carstate", "sm", "rest")

    # update_events, split the same way. It is 1.39 ms and 20% of the loop's
    # work, and the only phase that does not move with contention, so what
    # comes out of here is real. setup is the clear and the two add_from_msg
    # calls; device is deviceState, calibration and the lane-change checks;
    # mismatch is the panda safety block and its episode recorder; health is
    # the HW/system checks, liveParameters and locationd; rest is FCW onward.
    EVENT_NAMES = ("setup", "device", "mismatch", "health", "rest")

    # state_control, split the same way. At 2.3 ms it is the largest phase of
    # the loop, and the 2026-09-11 drive had it 81 us a frame slower than
    # 2026-09-10 at equal preemption with no change to its code. setup is the
    # vehicle model, ntune torque, CarControl and the stop-accel latch; long is
    # the accel PID loop and the lead picture; coast is predictive coasting and
    # the natural-decel learner; lat is the lag-adjusted curvature and
    # LaC.update; rest is the pedal profile, the saturation prompt and the NaN
    # guard.
    CONTROL_NAMES = ("setup", "long", "coast", "lat", "rest")

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
                ignore_avg_freq += ['lateralPlan', 'dynamicFollowData']
                # The vision chain -- camerad -> modeld -> locationd -> paramsd --
                # runs off one clock, so every road-camera frame the model overruns
                # is lost to all four at once. With the big supercombo the model
                # takes 31.9 ms of the 50 ms frame budget (20.7 ms before it), so on
                # 2026-09-05--01-13-56 3.2% of frames overran and the drop rate rose
                # from 0.65% cold to 5.30% at 75 C. That is enough to pull the
                # 100-sample average under 18 Hz and soft-disable on commIssue,
                # while the guard actually built for this -- modeldLagging at
                # frameDropPerc > 20 -- correctly reads the same drive as healthy.
                # Drop the blunt average-rate gate for these four and keep the
                # graded one. cameraMalfunction (all_alive on the camera packets)
                # and the alive/valid checks are untouched.
                ignore_avg_freq += ['modelV2', 'roadCameraState',
                                    'liveLocationKalman', 'liveParameters']
            # No driverMonitoringState: dmonitoringmodeld and dmonitoringd are
            # disabled in process_config, and a subscription with no publisher
            # would fail the alive check and block engagement on commIssue.
            self.sm = messaging.SubMaster(
                ['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman', 'dynamicFollowData',
                 'managerState', 'liveParameters', 'radarState'] + self.camera_packets + joystick_packet,
                ignore_alive=ignore, ignore_avg_freq=ignore_avg_freq)

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
        # The 59 deg model input + 0.9-style lateral MPC (always on since the
        # WideOptechLateral toggle was removed, see lateral_planner.py) moves the
        # demand more on straights; this smooths those small moves.
        self.straight_wobble_filter = StraightWobbleFilter()

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

        # publish_logs picks the lateralControlState union member from these
        # every frame. Neither can change for a car that is already fingerprinted,
        # and a capnp read costs more than the write it guards. pcmCruise is
        # deliberately not cached here -- state_transition() reassigns it.
        self._lat_tuning = self.CP.lateralTuning.which()
        self._steer_control_type_angle = (
          self.CP.steerControlType == car.CarParams.SteerControlType.angle)

        self.initialized = False
        self.state = State.disabled
        self.enabled = False
        self.active = False
        self.can_rcv_error = False
        self.soft_disable_timer = 0
        self.v_cruise_kph = 255
        self.v_cruise_kph_last = 0
        self.max_speed_clu = 0.
        self.speed_bump_active = False
        self.speed_bump_released = False
        self.atc_low_active = False
        self.curve_speed_ms = 0.
        self.curve_speed_limiter = CurveSpeedLimiter()
        # Speed the current curve was entered at, or None between curves.
        self._curve_entry_speed_ms = None
        # (modelV2 rcv_frame, speed tuning band, built profile). See
        # _model_curve_profile() for why those two keys are sufficient.
        self._curve_profile_cache = None
        self._curve_model_ticks = {}  # modelV2 logMonoTime -> (v_ego, curvature, v_cruise_kph)
        self.curve_pedal_coordinator = CurvePedalCoordinator(DT_CTRL)
        self.predictive_coasting = PredictiveCoastingCoordinator(DT_CTRL)
        self.predictive_coast_pedal_scale = 1.0
        self.pre_corner_coast = PreCornerCoast()
        self.pre_corner_lift_last = False
        self.corner_lift_speed_hold = CornerLiftSpeedHold()
        self.speed_limit_coast_active = False
        self.speed_limit_coast_target_ms = 0.0
        self.speed_limit_coast_distance_m = math.inf
        self.curve_plan_speed_ms = CURVE_SPEED_DISABLED
        self.curve_pedal_raw_accel = 0.0
        self.curve_pedal_final_accel = 0.0
        self.is_curv_driving = False
        # Corner-entry prompt, driven from the model curve profile in
        # cal_curve_speed and raised as an event in state_control.
        self.corner_alert = CornerAlert()
        self.atc_lane_guide = AtcLaneGuide()
        self.atc_lane_prompt = BLINKER_NONE
        # HUD traffic light guessed from the model's plan (traffic_light.py)
        self.traffic_light = TrafficLight()
        self._atc_rls = None
        self._atc_type = ""
        # modelV2 changes at 20 Hz; what the launch latch reads from it is
        # worked out once per model frame, not on every 100 Hz control frame.
        self._launch_lead_accel = 0.0
        self._model_go_plan_speed = float('nan')
        self.curv_speed = 0.0
        self.v_cruise_kph_limit = 0
        self.applyMaxSpeed = 0
        self.roadLimitSpeedActive = 0
        self.roadLimitSpeed = 0
        self.roadLimitSpeedLeftDist = 0

        # Seeded here so the first publish_logs frame has values without a
        # filesystem read; manager guarantees these keys exist at every start.
        # The Dynamic Follow menus are gone (follow_gap.py has one gap and no
        # settings). These keep their controlsState fields filled: no profile
        # name, the low-speed floor, and a neutral multiplier.
        self.dynamic_tr_min = 0.8
        self.dynamic_tr_mode = ''
        self.dynamic_tr_global_df_mod = 1.0

        self.slow_on_curves = Params().get_bool('SccSmootherSlowOnCurves')
        self.min_set_speed_clu = self.kph_to_clu(MIN_SET_SPEED_KPH)
        self.max_set_speed_clu = self.kph_to_clu(MAX_SET_SPEED_KPH)

        self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
        self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

        self.slowing_down = False
        self.slowing_down_alert = False
        self.slowing_down_sound_alert = False
        # The torque tuning actually in force, for the onroad LatA/Fri readout.
        # Refreshed off the controller in state_control; these are what shows
        # until the first frame gets there.
        self.torque_lat_accel_factor = 0.0
        self.torque_friction = 0.0
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
        self.model_fcw_counter = 0
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
        self.steer_sat_elapsed = 0.0
        self.turn_sat_hold_s = 0.0   # "핸들을 잡아 주세요" under the turn message
        self.turn_desire_side = log.LateralPlan.Desire.none
        self.turn_desire_hold_s = 0.0
        self.steer_sat_clear_elapsed = 0.0
        # Time left on the pedal hold. Set at the end of one control step and read
        # at the start of the next, since the steering result is not known yet
        # when the pedal is decided.
        self.steer_sat_accel_hold_s = 0.0

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
        # Community toggle: a confirmed lead launch may start the car from
        # standstill (stop_accel_boost.py). Read once a second below.
        self.standstill_auto_launch = self._read_standstill_auto_launch()
        # True while that standstill launch is what lets the pedal through
        # below 1 km/h; the CarController checks it.
        self.standstill_launch_active = False
        # Lead picture handed to the interceptor rise-rate limit. Seeded with
        # the no-lead case; only the longitudinal path below refreshes it, so
        # joystick mode never gets a lead-relaxed rise rate.
        self.pedal_rise_lead_valid = False
        self.pedal_rise_lead_distance = 0.0
        self.pedal_rise_lead_rel_speed = 0.0
        self.pedal_rise_desired_tr = PEDAL_RISE_DEFAULT_TR_S
        # Seeded True so that any path which never refreshes it -- joystick mode,
        # a fault before the longitudinal update runs -- keeps the instant drop.
        self.pedal_fall_hard_decel = True
        # Comma-pedal response comes from the user's CommaPedalResistance
        # profile alone. The driving-style learner that used to shade it was
        # removed: on this car openpilot cannot brake at all, so the driver
        # supplies every deceleration, and the brake-based evidence the learner
        # relied on could not tell "openpilot accelerated too hard" from
        # "openpilot was holding speed and I needed to slow". Measured on
        # 2026-09-01--07-13-55, that pushed the gain to 0.85-0.90 with no
        # upward signal available to bring it back.
        # CommaPedalResistance = auto hands the choice to CommaPedalAutoLearner
        # (comma_pedal_auto.py), which steps LOW / MID / HIGH by the driver's
        # gas presses ("more") and brake taps ("less") while openpilot is
        # accelerating. It only ever picks one of the three, and every drive
        # (each controlsd start) begins again from MID with no votes.
        self.comma_pedal_auto = CommaPedalAutoLearner(put_fn=put_nonblocking)
        self.comma_pedal_auto_selected = False
        # Traffic jam (60 s mean speed < 25 km/h): the follow and pedal AUTO
        # learners do not learn from the driver's presses there.
        self.jam_detector = JamDetector()
        self.traffic_jam = False
        self.follow_cap_limit = math.inf
        self.follow_cap_hold = SpeedHoldOffset(DT_CTRL)
        self.comma_pedal_profile = self._read_comma_pedal_profile(params)
        # Brake-minimizing follow control (follow_learner.py): coast
        # deceleration per speed, the lift margin and gap scale from the
        # driver's corrections. The first state seeds the gap scale from the
        # retired Dynamic Follow multiplier so the gap does not jump.
        self.follow_learner = FollowLearner(
          params.get(FOLLOW_LEARN_PARAM), put_fn=put_nonblocking,
          seed_scale=params.get("globalDfMod", encoding="utf8"))
        # Why the pedal sat at zero while the planner asked for accel
        # (pedal_hold_diag.py, /data/log/pedal_hold.jsonl).
        self.pedal_hold_diag = PedalHoldDiag(DT_CTRL)
        # Why the stop-and-go launch floor did or did not reach the pedal
        # (launch_floor_diag.py, /data/log/launch_floor.jsonl).
        self.launch_floor_diag = LaunchFloorDiag(DT_CTRL)
        self.corner_lift_last = False
        self.corner_lift_scale_last = 1.0
        self.comma_pedal_profile_controller = CommaPedalProfileController(
          self.comma_pedal_profile)
        self.comma_pedal_profile_gain = 1.0
        self.comma_pedal_effective_gain = 1.0
        self.comma_pedal_rise_scale = comma_pedal_profile_rise_scale(
          self.comma_pedal_profile)
        self.comma_pedal_profile_changing = False
        self.comma_pedal_raw_command = 0.0
        self.comma_pedal_styled_command = 0.0
        self.comma_pedal_final_command = 0.0

        self.wide_camera = TICI and params.get_bool('EnableWideCamera')
        self.disable_op_fcw = params.get_bool('DisableOpFcw')

        self.limited_lead = False

        # TODO: no longer necessary, aside from process replay
        self.sm['liveParameters'].valid = True

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
        self._control_core_diag_t = 0.0
        self._own_cpu_prev = None
        self._preempt_prev = None
        self._step_acc = [0.0] * len(self.STEP_NAMES)
        self._step_max = [0.0] * len(self.STEP_NAMES)
        self._step_frames = 0
        self._step_window_t = sec_since_boot()
        self._pub_acc = [0.0] * len(self.PUBLISH_NAMES)
        self._pub_max = [0.0] * len(self.PUBLISH_NAMES)
        self._ev_acc = [0.0] * len(self.EVENT_NAMES)
        self._ev_max = [0.0] * len(self.EVENT_NAMES)
        self._ctl_acc = [0.0] * len(self.CONTROL_NAMES)
        self._ctl_max = [0.0] * len(self.CONTROL_NAMES)
        self._smp_acc = [0.0] * len(self.SAMPLE_NAMES)
        self._smp_max = [0.0] * len(self.SAMPLE_NAMES)
        self._smp_marks = (0.0, 0.0, 0.0)

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

    MISMATCH_SNAPSHOT_SERVICES = ("pandaStates", "deviceState", "managerState")

    def _controls_mismatch_capture(self, panda_safety_matches, include_manager_processes=False):
        """The raw inputs of one mismatch snapshot, taken at 20 Hz.

        Turning them into the snapshot dict (_controls_mismatch_snapshot_from)
        is the expensive part -- a few dozen capnp reads and conversions -- and
        the dicts are only read when a mismatch episode is written, which has
        not happened since 2026-09-12. So the 20 Hz history keeps these tuples,
        and the message readers in them, and builds the dicts on that frame.
        controls_ready is self.initialized: controlsd sets the ControlsReady
        param itself when it initializes, and reading it back hit the
        filesystem on every sample.
        """
        sm = self.sm
        return (sm.frame, sec_since_boot(), self.enabled, self.active, self.initialized,
                self.panda_safety_ready, self.panda_safety_match_counter, panda_safety_matches,
                self.mismatch_counter, self.can_rcv_error, self.can_rcv_error_counter,
                sm['pandaStates'], sm['deviceState'], sm['peripheralState'], sm['managerState'],
                tuple((sm.valid[s], sm.alive[s], sm.freq_ok[s], sm.updated[s], sm.rcv_time[s],
                       sm.rcv_frame[s], sm.logMonoTime[s]) for s in self.MISMATCH_SNAPSHOT_SERVICES),
                include_manager_processes)

    def _controls_mismatch_snapshot_from(self, capture):
        (frame, now, enabled, active, controls_ready, panda_safety_ready, panda_safety_match_counter,
         panda_safety_matches, mismatch_counter, can_rcv_error, can_rcv_error_counter,
         panda_states, device_state, peripheral_state, manager_state, services,
         include_manager_processes) = capture
        snapshot = {
            "frame": int(frame),
            "started": bool(device_state.started),
            "enabled": bool(enabled),
            "active": bool(active),
            "controls_ready": bool(controls_ready),
            "panda_safety_ready": bool(panda_safety_ready),
            "panda_safety_match_counter": int(panda_safety_match_counter),
            "panda_safety_matches": bool(panda_safety_matches),
            "controls_allowed_mismatch_counter": int(mismatch_counter),
            "can_receive_error": bool(can_rcv_error),
            "can_receive_error_counter": int(can_rcv_error_counter),
            "charging_disabled": bool(device_state.chargingDisabled),
            "panda_count": len(panda_states),
            "peripheral_usb_power_mode": self._diagnostic_enum_value(
                peripheral_state.usbPowerMode),
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
            "services": {service: {
                "valid": bool(valid),
                "alive": bool(alive),
                "frequency_ok": bool(freq_ok),
                "updated": bool(updated),
                "receive_age_s": max(0.0, now - rcv_time) if rcv_time else None,
                "receive_frame": int(rcv_frame),
                "log_mono_time": int(log_mono_time),
            } for service, (valid, alive, freq_ok, updated, rcv_time, rcv_frame, log_mono_time)
              in zip(self.MISMATCH_SNAPSHOT_SERVICES, services)},
        }
        if include_manager_processes:
            snapshot["manager_processes"] = [{
                "name": process.name,
                "running": bool(process.running),
                "should_be_running": bool(process.shouldBeRunning),
                "pid": int(process.pid),
                "exit_code": int(process.exitCode),
            } for process in manager_state.processes]
        return snapshot

    def _controls_mismatch_snapshot(self, panda_safety_matches, include_manager_processes=False):
        return self._controls_mismatch_snapshot_from(
          self._controls_mismatch_capture(panda_safety_matches, include_manager_processes))

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
            trigger=self._controls_mismatch_snapshot_from(snapshot) if snapshot is not None else None,
            history=[self._controls_mismatch_snapshot_from(c) for c in self.controls_mismatch_history],
        )

    def _read_comma_pedal_profile(self, params):
        """The profile CommaPedalResistance selects, resolving AUTO to the
        learner's current pick."""
        raw = (params.get("CommaPedalResistance", encoding="utf8") or 'mid').strip().lower()
        auto = raw == 'auto'
        self.comma_pedal_auto_selected = auto
        return self.comma_pedal_auto.profile if auto else normalize_comma_pedal_profile(raw)

    def reset(self):
        self.max_speed_clu = 0.
        self.speed_bump_active = False
        self.atc_low_active = False
        self.curve_speed_ms = 0.
        self._curve_entry_speed_ms = None
        self.curve_speed_limiter.reset()
        self.curve_pedal_coordinator.reset()
        self.predictive_coasting.reset()
        self.predictive_coast_pedal_scale = 1.0
        self.steer_sat_accel_hold_s = 0.0
        self.pre_corner_coast.reset()
        self.corner_lift_speed_hold.reset()
        self.speed_limit_coast_active = False
        self.speed_limit_coast_target_ms = 0.0
        self.speed_limit_coast_distance_m = math.inf
        self.curve_plan_speed_ms = CURVE_SPEED_DISABLED
        self.curve_pedal_raw_accel = 0.0
        self.curve_pedal_final_accel = 0.0
        self.slowing_down = False
        self.slowing_down_alert = False
        self.slowing_down_sound_alert = False

    def _model_curve_profile(self, sm, v_ego, measured_curvature, control_min_speed_kph):
        """Build the model curve profile at most once per modelV2 frame.

        modelV2 and the lateralPlan derived from it land on different control
        ticks, so this used to run at 40 Hz and throw the second copy away.
        Everything in the profile is fixed by the model frame except two live
        inputs: the measured curvature, which occupies index 0 and nothing else,
        and v_ego, which acts only through the speed tuning band.
        """
        band = model_curve_tuning_band(v_ego)
        cache = self._curve_profile_cache

        # plannerd builds the same profile once per model frame and sends it in
        # lateralPlan (~1 ms of Python a model frame that used to land on this
        # loop on core 3). It is used whenever it was built for this speed
        # band; otherwise -- band boundary, or an older plannerd -- build here.
        lp = sm['lateralPlan']
        if sm.valid['lateralPlan'] and lp.curveProfileBuilt and lp.curveProfileBand == band:
            key = ('lp', sm.logMonoTime['lateralPlan'])
            if cache is None or cache[0] != key or cache[1] != band:
                valid = bool(lp.curveProfileValid)
                cache = (key, band, (list(lp.curveProfileCurvatures), list(lp.curveProfileTimes),
                                     list(lp.curveProfileDistances), valid,
                                     model_curve_band_diag(band, valid)))
                self._curve_profile_cache = cache
        rcv_frame = ('md', sm.rcv_frame['modelV2'])
        if cache is not None and cache[1] == band and (cache[0] == rcv_frame or cache[0][0] == 'lp'
                                                       and cache[0][1] == sm.logMonoTime['lateralPlan']):
            curvatures, times, distances, valid, diag = cache[2]
            if curvatures:
                measured = abs(float(measured_curvature)) if np.isfinite(measured_curvature) else 0.0
                curvatures = [measured] + curvatures[1:]
            # Diagnostic only -- the real speed gate is applied by the caller --
            # but it reads off the live speed, not the band.
            diag["model_profile_control_allowed"] = bool(
              float(v_ego) * CV.MS_TO_KPH >= float(control_min_speed_kph))
            return curvatures, times, distances, valid, diag

        model = sm['modelV2']
        result = build_v0813_model_curve_profile(
          model.position.t,
          model.orientationRate.z,
          model.velocity.x, model.velocity.y, model.velocity.z,
          model.position.x, model.position.y, model.position.z,
          measured_curvature, v_ego=v_ego,
          control_min_speed_kph=control_min_speed_kph)
        self._curve_profile_cache = (rcv_frame, band, result)
        return result

    def _curve_profile_from_planner(self, sm):
        """True when this model frame's curve profile comes from plannerd."""
        return bool(sm.valid['lateralPlan'] and sm['lateralPlan'].curveProfileBuilt)

    def cal_curve_speed(self, sm, v_ego, frame, measured_curvature):
        lateralPlan = sm['lateralPlan']
        if not self.slow_on_curves:
            # No curve profile is built below, so the prompt has nothing to
            # stand on and must not hold its last answer.
            self.corner_alert.reset()
            self.curve_speed_limiter.reset()
            self.curve_pedal_coordinator.reset()
            self.curve_speed_ms = CURVE_SPEED_DISABLED
            self.curve_plan_speed_ms = CURVE_SPEED_DISABLED
            return

        # modelV2 and lateralPlan are produced at 20 Hz while controlsd runs at
        # 100 Hz, and they land on different control ticks, so this runs at
        # 40 Hz. The profile itself is built once per model frame regardless --
        # see _model_curve_profile() -- and the confirmation below still counts
        # only fresh model evidence.
        lateral_plan_updated = bool(sm.updated['lateralPlan'])
        from_planner = self._curve_profile_from_planner(sm)
        if sm.updated['modelV2']:
            # What each modelV2 tick saw, by model frame, for the replay below.
            # Kept for a few frames: when plannerd runs late the next modelV2
            # can land before the lateralPlan of the one before it.
            self._curve_model_ticks[sm.logMonoTime['modelV2']] = (
              v_ego, measured_curvature, self.v_cruise_kph)
            while len(self._curve_model_ticks) > 4:
                self._curve_model_ticks.pop(next(iter(self._curve_model_ticks)))
        if from_planner:
            # The profile comes from plannerd in lateralPlan, ~20 ms after the
            # model frame. Both of that frame's ticks are replayed here when it
            # arrives, in the order they used to come and each with the inputs
            # it used to see: the modelV2 tick (model evidence, or the measured
            # fallback when the model is invalid) with the speed, curvature and
            # cruise speed captured when modelV2 landed, then the lateralPlan
            # tick (the lateralPlan fallback) with the live ones. The
            # limiter's gates and holds were
            # tuned on the car on that sequence, and a speed taken 20 ms later
            # flips borderline decisions at the slowdown's speed gate.
            if not lateral_plan_updated:
                return
            # A model frame this loop never saw as a modelV2 tick (two landed
            # within one control frame and only the newer was read) had no
            # modelV2 tick before either, so only its lateralPlan tick runs.
            model_inputs = self._curve_model_ticks.pop(lateralPlan.modelMonoTime, None)
            plan_tick = (False, True, v_ego, measured_curvature, self.v_cruise_kph)
            ticks = (((True, False) + model_inputs, plan_tick) if model_inputs is not None
                     else (plan_tick,))
        else:
            model_updated = bool(sm.updated['modelV2'])
            if not (model_updated or lateral_plan_updated):
                return
            ticks = ((model_updated, lateral_plan_updated, v_ego, measured_curvature, self.v_cruise_kph),)

        curvature_factor = 0.85 * ntune_scc_get("sccCurvatureFactor")
        for tick_model, tick_plan, tick_v, tick_curv, tick_cruise_kph in ticks:
            # CP.minSteerSpeed is the LKAS torque gate. It reaches the profile only
            # as the model_profile_control_allowed diagnostic; what gates the
            # slowdown itself is CURVE_SLOWDOWN_MIN_SPEED_KPH, below.
            (model_curvatures, model_times, model_distances,
             model_profile_valid, model_profile_diag) = self._model_curve_profile(
              sm, tick_v, tick_curv, float(self.CP.minSteerSpeed) * CV.MS_TO_KPH)

            # The prompt and the speed limiter read the same profile and each used
            # to validate and smooth it for itself. Do it once here and hand both
            # the result. Only worth it above the lower of their two speed gates --
            # below that neither gets as far as looking at it, and preparing it
            # would be new work on the frames that had none.
            profile_prepared = None
            if (model_profile_valid and
                    float(tick_v) * CV.MS_TO_KPH >= CORNER_ALERT_MIN_SPEED_KPH):
                profile_prepared = prepare_profile(
                  model_curvatures, time_idxs=model_times, distances=model_distances,
                  trusted=True)

            self._curve_tick(sm, tick_v, tick_curv, lateralPlan, tick_cruise_kph * CV.KPH_TO_MS,
                             curvature_factor, model_curvatures, model_times, model_distances,
                             model_profile_valid, model_profile_diag, profile_prepared,
                             tick_model, tick_plan)

    def _curve_tick(self, sm, v_ego, measured_curvature, lateralPlan, cruise_speed_ms,
                    curvature_factor, model_curvatures, model_times, model_distances,
                    model_profile_valid, model_profile_diag, profile_prepared,
                    model_updated, lateral_plan_updated):
        """One control tick of cal_curve_speed: the corner prompt and the limiter."""
        # Corner-entry prompt, decided here on the model profile rather than on
        # whether the curve slowdown engaged. The two are different questions:
        # the slowdown engages on any bend it can shave speed for, while the
        # prompt is for a corner the driver has to brake into themselves. Held
        # on self because state_control raises the event, one loop below.
        # Once per model frame, on its modelV2 tick only. It used to run on the
        # lateralPlan tick as well, feeding the same frame twice: two confirm
        # frames then took one model frame, so a single-frame spike of the
        # model's prediction over the gate raised it, and the 2.5 s hold ran
        # out in 1.25 s. Replayed over the 130 segments of 2026-11-24 the
        # double count raised 26 prompts against 20, and all six extra were
        # corners at 1.2-1.8 m/s^2 whose prediction flickered over 2.0 for
        # one frame at a time; the 20 real ones rise 0.05 s later (median).
        if model_updated:
            if model_profile_valid:
                self.corner_alert.update(model_curvatures, v_ego,
                                         distances=model_distances,
                                         time_idxs=model_times,
                                         curvature_factor=curvature_factor,
                                         prepared=profile_prepared)
            else:
                # Momentary model dropout, not a straight road -- let the hold run out.
                self.corner_alert.hold()

        # Coming off the throttle for a bend is a different question from
        # whether LKAS can steer, so this gate is its own number rather than
        # CP.minSteerSpeed's 10 km/h. Below CURVE_SLOWDOWN_MIN_SPEED_KPH the
        # adapter still runs for diagnostics but commands nothing. Confirmation
        # state is retained down to the release speed, so noise at the gate
        # cannot keep wiping it.
        v_ego_kph = float(v_ego) * CV.MS_TO_KPH
        if v_ego_kph < CURVE_SLOWDOWN_RELEASE_KPH:
            self.curve_speed_limiter.reset()

        if v_ego_kph < CURVE_SLOWDOWN_MIN_SPEED_KPH:
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
          # The fallbacks below hand over lateralPlan curvatures or a single
          # measured one, which are not what was prepared above.
          "prepared": profile_prepared if source.startswith("modelV2") else None,
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

    def _curve_entry_target(self, curve_detected, v_ego, physics_target_ms):
        """Curve target as a fixed fraction of the speed the curve was entered at.

        Latched on the first detected frame. Taking CURVE_ENTRY_SPEED_FACTOR of
        the live speed every frame instead would ratchet: each new target is
        below the speed it was derived from, so the car would keep slowing for
        as long as the curve lasted rather than settling.

        MIN_CURVE_SPEED still floors the result, so entering below about
        33 km/h asks for no reduction at all -- the floor is already at or
        above 90% of that entry speed.
        """
        if not curve_detected:
            self._curve_entry_speed_ms = None
            return physics_target_ms
        if self._curve_entry_speed_ms is None:
            self._curve_entry_speed_ms = float(v_ego)
        return max(self._curve_entry_speed_ms * CURVE_ENTRY_SPEED_FACTOR, MIN_CURVE_SPEED)

    # [크루즈 MAX 속도 설정] #
    def cal_max_speed(self, frame: int, vEgo, sm, CS, measured_curvature):

        road_speed_limiter = get_road_speed_limiter()
        # CS.vEgo is m/s; RoadSpeedLimiter and max_speed_clu use the active
        # cluster unit (km/h or mph).
        v_ego_clu = vEgo * self.speed_conv_to_clu

        apply_limit_speed, road_limit_speed, left_dist, first_started, max_speed_log = \
            road_speed_limiter_get_max_speed(v_ego_clu, self.is_metric)
        # 방지턱 감속 중: 방지턱 목표(BUMP_SPEED_KPH)를 V_CRUISE_MIN 예외로 받고, 필터 없이 바로 반영한다.
        was_bump_active = self.speed_bump_active
        self.speed_bump_active = bool(road_speed_limiter.speed_bump_active)
        # 방지턱을 지나 제한이 풀린 순간: 원래 속도로 즉시 돌아간다 (update_max_speed 참고).
        self.speed_bump_released = bool(was_bump_active and not self.speed_bump_active)
        # ATC 교차로 회전(15km/h): 방지턱처럼 V_CRUISE_MIN 아래 제한을 허용한다. 필터는 그대로 탄다.
        self.atc_low_active = bool(getattr(road_speed_limiter, 'atc_low_active', False))

        # print("apply_limit_speed : ", apply_limit_speed)
        # print("road_limit_speed : ", road_limit_speed)
        # print("left_dist : ", left_dist)
        # print("first_started : ", first_started)
        # print("max_speed_log : ", max_speed_log)

        curv_limit = 0
        self.cal_curve_speed(sm, vEgo, frame, measured_curvature)
        cruise_speed_ms = self.v_cruise_kph * CV.KPH_TO_MS
        if self.CP.enableGasInterceptor:
            # Read, not copied: last_diag is only rebuilt inside cal_curve_speed,
            # which returns early unless the model or the lateral plan updated,
            # so this dict changes 20 times a second and was being duplicated
            # 100. Nothing here mutates it.
            curve_diag = self.curve_speed_limiter.last_diag
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
              self._curve_entry_target(curve_detected, vEgo, raw_curve_speed_ms),
              selected_time_s=curve_diag.get("selected_time_s", None))
            self.curve_plan_speed_ms = (CURVE_SPEED_DISABLED if curve_plan_speed_ms is None
                                        else float(curve_plan_speed_ms))
        else:
            self.curve_plan_speed_ms = self._curve_entry_target(
              MIN_CURVE_SPEED <= self.curve_speed_ms < CURVE_SPEED_DISABLED,
              vEgo, self.curve_speed_ms)

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
        # Show the target actually being commanded. It used to report
        # MIN_CURVE_SPEED unconditionally, so the indicator read 30.0 km/h for
        # every bend however gentle; the target now varies with entry speed.
        self.curv_speed = (float(self.curve_plan_speed_ms) * CV.MS_TO_KPH
                           if self.is_curv_driving else 0.0)

        if road_speed_limiter.roadLimitSpeed is not None:
            camSpeedFactor = clip(road_speed_limiter.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)
            self.over_speed_limit = road_speed_limiter.roadLimitSpeed.camLimitSpeedLeftDist > 0 and \
                                    0 < road_limit_speed * camSpeedFactor < v_ego_clu + 2
        else:
            self.over_speed_limit = False

        max_speed_log = ""

        if apply_limit_speed >= self.kph_to_clu(V_CRUISE_MIN) or \
                ((self.speed_bump_active or self.atc_low_active) and apply_limit_speed > 0):  # 크루즈 최저 속도보다 큰 경우 설정 (방지턱·ATC 교차로 회전은 예외)

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


        self.update_max_speed(int(max_speed_clu + 0.5), CS,
                              curv_limit != 0 and curv_limit == int(max_speed_clu))
        # print("update_max_speed() value : ", self.max_speed_clu)

        return road_limit_speed, left_dist, max_speed_log

    def update_max_speed(self, max_speed, CS, limited_curv):
        if not CS.cruiseState.enabled or self.max_speed_clu <= 0:
            self.max_speed_clu = max_speed
        elif self.speed_bump_active:
            # 방지턱 프로파일은 이미 거리 기반으로 부드럽게 내려가고, 통과 후에는 플래너가
            # 2km/h/s로 천천히 올린다. 둘 다 이미 다듬어진 값이라 지연 필터를 거치지 않는다
            # (필터를 타면 회복이 1초쯤 더 늦어진다).
            self.max_speed_clu = max_speed
        elif self.speed_bump_released and max_speed > self.max_speed_clu:
            # 방지턱 통과 직후에도 원래 속도로 바로 돌아간다. 예전엔 아래 1초 시상수
            # 필터를 타서 실제 가속이 1초쯤 늦게 시작됐다(2026-11-16 1140.75초 해제 →
            # 1141.75초 가속 시작). 가속 세기는 여전히 플래너 한도와 페달 프로필이 정한다.
            self.max_speed_clu = max_speed
        else:
            kp = 0.02 if limited_curv else 0.01
            error = max_speed - self.max_speed_clu
            self.max_speed_clu = self.max_speed_clu + error * kp

    def update_events(self, CS):
        """Compute carEvents from carState"""
        e0 = sec_since_boot()

        self.events.clear()

        # Add startup event
        if self.startup_event is not None:
            self.events.add(self.startup_event)
            self.startup_event = None

        # Don't add any more events if not initialized
        if not self.initialized:
            self.events.add(EventName.controlsInitializing)
            # Still recorded, so the five phases keep tiling every frame the
            # step timer counts. Everything past setup is empty on this path,
            # which only runs for the first few seconds of a drive.
            e = sec_since_boot()
            self._record_events_timing(e0, e, e, e, e, e)
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

        car_event_names = self.CI.event_names
        if car_event_names is not None:
            self.events.events.extend(car_event_names)
        else:
            self.events.add_from_msg(CS.events)
        e1 = sec_since_boot()

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
        cpus = list(self.sm['deviceState'].cpuUsagePercent)
        # The last core is left out of the driver alert on EON: controlsd is
        # pinned there with RT priority, so it reads 100% for a whole healthy
        # drive and including it would latch the alert on permanently. That
        # core is watched by _record_control_core_load() instead.
        if max(cpus[:(-1 if EON else None)], default=0) > 95 and not SIMULATION:
          self.events.add(EventName.highCpuUsage)
        # REPLAY runs the loop off the wall clock, so its rate says nothing.
        if EON and not SIMULATION and not REPLAY:
            self._record_control_core_load(cpus)

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

        # Navigation (ATC) lane changes are the driver's: the car never moves
        # across for them (desire_helper.py). AtcLaneGuide tells the driver once
        # per instruction, just before the turn or fork point ("좌측/우측 차선으로
        # 이동하세요", one chime); stepped on modelV2 frames for its 20 Hz timers.
        rls = get_road_speed_limiter().roadLimitSpeed
        # roadLimitSpeed comes at 20 Hz: convert its text once per message.
        if rls is not self._atc_rls:
            self._atc_rls = rls
            self._atc_type = str(rls.atcType) if rls is not None else ""
        atc_type = self._atc_type
        if self.sm.updated['modelV2']:
            self.atc_lane_prompt = self.atc_lane_guide.update(
                atc_type, float(rls.xDistToTurn) if rls is not None else 0., CS.vEgo,
                CS.leftBlinker, CS.rightBlinker, md=self.sm['modelV2'],
                turn_info=int(rls.xTurnInfo) if rls is not None else 0)
        if self.atc_lane_prompt == BLINKER_LEFT:
            self.events.add(EventName.atcBlinkerLeft)
        elif self.atc_lane_prompt == BLINKER_RIGHT:
            self.events.add(EventName.atcBlinkerRight)

        # Traffic light guess for the HUD, once per model frame. Display and the
        # green-light chime only; it never acts on the car.
        if self.sm.updated['modelV2']:
            md = self.sm['modelV2']
            pos = md.position
            xs = pos.x
            acc = list(md.acceleration.x)[:11]
            lead = self.sm['radarState'].leadOne
            self.traffic_light.update(
              CS.vEgo, float(xs[len(xs) - 1]) if len(xs) else 1000.,
              sum(acc) / len(acc) if acc else 0., CS.standstill,
              CS.leftBlinker or CS.rightBlinker, CS.steeringAngleDeg,
              bool(lead.status), lead.dRel, lead.vLead,
              max((abs(y) for y in pos.y), default=0.))
            if self.traffic_light.green_event and self.enabled and not CS.gasPressed:
                self.events.add(EventName.trafficLightGreen)

        # Handle lane change. Under an ATC instruction the planner holds in
        # preLaneChange for the whole approach; the guide speaks for it, so the
        # pre-lane-change line ("○측차선으로 차선을 변경합니다") stays off there.
        # "... prepare" is the instruction still too far away: DesireHelper ignores it too.
        atc_active = atc_type.startswith(ATC_LEFT_TYPES + ATC_RIGHT_TYPES) and not atc_type.endswith("prepare")
        lc_state = self.sm['lateralPlan'].laneChangeState
        direction = self.sm['lateralPlan'].laneChangeDirection
        if lc_state == LaneChangeState.preLaneChange:
            if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
                    (CS.rightBlindspot and direction == LaneChangeDirection.right):
                self.events.add(EventName.laneChangeBlocked)
            elif atc_active:
                pass
            elif self.sm['lateralPlan'].autoLaneChangeEnabled and self.sm['lateralPlan'].autoLaneChangeTimer > 0:
                pass  # 2026-11-22, on request: no autoLaneChange countdown alert
            elif direction == LaneChangeDirection.left:
                self.events.add(EventName.preLaneChangeLeft)
            else:
                self.events.add(EventName.preLaneChangeRight)
        elif lc_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
            self.events.add(EventName.laneChange)

        #if not CS.canValid:
        #    self.events.add(EventName.canError)
        e2 = sec_since_boot()

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

        # The snapshot is only consumed by the 20 Hz history sampler and by the
        # first frame of a mismatch episode, so building one every 100 Hz frame
        # threw four out of five away -- along with the ControlsReady param read
        # inside it, which hits the filesystem.
        need_sample = (self.sm.frame - self.controls_mismatch_last_sample_frame
                       >= CONTROLS_MISMATCH_SAMPLE_FRAMES)
        need_episode = bool(controls_mismatch_reasons) and not self.controls_mismatch_active
        controls_mismatch_capture = None
        if need_sample or need_episode:
            controls_mismatch_capture = self._controls_mismatch_capture(
                panda_safety_matches, include_manager_processes=bool(controls_mismatch_reasons))
            self._record_controls_mismatch_snapshot(controls_mismatch_capture)
        self._record_controls_mismatch_episode(controls_mismatch_reasons, controls_mismatch_capture)
        e3 = sec_since_boot()

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
            communication_bad = not controlsd_communication_ok(
                self.sm, optional_validity_services=COMM_ISSUE_OPTIONAL_VALIDITY)
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

        # The lane-confidence watchdog that raised EventName.laneConfidenceLow
        # is gone: it is not wanted on this car. It sustained a dProb < 0.25
        # count and alerted after 8 s with a 30 s cooldown -- tuned on
        # 2026-08-26--12-34-51 to fire only twice in 11.5 min, since brief
        # dropouts are normal (33 of 72 under 0.5 s, median 0.99 s) and a
        # shorter sustain reached ~37/hour. The EventName stays in car.capnp,
        # where removing a field would break replay of older logs, but nothing
        # adds it now and the alert is gone from events.py.
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

        e4 = sec_since_boot()

        # Check for FCW (브레이크! 추돌위험)
        stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
        model_fcw_raw = (self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and
                         not stock_long_is_braking and CS.vEgo >= MODEL_FCW_MIN_SPEED)
        self.model_fcw_counter = self.model_fcw_counter + 1 if model_fcw_raw else 0
        model_fcw = self.model_fcw_counter >= MODEL_FCW_HOLD_FRAMES
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

        # df_manager.update() is not called: its result was discarded, and it
        # cost 144 us a frame on core 3 -- a Params file read plus a SubMaster
        # poll of its own. The planner keeps its own dfManager and uses that.
        self._record_events_timing(e0, e1, e2, e3, e4, sec_since_boot())

    def _record_events_timing(self, e0, e1, e2, e3, e4, e5):
        """Where update_events' 1.4 ms goes, over the same window as the steps.

        Worth splitting because of what the 2026-09-10 drive showed. Every
        other phase moves with contention -- corr(preempted_per_s, publish) is
        +0.93, control +0.92 -- and this one does not: +0.02 over 103 windows.
        It is the only phase whose cost is its own work rather than the core
        being taken away, which makes it the one place where anything cut is
        certain to come back.

        The five phases tile the whole method, including the not-initialized
        early return, so they sum to the "events" step from _record_step_timing
        and can be checked against it.
        """
        acc = self._ev_acc
        mx = self._ev_max
        d0 = e1 - e0
        d1 = e2 - e1
        d2 = e3 - e2
        d3 = e4 - e3
        d4 = e5 - e4
        acc[0] += d0
        acc[1] += d1
        acc[2] += d2
        acc[3] += d3
        acc[4] += d4
        if d0 > mx[0]: mx[0] = d0
        if d1 > mx[1]: mx[1] = d1
        if d2 > mx[2]: mx[2] = d2
        if d3 > mx[3]: mx[3] = d3
        if d4 > mx[4]: mx[4] = d4

    def _record_control_timing(self, c0, c1, c2, c3, c4, c5):
        """Where state_control's 2.3 ms goes, over the same window as the steps.

        The 2026-09-11 drive put control 81 us a frame above 2026-09-10 at
        equal preemption, which cancelled the 117 us that publish gave back,
        and nothing inside state_control was measured, so it could only be
        called situational. The five phases tile the whole method, joystick
        mode included, so they sum to the "control" step.
        """
        acc = self._ctl_acc
        mx = self._ctl_max
        d0 = c1 - c0
        d1 = c2 - c1
        d2 = c3 - c2
        d3 = c4 - c3
        d4 = c5 - c4
        acc[0] += d0
        acc[1] += d1
        acc[2] += d2
        acc[3] += d3
        acc[4] += d4
        if d0 > mx[0]: mx[0] = d0
        if d1 > mx[1]: mx[1] = d1
        if d2 > mx[2]: mx[2] = d2
        if d3 > mx[3]: mx[3] = d3
        if d4 > mx[4]: mx[4] = d4

    def _record_control_core_load(self, cpus):
        """Record the control core when the 100 Hz loop stops making its rate.

        highCpuUsage deliberately ignores this core, so until now nothing
        watched the one core that decides whether control runs on time. Usage
        alone is not a fault there -- an RT-pinned loop is supposed to own its
        core -- so this fires only once the loop has actually lost rate, which
        is the symptom that matters and the one that shows up as a growing
        cumLagMs.

        cpu_pct cannot say why the rate was lost. The control core reads a flat
        100.0% for every drive whatever controlsd does, because rtshield spins
        there at FIFO 1 for the sole purpose of keeping the core out of idle.
        own_cpu_pct and preempted_per_s split the two explanations: a loop that
        is off-rate at close to 100% of its own is doing more work than fits in
        a frame, while one that is off-rate well under that is not being given
        the core -- and on this device the thing that can take it is boardd,
        same core 3, FIFO 54 against this process's 53.
        """
        if self.sm.frame % CONTROL_CORE_CHECK_FRAMES:
            return
        now = sec_since_boot()
        # Sampled before the rate check so the window is always the ~1 s since
        # the last check, never the arbitrary gap since the last line written.
        own_cpu_pct = self._sample_own_cpu(now)
        avg_dt = self.rk.avg_dt
        if avg_dt <= 0.0 or 1.0 / avg_dt >= CONTROL_LOOP_MIN_HZ:
            return
        if now - self._control_core_diag_t < CONTROL_CORE_DIAG_PERIOD_S:
            return
        self._control_core_diag_t = now
        extra = self._sample_preemptions(now)
        if own_cpu_pct is not None:
            extra["own_cpu_pct"] = own_cpu_pct
        append_process_diagnostic(
            "controlsd_loop_lagging",
            loop_hz=round(1.0 / avg_dt, 2),
            avg_frame_ms=round(avg_dt * 1000.0, 3),
            cum_lag_ms=round(-self.rk.remaining * 1000.0, 1),
            control_core_pct=float(cpus[-1]) if cpus else 0.0,
            cpu_pct=[float(c) for c in cpus],
            **extra,
        )

    def _sample_own_cpu(self, now):
        """This process's own CPU share since the last check, in percent."""
        cpu_s = _read_own_cpu_seconds()
        prev = self._own_cpu_prev
        self._own_cpu_prev = (now, cpu_s)
        if cpu_s is None or prev is None or prev[1] is None:
            return None
        dt = now - prev[0]
        if dt <= 0.0:
            return None
        return round(100.0 * (cpu_s - prev[1]) / dt, 1)

    def _sample_preemptions(self, now):
        """Involuntary context switches per second since the last line written.

        Read here rather than every check because /proc/self/status is the
        expensive half of the pair, and this frame is already paying for a
        file append. Voluntary switches are left out: the loop sleeps once a
        frame by design, so only the involuntary ones say anything.
        """
        count = _read_own_preemptions()
        prev = self._preempt_prev
        self._preempt_prev = (now, count)
        if count is None or prev is None or prev[1] is None:
            return {}
        dt = now - prev[0]
        if dt <= 0.0:
            return {}
        # The window spans whatever ran between two lines, including stretches
        # where the loop was healthy, so it is reported alongside the rate.
        return {"preempted_per_s": round((count - prev[1]) / dt, 1),
                "preempt_window_s": round(dt, 1)}

    def data_sample(self):
        """Receive data from sockets and update carState"""

        # Update carState from CAN
        can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
        s_drain = sec_since_boot()
        CS = self.CI.update(self.CC, can_strs)
        s_cs = sec_since_boot()

        self.sm.update(0)
        self._smp_marks = (s_drain, s_cs, sec_since_boot())

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

        c0 = sec_since_boot()
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

            # What the onroad LatA/Fri readout shows. Read back off the
            # controller rather than off ntune so it is the clipped value that
            # is actually steering the car, and cached here so publish_logs
            # stays two float writes.
            fixed = getattr(self.LaC, 'fixed_torque_params', None)
            if fixed is not None:
                self.torque_lat_accel_factor = fixed['latAccelFactor']
                self.torque_friction = fixed['friction']


        lat_plan = self.sm['lateralPlan']
        long_plan = self.sm['longitudinalPlan']
        # Only when the plan itself is new. These three come off lateralPlan at
        # 20 Hz, so re-reading and re-casting them every frame handed the
        # controller the same numbers five times over. The setter stores them,
        # so they stay in effect between plans.
        if self.sm.updated['lateralPlan'] and hasattr(self.LaC, 'set_path_stability'):
            self.LaC.set_path_stability(
              bool(getattr(lat_plan, 'pathStabilityActive', False)),
              float(getattr(lat_plan, 'pathWobbleRangeM', 0.0)),
              int(getattr(lat_plan, 'pathWobbleFlips', 0)))

        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        # CarControl is reconstructed again in publish_logs. Use the same
        # helper in both places so activation flags cannot be dropped between
        # lateral control and the vehicle CarController.
        self._frame_lat_active, _ = apply_control_activation(
          CC, self.active, CS, self.CP, self.events.any(ET.OVERRIDE))

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
        if self.sm.frame % 100 == 0:
            self.standstill_auto_launch = self._read_standstill_auto_launch()
        # The launch floor follows the lead's own acceleration. radard publishes
        # aLeadK = 0 for vision leads, so take it from the model directly.
        if self.sm.updated['modelV2']:
            model_leads = self.sm['modelV2'].leadsV3
            self._launch_lead_accel = (float(model_leads[0].a[0])
                                       if len(model_leads) and model_leads[0].prob > 0.5 and
                                       len(model_leads[0].a) else 0.0)
            self._model_go_plan_speed = self._model_plan_speed(MODEL_GO_PLAN_T_S)
        launch_lead_accel = self._launch_lead_accel
        # nan while modelV2 is invalid, as _model_plan_speed itself returns.
        model_go_plan_speed = self._model_go_plan_speed if self.sm.valid['modelV2'] else float('nan')
        self.stop_accel_boost_active = self.stop_accel_boost_latch.update(
          boost_system_ready,
          dynamic_follow_valid and dynamic_follow.leadCatchupActive,
          CS.vEgo,
          brake_pressed=CS.brakePressed,
          gas_pressed=CS.gasPressed,
          lead_speed=dynamic_follow.leadSpeed if dynamic_follow_valid else 0.0,
          lead_relative_speed=dynamic_follow.leadRelativeSpeed if dynamic_follow_valid else 0.0,
          lead_distance=dynamic_follow.leadDistance if dynamic_follow_valid else 0.0,
          standstill_launch_enabled=self.standstill_auto_launch,
          lead_accel=launch_lead_accel,
          model_plan_speed=model_go_plan_speed)
        self.standstill_launch_active = bool(
          self.stop_accel_boost_active and self.stop_accel_boost_latch.standstill_launch)

        c1 = sec_since_boot()
        if not self.joystick_mode:
            # accel PID loop
            pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_kph * CV.KPH_TO_MS)
            # Follow accel cap (follow_accel_cap.py) on the speed PID's positive
            # limit: the plan alone does not bind, the wound-up integrator does.
            pid_accel_limits = self._apply_follow_accel_cap(CS, pid_accel_limits)
            t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL

            # Only evaluated while the latch would emit a floor; the reasons
            # go to launch_floor_diag.py so a refused floor says why.
            floor_block_reasons = (boost_floor_context_block_reasons(
              can_valid=CS.canValid,
              radar_valid=self.sm.valid['radarState'],
              radar_error=len(self.sm['radarState'].radarErrors) > 0,
              # Driver monitoring is off, so there is no awareness to lose.
              driver_aware=True,
              curv_driving=self.is_curv_driving,
              curve_active=(
                self.curve_pedal_coordinator.engaged or
                self.curve_pedal_coordinator.pedal_intervening),
              curve_target=self.curve_plan_speed_ms,
              speed_limit_active=self.speed_limit_coast_active,
              speed_limit_target=self.speed_limit_coast_target_ms,
              v_ego=CS.vEgo,
              fcw=long_plan.fcw,
              plan_valid=self.sm.valid['longitudinalPlan'],
              plan_age=t_since_plan,
              plan_full=len(long_plan.speeds) == CONTROL_N,
              plan_source=long_plan.longitudinalPlanSource)
              if self.stop_accel_boost_latch.floor_allowed else ())
            boost_floor_context_safe = bool(
              self.stop_accel_boost_latch.floor_allowed and not floor_block_reasons)
            boost_floor_accel = self.stop_accel_boost_latch.update_hill_response(
              boost_floor_context_safe, CS.aEgo)
            self.launch_floor_diag.update(
              self.stop_accel_boost_latch.latched, self.stop_accel_boost_active,
              self.stop_accel_boost_latch.floor_allowed, floor_block_reasons,
              boost_floor_accel, CS.vEgo, CS.aEgo, CS.gasPressed, CS.brakePressed,
              dynamic_follow.leadDistance if dynamic_follow_valid else 0.0,
              dynamic_follow.leadSpeed if dynamic_follow_valid else 0.0,
              str(long_plan.longitudinalPlanSource),
              lead_accel=self.stop_accel_boost_latch.lead_accel,
              match_floor=self.stop_accel_boost_latch.match_floor)
            raw_long_accel = self.LoC.update(
              self.active, CS, long_plan, pid_accel_limits, t_since_plan,
              self.stop_accel_boost_active, boost_floor_accel,
              self.stop_accel_boost_latch.driver_launch_handoff,
              stop_accel_boost_factor=self.stop_accel_boost_latch.boost_factor,
              stop_accel_boost_cap=self.stop_accel_boost_latch.accel_cap)
            lead_one = self.sm['radarState'].leadOne
            radar_valid = bool(self.sm.valid['radarState'] and
                               len(self.sm['radarState'].radarErrors) == 0)
            effective_tr = (dynamic_follow.mpcTR if dynamic_follow_valid else 1.3)
            # The longitudinal controller's own output is the accel. Four pedal
            # recovery assists used to be able to replace it here; they are gone.
            actuators.accel = raw_long_accel

            # Lead picture for the interceptor's rise-rate limit. Published here
            # rather than read again in the CarController so both see the same
            # radar frame the longitudinal decisions above were made on.
            self.pedal_rise_lead_valid = bool(lead_one.status and radar_valid)
            self.pedal_rise_lead_distance = float(
              lead_one.dRel if self.pedal_rise_lead_valid else 0.0)
            self.pedal_rise_lead_rel_speed = float(
              lead_one.vRel if self.pedal_rise_lead_valid else 0.0)
            self.pedal_rise_desired_tr = float(effective_tr)
            # Does the planner want real deceleration, or has it just stopped
            # asking for gas? Both look identical at the interceptor -- the
            # command is clipped at zero either way -- so the fall-rate limit
            # has to be told which one this is before it may soften the lift.
            self.pedal_fall_hard_decel = bool(
              long_plan.fcw or actuators.accel < PEDAL_FALL_HARD_DECEL)

            self.curve_pedal_raw_accel = float(actuators.accel)
            # Curve target shaping remains in the longitudinal plan. Lead,
            # curve, and speed-limit pedal lift are arbitrated once below so
            # independent smoothers cannot multiply each other.
            self.curve_pedal_final_accel = float(actuators.accel)
            c2 = sec_since_boot()

            # Predictive coasting supplies a final 0..1 pedal ceiling so the
            # profile response cannot add back pedal while a lead is consuming
            # the desired gap. It used to be gated on the driving-style learner
            # being enabled as well; with the learner gone it follows the
            # interceptor alone, which is what that gate evaluated to while the
            # learner was on.
            predictive_enabled = bool(self.CP.enableGasInterceptor)
            curve_diag = self.curve_speed_limiter.last_diag
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
              launch_boost_floor_active=bool(
                boost_floor_context_safe and boost_floor_accel > 0.0),
              # Was a learned offset; the learner is gone, so predictive
              # coasting uses its own unshifted low-speed behaviour.
              learned_low_speed_coast_offset_s=0.0,
              # Brake-minimizing lift: lift by what coasting can still shed
              # before the gap reaches the brake-free floor.
              coast_decel_ms2=self.follow_learner.coast_decel(CS.vEgo),
              reach_floor_tr=floor_tr(CS.vEgo),
              reach_margin=self.follow_learner.lift_margin)

            # Steering pinned: take the pedal off (applied below, together with
            # the pre-corner lift). The stopped-lead launch boost owns the pedal
            # outright and is left alone.
            steer_sat_lift = bool(self.steer_sat_accel_hold_s > 0.0)

            # Same lift, but BEFORE the corner: when the lateral plan predicts a
            # demand the 300 LKA torque cannot hold (the EPS faults above it),
            # coast into the corner so the same torque turns the car further.
            # See pre_corner_coast.py for the guards that keep it from firing on
            # straights, lifting too early, or holding after the corner.
            lat_plan_curvatures = lat_plan.curvatures
            pre_corner_lift = self.pre_corner_coast.update(
              plan_updated=bool(self.sm.updated['lateralPlan']),
              control_active=bool(self.active and CC.latActive),
              v_ego=CS.vEgo,
              curvatures=lat_plan_curvatures,
              plan_valid=bool(self.sm.valid['lateralPlan'] and lat_plan.mpcSolutionValid and
                              len(lat_plan_curvatures) == CONTROL_N),
              lane_change_active=bool(
                lat_plan.laneChangeState != log.LateralPlan.LaneChangeState.off),
              gas_pressed=bool(CS.gasPressed),
              brake_pressed=bool(CS.brakePressed),
              dt=DT_CTRL)
            if pre_corner_lift and not self.pre_corner_lift_last:
                cloudlog.event('preCornerCoast', vEgo=float(CS.vEgo),
                               peakLatAccel=float(self.pre_corner_coast.peak))
            self.pre_corner_lift_last = bool(pre_corner_lift)

            # Both lifts used to set the scale to 0 outright, which uphill bled
            # 3-4 km/h a second through the corner. The speed hold starts at 0
            # and gives pedal back only while the car slows harder than a flat
            # coast, so on a grade the car holds its speed instead of losing it.
            # Taking the min keeps it from ever adding pedal.
            corner_lift = bool(predictive_enabled and (steer_sat_lift or pre_corner_lift) and
                               not self.stop_accel_boost_active)
            corner_lift_scale = self.corner_lift_speed_hold.update(corner_lift, CS.aEgo, DT_CTRL)
            self.corner_lift_last = corner_lift
            self.corner_lift_scale_last = corner_lift_scale
            self.predictive_coast_pedal_scale = min(self.predictive_coast_pedal_scale,
                                                    corner_lift_scale)

            # NaturalDecelLearner ran here at 94 us a frame, measured on the
            # device, learning how hard this car slows on its own. Its only
            # reader was the BRAKE prompt's shadow, so it went with the prompt,
            # and the road pitch it needed -- calibratedOrientationNED, the
            # liveCalibration status and the blinker/lane-change context that
            # decided when a sample counted -- went with it.

            if self.corner_alert.active:
                self.events.add(EventName.curveEntry)
            c3 = sec_since_boot()

            # Steering PID loop and lateral MPC
            # lat_active = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
            #             CS.vEgo > self.CP.minSteerSpeed and not CS.standstill \
            #             and abs(CS.steeringAngleDeg) < self.CP.maxSteeringAngleDeg

            self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,
                                                                                   lat_plan.psis,
                                                                                   lat_plan.curvatures,
                                                                                   lat_plan.curvatureRates,
                                                                                   CC.latActive,
                                                                                   self.straight_wobble_filter,
                                                                                   self.sm.updated['lateralPlan'])
            actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, params,
                                                                                   self.last_actuators,
                                                                                   self.steer_limited,
                                                                                   self.desired_curvature,
                                                                                   self.desired_curvature_rate,
                                                                                   self.sm['liveLocationKalman'])
            c4 = sec_since_boot()
        else:
            # Joystick mode is not split: all of it lands in rest.
            c2 = c3 = c4 = c1
            self.predictive_coasting.reset()
            self.pre_corner_coast.reset()
            self.corner_lift_speed_hold.reset()
            # No longitudinal decision was made this frame, so nothing here has
            # established that easing off is what was meant.
            self.pedal_fall_hard_decel = True
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
            self.comma_pedal_profile = self._read_comma_pedal_profile(self.params)
        if self.comma_pedal_auto_selected and self.sm.frame % COMMA_PEDAL_AUTO_FRAMES == 0:
            lead = self.sm['radarState'].leadOne
            lead_headway_s = None
            lead_closing = False
            if lead.status:
                lead_headway_s = float(lead.dRel) / max(float(CS.vEgo), 1.0)
                lead_closing = float(lead.vRel) < -0.5
            self.comma_pedal_auto.update(
              now=sec_since_boot(), dt=COMMA_PEDAL_AUTO_FRAMES * DT_CTRL,
              v_ego=float(CS.vEgo), a_ego=float(CS.aEgo),
              gas_pressed=CS.gasPressed, brake_pressed=CS.brakePressed,
              long_active=bool(self.active and self.CP.enableGasInterceptor),
              op_accel=float(actuators.accel),
              op_hold=float(self.follow_cap_hold.x * hold_weight(CS.vEgo)),
              lead_headway_s=lead_headway_s, lead_closing=lead_closing,
              lat_accel=float(CS.yawRate) * float(CS.vEgo),
              lane_change=lat_plan.laneChangeState != LaneChangeState.off,
              jam=self.traffic_jam)
            # The learner may have just decided; use it from this frame on.
            self.comma_pedal_profile = self.comma_pedal_auto.profile
        if self.CP.enableGasInterceptor and self.sm.frame % COMMA_PEDAL_AUTO_FRAMES == 1:
            self.traffic_jam = self.jam_detector.update(CS.vEgo, COMMA_PEDAL_AUTO_FRAMES * DT_CTRL)
            follow_lead = self.sm['radarState'].leadOne
            self.follow_learner.update(
              now=sec_since_boot(), dt=COMMA_PEDAL_AUTO_FRAMES * DT_CTRL,
              v_ego=float(CS.vEgo), a_ego=float(CS.aEgo),
              pedal_cmd=float(self.last_actuators.gas),
              gas_pressed=CS.gasPressed, brake_pressed=CS.brakePressed,
              long_active=bool(self.active),
              op_accel=float(actuators.accel),
              lat_accel=float(CS.yawRate) * float(CS.vEgo),
              lane_change=lat_plan.laneChangeState != LaneChangeState.off,
              lead_status=bool(follow_lead.status),
              lead_dist=float(follow_lead.dRel) if follow_lead.status else 0.0,
              lead_vrel=float(follow_lead.vRel) if follow_lead.status else 0.0,
              jam=self.traffic_jam)
        pedal_profile_active = bool(
          self.active and self.CP.enableGasInterceptor and
          self.last_actuators.gas > 0.001 and
          not CS.gasPressed and not CS.brakePressed)
        self.comma_pedal_profile_gain = self.comma_pedal_profile_controller.update(
          self.comma_pedal_profile, CS.vEgo, pedal_profile_active, DT_CTRL)
        self.comma_pedal_profile_changing = bool(
          self.comma_pedal_profile_controller.changing)

        # The profile used to be applied twice -- once on the planner's cruise
        # acceleration ceiling and again as a multiplier on the pedal command --
        # so its authority swung between 0%, 18% and 39% depending on which of
        # the two happened to bind. The pedal-command copy is gone: it sat
        # inside the speed PID loop, which absorbed it (a higher pedal makes the
        # car reach the planned speed sooner, so the PID simply asks for less).
        # What is left is the ceiling, which sets how much acceleration may be
        # planned, and the rise scale below, which sets how fast the pedal is
        # allowed to get there.
        self.comma_pedal_effective_gain = float(self.comma_pedal_profile_gain)
        self.comma_pedal_rise_scale = comma_pedal_profile_rise_scale(
          self.comma_pedal_profile)

        # Steering-authority prompt (조향 제어 초과).
        # A tight city corner saturates the steering command for seconds before
        # the car measurably leaves the path, so waiting for 0.20 m of deviation
        # prompts the driver far too late. Sustained saturation alone raises it;
        # an actual deviation still raises it immediately.
        steer_saturated_now = bool(lac_log.active and lac_log.saturated and
                                   not CS.steeringPressed)
        # Held TURN_DESIRE_HOLD_S over a short gap: the turn desire re-pulse
        # (desire_helper TURN_REPULSE_*) drops it to none for 0.1 s.
        if lat_plan.desire in (log.LateralPlan.Desire.turnLeft, log.LateralPlan.Desire.turnRight):
            self.turn_desire_side = lat_plan.desire
            self.turn_desire_hold_s = TURN_DESIRE_HOLD_S
        else:
            self.turn_desire_hold_s = max(0.0, self.turn_desire_hold_s - DT_CTRL)
        turning = self.turn_desire_hold_s > 0.
        if steer_saturated_now:
            self.steer_sat_elapsed += DT_CTRL
            self.steer_sat_clear_elapsed = 0.0
        else:
            self.steer_sat_clear_elapsed += DT_CTRL
            if self.steer_sat_clear_elapsed >= STEER_SAT_CLEAR_S:
                self.steer_sat_elapsed = 0.0

        # lac_log.saturated already excludes hands-on and steer_limited frames.
        if lac_log.active and lac_log.saturated:
            self.steer_sat_accel_hold_s = STEER_SAT_ACCEL_HOLD_S
        else:
            self.steer_sat_accel_hold_s = max(0.0, self.steer_sat_accel_hold_s - DT_CTRL)

        if steer_saturated_now:
            dpath_points = lat_plan.dPathPoints
            deviating = False
            if len(dpath_points):
                # TODO use desired vs actual curvature
                deviating = ((actuators.steer > 0 and dpath_points[0] < -STEER_SAT_DEVIATION_M) or
                             (actuators.steer < 0 and dpath_points[0] > STEER_SAT_DEVIATION_M))
            # Under-turn: how much less lateral acceleration the car is making
            # than the plan asks for, in the direction of the turn. Only the
            # torque controller reports it; the others keep the plain hold.
            falling_short = True
            if hasattr(lac_log, 'desiredLateralAccel') and hasattr(lac_log, 'actualLateralAccel'):
                desired_lat = float(lac_log.desiredLateralAccel)
                falling_short = (math.copysign(1.0, desired_lat) *
                                 (desired_lat - float(lac_log.actualLateralAccel)) >= STEER_SAT_MIN_SHORTFALL)
            if deviating or (self.steer_sat_elapsed >= STEER_SAT_WARN_HOLD_S and falling_short):
                if turning:
                    self.turn_sat_hold_s = TURN_SAT_HOLD_S   # said inside the turn message
                else:
                    self.events.add(EventName.steerSaturated)

        # "좌회전 중 / 우회전 중" while the turn desire is out, with the steering
        # limit or the low-speed cut-off as its second line.
        self.turn_sat_hold_s = max(0.0, self.turn_sat_hold_s - DT_CTRL)
        # Only while the car is turning: the desire stays out while it waits
        # at a light or creeps in the turn queue (2026-11-23--08-45-38: 133 s
        # and 44 s, 80-93% of it under 10 km/h), which is no "회전 중".
        turn_left = self.turn_desire_side == log.LateralPlan.Desire.turnLeft
        wheel_toward = CS.steeringAngleDeg * (1. if turn_left else -1.)
        turning_now = turning and (CS.vEgo >= TURN_SLOW_KPH * CV.KPH_TO_MS or
                                   (CS.vEgo > TURN_MOVING_KPH * CV.KPH_TO_MS and
                                    wheel_toward > TURN_SHOW_WHEEL_DEG))
        if turning_now and self.enabled:
            TURN_STATE["left"] = turn_left
            TURN_STATE["sub"] = ("slow" if CS.vEgo < TURN_SLOW_KPH * CV.KPH_TO_MS else
                                 "sat" if self.turn_sat_hold_s > 0. else "")
            self.events.add(EventName.turningIndicatorOn)

        # Ensure no NaNs/Infs
        for p in ACTUATOR_FIELDS:
            attr = getattr(actuators, p)
            if not isinstance(attr, Number):
                continue

            if not math.isfinite(attr):
                cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
                setattr(actuators, p, 0.0)

        self._record_control_timing(c0, c1, c2, c3, c4, sec_since_boot())
        return actuators, lac_log

    def update_button_timers(self, buttonEvents):
        # increment timer for buttons still pressed
        for k in self.button_timers:
            if self.button_timers[k] > 0:
                self.button_timers[k] += 1

        for b in buttonEvents:
            if b.type.raw in self.button_timers:
                self.button_timers[b.type.raw] = 1 if b.pressed else 0

    def _model_plan_speed(self, t):
        """The driving model's planned speed t seconds ahead, nan when unavailable."""
        vx = self.sm['modelV2'].velocity.x
        if len(vx) != len(T_IDXS) or not self.sm.valid['modelV2']:
            return float('nan')
        return float(np.interp(t, T_IDXS, vx))

    def _apply_follow_accel_cap(self, CS, limits):
        """Lower the PID's positive accel limit while following a lead in flowing
        traffic, eased down at FOLLOW_CAP_EASE_MPS3 so the pedal never steps.

        The ceiling is acceleration above the speed-hold level, so it is added to
        what the command spends holding speed against drag and the grade."""
        hold = self.follow_cap_hold.update(
          bool(self.active and not CS.gasPressed and not CS.brakePressed),
          float(self.LoC.last_output_accel), float(CS.aEgo), float(CS.vEgo))
        target = math.inf
        if self.CP.enableGasInterceptor:
            lead = self.sm['radarState'].leadOne
            if lead.status and self.sm.valid['radarState']:
                fl = self.follow_learner
                target = follow_accel_cap(
                  float(CS.vEgo), float(lead.dRel), float(lead.vRel), fl.coast_decel(CS.vEgo),
                  fl.lift_margin, floor_tr(CS.vEgo), follow_band(CS.vEgo, fl.gap_scale)[2])
                target += hold * hold_weight(CS.vEgo)
        if math.isfinite(target):
            # Ease down from what the controller is actually sending now.
            prev = min(self.follow_cap_limit, float(limits[1]), float(self.LoC.last_output_accel))
            self.follow_cap_limit = max(0.0, target, prev - FOLLOW_CAP_EASE_MPS3 * DT_CTRL)
        else:
            self.follow_cap_limit = math.inf
        return limits[0], min(float(limits[1]), self.follow_cap_limit)

    def _read_standstill_auto_launch(self):
        """Community toggle; off if the key is not built into params yet."""
        try:
            return self.params.get_bool("StandstillAutoLaunch")
        except Exception:
            return False

    def _pedal_hold_snapshot(self, CS):
        """Pedal-path state for PedalHoldDiag; only called while the pedal is held at zero."""
        pc = self.predictive_coasting
        sp = pc.source_pressures
        cc = getattr(self.CI, 'CC', None)
        rl = getattr(cc, 'pedal_rise_limiter', None)
        r2 = lambda x: round(float(x), 2) if math.isfinite(float(x)) else None
        snap = {
            'acc_on': bool(CS.adaptiveCruise),
            'stop_boost': bool(self.stop_accel_boost_active),
            'raw': r2(getattr(self, 'comma_pedal_raw_command', 0.0)),
            'coast_scale': r2(self.predictive_coast_pedal_scale),
            'coast_phase': pc.phase,
            'coast_pressure': r2(pc.last_pressure),
            'coast_source': pc.dominant_source,
            'p_lead': r2(sp.get('lead', 0.0)),
            'p_curve': r2(sp.get('curve', 0.0)),
            'p_speed': r2(sp.get('speed_limit', 0.0)),
            'brake_latched': bool(pc.brake_latched),
            'lead_d': r2(pc.lead_distance_m),
            'lead_vrel': r2(pc.lead_rel_speed_ms),
            'gap_margin': r2(pc.distance_margin_m),
            'ttg': r2(pc.time_to_gap_s),
            'reach': r2(pc.reach_ratio),
            'corner_lift': bool(self.corner_lift_last),
            'corner_scale': r2(self.corner_lift_scale_last),
            'speed_limit_coast': bool(self.speed_limit_coast_active),
        }
        if rl is not None:
            snap.update(rise_holding=bool(rl.holding), rise_rate=r2(rl.rise_rate),
                        post_brake_s=r2(rl.post_brake_elapsed))
        return snap

    def publish_logs(self, CS, start_time, actuators, lac_log):
        """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

        p0 = sec_since_boot()

        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        CC.active = self.active
        CC.actuators = actuators
        # state_control computes these flags on a different temporary
        # CarControl message. Re-populate them on the message passed to CI.apply
        # and published to carControl; otherwise both fields default to false.
        # latActive is state_control's (same active, car state and params);
        # longActive is re-evaluated, as the events may have changed since.
        apply_control_activation(CC, self.active, CS, self.CP, self.events.any(ET.OVERRIDE),
                                 lat_active=self._frame_lat_active)

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

        # Bound once: a capnp field read costs more than a write, and this
        # message is read five times further down.
        lat_plan = self.sm['lateralPlan']
        right_lane_visible = lat_plan.rProb > 0.5
        left_lane_visible = lat_plan.lProb > 0.5

        totalCameraOffset = lat_plan.totalCameraOffset

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
            # right_lane_visible/left_lane_visible were read off this same
            # lateralPlan a few lines up and nothing has changed since.
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

        p1 = sec_since_boot()
        if not self.read_only and self.initialized:
            # send car controls over can
            self.last_actuators, can_sends = self.CI.apply(CC, self)
            self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
            CC.actuatorsOutput = self.last_actuators
            # Compare the values we already hold rather than reading them back
            # out of CC: a nested capnp read is ~10 us, and these two are the
            # actuators passed in and what CI.apply just returned.
            self.steer_limited = abs(actuators.steer - self.last_actuators.steer) > 1e-2
            if self.CP.enableGasInterceptor:
                self.pedal_hold_diag.update(
                    self.active, CS.vEgo, actuators.accel, self.last_actuators.gas,
                    CS.gasPressed, CS.brakePressed,
                    lambda: self._pedal_hold_snapshot(CS))
        p2 = sec_since_boot()

        # Only soft-disable forces decel now: the other trigger was driver
        # monitoring's awareness running out, and monitoring is off.
        force_decel = self.state == State.softDisabling

        # Curvature & Steering angle
        params = self.sm['liveParameters']

        steer_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
        curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, params.roll)
        # NDA Add.. (PSK). Only road_limit_speed is still published; left_dist
        # and max_speed_log are consumed inside cal_max_speed itself, so they
        # are unpacked and dropped rather than named.
        road_limit_speed, _, _ = self.cal_max_speed(
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

        # canMonoTimes is not written: reading the capnp list, rebuilding it as
        # a Python list and writing it back cost ~20 us a frame and nothing
        # reads the field. Left in the schema, so it reads as empty.
        controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
        controlsState.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan']
        controlsState.enabled = self.enabled
        controlsState.active = self.active
        controlsState.curvature = curvature
        controlsState.state = self.state
        # engageable is not written: nothing reads it, and the events.any() it
        # needed measures 11.3 us on the EON -- the most expensive of the
        # thirty dead writes removed here, because it was the only one that
        # had to compute its value first.
        controlsState.longControlState = self.LoC.long_control_state
        controlsState.vPid = float(self.LoC.v_pid)

        # Cruise SET
        # kph [applyMaxSpeed, cruiseMaxSpeed]
        # 방지턱 감속 중에는 하한을 MIN_SET_SPEED_KPH 대신 방지턱 속도(BUMP_SPEED_KPH)로 둔다.
        if self.speed_bump_active:
            min_set_kph = SpeedBumpPlanner.BUMP_SPEED_KPH
        elif self.atc_low_active:
            min_set_kph = ATC_LOW_FLOOR_KPH   # ATC 교차로 회전: 15km/h
        else:
            min_set_kph = MIN_SET_SPEED_KPH
        controlsState.applyMaxSpeed = float(
            clip(self.v_cruise_kph, min_set_kph, self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))
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

        # NDA. roadLimitSpeedActive and roadLimitSpeedLeftDist are not written:
        # nothing reads either, and get_active() does a socket recv (6.7 us)
        # purely to fill one of them. left_dist still comes back from
        # cal_max_speed, which computes it for its own use.
        controlsState.roadLimitSpeed = road_limit_speed

        controlsState.steerRatio = self.VM.sR
        # Report the effective CarParams value after the GM minimum clamp, not
        # the raw ntune request. v0.8.13's additional 0.2 s is logged below.
        controlsState.steerActuatorDelay = float(self.CP.steerActuatorDelay)

        # sccGasFactor, sccBrakeFactor and sccCurvatureFactor are not written:
        # nothing reads them, and the three ntune_scc_get calls they needed are
        # 9.7 us together. The tuning they report still applies -- it is read
        # where it is used, not here.

        # Curve slowdown state consumed by the onroad CURV indicator.
        controlsState.curvDriving = bool(self.is_curv_driving)
        controlsState.curvSpeed = float(self.curv_speed)

        # latAccelFactor and friction are the onroad LatA/Fri readout. They
        # were the live torque learner's report originally; torqued is
        # disabled, so what they carry now is the fixed ntune tuning that is
        # actually steering the car, cached in state_control off the
        # controller. latAccelOffset and totalBucketPoints stay unwritten --
        # the learner fed those and nothing reads them. The fields are left in
        # the schema either way (removing capnp fields would break replay of
        # every log recorded before this).
        controlsState.latAccelFactor = float(self.torque_lat_accel_factor)
        controlsState.friction = float(self.torque_friction)
        # The dynamicTorque*/modelCurvature*/lowSpeedTorque* fields were
        # written every frame from literals baked into the controller -- the
        # feature they describe is stubbed off, so they carried no information
        # at 2.2 us a write. They are gone, and with them the
        # get_dynamic_debug_torque_params() call that existed only to fill the
        # five of them that moved; the schema keeps the fields and they now
        # read as their defaults. The two laneCenterCorrection fields do have a
        # reader and come off lat_plan, not off that call, so they no longer
        # sit behind a guard that was never about them.
        controlsState.laneCenterCorrectionM = float(
          getattr(lat_plan, 'laneCenterCorrectionM', 0.0))
        controlsState.laneCenterCorrectionActive = bool(
          getattr(lat_plan, 'laneCenterCorrectionActive', False))

        #controlsState.cruiseGap = int(Params().get("cruiseGap", encoding="utf8"))
        controlsState.minTR = float(self.dynamic_tr_min)
        #controlsState.dynamicTRMode = int(self.sm['longitudinalPlan'].dynamicTRMode)
        controlsState.dynamicTRMode = self.dynamic_tr_mode
        controlsState.globalDfMod = float(self.dynamic_tr_global_df_mod)
        controlsState.dynamicTRValue = float(self.sm['dynamicFollowData'].mpcTR)
        # followingDistanceRawTR and followingDistanceLearnedTROffset are not
        # written: nothing reads either, and each was a nested capnp read
        # (~8.7 us) on top of the write.

        # Stop-and-go launch diagnostics. These report the controller-accepted
        # request and whether it actually raised the acceleration command.
        # Only stopAccelBoostActive survives here. The other nine of this group
        # -- Applied, RawAccel, FinalAccel, Factor, FloorAccel, HillExtraAccel
        # and both driverLaunchHandoff fields -- were written every frame and
        # read nowhere. The state they describe still exists on self.LoC and
        # self.stop_accel_boost_latch for whoever wants to look at it.
        controlsState.stopAccelBoostActive = bool(self.stop_accel_boost_active)
        # Speed bump being approached, for the UI (see log.capnp).
        bump_planner = get_road_speed_limiter().bump_planner
        controlsState.speedBumpTracking = bool(bump_planner.tracking)
        controlsState.speedBumpDist = float(bump_planner.left_dist)
        controlsState.trafficState = int(self.traffic_light.state)
        # drivingStyleAI* fields are left in the schema (removing capnp fields
        # would break replay of every log recorded before this) but nothing
        # writes them any more, so they read as their defaults.
        # commaPedalResistanceProfile went with them, and it was the expensive
        # one: a capnp Text write is 9.9 us against 2.0 for a scalar.
        controlsState.commaPedalProfileGain = float(self.comma_pedal_profile_gain)
        # commaPedalLearnedGain is no longer written -- nothing learns a gain.
        # Nor are EffectiveGain, ProfileChanging, RawCommand, StyledCommand,
        # FinalCommand or RiseScale: same story, no reader.

        controlsState.totalCameraOffset = totalCameraOffset

        lat_tuning = self._lat_tuning
        if self.joystick_mode:
          controlsState.lateralControlState.debugState = lac_log
        elif self._steer_control_type_angle:
          controlsState.lateralControlState.angleState = lac_log
        elif lat_tuning == 'pid':
          controlsState.lateralControlState.pidState = lac_log
        elif lat_tuning == 'lqr':
          controlsState.lateralControlState.lqrState = lac_log
        elif lat_tuning == 'indi':
          controlsState.lateralControlState.indiState = lac_log
        elif lat_tuning == 'torque':
          controlsState.lateralControlState.torqueState = lac_log

        p3 = sec_since_boot()
        self.pm.send('controlsState', dat)

        p4 = sec_since_boot()
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
        self._record_publish_timing(p0, p1, p2, p3, p4, sec_since_boot())

    def step(self):
        start_time = sec_since_boot()

        # Sample data from sockets and get a carState
        CS = self.data_sample()
        t_sample = sec_since_boot()

        self.update_events(CS)
        t_events = sec_since_boot()

        if not self.read_only and self.initialized:
            # Update control state
            self.state_transition(CS)
        t_transition = sec_since_boot()

        # Compute actuators (runs PID loops and lateral MPC)
        actuators, lac_log = self.state_control(CS)
        t_control = sec_since_boot()

        # Publish data
        self.publish_logs(CS, start_time, actuators, lac_log)
        t_publish = sec_since_boot()

        self.update_button_timers(CS.buttonEvents)
        self._record_step_timing(start_time, t_sample, t_events, t_transition,
                                 t_control, t_publish, sec_since_boot())

    def controlsd_thread(self):
        while True:
            self.step()
            self.rk.monitor_time()

    def _record_step_timing(self, t0, t1, t2, t3, t4, t5, t6):
        """Where the control loop's 10 ms actually goes, averaged over a window.

        Always on, unlike the Profiler this replaces. That one had to be turned
        on for a drive before it said anything, and its iter_ms was not usable
        even then: Profiler.reset() clears the per-checkpoint dictionary but
        not the running total those checkpoints are divided against, so the
        figure grew window over window -- 51 ms then 64 ms, on a loop whose
        frames were 12.9 ms. Its per-step numbers were sound; only the total
        was wrong. Measuring the same thing directly, with the publish_logs
        breakdown alongside it, costs 26 us a frame measured on the EON,
        0.26% of the budget, which is cheap enough not to need a flag.

        "wait" is data_sample. Only its drain_sock_raw block is slack; it also
        runs CI.update and the SubMaster poll, which sample_mean_ms splits
        out, and cpu_work_ms is the loop's work including those.
        """
        acc = self._step_acc
        mx = self._step_max
        d0 = t1 - t0
        d1 = t2 - t1
        d2 = t3 - t2
        d3 = t4 - t3
        d4 = t5 - t4
        d5 = t6 - t5
        acc[0] += d0
        acc[1] += d1
        acc[2] += d2
        acc[3] += d3
        acc[4] += d4
        acc[5] += d5
        sa = self._smp_acc
        sm_ = self._smp_max
        m0, m1, m2 = self._smp_marks
        s0 = m0 - t0
        s1 = m1 - m0
        s2 = m2 - m1
        s3 = t1 - m2
        sa[0] += s0
        sa[1] += s1
        sa[2] += s2
        sa[3] += s3
        if s0 > sm_[0]: sm_[0] = s0
        if s1 > sm_[1]: sm_[1] = s1
        if s2 > sm_[2]: sm_[2] = s2
        if s3 > sm_[3]: sm_[3] = s3
        # Straight-line rather than a loop over the pairs: at 100 Hz the range()
        # and its indexing cost more than the six comparisons they would save.
        if d0 > mx[0]: mx[0] = d0
        if d1 > mx[1]: mx[1] = d1
        if d2 > mx[2]: mx[2] = d2
        if d3 > mx[3]: mx[3] = d3
        if d4 > mx[4]: mx[4] = d4
        if d5 > mx[5]: mx[5] = d5

        self._step_frames += 1
        if self._step_frames < STEP_TIMING_WINDOW_FRAMES:
            return

        n = float(self._step_frames)
        total = sum(acc)
        names = self.STEP_NAMES
        append_process_diagnostic(
            "controlsd_step_timing",
            frames=self._step_frames,
            window_s=round(t6 - self._step_window_t, 3),
            step_ms=round(1000.0 * total / n, 3),
            work_ms=round(1000.0 * (total - acc[0]) / n, 3),
            # work_ms keeps its old meaning (everything after data_sample) so
            # drives stay comparable; this one counts data_sample's CPU too.
            cpu_work_ms=round(1000.0 * (total - self._smp_acc[0]) / n, 3),
            mean_ms={name: round(1000.0 * acc[i] / n, 3) for i, name in enumerate(names)},
            max_ms={name: round(1000.0 * mx[i], 3) for i, name in enumerate(names)},
            publish_mean_ms={name: round(1000.0 * self._pub_acc[i] / n, 3)
                             for i, name in enumerate(self.PUBLISH_NAMES)},
            publish_max_ms={name: round(1000.0 * self._pub_max[i], 3)
                            for i, name in enumerate(self.PUBLISH_NAMES)},
            events_mean_ms={name: round(1000.0 * self._ev_acc[i] / n, 3)
                            for i, name in enumerate(self.EVENT_NAMES)},
            events_max_ms={name: round(1000.0 * self._ev_max[i], 3)
                           for i, name in enumerate(self.EVENT_NAMES)},
            control_mean_ms={name: round(1000.0 * self._ctl_acc[i] / n, 3)
                             for i, name in enumerate(self.CONTROL_NAMES)},
            control_max_ms={name: round(1000.0 * self._ctl_max[i], 3)
                            for i, name in enumerate(self.CONTROL_NAMES)},
            sample_mean_ms={name: round(1000.0 * self._smp_acc[i] / n, 3)
                            for i, name in enumerate(self.SAMPLE_NAMES)},
            sample_max_ms={name: round(1000.0 * self._smp_max[i], 3)
                           for i, name in enumerate(self.SAMPLE_NAMES)},
        )
        # The write itself lands after t6, so it is outside every delta above.
        # It shows up as one longer loop period in a thousand and nowhere else.
        self._step_acc = [0.0] * len(names)
        self._step_max = [0.0] * len(names)
        self._step_frames = 0
        self._step_window_t = t6
        self._pub_acc = [0.0] * len(self.PUBLISH_NAMES)
        self._pub_max = [0.0] * len(self.PUBLISH_NAMES)
        self._ev_acc = [0.0] * len(self.EVENT_NAMES)
        self._ev_max = [0.0] * len(self.EVENT_NAMES)
        self._ctl_acc = [0.0] * len(self.CONTROL_NAMES)
        self._ctl_max = [0.0] * len(self.CONTROL_NAMES)
        self._smp_acc = [0.0] * len(self.SAMPLE_NAMES)
        self._smp_max = [0.0] * len(self.SAMPLE_NAMES)

    def _record_publish_timing(self, p0, p1, p2, p3, p4, p5):
        """Accumulate the publish_logs breakdown for the current step window.

        Emitted by _record_step_timing rather than separately: one line a
        window is easier to read than two that have to be matched up, and the
        two are counted over exactly the same frames.

        Worth breaking out because 850 profiler windows on 2026-09-08 put
        publish_logs at 4.52 ms a frame, the largest of the four checkpoints
        in 764 of them, against 2.37 for the control maths and 1.62 for the
        state transition. It is 278 lines long, so knowing it is the expensive
        phase is not yet knowing what to change.
        """
        acc = self._pub_acc
        mx = self._pub_max
        d0 = p1 - p0
        d1 = p2 - p1
        d2 = p3 - p2
        d3 = p4 - p3
        d4 = p5 - p4
        acc[0] += d0
        acc[1] += d1
        acc[2] += d2
        acc[3] += d3
        acc[4] += d4
        if d0 > mx[0]: mx[0] = d0
        if d1 > mx[1]: mx[1] = d1
        if d2 > mx[2]: mx[2] = d2
        if d3 > mx[3]: mx[3] = d3
        if d4 > mx[4]: mx[4] = d4


def main(sm=None, pm=None, logcan=None):
    controls = Controls(sm, pm, logcan)
    controls.controlsd_thread()


if __name__ == "__main__":
    main()
