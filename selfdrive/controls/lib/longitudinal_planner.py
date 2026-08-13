#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import interp

import cereal.messaging as messaging
from common.conversions import Conversions as CV
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.controls.lib.vision_lead_transition import update_vision_lead_transition
from selfdrive.controls.lib.model_data_validation import as_finite_vector
from selfdrive.swaglog import cloudlog


LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2  # car smoothly decel at .2m/s^2 when user is distracted
LEAD_LOSS_CRUISE_BLEND_S = 0.60
LEAD_LOSS_ACCEL_RISE_MPS3 = 0.70

# 가속도를 낮추어 엑셀 사용을 최소화합니다.
_A_CRUISE_MIN_V_FOLLOWING = [-1.2, -1.2, -1.1, -1.0, -0.8]
_A_CRUISE_MIN_V = [-0.8, -1.0, -0.8, -0.5, -0.3]
_A_CRUISE_MIN_BP = [0., 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS]

_A_CRUISE_MAX_V = [0.8, 0.7, 0.6, 0.5, 0.4]  # 최대 가속도를 낮추어 연비를 개선
_A_CRUISE_MAX_V_FOLLOWING = [1.2, 1.0, 0.8, 0.5, 0.4]
_A_CRUISE_MAX_BP = _A_CRUISE_MIN_BP

_A_TOTAL_MAX_V = [2.5, 3.0, 4.0]  # 회전 시 가속 제한을 낮춤
_A_TOTAL_MAX_BP = [0., 25., 55.]

def calc_cruise_accel_limits(v_ego):
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V_FOLLOWING)
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
    return [a_cruise_min, a_cruise_max]

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
    a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    return [a_target[0], min(a_target[1], a_x_allowed)]

def limit_stop_acceleration(v_ego, a_target):
    if v_ego < 0.5:  # 감속을 부드럽게 조정
      a_target = max(a_target, AWARENESS_DECEL / 2)
    return a_target

class Planner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.accel_limit_max = float(calc_cruise_accel_limits(init_v)[1])

    self.fcw = False
    self.vision_lead_present = False
    self.previous_vision_lead_present = False
    self.last_vision_lead_mono_time = 0
    self.lead_loss_blend_remaining = 0.0

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

  def update(self, sm):
    v_ego = sm['carState'].vEgo

    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # Detect a confirmed lead-loss edge using the source model timestamp. The
    # radar layer already absorbs short dropouts; once its bounded hold expires,
    # blend into cruise without deleting the previous acceleration state.
    self.vision_lead_present = bool(sm['radarState'].leadOne.status or sm['radarState'].leadTwo.status)
    vision_lead_mono_time = sm['radarState'].mdMonoTime
    self.previous_vision_lead_present, self.last_vision_lead_mono_time, vision_lead_lost = \
      update_vision_lead_transition(self.previous_vision_lead_present,
                                    self.last_vision_lead_mono_time,
                                    self.vision_lead_present, vision_lead_mono_time)

    if vision_lead_lost:
      self.lead_loss_blend_remaining = LEAD_LOSS_CRUISE_BLEND_S
      if self.CP.enableGasInterceptor:
        # Negative planned acceleration and zero both mean zero command to a
        # gas interceptor. Start the clear-road blend from that physical zero
        # point, while the rise limiter below prevents a cruise acceleration
        # jump. Aligning the speed state avoids an artificial wait for a stale
        # lead-follow trajectory to catch back up to measured speed.
        self.v_desired_filter.x = max(self.v_desired_filter.x, v_ego)
        self.a_desired = max(self.a_desired, 0.0)
    elif self.vision_lead_present:
      self.lead_loss_blend_remaining = 0.0

    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = 0.0
      self.lead_loss_blend_remaining = 0.0

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    accel_limits = calc_cruise_accel_limits(v_ego)
    self.accel_limit_max = float(accel_limits[1])
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    if force_slow_decel:
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)
    if self.CP.enableGasInterceptor and self.lead_loss_blend_remaining > 0.0:
      accel_limits_turns[1] = min(
        accel_limits_turns[1], self.a_desired + LEAD_LOSS_ACCEL_RISE_MPS3 * DT_MDL)
      self.lead_loss_blend_remaining = max(
        0.0, self.lead_loss_blend_remaining - DT_MDL)

    accel_limits_turns[1] = limit_stop_acceleration(v_ego, accel_limits_turns[1])

    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    model_x = as_finite_vector(sm['modelV2'].position.x, expected_size=33)
    model_v = as_finite_vector(sm['modelV2'].velocity.x, expected_size=33)
    model_a = as_finite_vector(sm['modelV2'].acceleration.x, expected_size=33)
    if model_x is not None and model_v is not None and model_a is not None:
      x = np.interp(T_IDXS_MPC, T_IDXS, model_x)
      v = np.interp(T_IDXS_MPC, T_IDXS, model_v)
      a = np.interp(T_IDXS_MPC, T_IDXS, model_a)
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
    self.mpc.update(sm['carState'], sm['radarState'], sm['modelV2'], v_cruise, x, v, a, prev_accel_constraint)
    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 5
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = self.vision_lead_present
    longitudinalPlan.accelLimitMax = self.accel_limit_max
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    pm.send('longitudinalPlan', plan_send)
