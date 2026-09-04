from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.stop_accel_boost import (
  apply_stop_accel_boost, STOP_ACCEL_BOOST_FACTOR, STOP_ACCEL_ZERO_EPS)
from selfdrive.modeld.constants import T_IDXS

LongCtrlState = car.CarControl.Actuators.LongControlState

# As per ISO 15622:2018 for all speeds
ACCEL_MIN_ISO = -3.5  # m/s^2
ACCEL_MAX_ISO = 2.0  # m/s^2

def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_future, brake_pressed, cruise_standstill,
                             stop_accel_boost_active=False):
  """Update longitudinal control state machine"""
  accelerating = v_target_future > v_target
  stopping_condition = not stop_accel_boost_active and ((v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < CP.vEgoStopping and
                        (v_target_future < CP.vEgoStopping and not accelerating)))

  starting_condition = stop_accel_boost_active or \
                       (v_target_future > CP.vEgoStarting and accelerating and not cruise_standstill)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    self.stop_accel_boost_applied = False
    self.stop_accel_boost_raw_accel = 0.0
    self.stop_accel_boost_final_accel = 0.0
    self.driver_launch_handoff_active = False
    self.driver_launch_handoff_shadow_accel = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan,
             stop_accel_boost_active=False, stop_accel_boost_floor=0.0,
             driver_launch_handoff=False,
             stop_accel_boost_factor=STOP_ACCEL_BOOST_FACTOR):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target) / self.CP.longitudinalActuatorDelayLowerBound - a_target

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target) / self.CP.longitudinalActuatorDelayUpperBound - a_target
      a_target = min(a_target_lower, a_target_upper)

      v_target_future = speeds[-1]
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0

    # TODO: This check is not complete and needs to be enforced by MPC
    a_target = clip(a_target, ACCEL_MIN_ISO, ACCEL_MAX_ISO)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Update state machine
    output_accel = self.last_output_accel
    gas_override = bool(CS.gasPressed)
    qualified_gas_handoff = bool(
      active and gas_override and driver_launch_handoff and not CS.brakePressed)
    launch_context_active = bool(stop_accel_boost_active or qualified_gas_handoff)
    self.driver_launch_handoff_active = qualified_gas_handoff
    self.driver_launch_handoff_shadow_accel = 0.0

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_future, CS.brakePressed,
                                                       CS.cruiseState.standstill,
                                                       launch_context_active)

    # Driver gas always suppresses automatic pedal output. During a confirmed
    # stopped-lead launch, however, it must not erase the launch/PID state. Keep
    # a frozen-integrator shadow calculation warm so the first post-release
    # control frame can resume lead tracking without a second dead period.
    if self.long_control_state == LongCtrlState.off or CS.brakePressed:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif gas_override and not qualified_gas_handoff:
      self.reset(CS.vEgo)
      output_accel = 0.

    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not launch_context_active and not self.CP.stoppingControl and \
                          CS.vEgo < 1.5 and v_target_future < 0.7 and v_target_future < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot or qualified_gas_handoff

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)

      # 가스 인터셉터(gas interceptor)는 음의 가속(감속)을 수행할 수 없습니다.
      # 일단 두 비전 리드(vision lead)가 모두 사라지면, 새로운 궤적을 복구하는 동안
      # 유효하지 않은 음의 피드포워드(stale negative feedforward) 값이
      # 양의 PID 속도 요청을 상쇄하지 않도록 해야 합니다.
      # 리드가 존재하는 동안에는 기존의 MPC 감속 동작이 그대로 유지됩니다.
      pedal_clear_road_recovery = self.CP.enableGasInterceptor and \
                                  not long_plan.hasLead and \
                                  error_deadzone > 0.0 and \
                                  v_target_future >= v_target and \
                                  a_target < 0.0
      if pedal_clear_road_recovery:
        a_target = 0.0
        # 단순히 오래된 피드포워드(stale feedforward) 값뿐만 아니라,
        # 음수 방향의 적분(negative integrator) 값도 여기서 초기화(drop)해야 합니다.
        #
        # PIDController는 제어값이 pos_limit이나 neg_limit에 도달했을 때만 적분 동작을 멈추는데,
        # 여기서 neg_limit은 ISO 감속 하한값(약 -3.5)으로 설정되어 있습니다.
        # 하지만 이 차량의 실제 액추에이터 한계값은 0입니다. 즉, 인터셉터(interceptor)는 음의 가속도를 낼 수 없으며,
        # carcontroller는 페달 값을 clip(acc_mult * accel, 0.0, 0.85)로 제한합니다.
        # 따라서 감속 상황에서 PID는 이미 0에 고정된 액추에이터를 상대로 적분값(i)을 계속 낮추게 되는데,
        # 이는 PID가 감지하지 못하는 전형적인 '포화 상태의 윈드업(saturation windup)' 현상입니다.
        #
        # 이렇게 낮아진 적분값(i)이 재가속을 지연시키는 원인이 됩니다. a_target을 0으로 설정하면 오래된 피드포워드 값은 제거되지만,
        # 적분값(i)은 페달 입력이 다시 발생하기 전까지 error * k_i * i_rate에 따라 0을 지나 다시 상승해야 하기 때문입니다.
        # 음수 방향으로 누적된 적분값은 실제 제어에 반영된 적이 없으므로, 굳이 유지할 필요가 없습니다.
        #
        # 이 로직은 복구 조건(recovery condition)에 한해 적용됩니다.
        # 이 조건은 선행 차량이 없고(no lead), 데드존을 벗어난 양수 속도 오차가 있으며,
        # 목표값이 상승하는 상황을 전제로 하므로, 실제로 감속이 필요한 상황에서는 작동하지 않습니다.
        self.pid.i = max(self.pid.i, 0.0)

      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo, feedforward=a_target, freeze_integrator=freeze_integrator)

      if prevent_overshoot:
        output_accel = min(output_accel, 0.0)

      if qualified_gas_handoff:
        self.driver_launch_handoff_shadow_accel = float(output_accel)
        output_accel = 0.0

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_accel > self.CP.stopAccel:
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
      self.reset(CS.vEgo)

    boost_allowed = stop_accel_boost_active and active and not gas_override and not CS.brakePressed
    self.stop_accel_boost_raw_accel = float(output_accel)
    final_accel = apply_stop_accel_boost(
      output_accel, CS.vEgo, boost_allowed, accel_limits,
      launch_floor_accel=stop_accel_boost_floor if boost_allowed else 0.0,
      boost_factor=stop_accel_boost_factor)
    self.stop_accel_boost_final_accel = float(final_accel)
    self.stop_accel_boost_applied = bool(boost_allowed and
                                         final_accel > output_accel + STOP_ACCEL_ZERO_EPS)
    # Do not replace the prepared controller history with the deliberately
    # suppressed zero output during a qualified manual launch.
    if not qualified_gas_handoff:
      self.last_output_accel = final_accel

    return final_accel
