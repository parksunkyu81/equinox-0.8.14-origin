from cereal import car
from common.clock import sec_since_boot
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.steer_scheduler import GMSteeringCommandScheduler
from selfdrive.car.gm.steering_limits import steer_delta_limits_ms
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN
from selfdrive.controls.lib.stop_accel_boost import pedal_command_allowed
from selfdrive.controls.lib.pedal_force_recovery import PEDAL_FORCE_RECOVERY_PEDAL_FLOOR
from selfdrive.controls.lib.comma_pedal_rise_limiter import CommaPedalRiseLimiter

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.0
    self.predictive_coast_styled_pedal = 0.0
    self.predictive_coast_pedal_scale = 1.0
    # Rise-rate limit and post-brake suppression; see the module docstring for
    # the drive-log measurements behind its constants.
    self.pedal_rise_limiter = CommaPedalRiseLimiter(DT_CTRL)

    # Dynamic steering delta diagnostics/state
    self._dyn_steer_limited_prev = False
    self._dyn_delta_up_last = 0
    self._dyn_delta_down_last = 0


    self.steer_command_scheduler = GMSteeringCommandScheduler()
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, c, enabled, CS, frame, controls, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Panda keeps the absolute low-speed ceiling at 14/20. Apply the actual
    # speed-dependent envelope here so highway lane changes and curves return
    # early to the stable 7/17 response.
    delta_up, delta_down = steer_delta_limits_ms(CS.out.vEgo)
    P.STEER_DELTA_UP = float(delta_up)
    P.STEER_DELTA_DOWN = float(delta_down)
    self._dyn_delta_up_last = float(delta_up)
    self._dyn_delta_down_last = float(delta_down)

    # Send CAN commands.
    can_sends = []

    brake_pressed = bool(CS.out.brakePressed)

    # Steering (50Hz)
    # 메시지를 너무 짧은 간격으로 전송할 때 발생하는 GM EPS 오류를 방지하십시오.
    # 현재 CS 프레임 내에서 다음 Panda 루프백 확인(loopback confirmation)을 방금 수신했다면 해당 전송을 건너뛰어야 합니다.
    lkas_enabled = c.latActive and not (
      CS.out.steerFaultTemporary or CS.out.steerFaultPermanent
    ) and CS.out.vEgo >= P.MIN_STEER_SPEED

    if not lkas_enabled:
      self.steer_rate_limited = False

    # Schedule from monotonic time instead of frame parity. Panda loopback can
    # arrive on every due (even) control frame; suppressing that frame would
    # otherwise starve steering commands indefinitely.
    steer_command_sent, idx = self.steer_command_scheduler.update(
      sec_since_boot(), CS.lka_steering_cmd_counter)
    if steer_command_sent:
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0  # LKAS가 비활성화된 경우에는 EPS에 조향 토크를 요청하지 않도록 0을 사용

      self.apply_steer_last = apply_steer  # 적용한 조향 토크를 저장합니다. 다음 주기의 토크 변화량 제한을 계산할 때 기준값으로 사용

      # GM EPS는 수신 메시지 카운터에 간극(gap)이 발생하면 결함(fault)을 보고합니다.
      # 시스템 해제 시점에 일시적으로 발생하는 OP/Panda 안전 동기화 문제를 처리하기 위해,
      # Panda 안전성 검사를 통과한 것으로 확인된 마지막 메시지를 기준으로 카운터를 증가시킵니다.
      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    # 현재 조향 토크가 제한됐는지를 상위 제어기인 controls에 전달
    # 조향 PID 제어기는 이 정보를 이용해 토크 제한 중 적분값이 과도하게 누적되는 현상 등을 방지
    # 현재 제한 상태를 조향 PID에 전달
    controls.gm_steer_torque_limited = bool(self.steer_rate_limited)

    # GM EPS의 메시지 카운터 규칙을 지키면서 OpenPilot의 조향 요청을 안전한 토크로 제한하여 전송하는 기능
    # ================================================================================================================== #

    if CS.CP.enableGasInterceptor:
      # 이것이 없으면 저속에서 너무 공격적입니다.
      # Automatic pedal output is never allowed below 1 km/h. The driver must
      # initiate motion; launch boost can begin only after measured speed has
      # crossed that threshold.
      pedal_speed_allowed = pedal_command_allowed(CS.out.vEgo, V_CRUISE_ENABLE_MIN)
      if CS.adaptive_Cruise and not brake_pressed and not CS.out.gasPressed and \
         pedal_speed_allowed:

        # Speed-dependent pedal conversion baseline. Tune this map from logged
        # pedal command versus measured vehicle acceleration before PID gains.
        '''acc_mult = interp(CS.out.vEgo,
                          [0., 10.0 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS,
                           60.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS],
                          [0.132, 0.145, 0.158, 0.185, 0.162, 0.173, 0.184]
                          )'''


        # 가속 멀티플라이어 설정
        acc_mult = interp(CS.out.vEgo,
                          [1., 20 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS],
                          [0.186, 0.178, 0.175, 0.170, 0.172, 0.184]
                          )

        # 연비 향상을 위해 클리핑
        # The user's CommaPedalResistance profile is the whole response. Keep
        # a controller-side clamp and the existing absolute comma-pedal ceiling.
        response_gain = clip(float(getattr(
          controls, 'comma_pedal_effective_gain', 1.0)), 0.82, 1.22)
        raw_pedal = clip(acc_mult * actuators.accel, 0.0, 0.85)
        # Apply the combined response first, then predictive coasting as the final
        # positive-pedal ceiling. Coasting can only remove pedal; it can never
        # create acceleration or override brake/FCW/longitudinal zero requests.
        styled_pedal = clip(acc_mult * actuators.accel * response_gain, 0.0, 0.85)
        hard_recovery = getattr(controls, 'pedal_force_recovery', None)
        lead_assist = getattr(controls, 'lead_coast_assist', None)
        lead_loss_assist = getattr(controls, 'lead_loss_cruise_assist', None)
        moving_gap_assist = getattr(controls, 'moving_gap_catchup_assist', None)
        recovery = (hard_recovery if hard_recovery is not None and hard_recovery.active else
                    lead_loss_assist if lead_loss_assist is not None and lead_loss_assist.active else
                    lead_assist if lead_assist is not None and lead_assist.active else
                    moving_gap_assist if moving_gap_assist is not None and moving_gap_assist.active else None)
        if recovery is not None:
          # Let the response profile shape raw recovery, but never scale the
          # calibrated recovery floor itself. Predictive coasting below remains
          # the final authority.
          raw_recovery_pedal = clip(acc_mult * recovery.raw_accel * response_gain, 0.0, 0.85)
          recovery_floor = (PEDAL_FORCE_RECOVERY_PEDAL_FLOOR if recovery is hard_recovery
                            else recovery.pedal_target)
          styled_pedal = clip(max(raw_recovery_pedal, recovery_floor), 0.0, 0.85)
        coast_scale = clip(float(getattr(controls, 'predictive_coast_pedal_scale', 1.0)), 0.0, 1.0)
        self.predictive_coast_styled_pedal = float(styled_pedal)
        self.predictive_coast_pedal_scale = float(coast_scale)
        pedal_target = float(styled_pedal * coast_scale)  # Actual comma-pedal command range: 0.00..0.85
        # The launch boost and the recovery floors are separately gated, carry
        # their own confirmation logic, and measured 0-3% of this driver's brake
        # presses, so they keep owning the pedal outright.
        pedal_bypass = bool(recovery is not None or
                            getattr(controls, 'stop_accel_boost_active', False))
        controls.comma_pedal_raw_command = float(raw_pedal)
        controls.comma_pedal_styled_command = float(styled_pedal)

        # self.comma_pedal = pedal_command
      else:
        pedal_target = 0.0
        pedal_bypass = False
        self.predictive_coast_styled_pedal = 0.0
        self.predictive_coast_pedal_scale = 1.0
        controls.comma_pedal_raw_command = 0.0
        controls.comma_pedal_styled_command = 0.0

      # Applied last, after coasting and the recovery floors, so it is the final
      # authority on how fast the command may grow. Runs on every frame -- brake
      # and gas frames included -- because the post-brake window is timed from
      # the driver's release.
      self.comma_pedal = float(self.pedal_rise_limiter.update(
        pedal_target, v_ego=CS.out.vEgo, brake_pressed=brake_pressed,
        gas_pressed=bool(CS.out.gasPressed), bypass=pedal_bypass))
      controls.comma_pedal_final_command = float(self.comma_pedal)
    else:
      self.comma_pedal = 0.0
      self.predictive_coast_styled_pedal = 0.0
      self.predictive_coast_pedal_scale = 1.0
      self.pedal_rise_limiter.reset()
      controls.comma_pedal_raw_command = 0.0
      controls.comma_pedal_styled_command = 0.0
      controls.comma_pedal_final_command = 0.0



    # 4프레임마다, 0~3으로 반복되는 순번을 붙여 가속페달 제어 명령을 전송
    if CS.CP.enableGasInterceptor and (frame % 4) == 0:
      idx = (frame // 4) % 4
      can_sends.append(create_gas_interceptor_command(self.packer_pt, self.comma_pedal, idx))

    # ================================================================================================================ #
    # LKA 토크가 인가되면 녹색 아이콘을 표시하고, 토크 한계에 근접하면 경고성 주황색 아이콘을 표시합니다.
    # 추가 신호가 없으면 LKA 아이콘은 약 5초 후에 사라집니다.
    # 편리하게도, 카메라 메시지를 주기적으로 전송하는 것은 킵얼라이브(keepalive) 역할도 겸합니다.
    # 킵얼라이브(keepalive) : 디바이스간의 데이터 링크가 잘 동작하고 있는지 확인하거나 데이터 링크가 끊어지는 것을 방지하기 위해서 디바이스 간에 서로 주고받는 메시지를 말한다.
    """lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status"""

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX
    # new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.comma_pedal)

    return new_actuators, can_sends
