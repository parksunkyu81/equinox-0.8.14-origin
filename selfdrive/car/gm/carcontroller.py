from cereal import car
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.0

    # Dynamic steering delta diagnostics/state
    self._dyn_steer_limited_prev = False
    self._dyn_delta_up_last = 0
    self._dyn_delta_down_last = 0

    self.accel = 0.0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, c, enabled, CS, frame, controls, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Send CAN commands.
    can_sends = []

    brake_pressed = bool(CS.out.brakePressed)

    # Steering (50Hz)
    # 메시지를 너무 짧은 간격으로 전송할 때 발생하는 GM EPS 오류를 방지하십시오.
    # 현재 CS 프레임 내에서 다음 Panda 루프백 확인(loopback confirmation)을 방금 수신했다면 해당 전송을 건너뛰어야 합니다.
    lkas_enabled = c.active and not (
      CS.out.steerFaultTemporary or CS.out.steerFaultPermanent
    ) and CS.out.vEgo > P.MIN_STEER_SPEED

    if not lkas_enabled:
      self.steer_rate_limited = False

    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last: # 차량에서 수신한 LKA 조향 메시지 카운터가 이전 값과 달라졌는지 확인
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter   # 카운터가 변경됐다면 차량의 새로운 조향 메시지가 수신된 것이므로, 마지막 카운터를 갱신
    elif (frame % P.STEER_STEP) == 0:  # 수신 카운터가 변경되지 않았고, 현재 프레임이 조향 명령 전송 주기일 때만 (2프레임마다 조향 명령을 계산하고 전송)
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
      # 차량에서 마지막으로 수신한 카운터에 1을 더해 다음 값을 만듭니다. (0 → 1 → 2 → 3)
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    # 현재 조향 토크가 제한됐는지를 상위 제어기인 controls에 전달
    # 조향 PID 제어기는 이 정보를 이용해 토크 제한 중 적분값이 과도하게 누적되는 현상 등을 방지
    # 현재 제한 상태를 조향 PID에 전달
    controls.gm_steer_torque_limited = bool(self.steer_rate_limited)

    # GM EPS의 메시지 카운터 규칙을 지키면서 OpenPilot의 조향 요청을 안전한 토크로 제한하여 전송하는 기능
    # ================================================================================================================== #

    if CS.CP.enableGasInterceptor:
      # 이것이 없으면 저속에서 너무 공격적입니다.
      pedal_speed_allowed = CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH
      if c.active and CS.adaptive_Cruise and not brake_pressed and not CS.out.gasPressed and \
         pedal_speed_allowed:

        # Speed-dependent pedal conversion baseline. Tune this map from logged
        # pedal command versus measured vehicle acceleration before PID gains.
        """acc_mult = interp(CS.out.vEgo,
                          [0., 10.0 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS,
                           60.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS],
                          [0.132, 0.145, 0.158, 0.185, 0.162, 0.173, 0.184]
                          # (기존: 0.12,0.132,0.144,0.168,0.18,0.192,0.204)
                          )

        pedal_command = float(clip(acc_mult * self.accel, 0., 0.75))"""

        # 가속 멀티플라이어 설정
        acc_mult = interp(CS.out.vEgo,
                          [0., 20 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS],
                          [0.185, 0.178, 0.175, 0.170, 0.172, 0.183]
                          #[0.18, 0.21, 0.23, 0.25]
                          )
        # 원래 가속 명령 계산
        pedal_command = acc_mult * actuators.accel
        # 연비 향상을 위해 클리핑
        self.comma_pedal = clip(pedal_command, 0., 0.85)  # 최대 0.8까지만 허용하여 연비 개선

        # self.comma_pedal = pedal_command
      else:
        self.comma_pedal = 0.0
    else:
      self.comma_pedal = 0.0



    # 4프레임마다, 0~3으로 반복되는 순번을 붙여 가속페달 제어 명령을 전송
    if CS.CP.enableGasInterceptor and (frame % 4) == 0:
      idx = (frame // 4) % 4
      can_sends.append(create_gas_interceptor_command(self.packer_pt, self.comma_pedal, idx))

    # ================================================================================================================ #
    # LKA 토크가 인가되면 녹색 아이콘을 표시하고, 토크 한계에 근접하면 경고성 주황색 아이콘을 표시합니다.
    # 추가 신호가 없으면 LKA 아이콘은 약 5초 후에 사라집니다.
    # 편리하게도, 카메라 메시지를 주기적으로 전송하는 것은 킵얼라이브(keepalive) 역할도 겸합니다.
    # 킵얼라이브(keepalive) : 디바이스간의 데이터 링크가 잘 동작하고 있는지 확인하거나 데이터 링크가 끊어지는 것을 방지하기 위해서 디바이스 간에 서로 주고받는 메시지를 말한다.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.comma_pedal)

    return new_actuators, can_sends
