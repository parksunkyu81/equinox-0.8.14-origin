from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN
from selfdrive.controls.lib.pedal_follow import pedal_follow_urgent
from selfdrive.ntune import ntune_scc_get
from common.params import Params
from common.realtime import sec_since_boot

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter

CREEP_SPEED = 2.5   # 4km
LEAD_CATCHUP_PEDAL_BOOST_BP = [20.0, 30.0, 40.0, 60.0, 70.0]
LEAD_CATCHUP_PEDAL_BOOST_V = [1.00, 1.08, 1.09, 1.10, 1.00]
LEAD_CATCHUP_PEDAL_BOOST_RISE_RATE = 0.15  # multiplier / second
LEAD_CATCHUP_PEDAL_BOOST_FALL_RATE = 0.20  # multiplier / second


# =====================================================================
# Dynamic GM steering torque delta map
# ---------------------------------------------------------------------
# 목적:
#   - 10~35km/h 저속/저중속 코너: DELTA_UP 14 / DELTA_DOWN 17 고정으로 조향 응답 최대 강화
#   - 35~45km/h bridge 코너: DELTA_UP 14→12 / DELTA_DOWN 17→16으로 추종력 유지
#   - 80~110km/h 고속: DELTA_UP 7~8 / DELTA_DOWN 14~15로 와리가리 억제
#   - 작은 조향 요구/운전자 개입/직전 limit 상황에서는 자동으로 보수화
#
# 주의:
#   이 값은 apply_std_steer_torque_limits() 호출 직전에만 P에 임시 적용하고,
#   호출 후 반드시 원래 P.STEER_DELTA_UP/DOWN으로 복원한다.
# =====================================================================
DYN_DELTA_UP_BP = [0.0, 8.0, 10.0, 30.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_DELTA_UP_V  = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

DYN_DELTA_DOWN_BP = [0.0, 8.0, 10.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_DELTA_DOWN_V  = [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]

# 저속에서 작은 조향 요구일 때 UP=14가 불필요하게 열리지 않도록 demand로 블렌딩한다.
DYN_DEMAND_BP = [0.04, 0.12, 0.24, 0.40]
DYN_DEMAND_V  = [0.00, 0.35, 0.75, 1.00]

DYN_LOW_BASE_UP_BP = [0.0, 10.0, 30.0, 45.0, 60.0, 80.0, 110.0]
DYN_LOW_BASE_UP_V  = [10.0, 10.0, 10.0, 10.0, 9.0, 8.0, 7.0]

DYN_LOW_BASE_DOWN_BP = [0.0, 10.0, 30.0, 45.0, 60.0, 80.0, 110.0]
DYN_LOW_BASE_DOWN_V  = [14.0, 15.0, 15.0, 15.0, 15.0, 14.0, 14.0]


def get_dynamic_steer_delta(v_ego, new_steer, last_steer, steer_max,
                            steering_pressed=False, steer_limited_prev=False):
  """Return fixed GM steering rate limits.

  The previous speed/demand based map changed LKAS torque slew while driving and
  made high-speed steering behavior hard to reason about. Keep the actual CAN
  command path on the same conservative 10/17 limits as CarControllerParams.
  """
  return 10, 17
class CarController():

  def get_lead(self, sm):
    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne
    return None

  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.0

    # Dynamic steering delta diagnostics/state
    self._dyn_steer_limited_prev = False
    self._dyn_delta_up_last = 0
    self._dyn_delta_down_last = 0

    self.accel = 0.0
    self.lead_catchup_pedal_boost = 1.0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    #self.RestartForceAccel = Params().get_bool('RestartForceAccel')

    # 종방향 캐시값은 update() 호출마다 현재 프레임 기준으로 갱신하고,
    # 여기서는 단순 초기값만 둔다.
    self.pedal_prev = 0.0
    self.accel_start_time = None

    self.params = CarControllerParams(CP)
    self.lead_catchup_enabled = Params().get_bool('StopAccelBoost')

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, c, enabled, CS, frame, controls, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Send CAN commands.
    can_sends = []

    # ---------------------------------------------------------------
    # Longitudinal path must be updated on every update() call.
    #
    # 이전 버전은 self.accel / self.comma_pedal 갱신이 steering send gate
    # 내부에만 있어서, 조향 메시지를 건너뛴 프레임에서는 현재 요청값이 아니라
    # 직전 캐시값이 new_actuators에 남았다.
    # 그 결과 req_accel>0 인데 applied_accel=0(또는 그 반대) 같은
    # stale mismatch가 생겨 멍때림/오탐 freeze를 유발했다.
    #
    # 따라서 종방향 계산은 매 프레임 먼저 갱신하고, CAN 전송 주기만 별도로 유지한다.
    # ---------------------------------------------------------------
    brake_pressed = bool(CS.out.brakePressed)
    lead = self.get_lead(controls.sm)
    # This car cannot command the brakes. A clearly closing lead therefore
    # gets an immediate gas cut in the final GM output layer as well.
    auto_follow = controls.df_manager.is_auto
    urgent_lead_closing = self.lead_catchup_enabled and \
                          pedal_follow_urgent(lead, CS.out.vEgo)
    requested_accel = float(clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    self.accel = min(requested_accel, 0.0) if brake_pressed or urgent_lead_closing else requested_accel

    if CS.CP.enableGasInterceptor:
      # 이것이 없으면 저속에서 너무 공격적입니다.
      if c.active and CS.adaptive_Cruise and not brake_pressed and \
         CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH:

        # 가속 멀티플라이어 설정
        # 속도별 가속 배율 - 전체적으로 절반으로 줄임
        """acc_mult = interp(CS.out.vEgo,
                          [0., 10.0 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS,
                           60.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS],
                          [0.132, 0.145, 0.158, 0.185, 0.162, 0.173, 0.184]
                          # (기존: 0.12,0.132,0.144,0.168,0.18,0.192,0.204)
                          ) """

        acc_mult = interp(CS.out.vEgo,
                          [0., 10.0 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS,
                           40.0 * CV.KPH_TO_MS, 60.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS, 100.0 * CV.KPH_TO_MS],
                          [0.132, 0.145, 0.158, 0.185,
                           0.182, 0.168, 0.178, 0.188]
                          )

        catchup_active = self.lead_catchup_enabled and not auto_follow and \
                         bool(controls.sm['dynamicFollowData'].leadCatchupActive) and \
                         lead is not None and lead.vRel > 0.0 and lead.aLeadK > -0.30
        if catchup_active:
          pedal_boost_target = interp(CS.out.vEgo * CV.MS_TO_KPH,
                                      LEAD_CATCHUP_PEDAL_BOOST_BP,
                                      LEAD_CATCHUP_PEDAL_BOOST_V)
        else:
          pedal_boost_target = 1.0

        if pedal_boost_target > self.lead_catchup_pedal_boost:
          self.lead_catchup_pedal_boost = min(pedal_boost_target,
                                              self.lead_catchup_pedal_boost +
                                              LEAD_CATCHUP_PEDAL_BOOST_RISE_RATE * DT_CTRL)
        else:
          self.lead_catchup_pedal_boost = max(pedal_boost_target,
                                              self.lead_catchup_pedal_boost -
                                              LEAD_CATCHUP_PEDAL_BOOST_FALL_RATE * DT_CTRL)
        acc_mult *= self.lead_catchup_pedal_boost

        pedal_command = acc_mult * self.accel
        self.comma_pedal = float(clip(pedal_command, 0., 0.75))
      else:
        self.lead_catchup_pedal_boost = 1.0
        self.comma_pedal = 0.0
    else:
      self.lead_catchup_pedal_boost = 1.0
      self.comma_pedal = 0.0

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = c.active and not (CS.out.steerFaultTemporary or CS.out.steerFaultPermanent) and CS.out.vEgo > P.MIN_STEER_SPEED
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        base_delta_up = int(P.STEER_DELTA_UP)
        base_delta_down = int(P.STEER_DELTA_DOWN)

        steering_pressed = bool(getattr(CS.out, 'steeringPressed', False))
        dyn_delta_up, dyn_delta_down = get_dynamic_steer_delta(
          CS.out.vEgo, new_steer, self.apply_steer_last, P.STEER_MAX,
          steering_pressed=steering_pressed,
          steer_limited_prev=self._dyn_steer_limited_prev,
        )

        try:
          # apply_std_steer_torque_limits()는 P.STEER_DELTA_UP/DOWN을 읽으므로
          # 이 호출 구간에서만 동적값을 임시 적용하고 즉시 복원한다.
          P.STEER_DELTA_UP = dyn_delta_up
          P.STEER_DELTA_DOWN = dyn_delta_down
          apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        finally:
          P.STEER_DELTA_UP = base_delta_up
          P.STEER_DELTA_DOWN = base_delta_down

        # 다음 프레임의 backoff 판단용. 요청 토크를 충분히 따라가지 못하면 limit으로 본다.
        self._dyn_steer_limited_prev = bool(abs(int(apply_steer) - int(new_steer)) > 1)
        self._dyn_delta_up_last = dyn_delta_up
        self._dyn_delta_down_last = dyn_delta_down
      else:
        apply_steer = 0
        self._dyn_steer_limited_prev = False

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    if CS.CP.enableGasInterceptor and (frame % 4) == 0:
      idx = (frame // 4) % 4
      can_sends.append(create_gas_interceptor_command(self.packer_pt, self.comma_pedal, idx))

    # Show green icon when LKA(차로이탈방지보조) torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.

    #lka_active = CS.lkas_status == 1
    #lka_critical = lka_active and abs(actuators.steer) > 0.9
    #lka_icon_status = (lka_active, lka_critical)
    #if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
    #  steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
    #  can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
    #  self.lka_icon_status_last = lka_icon_status

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.comma_pedal)

    return new_actuators, can_sends
