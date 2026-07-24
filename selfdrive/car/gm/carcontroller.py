from cereal import car
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from common.clock import sec_since_boot
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN
from selfdrive.car.gm.steer_scheduler import (GMSteeringCommandScheduler, GMSteeringLimitTracker,
                                              GM_STEER_RATE_DOWN,
                                              GM_STEER_RATE_UP)

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter

CREEP_SPEED = 2.5   # 4km


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
DYN_DELTA_UP_V  = [7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0]

DYN_DELTA_DOWN_BP = [0.0, 8.0, 10.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_DELTA_DOWN_V  = [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]

# 저속에서 작은 조향 요구일 때 UP=14가 불필요하게 열리지 않도록 demand로 블렌딩한다.
DYN_DEMAND_BP = [0.04, 0.12, 0.24, 0.40]
DYN_DEMAND_V  = [0.00, 0.35, 0.75, 1.00]

DYN_LOW_BASE_UP_BP = [0.0, 10.0, 30.0, 45.0, 60.0, 80.0, 110.0]
DYN_LOW_BASE_UP_V  = [7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0]

DYN_LOW_BASE_DOWN_BP = [0.0, 10.0, 30.0, 45.0, 60.0, 80.0, 110.0]
DYN_LOW_BASE_DOWN_V  = [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]


def get_dynamic_steer_delta(v_ego, new_steer, last_steer, steer_max,
                            steering_pressed=False, steer_limited_prev=False):
  """Return fixed GM steering rate limits.

  The previous speed/demand based map changed LKAS torque slew while driving and
  made high-speed steering behavior hard to reason about. Keep the actual CAN
  command path on the same conservative 7/17 limits as CarControllerParams.
  """
  return GM_STEER_RATE_UP, GM_STEER_RATE_DOWN
class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.0

    # Dynamic steering delta diagnostics/state
    self._dyn_steer_limited_prev = False
    self._dyn_delta_up_last = 0
    self._dyn_delta_down_last = 0

    self.accel = 0.0

    self.steer_command_scheduler = GMSteeringCommandScheduler()
    self.steer_limit_tracker = GMSteeringLimitTracker()
    self.lka_icon_status_last = (False, False)
    #self.RestartForceAccel = Params().get_bool('RestartForceAccel')

    # 종방향 캐시값은 update() 호출마다 현재 프레임 기준으로 갱신하고,
    # 여기서는 단순 초기값만 둔다.
    self.pedal_prev = 0.0
    self.accel_start_time = None

    self.params = CarControllerParams(CP)

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
    raw_requested_accel = float(clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    requested_accel = raw_requested_accel
    comfort_accel_cap = 0.0
    if CS.CP.enableGasInterceptor:
      # max(0.0, 값)은 상한이 음수가 되지 않도록 합니다.
      comfort_accel_cap = max(0.0, float(controls.sm['longitudinalPlan'].accelLimitMax))
      if requested_accel > 0.0:
        requested_accel = min(requested_accel, comfort_accel_cap)
    standstill_blocked = CS.out.standstill or CS.out.vEgo <= V_CRUISE_ENABLE_MIN * CV.KPH_TO_MS
    self.accel = min(requested_accel, 0.0) if brake_pressed or standstill_blocked else requested_accel

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
                          [0., 10.0 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS],
                          [0.15, 0.165, 0.18, 0.21, 0.23, 0.25]
                          )
        # 원래 가속 명령 계산
        pedal_command = acc_mult * actuators.accel
        # 연비 향상을 위해 클리핑
        self.comma_pedal = clip(pedal_command, 0., 0.85)  # 최대 0.8까지만 허용하여 연비 개선

        # self.comma_pedal = pedal_command
      else:
        pedal_command = 0.0
        self.comma_pedal = 0.0
    else:
      pedal_command = 0.0
      self.comma_pedal = 0.0

    # Keep legacy diagnostics populated for log/schema compatibility.
    controls.pedal_deadzone_boost_candidate = False
    controls.pedal_deadzone_boost_active = False
    controls.pedal_deadzone_raw_command = float(pedal_command)
    controls.pedal_deadzone_applied_command = float(self.comma_pedal)
    controls.pedal_deadzone_floor = 0.0
    # Preserve the pre-cap request for road-log PID/pedal-map tuning. Applied
    # acceleration remains available in carControl.actuatorsOutput.accel.
    controls.pedal_deadzone_accel_request = raw_requested_accel
    controls.pedal_deadzone_vehicle_accel = float(CS.out.aEgo)
    controls.pedal_comfort_accel_cap = float(comfort_accel_cap)

    # Steering (50 Hz). Loopback updates select the next rolling counter but do
    # not suppress a due command. The monotonic scheduler also prevents rapid
    # catch-up sends when controlsd resumes after a delayed frame.
    steer_command_sent, idx = self.steer_command_scheduler.update(
      sec_since_boot(), CS.lka_steering_cmd_counter)
    lkas_enabled = c.active and not (CS.out.steerFaultTemporary or CS.out.steerFaultPermanent) and \
                   CS.out.vEgo > P.MIN_STEER_SPEED
    requested_steer = None
    applied_steer = None
    if steer_command_sent:
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        requested_steer = new_steer
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

        applied_steer = apply_steer
        self._dyn_delta_up_last = dyn_delta_up
        self._dyn_delta_down_last = dyn_delta_down
      else:
        apply_steer = 0
        self._dyn_steer_limited_prev = False

      self.apply_steer_last = apply_steer
      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    # A non-send frame is the normal 50 Hz zero-order hold, not a new actuator
    # limit. Update this state only from a command that was actually transmitted.
    self.steer_limit_tracker.update(
      steer_command_sent, lkas_enabled,
      requested_torque=requested_steer, applied_torque=applied_steer,
    )
    self._dyn_steer_limited_prev = self.steer_limit_tracker.limited

    controls.gm_steer_command_sent = bool(steer_command_sent)
    controls.gm_steer_command_gap_ms = float(self.steer_command_scheduler.last_interval * 1000.0)
    controls.gm_steer_command_deadline_lag_ms = float(self.steer_command_scheduler.deadline_lag * 1000.0)
    controls.gm_steer_command_counter = int(idx if idx is not None else 0)
    controls.gm_steer_loopback_counter = int(CS.lka_steering_cmd_counter)
    controls.gm_steer_loopback_changed = bool(self.steer_command_scheduler.loopback_changed)
    controls.gm_steer_loopback_acked = bool(self.steer_command_scheduler.loopback_acked)
    controls.gm_steer_command_gap_fault = bool(self.steer_command_scheduler.gap_fault)
    controls.gm_lkas_status = int(CS.lkas_status)
    controls.gm_steer_command_active = bool(lkas_enabled)
    controls.gm_steer_command_torque = int(self.apply_steer_last)
    controls.gm_steer_requested_torque = int(self.steer_limit_tracker.requested_torque)
    controls.gm_steer_torque_limited = bool(self.steer_limit_tracker.limited)

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
