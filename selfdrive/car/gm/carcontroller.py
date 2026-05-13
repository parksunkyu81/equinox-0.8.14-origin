from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN
from selfdrive.ntune import ntune_scc_get
from common.params import Params

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter

CREEP_SPEED = 2.5   # 4km


# Equinox 2020 diesel dynamic steering torque delta map.
# The latcontrol_torque dynamic debug map is only advisory; actual rate limiting
# happens here through apply_std_steer_torque_limits().  Low speed gets more
# delta-up authority to reduce 10~30kph steer_clip, while high speed remains
# conservative to avoid highway weave.
DYN_STEER_DELTA_UP_BP = [0.0, 8.0, 10.0, 20.0, 30.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_STEER_DELTA_UP_V  = [10.0, 12.0, 15.0, 15.0, 14.0, 13.0, 12.0, 11.0, 8.0, 7.0, 6.0, 6.0]  # v32: 10~20kph 과한 상승/80kph+ 고속 변화율 완화
DYN_STEER_DELTA_DOWN_BP = [0.0, 10.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_STEER_DELTA_DOWN_V  = [14.0, 16.0, 16.0, 16.0, 15.0, 14.0, 14.0, 13.0, 13.0]  # v32: unwind도 고속에서 조금 더 부드럽게

# Conditional low-speed delta-up assist. Keep the base map moderate, but allow
# 10~28kph clean corners to climb to 17 when the EPS is not near max and the
# driver is not overriding.
CLEAN_DELTA_UP_ENABLE = True
CLEAN_DELTA_UP_MIN_KPH = 10.0
CLEAN_DELTA_UP_MAX_KPH = 24.0
CLEAN_DELTA_UP_VALUE = 16
CLEAN_DELTA_UP_MIN_REQ = 0.20
CLEAN_DELTA_UP_MAX_REQ = 0.78
CLEAN_DELTA_UP_MAX_LAST = 0.72
CLEAN_DELTA_UP_RISING_MIN = 0.018  # v32: 진짜 clean rising corner에서만 delta-up 보조

STOP_ACCEL_BOOST_ENTRY_SPEED = 1.0
STOP_ACCEL_BOOST_EXIT_SPEED = 20.0 * CV.KPH_TO_MS
STOP_ACCEL_BOOST_MAX_FRAMES = 4.0 / DT_CTRL
STOP_ACCEL_BOOST_GAIN = 1.10
STOP_ACCEL_BOOST_MIN_DREL = 3.0
STOP_ACCEL_BOOST_EXIT_DREL = 2.8
STOP_ACCEL_BOOST_MAX_DREL = 18.0
STOP_ACCEL_BOOST_MIN_VLEAD = 0.30
STOP_ACCEL_BOOST_MIN_VREL = 0.15
STOP_ACCEL_BOOST_EXIT_VREL = -0.5
STOP_ACCEL_BOOST_EXIT_ACCEL = -5.0
STOP_ACCEL_BOOST_START_ACCEL = -5.0
STOP_ACCEL_BOOST_MIN_PEDAL = 0.17
STOP_ACCEL_BOOST_MIN_PEDAL_FRAMES = 0.8 / DT_CTRL
STOP_ACCEL_BOOST_MIN_PEDAL_SPEED = 5.0 * CV.KPH_TO_MS


class CarController():

  def get_lead(self, sm):
    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne
    return None

  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.0
    self.accel = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.params_memory = Params()
    self.stop_accel_boost = self.params_memory.get_bool('StopAccelBoost')
    self.stop_accel_boost_active = False
    self.stop_accel_boost_start_frame = 0

    self.params = CarControllerParams(CP)

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])


  def _clean_low_speed_delta_up_allowed(self, v_kph, new_steer, CS):
    if not CLEAN_DELTA_UP_ENABLE:
      return False

    try:
      v = float(v_kph)
    except Exception:
      v = 0.0
    if v < CLEAN_DELTA_UP_MIN_KPH or v > CLEAN_DELTA_UP_MAX_KPH:
      return False

    try:
      steering_pressed = bool(getattr(CS.out, 'steeringPressed', False)) or bool(getattr(CS, 'steeringPressed', False))
    except Exception:
      steering_pressed = False
    if steering_pressed:
      return False

    try:
      steer_max = float(getattr(self.params, 'STEER_MAX', 300))
      if steer_max <= 1e-6:
        steer_max = 300.0
      req = float(new_steer) / steer_max
      last = float(self.apply_steer_last) / steer_max
    except Exception:
      return False

    abs_req = abs(req)
    abs_last = abs(last)
    if abs_req < CLEAN_DELTA_UP_MIN_REQ or abs_req > CLEAN_DELTA_UP_MAX_REQ:
      return False
    if abs_last > CLEAN_DELTA_UP_MAX_LAST:
      return False

    # Only help when torque is rising in the same direction.  Sign flips or
    # near-center corrections should stay on the base map to avoid twitching.
    same_direction = (req * last) >= -0.02
    rising = abs_req > (abs_last + float(CLEAN_DELTA_UP_RISING_MIN))
    return bool(same_direction and rising)

  def _dynamic_steer_deltas(self, v_ego, new_steer=None, CS=None):
    try:
      v_kph = float(v_ego) * CV.MS_TO_KPH
    except Exception:
      v_kph = 0.0

    try:
      up = int(round(interp(v_kph, DYN_STEER_DELTA_UP_BP, DYN_STEER_DELTA_UP_V)))
      down = int(round(interp(v_kph, DYN_STEER_DELTA_DOWN_BP, DYN_STEER_DELTA_DOWN_V)))
    except Exception:
      up = int(getattr(self.params, 'STEER_DELTA_UP', 10))
      down = int(getattr(self.params, 'STEER_DELTA_DOWN', 17))

    try:
      if new_steer is not None and CS is not None and self._clean_low_speed_delta_up_allowed(v_kph, new_steer, CS):
        up = max(up, int(CLEAN_DELTA_UP_VALUE))
    except Exception:
      pass

    return max(1, up), max(1, down)

  def _stop_accel_boost_lead(self, controls):
    try:
      return self.get_lead(controls.sm)
    except Exception:
      return None

  def _stop_accel_boost_lead_moving(self, lead):
    return lead is not None and lead.status and \
      lead.vLead > STOP_ACCEL_BOOST_MIN_VLEAD and lead.vRel > STOP_ACCEL_BOOST_MIN_VREL

  def _stop_accel_boost_allowed(self, c, CS, frame, controls, actuators):
    if not self.stop_accel_boost:
      self.stop_accel_boost_active = False
      return False, None

    lead = self._stop_accel_boost_lead(controls)
    common_allowed = c.active and CS.adaptive_Cruise and not CS.out.brakePressed and not CS.out.gasPressed
    lead_valid = lead is not None and lead.status
    if not common_allowed or not lead_valid:
      self.stop_accel_boost_active = False
      return False, lead

    boost_timed_out = self.stop_accel_boost_active and frame - self.stop_accel_boost_start_frame > STOP_ACCEL_BOOST_MAX_FRAMES
    if (CS.out.vEgo >= STOP_ACCEL_BOOST_EXIT_SPEED or
            lead.dRel <= STOP_ACCEL_BOOST_EXIT_DREL or
            lead.vRel < STOP_ACCEL_BOOST_EXIT_VREL or
            actuators.accel < STOP_ACCEL_BOOST_EXIT_ACCEL or
            boost_timed_out):
      self.stop_accel_boost_active = False
      return False, lead

    lead_moving = self._stop_accel_boost_lead_moving(lead)
    if not lead_moving:
      self.stop_accel_boost_active = False
      return False, lead

    if self.stop_accel_boost_active:
      return True, lead

    start_allowed = (CS.out.vEgo < STOP_ACCEL_BOOST_ENTRY_SPEED and
                     STOP_ACCEL_BOOST_MIN_DREL < lead.dRel < STOP_ACCEL_BOOST_MAX_DREL and
                     lead_moving and
                     actuators.accel > STOP_ACCEL_BOOST_START_ACCEL)
    if start_allowed:
      self.stop_accel_boost_active = True
      self.stop_accel_boost_start_frame = frame
      return True, lead

    return False, lead

  def _stop_accel_boost_pedal(self, pedal_command, lead, v_ego, frame):
    if lead is None:
      return pedal_command

    boost_min = interp(lead.dRel,
                       [STOP_ACCEL_BOOST_MIN_DREL, 8.0, STOP_ACCEL_BOOST_MAX_DREL],
                       [0.06, 0.12, 0.18])
    boost_min *= interp(lead.vLead,
                        [STOP_ACCEL_BOOST_MIN_VLEAD, 2.0, 5.0],
                        [0.7, 1.0, 1.2])
    boost_min *= interp(v_ego,
                        [0.0, 8.0 * CV.KPH_TO_MS, STOP_ACCEL_BOOST_EXIT_SPEED],
                        [1.0, 0.65, 0.0])
    boost_min *= STOP_ACCEL_BOOST_GAIN

    boost_elapsed = frame - self.stop_accel_boost_start_frame
    if v_ego < STOP_ACCEL_BOOST_MIN_PEDAL_SPEED and boost_elapsed <= STOP_ACCEL_BOOST_MIN_PEDAL_FRAMES:
      boost_min = max(boost_min, STOP_ACCEL_BOOST_MIN_PEDAL)

    return max(pedal_command, boost_min)

  def update(self, c, enabled, CS, frame, controls, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Send CAN commands.
    can_sends = []

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = c.active and not (CS.out.steerFaultTemporary or CS.out.steerFaultPermanent) and CS.out.vEgo > P.MIN_STEER_SPEED
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))

        # Apply speed-based delta limits to the actual GM steering command path.
        # Mutate the params object only around this limiter call and restore it
        # immediately, so the rest of CarControllerParams stays unchanged.
        base_delta_up = int(getattr(P, 'STEER_DELTA_UP', 10))
        base_delta_down = int(getattr(P, 'STEER_DELTA_DOWN', 17))
        dyn_delta_up, dyn_delta_down = self._dynamic_steer_deltas(CS.out.vEgo, new_steer, CS)
        try:
          P.STEER_DELTA_UP = dyn_delta_up
          P.STEER_DELTA_DOWN = dyn_delta_down
          apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        finally:
          try:
            P.STEER_DELTA_UP = base_delta_up
            P.STEER_DELTA_DOWN = base_delta_down
          except Exception:
            pass
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

      self.accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      if CS.CP.enableGasInterceptor:
        if (frame % 50) == 0:
          self.stop_accel_boost = self.params_memory.get_bool('StopAccelBoost')

        restart_boost_allowed, restart_boost_lead = self._stop_accel_boost_allowed(c, CS, frame, controls, actuators)
        allow_pedal = c.active and CS.adaptive_Cruise and (
          CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH or restart_boost_allowed)
        # 이것이 없으면 저속에서 너무 공격적입니다.
        if allow_pedal:

          # 가속 멀티플라이어 설정
          acc_mult = interp(CS.out.vEgo,
                            [0., 10.0 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS],
                            [0.15, 0.165, 0.18, 0.21, 0.23, 0.25]
                            )
          # 원래 가속 명령 계산
          pedal_command = acc_mult * actuators.accel
          if restart_boost_allowed:
            pedal_command = self._stop_accel_boost_pedal(pedal_command, restart_boost_lead, CS.out.vEgo, frame)
          # 연비 향상을 위해 클리핑
          self.comma_pedal = clip(pedal_command, 0., 0.85)  # 최대 0.8까지만 허용하여 연비 개선

          if restart_boost_allowed:
            self.comma_pedal = clip(self.comma_pedal, 0., 0.35)

          # longitudinal with FrogPilot
          """zero = 0.15625  # 40/256
          if actuators.accel > 0.:
            # Scales the accel from 0-1 to 0.156-1
            self.comma_pedal = clip(((1 - zero) * actuators.accel + zero), 0., 1.)
          else:
            # if accel is negative, -0.1 -> 0.015625
            self.comma_pedal = clip(zero + actuators.accel, 0., zero)  # Make brake the same size as gas, but clip to regen
          """
          # End...

        elif not allow_pedal:
          self.comma_pedal = 0.0

        if (frame % 4) == 0:
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
    new_actuators.accel = self.accel
    new_actuators.gas = self.comma_pedal

    return new_actuators, can_sends
