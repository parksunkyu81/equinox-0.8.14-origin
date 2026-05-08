#!/usr/bin/env python3
from cereal import car
from common.numpy_fast import interp
from math import fabs
from common.conversions import Conversions as CV
from selfdrive.ntune import ntune_scc_get
from selfdrive.car.gm.values import CAR, CruiseButtons, CarControllerParams, NO_ASCM
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, \
    get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.ntune import ntune_common_get, ntune_torque_get
from common.params import Params
from decimal import Decimal

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter


def get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED):
    x = ANGLE * (desired_angle + ANGLE_OFFSET)
    sigmoid = x / (1 + fabs(x))
    return (SIGMOID_SPEED * sigmoid * v_ego) + (SIGMOID * sigmoid) + (SPEED * v_ego)

class CarInterface(CarInterfaceBase):

    @staticmethod
    def get_pid_accel_limits(CP, current_speed, cruise_speed):
       params = CarControllerParams(CP)
       return params.ACCEL_MIN, params.ACCEL_MAX
       #v_current_kph = current_speed * CV.MS_TO_KPH
       #accel_max_bp = [10., 20., 50.]
       #accel_max_v = [0.7, 1.0, 0.95]
       #return params.ACCEL_MIN, interp(v_current_kph, accel_max_bp, accel_max_v)

    # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
    @staticmethod
    def get_steer_feedforward_bolt_euv(desired_angle, v_ego):
        ANGLE = 0.0758345580739845
        ANGLE_OFFSET = 0.31396926577596984
        SIGMOID_SPEED = 0.04367532050459129
        SIGMOID = 0.43144116109994846
        SPEED = -0.002654134623368279
        return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)

    @staticmethod
    def get_steer_feedforward_bolt(desired_angle, v_ego):
        ANGLE = 0.06370624896135679
        ANGLE_OFFSET = 0.32536345911579184
        SIGMOID_SPEED = 0.06479105208670367
        SIGMOID = 0.34485246691603205
        SPEED = -0.0010645479469461995
        return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)

    def get_steer_feedforward_function(self):
        lateral_control = Params().get("LateralControl", encoding='utf-8')
        if lateral_control == 'PID':
            return self.get_steer_feedforward_bolt_euv    # bolt EUV
        else:
            return CarInterfaceBase.get_steer_feedforward_default

    @staticmethod
    def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
        ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
        ret.carName = "gm"
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
        ret.alternativeExperience = 1  # UNSAFE_DISABLE_DISENGAGE_ON_GAS
        ret.pcmCruise = False  # stock cruise control is kept off
        ret.openpilotLongitudinalControl = True  # ASCM vehicles use OP for long
        ret.radarOffCan = False  # ASCM vehicles (typically) have radar

        # Default to normal torque limits
        ret.safetyConfigs[0].safetyParam = 0

        # Start with a baseline lateral tuning for all GM vehicles. Override tuning as needed in each model section below.
        ret.enableGasInterceptor = 0x201 in fingerprint[0]
        #ret.minEnableSpeed = 18 * CV.MPH_TO_MS
        ret.minSteerSpeed = 11 * CV.KPH_TO_MS
        ret.minEnableSpeed = -1
        #ret.steerRateCost = 0.35  # def : 2.0

        # steerActuatorDelay, steerMaxV 커질수록 인으로 붙고, scale 작을수록 인으로 붙는다.
        # steeractuatordelay는 계산된 주행곡선을 좀더 빠르게 혹은 느리게 반영할지를 결정합니다
        ret.steerActuatorDelay = max(ntune_common_get('steerActuatorDelay'), 0.1)

        ret.mass = 3500. * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.72
        ret.centerToFront = ret.wheelbase * 0.4
        ret.steerRatio = 16.8  # default Equinox 2019
        # no rear steering, at least on the listed cars above
        ret.steerRatioRear = 0.
        ret.steerControlType = car.CarParams.SteerControlType.torque

        tire_stiffness_factor = 0.444  # 1. 을 기준으로 줄면 민감(오버), 커지면 둔감(언더) DEF : 0.5
        ret.maxSteeringAngleDeg = 1000.  # 최대 조향 각도
        #ret.disableLateralLiveTuning = True

        lateral_control = Params().get("LateralControl", encoding='utf-8')
        if lateral_control == 'INDI':
            ret.lateralTuning.init('indi')
            ret.lateralTuning.indi.innerLoopGainBP = [0.]
            ret.lateralTuning.indi.innerLoopGainV = [3.3]
            ret.lateralTuning.indi.outerLoopGainBP = [0.]
            ret.lateralTuning.indi.outerLoopGainV = [2.8]
            ret.lateralTuning.indi.timeConstantBP = [0.]
            ret.lateralTuning.indi.timeConstantV = [1.4]
            ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
            ret.lateralTuning.indi.actuatorEffectivenessV = [1.8]
        elif lateral_control == 'LQR':
            ret.lateralTuning.init('lqr')
            ret.lateralTuning.lqr.scale = 1955.0
            ret.lateralTuning.lqr.ki = 0.0315
            ret.lateralTuning.lqr.dcGain = 0.002237852961363602
            ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
            ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
            ret.lateralTuning.lqr.c = [1., 0.]
            ret.lateralTuning.lqr.k = [-110.73572306, 451.22718255]
            ret.lateralTuning.lqr.l = [0.3233671, 0.3185757]
        elif lateral_control == 'PID':
            ret.lateralTuning.init('pid')
            ret.minEnableSpeed = -1
            ret.mass = 1616. + STD_CARGO_KG
            ret.wheelbase = 2.60096
            ret.steerRatio = 16.8
            ret.steerRatioRear = 0.
            ret.centerToFront = 2.0828  # ret.wheelbase * 0.4 # wild guess
            tire_stiffness_factor = 1.0
            # still working on improving lateral
            # ret.steerRateCost = 0.5
            ret.steerActuatorDelay = 0.
            ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[10., 41.0], [10., 41.0]]
            ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.14, 0.24], [0.01, 0.021]]
            # ret.lateralTuning.pid.kdBP = [0.]
            # ret.lateralTuning.pid.kdV = [0.5]
            ret.lateralTuning.pid.kf = 1.  # for get_steer_feedforward_bolt()
        else:
            params = Params()
            ret.lateralTuning.init('torque')

            try:
              # torque_lat_accel_factor = ntune_torque_get('latAccelFactor')  # LAT_ACCEL_FACTOR
              # torque_friction = ntune_torque_get('friction')  # FRICTION
              torque_lat_accel_factor = 1.88  # LAT_ACCEL_FACTOR
              torque_friction = 0.255  # FRICTION
            except:
              torque_lat_accel_factor = float(
                    Decimal(params.get("TorqueMaxLatAccel", encoding="utf8")) * Decimal('0.1'))  # LAT_ACCEL_FACTOR
              torque_friction = float(
                    Decimal(params.get("TorqueFriction", encoding="utf8")) * Decimal('0.001'))  # FRICTION
            CarInterfaceBase.configure_torque_tune(ret.lateralTuning, torque_lat_accel_factor, torque_friction)


        # TODO: get actual value, for now starting with reasonable value for
        # civic and scaling by mass and wheelbase
        ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

        # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
        # mass and CG position, so all cars will have approximately similar dyn behaviors
        ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                             tire_stiffness_factor=tire_stiffness_factor)

        # longitudinal
        # GPT recommand code
        # 가속감이 줄어들고, 더 부드러운 가속 및 감속이 가능해지며, 연비가 개선
        ret.longitudinalTuning.kpBP = [0., 5. * CV.KPH_TO_MS, 10. * CV.KPH_TO_MS, 20. * CV.KPH_TO_MS,
                                       30. * CV.KPH_TO_MS, 50. * CV.KPH_TO_MS, 60. * CV.KPH_TO_MS,
                                       80. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
        #ret.longitudinalTuning.kpV = [0.86, 0.79, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4]  # Reduced kpV values for smoother acceleration
        ret.longitudinalTuning.kpV = [0.8, 0.74, 0.66, 0.61, 0.56, 0.51, 0.46, 0.42, 0.38]
        ret.longitudinalTuning.kiBP = [0., 25. * CV.KPH_TO_MS, 130. * CV.KPH_TO_MS]
        #ret.longitudinalTuning.kiV = [0.18, 0.13, 0.1]  # [0.1, 0.075, 0.05] Ki 값을 높이면 시스템이 오차를 빠르게 보상하려고 하여 반응이 빨라짐
        ret.longitudinalTuning.kiV = [0.16, 0.12, 0.08]
        ret.longitudinalActuatorDelayLowerBound = 0.05  # 값을 줄이면 액츄에이터의 지연시간이 감소하여 보다 빠른 응답
        ret.longitudinalActuatorDelayUpperBound = 0.1

        ret.steerLimitTimer = 0.4
        ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

        return ret

    def _update(self, c: car.CarControl) -> car.CarState:
        pass

    # returns a car.CarState
    def update(self, c, can_strings):
        self.cp.update_strings(can_strings)
        self.cp_loopback.update_strings(can_strings)

        ret = self.CS.update(self.cp, self.cp_loopback)

        ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
        ret.cruiseState.enabled = ret.cruiseState.available

        buttonEvents = []

        if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
            be = car.CarState.ButtonEvent.new_message()
            be.type = ButtonType.unknown
            if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
                be.pressed = True
                but = self.CS.cruise_buttons
            else:
                be.pressed = False
                but = self.CS.prev_cruise_buttons
            if but == CruiseButtons.RES_ACCEL:
                if not (ret.cruiseState.enabled and ret.standstill):
                    be.type = ButtonType.accelCruise  # Suppress resume button if we're resuming from stop so we don't adjust speed.
            elif but == CruiseButtons.DECEL_SET:
                be.type = ButtonType.decelCruise
            elif but == CruiseButtons.CANCEL:
                be.type = ButtonType.cancel
            elif but == CruiseButtons.MAIN:
                be.type = ButtonType.altButton3
            buttonEvents.append(be)

        ret.buttonEvents = buttonEvents
        # TODO: JJS Move this to appropriate place (check other brands)
        EXTRA_GEARS = [GearShifter.sport, GearShifter.low, GearShifter.eco, GearShifter.manumatic]
        events = self.create_common_events(ret, extra_gears=EXTRA_GEARS, pcm_enable=self.CS.CP.pcmCruise)

        # if ret.vEgo < self.CP.minEnableSpeed:
        #  events.add(EventName.belowEngageSpeed)
        # if self.CS.park_brake:
        #  events.add(EventName.parkBrake)
        # if ret.cruiseState.standstill:
        #  events.add(EventName.resumeRequired)
        # if (self.CS.CP.carFingerprint not in NO_ASCM) and self.CS.pcm_acc_status == AccState.FAULTED:
        #  events.add(EventName.accFaulted)
        # if ret.vEgo < self.CP.minSteerSpeed:
        #  events.add(car.CarEvent.EventName.belowSteerSpeed)

        # handle button presses
        # for b in ret.buttonEvents:
        # do enable on both accel and decel buttons
        #  if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
        #    events.add(EventName.buttonEnable)
        # do disable on button down
        #  if b.type == ButtonType.cancel and b.pressed:
        #    events.add(EventName.buttonCancel)

        ###

        if self.CP.enableGasInterceptor:
            # 가속 페달 장착 차량
            for b in ret.buttonEvents:
                if (b.type in [ButtonType.decelCruise, ButtonType.accelCruise]) and not b.pressed:
                    self.CS.adaptive_Cruise = True
                    self.CS.enable_lkas = True
                    events.add(EventName.buttonEnable)
                    break
                if (b.type == ButtonType.cancel and b.pressed):
                    self.CS.adaptive_Cruise = False
                    self.CS.enable_lkas = False
                    events.add(EventName.buttonCancel)
                    break
                if (b.type == ButtonType.altButton3 and b.pressed):
                    # MAIN 버튼은 단순히 main_on 플래그를 토글하는 역할로만 두고
                    # adaptive_Cruise 상태는 main_on 상태에 맞춰 아래에서 반영
                    break

            # MAIN 상태에 따라 adaptiveCruise 반영
            if self.CS.main_on:
                self.CS.adaptive_Cruise = False
                self.CS.enable_lkas = True
            else:
                self.CS.adaptive_Cruise = True
                self.CS.enable_lkas = True

        else:
            # 페달 없는 차량
            if self.CS.main_on:
                self.CS.adaptive_Cruise = True
                self.CS.enable_lkas = True
            else:
                self.CS.adaptive_Cruise = False
                self.CS.enable_lkas = True

        ###

        ret.events = events.to_msg()

        # copy back carState packet to CS
        self.CS.out = ret.as_reader()

        return self.CS.out

    def apply(self, c, controls):
        hud_control = c.hudControl
        hud_v_cruise = hud_control.setSpeed
        if hud_v_cruise > 70:
            hud_v_cruise = 0

        # For Openpilot, "enabled" includes pre-enable.
        # In GM, PCM faults out if ACC command overlaps user gas.
        # Does not apply when no built-in ACC
        # TODO: This isn't working right... should maybe use unsafe blah blah
        # pedal was disengaging
        if not self.CP.enableGasInterceptor or self.CP.carFingerprint in NO_ASCM:
            enabled = c.enabled  # and not self.CS.out.gasPressed
        else:
            enabled = c.enabled

        new_actuators, can_sends = self.CC.update(c, enabled, self.CS, self.frame,
                                                  controls,
                                                  c.actuators,
                                                  hud_v_cruise, hud_control.lanesVisible,
                                                  hud_control.leadVisible, hud_control.visualAlert)

        self.frame += 1
        return new_actuators, can_sends
