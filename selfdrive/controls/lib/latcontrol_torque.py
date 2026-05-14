import math

from cereal import log
from common.numpy_fast import interp, clip
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from common.params import Params
from decimal import Decimal
import cereal.messaging as messaging

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

# LOW_SPEED_X = [0, 10, 20, 30] #comma
# LOW_SPEED_Y = [15, 13, 10, 5] #comma
LOW_SPEED_X = [0, 5, 10, 20, 30]
LOW_SPEED_Y = [15, 10, 0, 0, 5]

# ============================================================
# Equinox 2020 Diesel low-speed curvature compensation
# ------------------------------------------------------------
# CS.vEgo is m/s, not km/h.
# The previous table [0, 10, 20] kept LOW_SPEED_FACTOR alive up to
# 72km/h, which can cause mid/high-speed oscillation. This table keeps
# the very-low-speed corner response, then removes the helper by 50km/h.
# 3.00m/s  = 10.8km/h
# 5.00m/s  = 18.0km/h
# 8.33m/s  = 30.0km/h
# 13.89m/s = 50.0km/h
# ============================================================
LOW_SPEED_FACTOR_ENABLED_DEFAULT = True
LOW_SPEED_FACTOR_BP = [0.0, 3.0, 5.0, 8.33, 13.89]
LOW_SPEED_FACTOR_V = [420.0, 420.0, 260.0, 80.0, 0.0]

# Fallback when IsLowSpeedFactor is explicitly disabled.
# Still leaves a small low-speed helper, but removes it by 30km/h.
LOW_SPEED_FACTOR_OFF_BP = [0.0, 3.0, 5.0, 8.33]
LOW_SPEED_FACTOR_OFF_V = [300.0, 300.0, 120.0, 0.0]

# ==============================
# Curvature request guard
# 30km/h 이상에서 목표 곡률(desired_curvature) 변화를 완화한다.
# 중속에서는 부드럽게, 고속에서는 더 보수적으로 적용한다.
# ==============================
HS_CURV_GUARD_ON_KPH = 40.0

# update 1회당 desired_curvature 최대 변화량
HS_CURV_DELTA_MAX_BP = [30.0, 45.0, 70.0, 90.0, 110.0, 130.0]
HS_CURV_DELTA_MAX_V = [0.00100, 0.00072, 0.00050, 0.00032, 0.00020, 0.00014]  # v32: 80~110kph 고속 곡률 변화 더 보수화

# desired_curvature_rate 절대값 제한
HS_CURV_RATE_MAX_BP = [30.0, 45.0, 70.0, 90.0, 110.0, 130.0]
HS_CURV_RATE_MAX_V = [0.030, 0.023, 0.016, 0.010, 0.0065, 0.0045]  # v32: 80~110kph rate spike 억제

# 저역통과 필터 alpha (작을수록 더 부드러움)
HS_CURV_ALPHA_BP = [30.0, 45.0, 70.0, 90.0, 110.0, 130.0]
HS_CURV_ALPHA_V = [0.58, 0.50, 0.40, 0.30, 0.20, 0.14]  # v32: 고속 desired_curvature LPF 강화

# 좌/우 부호가 갑자기 뒤집히는 경우 완화
HS_SIGN_FLIP_MIN_CURV = 0.0008
HS_SIGN_FLIP_KEEP_RATIO = 0.12

# steer_limited / saturated 직후 몇 프레임 더 보수적으로 유지할지
HS_LIMIT_HOLD_BP = [30.0, 60.0, 90.0, 110.0, 130.0]
HS_LIMIT_HOLD_V = [4.0, 6.0, 8.0, 12.0, 16.0]

HS_LIMIT_DELTA_SHRINK = 0.55
HS_LIMIT_ALPHA_SHRINK = 0.75

# Low-speed adaptive slew guard.
# It does not reduce steady-state steering authority. It only slows a sudden
# jump when the requested steer is far ahead of the last actually applied steer.
LS_ADAPTIVE_SLEW_MIN_KPH = 8.0
LS_ADAPTIVE_SLEW_FULL_ON_KPH = 12.0
LS_ADAPTIVE_SLEW_FULL_OFF_KPH = 28.0
LS_ADAPTIVE_SLEW_MAX_KPH = 34.0
LS_ADAPTIVE_SLEW_GAP_START = 0.45
LS_ADAPTIVE_SLEW_GAP_FULL = 1.00
LS_ADAPTIVE_SLEW_ALLOW_GAP_BP = [8.0, 12.0, 20.0, 28.0, 34.0]
LS_ADAPTIVE_SLEW_ALLOW_GAP_V = [1.00, 0.78, 0.72, 0.78, 1.00]

# Safety output torque slew guard.
# This is separate from model curvature smoothing and protects against abrupt
# torque jumps when the controller/car delta-up is too permissive.
STABLE_TORQUE_SLEW_ENABLED = True
# 저속은 더 빠르게, 고속은 더 안정적으로 torque slew 제한.
# 목적: 10~30kph 코너 추종력 확보 + 80~110kph 와리가리 억제.
STABLE_TORQUE_SLEW_KPH_BP = [0.0, 10.0, 20.0, 30.0, 35.0, 40.0, 45.0, 70.0, 90.0, 110.0, 130.0]
STABLE_TORQUE_UP_V =       [0.065, 0.092, 0.098, 0.090, 0.078, 0.062, 0.048, 0.026, 0.018, 0.014, 0.012]  # v32: 80~110kph output slew 보수화
STABLE_TORQUE_DOWN_V =     [0.085, 0.115, 0.120, 0.108, 0.098, 0.078, 0.062, 0.038, 0.029, 0.022, 0.018]
STABLE_TORQUE_LIMITED_SHRINK = 0.85

# ==============================
# Dynamic effective torque profile
# ==============================
# liveTorqueParameters 자체는 학습값 그대로 유지하고,
# 실제 torque 계산에만 임시 effective latAccelFactor/friction을 적용한다.
DYN_TORQUE_PROFILE_ENABLED = True

# 로그 기반 목표값: 저속은 차가 실제로 받아줄 수 있는 범위로 완화하고, 고속은 안정성을 유지한다.
# 2026-05-11 logfix: 10~30kph steer_clip/rate_limit가 높아 latAF/friction 목표를 한 단계 완화.
DYN_LAT_FACTOR_BP = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 45.0, 60.0, 80.0, 100.0, 110.0, 130.0]
# v5 10~45kph 통합 개선:
#  - 10~30kph는 1.74~1.76 계열로 완화해 요구 토크가 적용 한계를 계속 앞지르지 않게 한다.
#  - 30~35kph는 부드러운 bridge로 연결한다.
#  - 60kph 이상은 기존 고속 안정 profile을 유지한다.
DYN_LAT_FACTOR_V  = [1.88, 1.76, 1.745, 1.74, 1.75, 1.76, 1.80, 1.86, 1.91, 1.93, 1.95, 1.96, 1.96]  # v33: 10~35kph clip-heavy logs need more authority
DYN_FRICTION_BP   = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 45.0, 60.0, 80.0, 100.0, 110.0, 130.0]
DYN_FRICTION_V    = [0.255, 0.286, 0.290, 0.290, 0.286, 0.282, 0.274, 0.266, 0.256, 0.252, 0.248, 0.246, 0.246]  # v33: strengthen 10~35kph static-friction assist

# 실제 CarController의 STEER_DELTA_UP/DOWN은 carcontroller 쪽에서 적용해야 한다.
# 아래 맵은 이 파일 안에서는 torque slew와 디버그용 목표값으로만 사용한다.
DYN_DELTA_UP_BP   = [0.0, 10.0, 30.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_DELTA_UP_V    = [10.0, 14.0, 14.0, 14.0, 13.0, 12.0, 9.0, 8.0, 7.0, 7.0]
DYN_DELTA_DOWN_BP = [0.0, 10.0, 35.0, 40.0, 45.0, 60.0, 80.0, 100.0, 110.0]
DYN_DELTA_DOWN_V  = [14.0, 17.0, 17.0, 17.0, 16.0, 15.0, 15.0, 14.0, 14.0]

# 코너 강도 판정. 세 값 중 가장 큰 값을 사용한다.
# 코너 감지 민감도 보정.
# 기존 임계가 높으면 완만한 저속 코너에서 corner_strength=0에 가까워져
# effective LatAccelFactor/Friction이 표시상/체감상 거의 변하지 않는 문제가 있었다.
DYN_CURV_STRENGTH_BP = [0.00018, 0.00135]
DYN_LATACC_STRENGTH_BP = [0.035, 0.65]
DYN_STEER_STRENGTH_BP = [0.012, 0.18]

# 저속/저중속 부스트 속도 게이트: 10~35kph 완전 ON, 35~45kph bridge로 점진 완화.
# v4: 10~30kph 강한 개선을 35kph까지 유지하고, 35~45kph 추종력 공백을 제거한다.
DYN_LOW_SPEED_GATE_BP = [0.0, 8.0, 10.0, 30.0, 35.0, 40.0, 45.0, 50.0]
DYN_LOW_SPEED_GATE_V  = [0.0, 0.0, 1.00, 1.00, 1.00, 0.70, 0.40, 0.0]

# 45~60kph 중속 코너 보조 게이트.
# 저속 부스트는 35kph 이후 줄이되, 램프/완만한 중속 코너에서 기본값으로 너무 빨리 죽지 않게 한다.
DYN_MID_SPEED_GATE_BP = [35.0, 40.0, 45.0, 55.0, 60.0, 70.0]
DYN_MID_SPEED_GATE_V  = [0.25, 0.45, 0.55, 0.55, 0.30, 0.0]

# 고속 안정 게이트: 60kph 이상에서는 조향을 둔감하게 만들어 와리가리 억제.
DYN_HIGH_SPEED_GATE_BP = [45.0, 60.0, 80.0, 110.0, 130.0]
DYN_HIGH_SPEED_GATE_V  = [0.0, 0.30, 0.85, 1.00, 1.00]  # v32: 60~90kph부터 안정 profile 조기 반영

# 부스트 램프/홀드. 프레임 기반이며 controls update 주기에 독립적으로 안전하게 동작한다.
DYN_BOOST_RISE_STEP = 0.10
# v32: 10~20kph clip이 42%까지 오른 원인을 줄이기 위해 저속 코너 부스트 상승을 별도 완화한다.
# v33: let low-speed effective torque reach the target earlier in the
# 10~35kph bands where 2026-05-13/14 logs still show repeated steer_clip.
DYN_BOOST_RISE_LOW1020_STEP = 0.080
DYN_BOOST_RISE_LOW2035_STEP = 0.095
DYN_BOOST_FALL_STEP = 0.035
DYN_LOW_SPEED_HOLD_FRAMES = 70  # 약 0.70초 @100Hz: 과한 저속 boost hold 완화

# limit 상황에서는 더 밀어붙이지 않고 부스트를 줄인다.
DYN_STEER_LIMITED_BOOST_MULT = 0.70
DYN_TORQUE_SLEW_ACTIVE_MULT = 0.76

# steeringPressed가 True라고 해서 저속 코너 dynamic boost를 0으로 끄면,
# 운전자가 핸들을 살짝 잡은 일반 코너에서 LatAccelFactor/Friction 보조가 전혀 체감되지 않는다.
# 강한 운전자 조향 개입은 차단하되, 가벼운 steeringPressed 상태에서는 저속 코너 보조를 일부 유지한다.
DYN_STEERING_PRESSED_LOW_BOOST_MULT = 0.65
DYN_STEERING_PRESSED_MID_BOOST_MULT = 0.55
DYN_STEERING_PRESSED_LOW_MIN_BOOST = 0.65
DYN_STEERING_PRESSED_BRIDGE_MIN_BOOST = 0.55
DYN_DRIVER_TORQUE_HARD_DISABLE = 30.0

# v2: 이전 프레임의 강한 rate-limit/추종 gap을 직접 backoff 입력으로 사용한다.
# 외부 인터페이스를 바꾸지 않고, 직전 프레임에서 low-speed slew 또는 stable torque slew가
# 큰 gap을 만들었는지 저장했다가 다음 프레임 dynamic boost를 줄인다.
DYN_RATE_LIMITED_STRONG_BOOST_MULT = 0.66
DYN_RATE_LIMITED_STRONG_TRACKING_GAP = 0.40
DYN_RATE_LIMITED_STRONG_OUTPUT_GAP = 0.16

# 저속 코너 최소 boost 보장값. 제한이 감지되면 기존처럼 1.00으로 다시 밀어붙이지 않고
# 0.72~0.86 범위로 후퇴시켜 steer_clip/rate_limit 반복을 줄인다.
DYN_LOW35_MIN_BOOST_NORMAL = 0.86  # v32: 10~20kph 작은/완만 코너에서 과요구 완화
DYN_LOW35_MIN_BOOST_LIMITED = 0.76
DYN_LOW35_MIN_BOOST_STRONG = 0.66
DYN_BRIDGE_MIN_BOOST_NORMAL_BP = [35.0, 40.0, 45.0]
DYN_BRIDGE_MIN_BOOST_NORMAL_V = [0.78, 0.65, 0.52]
DYN_BRIDGE_MIN_BOOST_LIMITED_MULT = 0.82
DYN_BRIDGE_MIN_BOOST_STRONG_MULT = 0.72

# 최종 안전 클램프
DYN_LAT_FACTOR_MIN = 1.68
DYN_LAT_FACTOR_MAX = 1.965
DYN_FRICTION_MIN = 0.245
DYN_FRICTION_MAX = 0.288


class LatControlTorque(LatControl):
    def __init__(self, CP, CI):
        super().__init__(CP, CI)
        self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                                 k_f=CP.lateralTuning.torque.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
        self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
        self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
        self.steering_angle_deadzone_deg = CP.lateralTuning.torque.steeringAngleDeadzoneDeg
        self.params = Params()
        self.update_live_torque_params(CP.lateralTuning.torque.latAccelFactor, CP.lateralTuning.torque.latAccelOffset,
                                       CP.lateralTuning.torque.friction)

        # high-speed conservative guard state
        self._hs_prev_desired_curvature = 0.0
        self._hs_prev_desired_curvature_rate = 0.0
        self._hs_guard_hold_frames = 0
        self._stable_prev_output_torque = 0.0
        self._stable_torque_slew_gap = 0.0
        self._stable_torque_slew_active = False

        # dynamic effective torque state
        self._dyn_corner_boost = 0.0
        self._dyn_corner_hold_frames = 0
        # _dyn_base_live_torque_params는 torqued/liveTorqueParameters에서 받은 "학습 기준값"이고,
        # live_torque_params는 현재 프레임에 실제 적용되는 effective 값을 보이도록 갱신한다.
        # 이렇게 하지 않으면 실제 토크 계산은 dynamic 값을 쓰더라도 디버그/화면에서는 고정값처럼 보인다.
        self._dyn_base_live_torque_params = dict(self.live_torque_params)
        self._dyn_last_effective_params = dict(self.live_torque_params)
        self._dyn_effective_active = False
        self._dyn_last_blend = 0.0
        self._dyn_last_corner_strength = 0.0
        self._dyn_last_low_speed_gate = 0.0
        self._dyn_last_mid_speed_gate = 0.0
        self._dyn_last_high_speed_gate = 0.0
        self._dyn_prev_rate_limited_strong = False
        self._dyn_prev_rate_limit_err = 0.0
        self._dyn_last_rate_limited_strong = False
        self._dyn_last_target_delta_up = 10.0
        self._dyn_last_target_delta_down = 14.0

    def _get_bool_param_default(self, name, default_value):
        """Read Params bool safely; use default_value when the key is missing."""
        try:
            raw = self.params.get(name)
            if raw is None:
                return bool(default_value)
            return bool(self.params.get_bool(name))
        except Exception:
            return bool(default_value)

    def _low_speed_factor(self, v_ego):
        """Equinox 2020 diesel low-speed factor. v_ego is m/s."""
        try:
            v = float(v_ego)
            if not math.isfinite(v):
                v = 0.0
        except Exception:
            v = 0.0

        is_low_speed_factor = self._get_bool_param_default(
            'IsLowSpeedFactor', LOW_SPEED_FACTOR_ENABLED_DEFAULT
        )
        if is_low_speed_factor:
            return float(interp(v, LOW_SPEED_FACTOR_BP, LOW_SPEED_FACTOR_V))
        return float(interp(v, LOW_SPEED_FACTOR_OFF_BP, LOW_SPEED_FACTOR_OFF_V))

    def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction, totalBucketPoints=0):
        base_params = {
            'latAccelFactor': latAccelFactor,
            'friction': friction,
            'latAccelOffset': latAccelOffset,
            'totalBucketPoints': totalBucketPoints,
        }
        # BUGFIX:
        # - torqued가 publish하는 값은 학습/앵커 기준값이다.
        # - dynamic profile은 latcontrol_torque 내부에서 프레임별 effective 값으로 적용된다.
        # - 따라서 base와 effective를 분리해야 한다. 그렇지 않으면 디버그에서 self.live_torque_params가
        #   항상 base 값으로 보이거나, 반대로 base가 effective로 오염되어 코너 종료 후 원복되지 않는다.
        self._dyn_base_live_torque_params = dict(base_params)
        if bool(getattr(self, '_dyn_effective_active', False)):
            self.live_torque_params = dict(getattr(self, '_dyn_last_effective_params', base_params))
        else:
            self.live_torque_params = dict(base_params)

    def _guard_high_speed_curvature(self, v_ego, desired_curvature, desired_curvature_rate):
        v_kph = float(v_ego) * 3.6

        try:
            curv_in = float(desired_curvature)
            if not math.isfinite(curv_in):
                curv_in = 0.0
        except Exception:
            curv_in = 0.0

        try:
            rate_in = float(desired_curvature_rate)
            if not math.isfinite(rate_in):
                rate_in = 0.0
        except Exception:
            rate_in = 0.0

        if v_kph < HS_CURV_GUARD_ON_KPH:
            self._hs_prev_desired_curvature = curv_in
            self._hs_prev_desired_curvature_rate = rate_in
            return curv_in, rate_in, False

        prev = float(getattr(self, "_hs_prev_desired_curvature", 0.0) or 0.0)
        hold_frames = int(max(0, getattr(self, "_hs_guard_hold_frames", 0) or 0))

        # 고속에서 좌/우 부호가 갑자기 뒤집히면 일단 보수적으로 눌러줌
        if (abs(prev) >= HS_SIGN_FLIP_MIN_CURV and
                abs(curv_in) >= HS_SIGN_FLIP_MIN_CURV and
                (prev * curv_in) < 0.0):
            curv_in = math.copysign(min(abs(curv_in), abs(prev) * HS_SIGN_FLIP_KEEP_RATIO), prev)
            rate_in = 0.0

        delta_max = float(interp(v_kph, HS_CURV_DELTA_MAX_BP, HS_CURV_DELTA_MAX_V))
        alpha = float(interp(v_kph, HS_CURV_ALPHA_BP, HS_CURV_ALPHA_V))
        rate_max = float(interp(v_kph, HS_CURV_RATE_MAX_BP, HS_CURV_RATE_MAX_V))

        # 최근에 steer limit / saturation이 있었다면 잠깐 더 강하게 보수화
        if hold_frames > 0:
            delta_max *= HS_LIMIT_DELTA_SHRINK
            alpha *= HS_LIMIT_ALPHA_SHRINK
            rate_max *= HS_LIMIT_DELTA_SHRINK

        curv_rl = float(clip(curv_in, prev - delta_max, prev + delta_max))
        curv_out = float((alpha * curv_rl) + ((1.0 - alpha) * prev))
        rate_out = float(clip(rate_in, -rate_max, rate_max))

        self._hs_prev_desired_curvature = curv_out
        self._hs_prev_desired_curvature_rate = rate_out
        return curv_out, rate_out, True

    def _guard_low_speed_steer_slew(self, v_ego, requested_steer, last_actuators, steering_pressed):
        v_kph = float(v_ego) * 3.6
        if (
                v_kph <= LS_ADAPTIVE_SLEW_MIN_KPH or
                v_kph >= LS_ADAPTIVE_SLEW_MAX_KPH or
                bool(steering_pressed)
        ):
            return float(requested_steer), False

        try:
            applied_steer = float(getattr(last_actuators, "steer", 0.0))
            if not math.isfinite(applied_steer):
                applied_steer = 0.0
        except Exception:
            applied_steer = 0.0

        requested_steer = float(clip(requested_steer, -self.steer_max, self.steer_max))
        req_abs = abs(requested_steer)
        app_abs = abs(applied_steer)

        same_direction = (requested_steer * applied_steer) >= -0.02
        tracking_gap = req_abs - app_abs
        if (not same_direction) or tracking_gap <= LS_ADAPTIVE_SLEW_GAP_START:
            return requested_steer, False

        allowed_gap = float(interp(v_kph, LS_ADAPTIVE_SLEW_ALLOW_GAP_BP, LS_ADAPTIVE_SLEW_ALLOW_GAP_V))
        limited_abs = min(req_abs, app_abs + allowed_gap)
        if limited_abs >= req_abs:
            return requested_steer, False

        speed_weight = float(interp(
            v_kph,
            [LS_ADAPTIVE_SLEW_MIN_KPH, LS_ADAPTIVE_SLEW_FULL_ON_KPH,
             LS_ADAPTIVE_SLEW_FULL_OFF_KPH, LS_ADAPTIVE_SLEW_MAX_KPH],
            [0.0, 1.0, 1.0, 0.0]
        ))
        gap_weight = float(interp(
            tracking_gap,
            [LS_ADAPTIVE_SLEW_GAP_START, LS_ADAPTIVE_SLEW_GAP_FULL],
            [0.0, 1.0]
        ))
        blend = float(clip(speed_weight * gap_weight, 0.0, 1.0))
        out_abs = (req_abs * (1.0 - blend)) + (limited_abs * blend)
        return math.copysign(out_abs, requested_steer), True

    def _safe_float(self, val, fallback=0.0):
        try:
            out = float(val)
            if math.isfinite(out):
                return out
        except Exception:
            pass
        return float(fallback)

    def _get_dynamic_torque_params(self, v_ego, desired_curvature, desired_lateral_accel,
                                   actual_lateral_accel, steer_limited=False, steering_pressed=False,
                                   last_actuators=None, rate_limited_strong=False, rate_limit_err=0.0,
                                   driver_steering_torque=0.0):
        """
        속도/코너 강도 기반 effective torque params.
        - liveTorqueParameters 학습값은 건드리지 않는다.
        - torque_from_lateral_accel()에 넣는 임시 파라미터만 변경한다.
        - 10~35kph: latAccelFactor 낮춤 + friction 올림 = 저속 코너 최대 조향.
        - 60~110kph: latAccelFactor 올림 + friction 낮춤 = 고속 와리가리 억제.
        """
        base_params = dict(getattr(self, '_dyn_base_live_torque_params', self.live_torque_params))

        if not DYN_TORQUE_PROFILE_ENABLED:
            self._dyn_effective_active = False
            self._dyn_last_blend = 0.0
            self.live_torque_params = dict(base_params)
            return self.live_torque_params

        base_lat = self._safe_float(base_params.get('latAccelFactor', 1.88), 1.88)
        base_fric = self._safe_float(base_params.get('friction', 0.255), 0.255)
        base_off = self._safe_float(base_params.get('latAccelOffset', 0.0), 0.0)
        total_pts = base_params.get('totalBucketPoints', 0)

        v_kph = self._safe_float(v_ego, 0.0) * 3.6
        desired_curv_abs = abs(self._safe_float(desired_curvature, 0.0))
        desired_lat_abs = abs(self._safe_float(desired_lateral_accel, 0.0))

        try:
            steer_abs = abs(float(getattr(last_actuators, 'steer', 0.0))) if last_actuators is not None else 0.0
            if not math.isfinite(steer_abs):
                steer_abs = 0.0
        except Exception:
            steer_abs = 0.0

        curv_w = float(interp(desired_curv_abs, DYN_CURV_STRENGTH_BP, [0.0, 1.0]))
        latacc_w = float(interp(desired_lat_abs, DYN_LATACC_STRENGTH_BP, [0.0, 1.0]))
        steer_w = float(interp(steer_abs, DYN_STEER_STRENGTH_BP, [0.0, 1.0]))
        corner_strength = float(clip(max(curv_w, latacc_w, steer_w), 0.0, 1.0))

        low_gate = float(clip(interp(v_kph, DYN_LOW_SPEED_GATE_BP, DYN_LOW_SPEED_GATE_V), 0.0, 1.0))
        mid_gate = float(clip(interp(v_kph, DYN_MID_SPEED_GATE_BP, DYN_MID_SPEED_GATE_V), 0.0, 1.0))
        high_gate = float(clip(interp(v_kph, DYN_HIGH_SPEED_GATE_BP, DYN_HIGH_SPEED_GATE_V), 0.0, 1.0))

        rate_err = abs(self._safe_float(rate_limit_err, 0.0))
        strong_rate_limited = bool(rate_limited_strong) or (rate_err >= float(DYN_RATE_LIMITED_STRONG_OUTPUT_GAP))

        low_boost_target = corner_strength * low_gate
        mid_boost_target = corner_strength * mid_gate

        driver_torque_abs = abs(self._safe_float(driver_steering_torque, 0.0))
        strong_driver_override = bool(steering_pressed) and (driver_torque_abs >= float(DYN_DRIVER_TORQUE_HARD_DISABLE))

        # steeringPressed가 들어와도 저속 코너 보조를 완전히 끄지 않는다.
        # - 강한 운전자 토크면 OP와 싸우지 않도록 0으로 차단
        # - 가벼운 hand-on/미세 개입이면 보조를 일부 유지
        if strong_driver_override:
            low_boost_target = 0.0
            mid_boost_target = 0.0
            self._dyn_corner_hold_frames = 0
        elif bool(steering_pressed):
            low_boost_target *= float(DYN_STEERING_PRESSED_LOW_BOOST_MULT)
            mid_boost_target *= float(DYN_STEERING_PRESSED_MID_BOOST_MULT)

        # 제한이 걸리면 더 밀지 않고 부스트를 줄임.
        # strong rate-limit은 단순 limit보다 더 강하게 backoff한다.
        if bool(strong_rate_limited):
            low_boost_target *= float(DYN_RATE_LIMITED_STRONG_BOOST_MULT)
            mid_boost_target *= float(DYN_RATE_LIMITED_STRONG_BOOST_MULT)
        elif bool(steer_limited):
            low_boost_target *= float(DYN_STEER_LIMITED_BOOST_MULT)
            mid_boost_target *= float(DYN_STEER_LIMITED_BOOST_MULT)
        if bool(getattr(self, '_stable_torque_slew_active', False)):
            low_boost_target *= float(DYN_TORQUE_SLEW_ACTIVE_MULT)
            mid_boost_target *= float(DYN_TORQUE_SLEW_ACTIVE_MULT)

        # 10~35km/h는 로그상 clip/rate가 가장 심한 저속~저중속 코너 영역이므로
        # 작은 코너 요구라도 부스트를 충분히 확보한다.
        # BUGFIX: 기존 corner_strength 임계가 높으면 완만한 코너에서 dynamic 값이 거의 안 변했다.
        # desired curvature / lateral accel / 이전 steer 중 하나라도 코너 힌트가 있으면 최소 부스트를 보장한다.
        turning_hint = bool(
            (desired_curv_abs >= 0.00018) or
            (desired_lat_abs >= 0.035) or
            (steer_abs >= 0.012)
        )
        if (10.0 <= v_kph <= 35.0) and turning_hint and (not strong_driver_override):
            if bool(strong_rate_limited):
                low35_min_boost = float(DYN_LOW35_MIN_BOOST_STRONG)
            elif bool(steer_limited) or bool(getattr(self, '_stable_torque_slew_active', False)):
                low35_min_boost = float(DYN_LOW35_MIN_BOOST_LIMITED)
            else:
                low35_min_boost = float(DYN_LOW35_MIN_BOOST_NORMAL)

            # v32: 10~20kph는 clip이 가장 높았던 구간이다.
            # 최소 boost를 그대로 강제하면 desired torque가 적용 한계를 앞질러
            # steer_clip만 늘 수 있어 20kph까지는 단계적으로 낮춘다.
            low_speed_clip_relief = float(interp(v_kph, [10.0, 15.0, 20.0, 30.0, 35.0],
                                                 [0.78, 0.82, 0.88, 1.00, 1.00]))
            low35_min_boost *= low_speed_clip_relief

            if bool(steering_pressed):
                low35_min_boost *= float(DYN_STEERING_PRESSED_LOW_MIN_BOOST)
            low_boost_target = max(low_boost_target, low35_min_boost * low_gate)
        elif (35.0 < v_kph <= 45.0) and turning_hint and (not strong_driver_override):
            bridge_min_boost = float(interp(v_kph, DYN_BRIDGE_MIN_BOOST_NORMAL_BP, DYN_BRIDGE_MIN_BOOST_NORMAL_V))
            if bool(strong_rate_limited):
                bridge_min_boost *= float(DYN_BRIDGE_MIN_BOOST_STRONG_MULT)
            elif bool(steer_limited) or bool(getattr(self, '_stable_torque_slew_active', False)):
                bridge_min_boost *= float(DYN_BRIDGE_MIN_BOOST_LIMITED_MULT)
            if bool(steering_pressed):
                bridge_min_boost *= float(DYN_STEERING_PRESSED_BRIDGE_MIN_BOOST)
            low_boost_target = max(low_boost_target, bridge_min_boost)

        # 코너 종료 후 짧게 유지한 뒤 천천히 감쇠.
        if low_boost_target > 0.08:
            self._dyn_corner_hold_frames = int(DYN_LOW_SPEED_HOLD_FRAMES)
        elif self._dyn_corner_hold_frames > 0:
            self._dyn_corner_hold_frames -= 1
            low_boost_target = max(low_boost_target, min(float(self._dyn_corner_boost), 0.65) * low_gate)

        # 램프 적용: 갑자기 토크 성격이 바뀌지 않도록 함.
        cur_boost = float(getattr(self, '_dyn_corner_boost', 0.0) or 0.0)
        if low_boost_target > cur_boost:
            # v32: 10~20kph는 상승을 가장 느리게, 20~35kph는 중간, 그 외는 기본값.
            rise_step = float(DYN_BOOST_RISE_STEP)
            if 10.0 <= v_kph < 20.0:
                rise_step = float(DYN_BOOST_RISE_LOW1020_STEP)
            elif 20.0 <= v_kph < 35.0:
                rise_step = float(DYN_BOOST_RISE_LOW2035_STEP)
            cur_boost = min(low_boost_target, cur_boost + rise_step)
        else:
            cur_boost = max(low_boost_target, cur_boost - float(DYN_BOOST_FALL_STEP))
        cur_boost = float(clip(cur_boost, 0.0, 1.0))
        self._dyn_corner_boost = cur_boost

        target_lat = float(interp(v_kph, DYN_LAT_FACTOR_BP, DYN_LAT_FACTOR_V))
        target_fric = float(interp(v_kph, DYN_FRICTION_BP, DYN_FRICTION_V))

        # 저속은 코너 강도 기반, 중속은 완만한 코너 보조, 고속은 안정 게이트 기반으로 블렌딩.
        blend = float(clip(max(cur_boost, mid_boost_target, high_gate), 0.0, 1.0))
        self._dyn_last_blend = float(blend)

        eff_lat = base_lat + (target_lat - base_lat) * blend
        eff_fric = base_fric + (target_fric - base_fric) * blend

        eff_lat = float(clip(eff_lat, DYN_LAT_FACTOR_MIN, DYN_LAT_FACTOR_MAX))
        eff_fric = float(clip(eff_fric, DYN_FRICTION_MIN, DYN_FRICTION_MAX))

        self._dyn_last_corner_strength = corner_strength
        self._dyn_last_low_speed_gate = low_gate
        self._dyn_last_mid_speed_gate = mid_gate
        self._dyn_last_high_speed_gate = high_gate
        self._dyn_last_rate_limited_strong = bool(strong_rate_limited)
        self._dyn_last_target_delta_up = float(interp(v_kph, DYN_DELTA_UP_BP, DYN_DELTA_UP_V))
        self._dyn_last_target_delta_down = float(interp(v_kph, DYN_DELTA_DOWN_BP, DYN_DELTA_DOWN_V))

        self._dyn_last_effective_params = {
            'latAccelFactor': eff_lat,
            'friction': eff_fric,
            'latAccelOffset': base_off,
            'totalBucketPoints': total_pts,
        }
        # BUGFIX: 실제 적용 effective 값을 self.live_torque_params에도 반영한다.
        # base는 _dyn_base_live_torque_params에 따로 보존하므로, 코너가 끝나면 원래 학습값으로 정상 복귀한다.
        self._dyn_effective_active = bool(blend > 1e-4)
        self.live_torque_params = dict(self._dyn_last_effective_params if self._dyn_effective_active else base_params)
        return self.live_torque_params


    def get_dynamic_debug_torque_params(self):
        """Return the last frame's dynamic effective torque state for controlsd/UI/logging."""
        try:
            base = dict(getattr(self, '_dyn_base_live_torque_params', self.live_torque_params))
        except Exception:
            base = dict(getattr(self, 'live_torque_params', {}))
        try:
            eff = dict(getattr(self, '_dyn_last_effective_params', base))
        except Exception:
            eff = dict(base)
        return {
            'active': bool(getattr(self, '_dyn_effective_active', False)),
            'blend': float(getattr(self, '_dyn_last_blend', 0.0) or 0.0),
            'corner_strength': float(getattr(self, '_dyn_last_corner_strength', 0.0) or 0.0),
            'low_gate': float(getattr(self, '_dyn_last_low_speed_gate', 0.0) or 0.0),
            'mid_gate': float(getattr(self, '_dyn_last_mid_speed_gate', 0.0) or 0.0),
            'high_gate': float(getattr(self, '_dyn_last_high_speed_gate', 0.0) or 0.0),
            'latAccelFactor': float(eff.get('latAccelFactor', base.get('latAccelFactor', 0.0)) or 0.0),
            'friction': float(eff.get('friction', base.get('friction', 0.0)) or 0.0),
            'latAccelOffset': float(eff.get('latAccelOffset', base.get('latAccelOffset', 0.0)) or 0.0),
            'baseLatAccelFactor': float(base.get('latAccelFactor', 0.0) or 0.0),
            'baseFriction': float(base.get('friction', 0.0) or 0.0),
            'targetDeltaUp': float(getattr(self, '_dyn_last_target_delta_up', 0.0) or 0.0),
            'targetDeltaDown': float(getattr(self, '_dyn_last_target_delta_down', 0.0) or 0.0),
            'rateLimitedStrong': bool(getattr(self, '_dyn_last_rate_limited_strong', False)),
        }

    def _guard_output_torque_slew(self, v_ego, output_torque, steering_pressed=False, steer_limited=False):
        if (not STABLE_TORQUE_SLEW_ENABLED) or bool(steering_pressed):
            self._stable_prev_output_torque = float(output_torque)
            self._stable_torque_slew_active = False
            return float(output_torque)

        try:
            v_kph = float(v_ego) * 3.6
        except Exception:
            v_kph = 0.0

        prev = float(getattr(self, "_stable_prev_output_torque", 0.0) or 0.0)
        target = float(clip(float(output_torque), -self.steer_max, self.steer_max))

        same_sign = (prev * target) >= 0.0
        increasing_abs = same_sign and (abs(target) > abs(prev))
        lim = float(interp(
            v_kph,
            STABLE_TORQUE_SLEW_KPH_BP,
            STABLE_TORQUE_UP_V if increasing_abs else STABLE_TORQUE_DOWN_V
        ))
        if bool(steer_limited):
            lim *= float(STABLE_TORQUE_LIMITED_SHRINK)

        out = float(clip(target, prev - lim, prev + lim))
        self._stable_prev_output_torque = out
        self._stable_torque_slew_gap = float(abs(out - target))
        self._stable_torque_slew_active = bool(self._stable_torque_slew_gap > 1e-6)
        return out

    def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate,
               llk):
        pid_log = log.ControlsState.LateralTorqueState.new_message()

        if CS.vEgo < MIN_STEER_SPEED or not active:
            output_torque = 0.0
            pid_log.active = False
            angle_steers_des = 0.0

            self._hs_prev_desired_curvature = 0.0
            self._hs_prev_desired_curvature_rate = 0.0
            self._hs_guard_hold_frames = 0
            self._stable_prev_output_torque = 0.0
            self._stable_torque_slew_gap = 0.0
            self._stable_torque_slew_active = False
            self._dyn_prev_rate_limited_strong = False
            self._dyn_prev_rate_limit_err = 0.0
            self._dyn_effective_active = False
            self._dyn_last_blend = 0.0
            if hasattr(self, '_dyn_base_live_torque_params'):
                self.live_torque_params = dict(self._dyn_base_live_torque_params)
        else:
            if self.use_steering_angle:
                actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
                                                      CS.vEgo, params.roll)
                curvature_deadzone = abs(
                    VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
            else:
                actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
                                                         CS.vEgo, params.roll)
                actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
                actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
                curvature_deadzone = 0.0

            desired_curvature, desired_curvature_rate, hs_guard_active = self._guard_high_speed_curvature(
                CS.vEgo, desired_curvature, desired_curvature_rate
            )
            desired_lateral_accel = desired_curvature * CS.vEgo ** 2

            # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
            # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
            actual_lateral_accel = actual_curvature * CS.vEgo ** 2
            lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

            # Equinox 2020 diesel: CS.vEgo is m/s.
            # Keep low-speed corner response, but remove LOW_SPEED_FACTOR by 50km/h.
            low_speed_factor = self._low_speed_factor(CS.vEgo)

            setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
            measurement = actual_lateral_accel + low_speed_factor * actual_curvature

            error = setpoint - measurement

            effective_torque_params = self._get_dynamic_torque_params(
                CS.vEgo, desired_curvature, desired_lateral_accel, actual_lateral_accel,
                steer_limited=steer_limited, steering_pressed=CS.steeringPressed,
                last_actuators=last_actuators,
                rate_limited_strong=getattr(self, '_dyn_prev_rate_limited_strong', False),
                rate_limit_err=getattr(self, '_dyn_prev_rate_limit_err', 0.0),
                driver_steering_torque=getattr(CS, 'steeringTorque', 0.0)
            )

            pid_log.error = self.torque_from_lateral_accel(lateral_accel_value=error,
                                                           torque_params=effective_torque_params)

            ff = self.torque_from_lateral_accel(
                lateral_accel_value=desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY,
                torque_params=effective_torque_params,
                lateral_accel_error=error,
                lateral_accel_deadzone=lateral_accel_deadzone,
                friction_compensation=True
            )
            freeze_integrator = (
                steer_limited or
                CS.steeringPressed or
                CS.vEgo < 5 or
                (hs_guard_active and self._hs_guard_hold_frames > 0)
            )
            output_torque = self.pid.update(pid_log.error,
                                            feedforward=ff,
                                            speed=CS.vEgo,
                                            freeze_integrator=freeze_integrator)

            requested_steer_raw = -output_torque
            requested_steer = requested_steer_raw
            requested_steer, low_speed_slew_active = self._guard_low_speed_steer_slew(
                CS.vEgo, requested_steer, last_actuators, CS.steeringPressed
            )
            if low_speed_slew_active:
                output_torque = -requested_steer
                self.pid.control = output_torque

            output_torque = self._guard_output_torque_slew(
                CS.vEgo, output_torque, CS.steeringPressed, bool(steer_limited)
            )
            self.pid.control = output_torque

            # v2: 다음 프레임 dynamic boost에 직접 넣을 strong rate-limit proxy를 저장한다.
            # low-speed slew에서 requested가 실제 적용 가능 범위보다 크게 앞서거나,
            # output torque slew가 target을 크게 잘라낸 경우에는 다음 프레임 부스트를 줄인다.
            try:
                applied_last = float(getattr(last_actuators, 'steer', 0.0)) if last_actuators is not None else 0.0
                if not math.isfinite(applied_last):
                    applied_last = 0.0
            except Exception:
                applied_last = 0.0
            same_direction = (float(requested_steer_raw) * applied_last) >= -0.02
            if same_direction:
                tracking_gap = max(0.0, abs(float(requested_steer_raw)) - abs(applied_last))
            else:
                tracking_gap = abs(float(requested_steer_raw) - applied_last)
            stable_gap = float(getattr(self, '_stable_torque_slew_gap', 0.0) or 0.0)
            dyn_rate_err = max(stable_gap, tracking_gap if bool(low_speed_slew_active) else 0.0)
            self._dyn_prev_rate_limit_err = float(dyn_rate_err)
            self._dyn_prev_rate_limited_strong = bool(
                (bool(low_speed_slew_active) and tracking_gap >= float(DYN_RATE_LIMITED_STRONG_TRACKING_GAP)) or
                (bool(getattr(self, '_stable_torque_slew_active', False)) and stable_gap >= float(DYN_RATE_LIMITED_STRONG_OUTPUT_GAP))
            )

            pid_log.active = True
            pid_log.p = self.pid.p
            pid_log.i = self.pid.i
            pid_log.d = self.pid.d
            pid_log.f = self.pid.f
            pid_log.output = -output_torque
            pid_log.actualLateralAccel = actual_lateral_accel
            pid_log.desiredLateralAccel = desired_lateral_accel
            pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

            if hs_guard_active:
                limited_now = bool(steer_limited or pid_log.saturated)

                if limited_now:
                    self._hs_guard_hold_frames = max(
                        int(self._hs_guard_hold_frames),
                        int(round(interp(CS.vEgo * 3.6, HS_LIMIT_HOLD_BP, HS_LIMIT_HOLD_V)))
                    )
                elif self._hs_guard_hold_frames > 0:
                    self._hs_guard_hold_frames -= 1
            else:
                self._hs_guard_hold_frames = 0

            angle_steers_des = math.degrees(
                VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll)) + params.angleOffsetDeg

        # TODO left is positive in this convention
        return -output_torque, angle_steers_des, pid_log
