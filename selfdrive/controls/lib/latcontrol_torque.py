import math

from cereal import log
from common.numpy_fast import interp, clip
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from common.params import Params
from decimal import Decimal
import cereal.messaging as messaging
from selfdrive.controls.lib.torque_tuning_config import (
    CONTROLLER_FRICTION_MAX, CONTROLLER_FRICTION_MIN,
    CONTROLLER_LAT_ACCEL_MAX, CONTROLLER_LAT_ACCEL_MIN,
    equinox_steer_delta_profile, read_torque_tuning_config,
)

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

# ==============================
# Curvature request guard
# 60km/h 이상에서 목표 곡률(desired_curvature) 변화를 완화한다.
# 중저속은 추종력을 우선하고, 고속에서는 안정성을 우선한다.
# ==============================
HS_CURV_GUARD_ON_KPH = 60.0

# 60km/h 미만에서는 모델 곡률을 그대로 사용해 중저속 코너 추종력을 확보한다.
# 60km/h 이상부터만 속도에 따라 점진적으로 곡률 변화량을 완화한다.
HS_CURV_DELTA_MAX_BP = [60.0, 70.0, 90.0, 110.0, 130.0]
HS_CURV_DELTA_MAX_V = [0.00080, 0.00062, 0.00040, 0.00025, 0.00017]

# desired_curvature_rate 절대값 제한
HS_CURV_RATE_MAX_BP = [60.0, 70.0, 90.0, 110.0, 130.0]
HS_CURV_RATE_MAX_V = [0.024, 0.018, 0.012, 0.008, 0.005]

# 저역통과 필터 alpha (작을수록 더 부드러움)
HS_CURV_ALPHA_BP = [60.0, 70.0, 90.0, 110.0, 130.0]
HS_CURV_ALPHA_V = [0.60, 0.50, 0.38, 0.27, 0.18]

# 좌/우 부호가 갑자기 뒤집히는 경우 완화
HS_SIGN_FLIP_MIN_CURV = 0.00045
HS_SIGN_FLIP_KEEP_RATIO = 0.12

# steer_limited / saturated 직후 몇 프레임 더 보수적으로 유지할지
HS_LIMIT_HOLD_BP = [60.0, 70.0, 90.0, 110.0, 130.0]
HS_LIMIT_HOLD_V = [3.0, 5.0, 8.0, 12.0, 16.0]

# 제한 직후에도 코너 요구를 과도하게 죽이지 않도록 완화한다.
HS_LIMIT_DELTA_SHRINK = 0.70
HS_LIMIT_ALPHA_SHRINK = 0.85

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
LS_ADAPTIVE_SLEW_ALLOW_GAP_V = [1.00, 0.92, 0.88, 0.92, 1.00]

# Safety output torque slew guard.
# This is separate from model curvature smoothing and protects against abrupt
# torque jumps when the controller/car delta-up is too permissive.
STABLE_TORQUE_SLEW_ENABLED = True
# 저속은 더 빠르게, 고속은 더 안정적으로 torque slew 제한.
# 목적: 10~30kph 코너 추종력 확보 + 80~110kph 와리가리 억제.
STABLE_TORQUE_SLEW_KPH_BP = [0.0, 10.0, 20.0, 30.0, 35.0, 40.0, 45.0, 70.0, 90.0, 110.0, 130.0]
STABLE_TORQUE_UP_V =       [0.075, 0.110, 0.120, 0.115, 0.105, 0.090, 0.075, 0.030, 0.022, 0.017, 0.014]
STABLE_TORQUE_DOWN_V =     [0.100, 0.140, 0.150, 0.145, 0.130, 0.115, 0.095, 0.045, 0.036, 0.028, 0.022]
STABLE_TORQUE_LIMITED_SHRINK = 0.85

# Slew limiter anti-windup.
# 출력이 증가 방향으로 제한되는 동안에는 PID I 적분을 누적하지 않고,
# 토크를 줄이거나 반대 방향으로 전환하는 동안에는 남은 I 값을 완만하게 감쇠한다.
STABLE_TORQUE_ANTI_WINDUP_ENABLED = True
STABLE_TORQUE_I_RELEASE_DECAY = 0.985

# 속도별 최종 토크 상한.
# 60km/h 미만은 1.0을 유지해 GM STEER_MAX=300을 모두 사용할 수 있다.
# 고속에서는 기본 상한을 낮추되 실제 코너 횡가속 요구가 크면 headroom을 추가해
# 고속도로 코너를 놓치지 않도록 한다.
SPEED_TORQUE_CAP_ENABLED = True
SPEED_TORQUE_CAP_KPH_BP = [0.0, 50.0, 60.0, 70.0, 90.0, 110.0, 130.0]
SPEED_TORQUE_CAP_BASE_V = [1.00, 1.00, 0.98, 0.93, 0.86, 0.80, 0.76]
SPEED_TORQUE_CAP_LATACC_BP = [0.0, 0.25, 0.60, 1.00, 1.50]
SPEED_TORQUE_CAP_HEADROOM_V = [0.00, 0.02, 0.06, 0.12, 0.20]
SPEED_TORQUE_CAP_MIN = 0.70
SPEED_TORQUE_CAP_MAX = 1.00

# ==============================
# Dynamic effective torque profile
# ==============================
# liveTorqueParameters 자체는 학습값 그대로 유지하고,
# 실제 torque 계산에만 임시 effective latAccelFactor/friction을 적용한다.
DYN_TORQUE_PROFILE_ENABLED = True

# Speed breakpoints for small relative adjustments around the learned base.
DYN_LAT_FACTOR_BP = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 45.0, 60.0, 80.0, 100.0, 110.0, 130.0]
# v4 10~45kph 통합 개선:
#  - 10~30kph는 안전 클램프 하한/상한까지 사용해 강한 저속 코너 보조
#  - 30~35kph는 저속 강한 개선을 그대로 연장
#  - 35~45kph는 bridge 구간으로 LatAccelFactor/Friction을 점진 완화해 추종력과 안정성을 같이 확보
DYN_FRICTION_BP   = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 45.0, 60.0, 80.0, 100.0, 110.0, 130.0]

# Equinox uses the learned live values as the base. These small relative scales
# improve low/mid-speed corner authority and add high-speed stability without
# pulling a well-learned vehicle back toward a hard-coded absolute target.
DYN_LAT_FACTOR_SCALE_V = [1.000, 0.980, 0.960, 0.940, 0.930, 0.930, 0.940,
                          0.960, 0.985, 1.000, 1.010, 1.015, 1.020]
DYN_FRICTION_SCALE_V = [1.000, 1.015, 1.035, 1.050, 1.055, 1.050, 1.040,
                        1.025, 1.010, 1.000, 0.990, 0.985, 0.980]
DYN_PROFILE_MIN_POINTS = 300
DYN_PROFILE_FULL_POINTS = 2500
DYN_PROFILE_LOW_SPEED_MIN_GATE = 0.35

# 속도별 dynamic 최종 클램프. 저속에서는 latAccelFactor를 더 낮출 수 있고,
# 고속으로 갈수록 base 값 근처로 복귀한다.
DYN_LAT_FACTOR_MIN_SCALE_BP = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 130.0]
DYN_LAT_FACTOR_MIN_SCALE_V = [0.98, 0.94, 0.88, 0.88, 0.91, 0.94, 0.97, 0.99, 1.00]
DYN_FRICTION_MAX_SCALE_BP = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 130.0]
DYN_FRICTION_MAX_SCALE_V = [1.05, 1.10, 1.15, 1.15, 1.12, 1.10, 1.08, 1.05, 1.02]

# Actual STEER_DELTA_UP/DOWN is now selected in GM CarController through the shared profile.

# 코너 강도 판정. 세 값 중 가장 큰 값을 사용한다.
# 코너 감지 민감도 보정.
# 기존 임계가 높으면 완만한 저속 코너에서 corner_strength=0에 가까워져
# effective LatAccelFactor/Friction이 표시상/체감상 거의 변하지 않는 문제가 있었다.
DYN_CURV_STRENGTH_BP = [0.00025, 0.00160]
DYN_LATACC_STRENGTH_BP = [0.05, 0.75]
DYN_STEER_STRENGTH_BP = [0.015, 0.22]

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
DYN_HIGH_SPEED_GATE_V  = [0.0, 0.20, 0.75, 1.00, 1.00]

# 부스트 램프/홀드. 프레임 기반이며 controls update 주기에 독립적으로 안전하게 동작한다.
DYN_BOOST_RISE_STEP = 0.14
DYN_BOOST_FALL_STEP = 0.025
DYN_LOW_SPEED_HOLD_FRAMES = 80  # 약 0.8초 @100Hz 근처

# limit 상황에서는 더 밀어붙이지 않고 부스트를 줄인다.
DYN_STEER_LIMITED_BOOST_MULT = 0.75
DYN_TORQUE_SLEW_ACTIVE_MULT = 0.85
# 50km/h 이하에서는 제한이 감지돼도 코너 보조를 거의 유지한다.
DYN_LOW_SPEED_LIMIT_BACKOFF_MULT = 0.95

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
DYN_RATE_LIMITED_STRONG_BOOST_MULT = 0.85
DYN_RATE_LIMITED_STRONG_TRACKING_GAP = 0.45
DYN_RATE_LIMITED_STRONG_OUTPUT_GAP = 0.18

# 최종 안전 클램프
DYN_LAT_FACTOR_MIN = 1.75
DYN_LAT_FACTOR_MAX = 2.42
DYN_FRICTION_MIN = 0.165
DYN_FRICTION_MAX = 0.305

# Separate low-speed adaptation. The base torqued learner is restricted to
# medium/high speed; these three bounded gains correct 10~40km/h tracking
# without contaminating the vehicle-wide latAccelFactor/friction estimate.
LOW_SPEED_GAIN_BIN_EDGES_KPH = [10.0, 20.0, 30.0, 40.0]
LOW_SPEED_GAIN_DEFAULTS = [1.020, 1.040, 1.030]
LOW_SPEED_GAIN_PARAM_KEYS = [
    "TorqueLowSpeedGain10_20",
    "TorqueLowSpeedGain20_30",
    "TorqueLowSpeedGain30_40",
]
LOW_SPEED_GAIN_STATE_VERSION = 1
LOW_SPEED_GAIN_MIN = 0.97
LOW_SPEED_GAIN_MAX = 1.10
LOW_SPEED_GAIN_MIN_POINTS = 500
LOW_SPEED_GAIN_FULL_POINTS = 2000
LOW_SPEED_GAIN_MAX_STEP = 0.00005
LOW_SPEED_GAIN_TARGET_WEIGHT = 0.20
LOW_SPEED_GAIN_SAVE_SAMPLES = 3000
LOW_SPEED_FRICTION_SCALE_BP = [10.0, 20.0, 30.0, 40.0, 50.0]
LOW_SPEED_FRICTION_SCALE_V = [1.02, 1.05, 1.06, 1.03, 1.00]

# A learned center offset is applied to feedforward even on a straight road.
# Keep the final controller-side clamp independent from torqued so a stale or
# malformed publisher can never create a large continuous steering bias.
# 직선 지속 쏠림 방지용 이중 안전장치. torqued가 stale/non-zero 값을 publish해도
# The offset remains disabled by Params by default and is hard-clamped when explicitly enabled.
LAT_ACCEL_OFFSET_COMP_ENABLED = True
LAT_ACCEL_OFFSET_ABS_MAX = 0.01
LAT_ACCEL_OFFSET_DEADBAND = 0.003
CENTER_OFFSET_PARAM_KEYS = (
    "TorqueCenterOffset20_40",
    "TorqueCenterOffset40_60",
    "TorqueCenterOffset60_100",
)
CENTER_OFFSET_COUNT_KEYS = tuple(key + "Count" for key in CENTER_OFFSET_PARAM_KEYS)
CENTER_OFFSET_ABS_MAX_BY_BIN = (0.006, 0.008, 0.010)
CENTER_OFFSET_EFFECTIVE_DEADBAND = 0.0005

# Directional torque balance.
# latAccelOffset remains the straight-line bias correction. These small,
# bounded per-direction assists compensate left/right corner response
# differences without moving the straight offset.
DIRECTIONAL_TORQUE_COMP_ENABLED = True
DIRECTIONAL_TORQUE_MIN_SPEED = 5.0
DIRECTIONAL_TORQUE_MIN_LAT_ACCEL = 0.12
DIRECTIONAL_TORQUE_ERROR_DEADBAND = 0.035
DIRECTIONAL_TORQUE_ERROR_FULL = 0.28
DIRECTIONAL_TORQUE_STEP = 0.00008
DIRECTIONAL_TORQUE_ASSIST_MIN = 0.97
DIRECTIONAL_TORQUE_ASSIST_MAX = 1.03
DIRECTIONAL_TORQUE_FRICTION_GAIN = 0.55


class LatControlTorque(LatControl):
    def __init__(self, CP, CI):
        super().__init__(CP, CI)
        self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                                 k_f=CP.lateralTuning.torque.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
        self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
        self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
        self.steering_angle_deadzone_deg = CP.lateralTuning.torque.steeringAngleDeadzoneDeg
        self._is_equinox_torque_profile = str(getattr(CP, 'carFingerprint', '')) == "CHEVROLET EQUINOX NO RADAR"
        self.update_live_torque_params(CP.lateralTuning.torque.latAccelFactor, CP.lateralTuning.torque.latAccelOffset,
                                       CP.lateralTuning.torque.friction)

        # high-speed conservative guard state
        self._hs_prev_desired_curvature = 0.0
        self._hs_prev_desired_curvature_rate = 0.0
        self._hs_guard_hold_frames = 0
        self._stable_prev_output_torque = 0.0
        self._stable_torque_slew_gap = 0.0
        self._stable_torque_slew_active = False
        self._stable_torque_slew_windup_block = False
        self._speed_torque_cap = float(self.steer_max)
        self._speed_torque_cap_gap = 0.0
        self._speed_torque_cap_active = False
        self._speed_torque_cap_windup_block = False

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
        self._dir_torque_assist_left = 1.0
        self._dir_torque_assist_right = 1.0
        self._dir_torque_last_side = 0

        self._torque_config_params = Params()
        self._torque_runtime_config = read_torque_tuning_config(self._torque_config_params, migrate=False)
        self._torque_config_counter = 0
        self._center_offset_enabled = bool(self._torque_runtime_config.center_offset_enabled)
        self._directional_comp_enabled = bool(self._torque_runtime_config.directional_comp_enabled)
        self._center_offsets = [0.0, 0.0, 0.0]
        self._center_offset_counts = [0, 0, 0]
        self._load_center_offsets()
        self._low_speed_gains = list(LOW_SPEED_GAIN_DEFAULTS)
        self._low_speed_gain_counts = [0, 0, 0]
        self._low_speed_gain_save_counter = 0
        self._low_speed_gain_sample_counter = 0
        self._directional_gain_sample_counter = 0
        self._load_low_speed_gains()

    def _load_low_speed_gains(self):
        try:
            raw_version = self._torque_config_params.get('TorqueLowSpeedGainVersion')
            version = int(raw_version.decode('utf-8')) if raw_version is not None else 0
        except Exception:
            version = 0
        if version != LOW_SPEED_GAIN_STATE_VERSION:
            return
        for i, key in enumerate(LOW_SPEED_GAIN_PARAM_KEYS):
            try:
                raw = self._torque_config_params.get(key)
                if raw is not None:
                    val = float(raw.decode('utf-8', errors='ignore').strip())
                    if math.isfinite(val):
                        self._low_speed_gains[i] = float(clip(val, LOW_SPEED_GAIN_MIN, LOW_SPEED_GAIN_MAX))
                raw_count = self._torque_config_params.get(key + 'Count')
                if raw_count is not None:
                    self._low_speed_gain_counts[i] = max(0, int(float(raw_count.decode('utf-8', errors='ignore').strip())))
            except Exception:
                pass

    def _save_low_speed_gains(self):
        try:
            self._torque_config_params.put('TorqueLowSpeedGainVersion',
                                           str(LOW_SPEED_GAIN_STATE_VERSION).encode('utf-8'))
            for i, key in enumerate(LOW_SPEED_GAIN_PARAM_KEYS):
                self._torque_config_params.put(key, ('%.5f' % self._low_speed_gains[i]).encode('utf-8'))
                self._torque_config_params.put(key + 'Count', str(int(self._low_speed_gain_counts[i])).encode('utf-8'))
        except Exception:
            pass

    def _load_center_offsets(self):
        for i, key in enumerate(CENTER_OFFSET_PARAM_KEYS):
            try:
                raw = self._torque_config_params.get(key)
                if raw is not None:
                    val = float(raw.decode('utf-8', errors='ignore').strip())
                    if math.isfinite(val):
                        self._center_offsets[i] = float(clip(
                            val, -CENTER_OFFSET_ABS_MAX_BY_BIN[i], CENTER_OFFSET_ABS_MAX_BY_BIN[i]))
                raw_count = self._torque_config_params.get(CENTER_OFFSET_COUNT_KEYS[i])
                if raw_count is not None:
                    self._center_offset_counts[i] = max(
                        0, int(float(raw_count.decode('utf-8', errors='ignore').strip())))
            except Exception:
                pass

    def _resolved_center_offsets(self, fallback=0.0):
        """Fill an unlearned speed bin from the nearest learned bin.

        This prevents a temporary zero-offset hole at 40 or 60 km/h while a new
        bin is still collecting its first valid straight-road window.
        """
        values = [float(x) for x in self._center_offsets]
        valid = [int(c) > 0 for c in self._center_offset_counts]
        fallback = float(clip(float(fallback), -LAT_ACCEL_OFFSET_ABS_MAX, LAT_ACCEL_OFFSET_ABS_MAX))
        if not any(valid):
            return fallback, fallback, fallback

        resolved = list(values)
        for i in range(3):
            if valid[i]:
                continue
            nearest = min((j for j in range(3) if valid[j]), key=lambda j: abs(j - i))
            resolved[i] = values[nearest]
        return tuple(resolved)

    def _get_center_offset_for_speed(self, v_kph, fallback=0.0):
        if not (LAT_ACCEL_OFFSET_COMP_ENABLED and getattr(self, '_center_offset_enabled', False)):
            return 0.0
        try:
            low, mid, high = self._resolved_center_offsets(fallback)
            # 10~20km/h applies only a weak fraction of the low-speed value.
            # Above 60km/h the highway value is applied at 100%.
            off = float(interp(float(v_kph),
                               [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 100.0],
                               [0.0, 0.0, 0.25 * low, low, 0.5 * (low + mid), mid, high, high]))
            off = float(clip(off, -LAT_ACCEL_OFFSET_ABS_MAX, LAT_ACCEL_OFFSET_ABS_MAX))
            if abs(off) < CENTER_OFFSET_EFFECTIVE_DEADBAND:
                return 0.0
            return off
        except Exception:
            return float(fallback)

    def _reload_runtime_torque_config(self):
        self._torque_config_counter += 1
        if self._torque_config_counter < 50:
            return
        self._torque_config_counter = 0
        try:
            self._torque_runtime_config = read_torque_tuning_config(self._torque_config_params, migrate=False)
            self._center_offset_enabled = bool(self._torque_runtime_config.center_offset_enabled)
            self._directional_comp_enabled = bool(self._torque_runtime_config.directional_comp_enabled)
            self._load_center_offsets()
        except Exception:
            pass

    @staticmethod
    def _low_speed_gain_bin(v_kph):
        if 10.0 <= v_kph < 20.0:
            return 0
        if 20.0 <= v_kph < 30.0:
            return 1
        if 30.0 <= v_kph < 40.0:
            return 2
        return None

    def _update_low_speed_gain(self, v_kph, desired_lateral_accel, actual_lateral_accel,
                               steering_pressed, steer_limited, rate_limited_strong):
        idx = self._low_speed_gain_bin(v_kph)
        if idx is None or not self._is_equinox_torque_profile or not self._torque_runtime_config.enabled:
            return
        self._low_speed_gain_sample_counter = (self._low_speed_gain_sample_counter + 1) % 10
        if self._low_speed_gain_sample_counter != 0:
            return
        if bool(steering_pressed) or bool(steer_limited) or bool(rate_limited_strong):
            return

        desired = self._safe_float(desired_lateral_accel, 0.0)
        actual = self._safe_float(actual_lateral_accel, 0.0)
        desired_abs = abs(desired)
        actual_abs = abs(actual)
        if desired_abs < 0.15 or desired_abs > 1.30 or actual_abs < 0.05:
            return
        if desired * actual <= 0.0 or abs(desired - actual) > 0.60:
            return

        ratio = desired_abs / max(actual_abs, 0.05)
        target = 1.0 + LOW_SPEED_GAIN_TARGET_WEIGHT * (ratio - 1.0)
        target = float(clip(target, LOW_SPEED_GAIN_MIN, LOW_SPEED_GAIN_MAX))
        current = float(self._low_speed_gains[idx])
        current += float(clip(target - current, -LOW_SPEED_GAIN_MAX_STEP, LOW_SPEED_GAIN_MAX_STEP))
        self._low_speed_gains[idx] = float(clip(current, LOW_SPEED_GAIN_MIN, LOW_SPEED_GAIN_MAX))
        self._low_speed_gain_counts[idx] += 1
        self._low_speed_gain_save_counter += 1
        if self._low_speed_gain_save_counter >= LOW_SPEED_GAIN_SAVE_SAMPLES:
            self._low_speed_gain_save_counter = 0
            self._save_low_speed_gains()

    def _get_low_speed_gain(self, v_kph):
        centers = [15.0, 25.0, 35.0]
        effective = []
        for i in range(3):
            confidence = float(interp(float(self._low_speed_gain_counts[i]),
                                      [LOW_SPEED_GAIN_MIN_POINTS, LOW_SPEED_GAIN_FULL_POINTS],
                                      [0.0, 1.0]))
            effective.append(float(LOW_SPEED_GAIN_DEFAULTS[i] +
                                   (self._low_speed_gains[i] - LOW_SPEED_GAIN_DEFAULTS[i]) * confidence))
        return float(interp(v_kph, [10.0] + centers + [45.0],
                            [1.0] + effective + [1.0]))

    def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction, totalBucketPoints=0):
        if not (LAT_ACCEL_OFFSET_COMP_ENABLED and getattr(self, '_center_offset_enabled', False)):
            safe_offset = 0.0
        else:
            try:
                safe_offset = float(clip(float(latAccelOffset),
                                         -LAT_ACCEL_OFFSET_ABS_MAX,
                                         LAT_ACCEL_OFFSET_ABS_MAX))
                if abs(safe_offset) < LAT_ACCEL_OFFSET_DEADBAND:
                    safe_offset = 0.0
            except Exception:
                safe_offset = 0.0
        base_params = {
            'latAccelFactor': latAccelFactor,
            'friction': friction,
            'latAccelOffset': safe_offset,
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

    def _get_directional_torque_params(self, torque_params, v_ego, desired_lateral_accel,
                                       actual_lateral_accel, steering_pressed=False,
                                       steer_limited=False):
        if not (DIRECTIONAL_TORQUE_COMP_ENABLED and getattr(self, '_directional_comp_enabled', False)):
            return torque_params

        desired_lat = self._safe_float(desired_lateral_accel, 0.0)
        actual_lat = self._safe_float(actual_lateral_accel, 0.0)
        if abs(desired_lat) < DIRECTIONAL_TORQUE_MIN_LAT_ACCEL:
            self._dir_torque_last_side = 0
            return torque_params

        side = 1 if desired_lat > 0.0 else -1
        self._dir_torque_last_side = side

        assist_attr = '_dir_torque_assist_left' if side > 0 else '_dir_torque_assist_right'
        assist = self._safe_float(getattr(self, assist_attr, 1.0), 1.0)

        can_learn = (
            self._safe_float(v_ego, 0.0) >= DIRECTIONAL_TORQUE_MIN_SPEED and
            not bool(steering_pressed) and
            not bool(steer_limited)
        )
        self._directional_gain_sample_counter = (self._directional_gain_sample_counter + 1) % 10
        can_learn = can_learn and self._directional_gain_sample_counter == 0
        if can_learn:
            signed_error = (desired_lat - actual_lat) * float(side)
            error_mag = abs(signed_error)
            if error_mag > DIRECTIONAL_TORQUE_ERROR_DEADBAND:
                learn_w = float(clip(
                    (error_mag - DIRECTIONAL_TORQUE_ERROR_DEADBAND) /
                    max(DIRECTIONAL_TORQUE_ERROR_FULL - DIRECTIONAL_TORQUE_ERROR_DEADBAND, 1e-3),
                    0.0, 1.0
                ))
                if signed_error > 0.0:
                    assist += DIRECTIONAL_TORQUE_STEP * learn_w
                else:
                    assist -= DIRECTIONAL_TORQUE_STEP * learn_w
                assist = float(clip(assist, DIRECTIONAL_TORQUE_ASSIST_MIN, DIRECTIONAL_TORQUE_ASSIST_MAX))
                setattr(self, assist_attr, assist)

        if abs(assist - 1.0) < 1e-5:
            return torque_params

        out = dict(torque_params)
        base_lat = self._safe_float(out.get('latAccelFactor', 1.0), 1.0)
        base_fric = self._safe_float(out.get('friction', 0.0), 0.0)
        if base_lat > 1e-3:
            out['latAccelFactor'] = base_lat / assist
        out['friction'] = max(0.0, base_fric * (1.0 + ((assist - 1.0) * DIRECTIONAL_TORQUE_FRICTION_GAIN)))
        return out

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
            curv_in = 0.0
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
        """Apply one bounded low-speed model on top of the learned base.

        torqued owns the medium/high-speed vehicle model. This controller only
        adapts a small 10~40km/h gain and a bounded friction fallback. High-speed
        stability is handled by curvature filtering and the final torque cap,
        avoiding the old duplicate high-speed latAccelFactor/friction scaling.
        """
        self._reload_runtime_torque_config()
        base_params = dict(getattr(self, '_dyn_base_live_torque_params', self.live_torque_params))

        if (not DYN_TORQUE_PROFILE_ENABLED or not self._is_equinox_torque_profile or
                not self._torque_runtime_config.enabled):
            self._dyn_effective_active = False
            self._dyn_last_blend = 0.0
            self.live_torque_params = dict(base_params)
            return self.live_torque_params

        base_lat = self._safe_float(base_params.get('latAccelFactor', 2.05), 2.05)
        base_fric = self._safe_float(base_params.get('friction', 0.230), 0.230)
        published_off = self._safe_float(base_params.get('latAccelOffset', 0.0), 0.0)
        total_pts = base_params.get('totalBucketPoints', 0)
        v_kph = self._safe_float(v_ego, 0.0) * 3.6
        base_off = self._get_center_offset_for_speed(v_kph, fallback=published_off)
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
        low_gate = float(clip(interp(v_kph, [8.0, 10.0, 35.0, 40.0, 45.0],
                                     [0.0, 1.0, 1.0, 0.55, 0.0]), 0.0, 1.0))

        strong_rate_limited = bool(rate_limited_strong) or (
            abs(self._safe_float(rate_limit_err, 0.0)) >= float(DYN_RATE_LIMITED_STRONG_OUTPUT_GAP))
        self._update_low_speed_gain(v_kph, desired_lateral_accel, actual_lateral_accel,
                                    steering_pressed, steer_limited, strong_rate_limited)

        driver_torque_abs = abs(self._safe_float(driver_steering_torque, 0.0))
        strong_driver_override = bool(steering_pressed) and driver_torque_abs >= float(DYN_DRIVER_TORQUE_HARD_DISABLE)
        blend = corner_strength * low_gate
        if strong_driver_override:
            blend = 0.0
        elif bool(steering_pressed):
            blend *= 0.45
        if bool(strong_rate_limited):
            blend *= 0.65
        elif bool(steer_limited):
            blend *= 0.80
        blend = float(clip(blend, 0.0, 1.0))

        low_speed_gain = self._get_low_speed_gain(v_kph)
        applied_gain = 1.0 + (low_speed_gain - 1.0) * blend
        eff_lat = base_lat / max(applied_gain, 0.90)
        friction_scale = float(interp(v_kph, LOW_SPEED_FRICTION_SCALE_BP, LOW_SPEED_FRICTION_SCALE_V))
        eff_fric = base_fric * (1.0 + (friction_scale - 1.0) * blend)
        eff_lat = float(clip(eff_lat, CONTROLLER_LAT_ACCEL_MIN, CONTROLLER_LAT_ACCEL_MAX))
        eff_fric = float(clip(eff_fric, CONTROLLER_FRICTION_MIN, CONTROLLER_FRICTION_MAX))

        delta_up, delta_down = equinox_steer_delta_profile(
            v_kph, self._torque_runtime_config,
            steering_pressed=steering_pressed, driver_torque=driver_steering_torque,
            reversing=False)
        self._dyn_last_target_delta_up = float(delta_up)
        self._dyn_last_target_delta_down = float(delta_down)
        self._dyn_last_corner_strength = corner_strength
        self._dyn_last_low_speed_gate = low_gate
        self._dyn_last_mid_speed_gate = 0.0
        self._dyn_last_high_speed_gate = 0.0
        self._dyn_last_rate_limited_strong = bool(strong_rate_limited)
        self._dyn_last_blend = blend
        self._dyn_last_low_speed_gain = float(low_speed_gain)
        self._dyn_last_low_speed_gain_applied = float(applied_gain)

        self._dyn_last_effective_params = {
            'latAccelFactor': eff_lat,
            'friction': eff_fric,
            'latAccelOffset': base_off,
            'totalBucketPoints': total_pts,
        }
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
            'dirAssistLeft': float(getattr(self, '_dir_torque_assist_left', 1.0) or 1.0),
            'dirAssistRight': float(getattr(self, '_dir_torque_assist_right', 1.0) or 1.0),
            'dirAssistSide': int(getattr(self, '_dir_torque_last_side', 0) or 0),
            'targetDeltaUp': float(getattr(self, '_dyn_last_target_delta_up', 0.0) or 0.0),
            'targetDeltaDown': float(getattr(self, '_dyn_last_target_delta_down', 0.0) or 0.0),
            'lowSpeedGain': float(getattr(self, '_dyn_last_low_speed_gain', 1.0) or 1.0),
            'lowSpeedGainApplied': float(getattr(self, '_dyn_last_low_speed_gain_applied', 1.0) or 1.0),
            'lowSpeedGainCounts': list(getattr(self, '_low_speed_gain_counts', [0, 0, 0])),
            'rateLimitedStrong': bool(getattr(self, '_dyn_last_rate_limited_strong', False)),
            'speedTorqueCap': float(getattr(self, '_speed_torque_cap', self.steer_max) or self.steer_max),
            'speedTorqueCapActive': bool(getattr(self, '_speed_torque_cap_active', False)),
            'speedTorqueCapGap': float(getattr(self, '_speed_torque_cap_gap', 0.0) or 0.0),
        }

    def _get_speed_torque_cap(self, v_ego, desired_lateral_accel=0.0, desired_curvature=0.0):
        if (not SPEED_TORQUE_CAP_ENABLED) or (not self._is_equinox_torque_profile):
            return float(self.steer_max)
        try:
            v_kph = max(0.0, float(v_ego) * 3.6)
        except Exception:
            v_kph = 0.0
        try:
            lat_abs = abs(float(desired_lateral_accel))
            if not math.isfinite(lat_abs):
                lat_abs = 0.0
        except Exception:
            lat_abs = 0.0
        try:
            curv_abs = abs(float(desired_curvature))
            if not math.isfinite(curv_abs):
                curv_abs = 0.0
        except Exception:
            curv_abs = 0.0

        base_norm = float(interp(v_kph, SPEED_TORQUE_CAP_KPH_BP, SPEED_TORQUE_CAP_BASE_V))
        headroom = float(interp(lat_abs, SPEED_TORQUE_CAP_LATACC_BP, SPEED_TORQUE_CAP_HEADROOM_V))
        # 짧고 급한 고속 코너는 횡가속 계산이 올라오기 전에도 약간의 여유를 준다.
        if curv_abs >= 0.0010:
            headroom = max(headroom, 0.06)
        cap_norm = float(clip(base_norm + headroom, SPEED_TORQUE_CAP_MIN, SPEED_TORQUE_CAP_MAX))
        return float(cap_norm * self.steer_max)

    def _guard_output_torque_slew(self, v_ego, output_torque, steering_pressed=False, steer_limited=False):
        if (not STABLE_TORQUE_SLEW_ENABLED) or bool(steering_pressed):
            self._stable_prev_output_torque = float(output_torque)
            self._stable_torque_slew_gap = 0.0
            self._stable_torque_slew_active = False
            self._stable_torque_slew_windup_block = False
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
        # 증가 방향 제한에서만 적분 windup을 차단한다. 감속/부호 전환에서는
        # 적분기가 자연스럽게 풀릴 수 있도록 별도로 표시한다.
        self._stable_torque_slew_windup_block = bool(
            self._stable_torque_slew_active and increasing_abs
        )
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
            self._stable_torque_slew_windup_block = False
            self._speed_torque_cap = float(self.steer_max)
            self._speed_torque_cap_gap = 0.0
            self._speed_torque_cap_active = False
            self._speed_torque_cap_windup_block = False
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

            isLowSpeed = Params().get_bool('IsLowSpeedFactor')
            if isLowSpeed:
                low_speed_factor = interp(CS.vEgo, [0.0, 3.0, 5.0, 8.33, 13.89], [420.0, 420.0, 260.0, 80.0, 0.0])
            else:
                low_speed_factor = interp(CS.vEgo, [0.0, 3.0, 5.0, 8.33], [300.0, 300.0, 120.0, 0.0])

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
            effective_torque_params = self._get_directional_torque_params(
                effective_torque_params, CS.vEgo, desired_lateral_accel, actual_lateral_accel,
                steering_pressed=CS.steeringPressed, steer_limited=steer_limited
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
            # 현재 프레임에서 slew 제한이 새로 발생할 수 있으므로 I 값을 보관했다가
            # 증가 방향 제한이면 update 후 복원한다. 이전 프레임부터 증가 제한 중이면
            # 이번 프레임은 처음부터 freeze한다. 감속/반전 제한은 freeze하지 않는다.
            try:
                pid_i_before_update = float(self.pid.i)
            except Exception:
                pid_i_before_update = 0.0

            freeze_integrator = (
                steer_limited or
                CS.steeringPressed or
                CS.vEgo < 5 or
                bool(getattr(self, '_stable_torque_slew_windup_block', False)) or
                bool(getattr(self, '_speed_torque_cap_windup_block', False)) or
                (hs_guard_active and self._hs_guard_hold_frames > 0)
            )
            output_torque = self.pid.update(pid_log.error,
                                            feedforward=ff,
                                            speed=CS.vEgo,
                                            freeze_integrator=freeze_integrator)

            # 속도별 상한은 slew 전에 적용해 상한 진입/복귀도 부드럽게 만든다.
            pid_target_before_speed_cap = float(output_torque)
            speed_cap = self._get_speed_torque_cap(CS.vEgo, desired_lateral_accel, desired_curvature)
            output_torque = float(clip(output_torque, -speed_cap, speed_cap))
            self._speed_torque_cap = float(speed_cap)
            self._speed_torque_cap_gap = float(abs(pid_target_before_speed_cap - output_torque))
            self._speed_torque_cap_active = bool(self._speed_torque_cap_gap > 1e-6)
            self._speed_torque_cap_windup_block = bool(self._speed_torque_cap_active)

            requested_steer_raw = -output_torque
            requested_steer = requested_steer_raw
            requested_steer, low_speed_slew_active = self._guard_low_speed_steer_slew(
                CS.vEgo, requested_steer, last_actuators, CS.steeringPressed
            )
            if low_speed_slew_active:
                output_torque = -requested_steer
                self.pid.control = output_torque

            stable_prev_before_slew = float(getattr(self, '_stable_prev_output_torque', 0.0) or 0.0)
            pid_target_before_slew = float(output_torque)
            output_torque = self._guard_output_torque_slew(
                CS.vEgo, output_torque, CS.steeringPressed, bool(steer_limited)
            )

            if STABLE_TORQUE_ANTI_WINDUP_ENABLED:
                windup_limited_now = bool(
                    low_speed_slew_active or
                    getattr(self, '_stable_torque_slew_windup_block', False) or
                    getattr(self, '_speed_torque_cap_windup_block', False)
                )
                if windup_limited_now and not freeze_integrator:
                    # PID가 요구한 증가량을 실제 출력이 따라가지 못한 프레임의 I 누적을 취소한다.
                    try:
                        self.pid.i = float(pid_i_before_update)
                    except Exception:
                        pass
                elif bool(getattr(self, '_stable_torque_slew_active', False)):
                    # 토크 감소 또는 좌/우 반전이 slew에 막힌 경우 남아 있는 I를 천천히 풀어
                    # 커브 종료 후 이전 방향으로 미는 시간을 줄인다.
                    reducing_or_reversing = bool(
                        (stable_prev_before_slew * pid_target_before_slew) < 0.0 or
                        abs(pid_target_before_slew) < abs(stable_prev_before_slew)
                    )
                    if reducing_or_reversing:
                        try:
                            self.pid.i = float(self.pid.i) * float(STABLE_TORQUE_I_RELEASE_DECAY)
                        except Exception:
                            pass

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
            speed_cap_gap = float(getattr(self, '_speed_torque_cap_gap', 0.0) or 0.0)
            dyn_rate_err = max(stable_gap, speed_cap_gap, tracking_gap if bool(low_speed_slew_active) else 0.0)
            self._dyn_prev_rate_limit_err = float(dyn_rate_err)
            self._dyn_prev_rate_limited_strong = bool(
                (bool(low_speed_slew_active) and tracking_gap >= float(DYN_RATE_LIMITED_STRONG_TRACKING_GAP)) or
                (bool(getattr(self, '_stable_torque_slew_active', False)) and stable_gap >= float(DYN_RATE_LIMITED_STRONG_OUTPUT_GAP)) or
                (bool(getattr(self, '_speed_torque_cap_active', False)) and speed_cap_gap >= float(DYN_RATE_LIMITED_STRONG_OUTPUT_GAP))
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
