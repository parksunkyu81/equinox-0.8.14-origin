import math

from cereal import log
from common.numpy_fast import interp, clip
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from common.params import Params, put_nonblocking
from decimal import Decimal
import cereal.messaging as messaging
from selfdrive.car.gm.steering_limits import STEER_DELTA_BP_KPH, STEER_DELTA_DOWN_V, STEER_DELTA_UP_V
from selfdrive.controls.lib.low_speed_torque_guard import LowSpeedTorqueReversalGuard
from selfdrive.controls.lib.torque_authority import (DynamicTorqueAuthorityScheduler,
                                                     LateralResponseCompensator,
                                                     LAT_FACTOR_ABS_MIN, FRICTION_ABS_MAX,
                                                     effective_torque_params)
from selfdrive.controls.lib.v0813_lateral_compat import V0813CurvatureGuard

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

# steer_limited / saturated 직후 몇 프레임 더 보수적으로 유지할지
HS_LIMIT_HOLD_BP = [30.0, 60.0, 90.0, 110.0, 130.0]
HS_LIMIT_HOLD_V = [4.0, 6.0, 8.0, 12.0, 16.0]

# Low-speed adaptive slew guard.
# It does not reduce steady-state steering authority. It only slows a sudden
# jump when the requested steer is far ahead of the last actually applied steer.
LS_ADAPTIVE_SLEW_MIN_KPH = 10.0
LS_ADAPTIVE_SLEW_FULL_ON_KPH = 12.0
LS_ADAPTIVE_SLEW_FULL_OFF_KPH = 28.0
LS_ADAPTIVE_SLEW_MAX_KPH = 34.0
LS_ADAPTIVE_SLEW_GAP_START = 0.45
LS_ADAPTIVE_SLEW_GAP_FULL = 1.00
LS_ADAPTIVE_SLEW_ALLOW_GAP_BP = [10.0, 12.0, 20.0, 28.0, 34.0]
LS_ADAPTIVE_SLEW_ALLOW_GAP_V = [1.00, 0.78, 0.72, 0.78, 1.00]

# Safety output torque slew guard.
# This is separate from model curvature smoothing and protects against abrupt
# torque jumps when the controller/car delta-up is too permissive.
STABLE_TORQUE_SLEW_ENABLED = True
# 저속은 더 빠르게, 고속은 더 안정적으로 torque slew 제한.
# 목적: 10~30kph 코너 추종력 확보 + 80~110kph 와리가리 억제.
STABLE_TORQUE_SLEW_KPH_BP = [0.0, 10.0, 20.0, 30.0, 35.0, 40.0, 45.0, 70.0, 90.0, 110.0, 130.0]
STABLE_TORQUE_UP_V =       [0.065, 0.090, 0.095, 0.090, 0.080, 0.065, 0.050, 0.028, 0.021, 0.016, 0.013]
STABLE_TORQUE_DOWN_V =     [0.085, 0.115, 0.120, 0.110, 0.100, 0.080, 0.065, 0.040, 0.032, 0.025, 0.020]
STABLE_TORQUE_LIMITED_SHRINK = 0.85

# ==============================
# Dynamic effective torque profile
# ==============================
# liveTorqueParameters 자체는 학습값 그대로 유지하고,
# 실제 torque 계산에만 임시 effective latAccelFactor/friction을 적용한다.
DYN_TORQUE_PROFILE_ENABLED = True

# v4 10~45kph 통합 개선:
#  - 10~30kph는 안전 클램프 하한/상한까지 사용해 강한 저속 코너 보조
#  - 30~35kph는 저속 강한 개선을 그대로 연장
#  - 35~45kph는 bridge 구간으로 LatAccelFactor/Friction을 점진 완화해 추종력과 안정성을 같이 확보

# 실제 CarController의 STEER_DELTA_UP/DOWN은 carcontroller 쪽에서 적용해야 한다.
# 아래 맵은 이 파일 안에서는 torque slew와 디버그용 목표값으로만 사용한다.
DYN_DELTA_UP_BP = STEER_DELTA_BP_KPH
DYN_DELTA_UP_V = STEER_DELTA_UP_V
DYN_DELTA_DOWN_BP = STEER_DELTA_BP_KPH
DYN_DELTA_DOWN_V = STEER_DELTA_DOWN_V

# Driver override and rate-limit thresholds remain controller inputs; boost
# shape, hold and direction damping are centralized in torque_authority.py.
DYN_DRIVER_TORQUE_HARD_DISABLE = 2.0
DYN_RATE_LIMITED_STRONG_TRACKING_GAP = 0.45
DYN_RATE_LIMITED_STRONG_OUTPUT_GAP = 0.18

# 절대 파라미터 클램프는 torque_authority.py에서 단일 관리한다.
# A learned center offset is applied to feedforward even on a straight road.
# Keep the final controller-side clamp independent from torqued so a stale or
# malformed publisher can never create a large continuous steering bias.
LAT_ACCEL_OFFSET_ABS_MAX = 0.03
LAT_ACCEL_OFFSET_DEADBAND = 0.003

# Directional torque balance.
# latAccelOffset remains the straight-line bias correction. These small,
# bounded per-direction assists compensate left/right corner response
# differences without moving the straight offset.
DIRECTIONAL_TORQUE_COMP_ENABLED = False
DIRECTIONAL_TORQUE_MIN_SPEED = 5.0
DIRECTIONAL_TORQUE_MIN_LAT_ACCEL = 0.12
DIRECTIONAL_TORQUE_ERROR_DEADBAND = 0.035
DIRECTIONAL_TORQUE_ERROR_FULL = 0.28
DIRECTIONAL_TORQUE_STEP = 0.00045
DIRECTIONAL_TORQUE_ASSIST_MIN = 0.94
DIRECTIONAL_TORQUE_ASSIST_MAX = 1.08
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
        self._model_curvature_guard = V0813CurvatureGuard()
        self._stable_prev_output_torque = 0.0
        self._stable_torque_slew_gap = 0.0
        self._stable_torque_slew_active = False
        self._low_speed_reversal_guard = LowSpeedTorqueReversalGuard()
        self._ls_raw_requested_steer = 0.0
        self._ls_guarded_requested_steer = 0.0
        self._ls_applied_steer = 0.0

        # dynamic effective torque state
        self._dyn_scheduler = DynamicTorqueAuthorityScheduler()
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
        self._dyn_last_authority_ceiling = 0.0
        self._dyn_last_direction_reversal = False
        self._dyn_last_direction_damping = False
        self._dyn_prev_rate_limited_strong = False
        self._dyn_prev_rate_limit_err = 0.0
        self._dyn_last_rate_limited_strong = False
        self._dyn_last_target_delta_up = 12.0
        self._dyn_last_target_delta_down = 20.0
        self._path_stability_active = False
        self._path_wobble_range_m = 0.0
        self._path_wobble_flips = 0
        try:
            response_state = Params().get('TorqueResponseBins')
        except Exception:
            response_state = None
        self._response_compensator = LateralResponseCompensator(response_state)
        self._response_last_saved_count = self._response_compensator.update_count
        self._dir_torque_assist_left = 1.0
        self._dir_torque_assist_right = 1.0
        self._dir_torque_last_side = 0

    def set_path_stability(self, active, range_m=0.0, flips=0):
        self._path_stability_active = bool(active)
        self._path_wobble_range_m = max(0.0, self._safe_float(range_m, 0.0))
        try:
            self._path_wobble_flips = max(0, int(flips))
        except Exception:
            self._path_wobble_flips = 0

    def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction, totalBucketPoints=0):
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
        if not DIRECTIONAL_TORQUE_COMP_ENABLED:
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
        curv_out, rate_out, guard_active = self._model_curvature_guard.update(
            float(v_ego) * 3.6,
            desired_curvature,
            desired_curvature_rate,
            limited_hold=int(max(0, getattr(self, "_hs_guard_hold_frames", 0) or 0)) > 0,
        )
        # Retain these attributes for existing diagnostics and forks which read
        # them, while the v0.8.13 guard owns the actual state transition.
        self._hs_prev_desired_curvature = float(curv_out)
        self._hs_prev_desired_curvature_rate = float(rate_out)
        return curv_out, rate_out, guard_active

    def _guard_low_speed_steer_slew(self, v_ego, requested_steer, last_actuators, steering_pressed):
        v_kph = float(v_ego) * 3.6
        if (
                v_kph < LS_ADAPTIVE_SLEW_MIN_KPH or
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
        """Apply bounded transient authority without changing learned values."""
        base_params = dict(getattr(self, '_dyn_base_live_torque_params', self.live_torque_params))

        if not DYN_TORQUE_PROFILE_ENABLED or not self._is_equinox_torque_profile:
            self._dyn_scheduler.reset()
            self._dyn_effective_active = False
            self._dyn_last_blend = 0.0
            self.live_torque_params = dict(base_params)
            return self.live_torque_params

        base_lat = self._safe_float(base_params.get('latAccelFactor', 1.88), 1.88)
        base_fric = self._safe_float(base_params.get('friction', 0.255), 0.255)
        base_off = self._safe_float(base_params.get('latAccelOffset', 0.0), 0.0)
        total_pts = base_params.get('totalBucketPoints', 0)

        v_kph = self._safe_float(v_ego, 0.0) * 3.6
        rate_err = abs(self._safe_float(rate_limit_err, 0.0))
        strong_rate_limited = bool(rate_limited_strong) or (rate_err >= float(DYN_RATE_LIMITED_STRONG_OUTPUT_GAP))

        driver_torque_abs = abs(self._safe_float(driver_steering_torque, 0.0))
        strong_driver_override = bool(steering_pressed) and (driver_torque_abs >= float(DYN_DRIVER_TORQUE_HARD_DISABLE))

        # Demand-only scheduling prevents the applied steer from feeding back
        # into boost. The scheduler owns ramp, hold and reversal damping.
        dyn = self._dyn_scheduler.update(
            v_kph, self._safe_float(desired_curvature, 0.0),
            self._safe_float(desired_lateral_accel, 0.0),
            steering_pressed=bool(steering_pressed),
            strong_driver_override=strong_driver_override,
            steer_limited=bool(steer_limited),
            strong_rate_limited=strong_rate_limited,
            torque_slew_active=bool(getattr(self, '_stable_torque_slew_active', False)),
            output_reversal_active=bool(self._low_speed_reversal_guard.boost_suppressed),
            path_unstable=bool(self._path_stability_active))
        authority_request = float(dyn['authorityRequest'])
        self._dyn_corner_boost = authority_request
        self._dyn_corner_hold_frames = int(dyn['holdFrames'])

        # 저속/중속 권한과 학습 신뢰도를 단일 스케줄러에서 블렌딩한다.
        eff_lat, eff_fric, blend = effective_torque_params(
            base_lat, base_fric, v_kph, authority_request, total_pts
        )
        response_scale = self._response_compensator.update(
            v_kph, self._safe_float(desired_lateral_accel, 0.0),
            self._safe_float(actual_lateral_accel, 0.0),
            steering_pressed=bool(steering_pressed),
            steer_limited=bool(steer_limited),
            # Normal torque slew is expected while following a real curve. The
            # stability timer below lets it settle; only a strong rate limit is
            # unsafe for response learning.
            rate_limited=bool(strong_rate_limited),
            reversal_active=bool(dyn['directionDamping'] or self._low_speed_reversal_guard.boost_suppressed),
            path_unstable=bool(self._path_stability_active))
        eff_lat = float(clip(eff_lat / response_scale, LAT_FACTOR_ABS_MIN, eff_lat))
        eff_fric = float(clip(eff_fric * (1.0 + 0.15 * (response_scale - 1.0)),
                              eff_fric, FRICTION_ABS_MAX))
        if (self._response_compensator.dirty and
                self._response_compensator.update_count - self._response_last_saved_count >= 300):
            try:
                put_nonblocking('TorqueResponseBins', self._response_compensator.serialize())
                self._response_last_saved_count = self._response_compensator.update_count
                self._response_compensator.dirty = False
            except Exception:
                pass
        self._dyn_last_blend = float(blend)

        self._dyn_last_corner_strength = float(dyn['cornerStrength'])
        self._dyn_last_low_speed_gate = float(dyn['lowGate'])
        self._dyn_last_mid_speed_gate = float(dyn['midGate'])
        self._dyn_last_high_speed_gate = float(dyn['highGate'])
        self._dyn_last_authority_ceiling = float(dyn['authorityCeiling'])
        self._dyn_last_direction_reversal = bool(dyn['directionReversal'])
        self._dyn_last_direction_damping = bool(dyn['directionDamping'])
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
        self._dyn_effective_active = bool(blend > 1e-4 or response_scale > 1.0001)
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
        if not bool(getattr(self, '_dyn_effective_active', False)):
            eff = dict(base)
        debug = {
            'active': bool(getattr(self, '_dyn_effective_active', False)),
            'blend': float(getattr(self, '_dyn_last_blend', 0.0) or 0.0),
            'corner_strength': float(getattr(self, '_dyn_last_corner_strength', 0.0) or 0.0),
            'low_gate': float(getattr(self, '_dyn_last_low_speed_gate', 0.0) or 0.0),
            'mid_gate': float(getattr(self, '_dyn_last_mid_speed_gate', 0.0) or 0.0),
            'high_gate': float(getattr(self, '_dyn_last_high_speed_gate', 0.0) or 0.0),
            'authorityCeiling': float(getattr(self, '_dyn_last_authority_ceiling', 0.0) or 0.0),
            'directionReversal': bool(getattr(self, '_dyn_last_direction_reversal', False)),
            'directionDamping': bool(getattr(self, '_dyn_last_direction_damping', False)),
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
            'rateLimitedStrong': bool(getattr(self, '_dyn_last_rate_limited_strong', False)),
        }
        response_debug = self._response_compensator.diagnostics()
        debug.update({
            'responseScale': float(response_debug['scale']),
            'responseRatio': float(response_debug['ratio']),
            'responseBin': int(response_debug['bin']),
            'responseStable': bool(response_debug['stable']),
            'responseFrozen': bool(response_debug['frozen']),
            'responseUpdateCount': int(response_debug['updateCount']),
            'pathStabilityActive': bool(self._path_stability_active),
            'pathWobbleRangeM': float(self._path_wobble_range_m),
            'pathWobbleFlips': int(self._path_wobble_flips),
        })
        debug.update(self._model_curvature_guard.diagnostics())
        reversal_debug = self._low_speed_reversal_guard.diagnostics()
        debug.update({
            'lowSpeedTorqueGuardActive': bool(reversal_debug['active']),
            'lowSpeedTorqueGuardState': int(reversal_debug['state']),
            'lowSpeedTorqueRawSteer': float(getattr(
                self, '_ls_raw_requested_steer', reversal_debug['rawSteer']) or 0.0),
            'lowSpeedTorqueGuardedSteer': float(getattr(
                self, '_ls_guarded_requested_steer', reversal_debug['guardedSteer']) or 0.0),
            'lowSpeedTorqueAppliedSteer': float(getattr(
                self, '_ls_applied_steer', reversal_debug['appliedSteer']) or 0.0),
            'lowSpeedTorqueConfirmMs': int(reversal_debug['confirmMs']),
            'lowSpeedTorqueReversalCount': int(reversal_debug['reversalCount']),
            'lowSpeedTorqueBoostSuppressed': bool(reversal_debug['boostSuppressed']),
        })
        return debug

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
            self._model_curvature_guard.reset()
            self._stable_prev_output_torque = 0.0
            self._stable_torque_slew_gap = 0.0
            self._stable_torque_slew_active = False
            self._low_speed_reversal_guard.reset(0.0)
            self._ls_raw_requested_steer = 0.0
            self._ls_guarded_requested_steer = 0.0
            self._ls_applied_steer = 0.0
            self._dyn_scheduler.reset()
            self._dyn_prev_rate_limited_strong = False
            self._dyn_prev_rate_limit_err = 0.0
            self._dyn_effective_active = False
            self._dyn_last_blend = 0.0
            self._response_compensator.reset_transient()
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
            freeze_integrator = (
                steer_limited or
                CS.steeringPressed or
                CS.vEgo < 5 or
                self._low_speed_reversal_guard.active or
                (hs_guard_active and self._hs_guard_hold_frames > 0)
            )
            pid_i_before_update = float(self.pid.i)
            output_torque = self.pid.update(pid_log.error,
                                            feedforward=ff,
                                            speed=CS.vEgo,
                                            freeze_integrator=freeze_integrator)

            requested_steer_raw = -output_torque
            self._ls_raw_requested_steer = float(requested_steer_raw)
            requested_steer = requested_steer_raw
            requested_steer, low_speed_slew_active = self._guard_low_speed_steer_slew(
                CS.vEgo, requested_steer, last_actuators, CS.steeringPressed
            )
            if low_speed_slew_active:
                output_torque = -requested_steer
                self.pid.control = output_torque

            try:
                applied_last = float(getattr(last_actuators, 'steer', 0.0)) if last_actuators is not None else 0.0
                if not math.isfinite(applied_last):
                    applied_last = 0.0
            except Exception:
                applied_last = 0.0
            self._ls_applied_steer = float(applied_last)

            requested_steer = self._low_speed_reversal_guard.update(
                CS.vEgo * 3.6, requested_steer, applied_last,
                desired_curvature=desired_curvature,
                desired_lateral_accel=desired_lateral_accel,
                steering_pressed=CS.steeringPressed,
                active=active,
            )
            reversal_limited = bool(
                abs(float(requested_steer) - float(requested_steer_raw)) > 1e-6)
            if reversal_limited:
                output_torque = -requested_steer
                # Do not accumulate integral behind a guarded output.  During
                # a reversal, gently unwind existing I torque toward neutral.
                self.pid.i = pid_i_before_update
                if self._low_speed_reversal_guard.active:
                    self.pid.i *= 0.98
                self.pid.control = output_torque

            output_torque = self._guard_output_torque_slew(
                CS.vEgo, output_torque, CS.steeringPressed, bool(steer_limited)
            )
            self.pid.control = output_torque
            self._ls_guarded_requested_steer = float(-output_torque)

            # v2: 다음 프레임 dynamic boost에 직접 넣을 strong rate-limit proxy를 저장한다.
            # low-speed slew에서 requested가 실제 적용 가능 범위보다 크게 앞서거나,
            # output torque slew가 target을 크게 잘라낸 경우에는 다음 프레임 부스트를 줄인다.
            same_direction = (float(requested_steer_raw) * applied_last) >= -0.02
            if same_direction:
                tracking_gap = max(0.0, abs(float(requested_steer_raw)) - abs(applied_last))
            else:
                tracking_gap = abs(float(requested_steer_raw) - applied_last)
            stable_gap = float(getattr(self, '_stable_torque_slew_gap', 0.0) or 0.0)
            reversal_gap = abs(float(requested_steer_raw) - float(self._ls_guarded_requested_steer))
            dyn_rate_err = max(stable_gap, reversal_gap,
                               tracking_gap if bool(low_speed_slew_active) else 0.0)
            self._dyn_prev_rate_limit_err = float(dyn_rate_err)
            self._dyn_prev_rate_limited_strong = bool(
                self._low_speed_reversal_guard.boost_suppressed or
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
