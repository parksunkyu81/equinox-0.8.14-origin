import math
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from selfdrive.modeld.constants import T_IDXS
from selfdrive.ntune import ntune_common_get
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.v0813_lateral_compat import compensated_steer_delay

# 경고: 이 값은 모델의 훈련 분포를 기반으로 결정되었으며,
# 이 속도 이상의 모델 예측은 예측할 수 없습니다.

# kph
V_CRUISE_MAX = 145
V_CRUISE_MIN = 20
V_CRUISE_DELTA_MI = 5 * CV.MPH_TO_KPH
V_CRUISE_DELTA_KM = 20
V_CRUISE_ENABLE_MIN = 1

LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

# Ceiling on how hard the plan may ask the car to turn. get_lag_adjusted_curvature
# rate-limits the target, but it anchors that limit on curvatures[0], so nothing
# bounded the magnitude when the planner itself emitted an implausible value: on
# the 2026-09-04 drive the curve fallback asked for 6.3 m/s^2 at 28 km/h while the
# car was tracking a nearly straight path, and the torque controller sat pinned at
# full output for 0.73 s with no alert raised.
#
# Expressed as lateral acceleration so it scales with speed -- about a 26 m radius
# at 28 km/h and a 350 m one at 100 km/h. This is a guard against a broken plan,
# not a tuning knob for how the car corners.
#
# 3.0 -> 2.2. The old figure was set before 8f829051, when the lane-blend divide
# bug and its 1.8 s of MPC ramp were still in the drives and inflated every
# measurement of what a curve "needs". Re-measured with those frames and a 2.5 s
# window around them excluded, over 3379 frames where the car was genuinely
# turning (|measured curvature| >= 0.008 averaged over 3 s, which is derived from
# steering angle and so is independent of the lane lines):
#
#   2026-09-05--09-24-53   p99 1.87   max 1.91     2026-09-05--06-16-06  p99 1.91  max 2.19
#   2026-09-05--07-36-07   p99 1.82   max 1.83     2026-09-04--09-02-52  p99 2.05  max 2.06 (stock)
#
# So 2.2 clips none of the recorded cornering, and it cuts the worst case a
# broken plan can deliver by 27%. It is deliberately tight: the headroom over the
# 2.19 m/s^2 maximum is 0.5%, so a curve sharper than anything in these four
# drives would be under-steered and run wide. If that shows up, 2.5 restores a
# 14% margin and still improves on 3.0 -- this constant is the only thing to
# change.
MAX_LATERAL_ACCEL = 2.2

# Backstop for the low-speed end, where the lateral-accel ceiling alone allows
# curvatures tighter than the car can physically steer. ~5 m turning radius.
MAX_CURVATURE = 0.2

ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}

class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)

def update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled, metric):
  # 크루즈 컨트롤이 활성화되지 않은 경우 속도 변경을 적용하지 않음
  if not enabled:
    return v_cruise_kph

  long_press = False
  button_type = None

  # 기본 속도 증가 값 설정
  v_cruise_delta = 5  # 짧은 누름일 때 5km/h 증가

  for b in buttonEvents:
    if b.type.raw in button_timers and not b.pressed:
      if button_timers[b.type.raw] > CRUISE_LONG_PRESS:
        return v_cruise_kph  # 길게 누름 종료
      button_type = b.type.raw
      break
  else:
    for k in button_timers.keys():
      if button_timers[k] and button_timers[k] % CRUISE_LONG_PRESS == 0:
        button_type = k
        long_press = True
        break

  if button_type:
    if long_press:
      v_cruise_delta = 10  # 길게 누를 때 10km/h 증가
    if long_press and v_cruise_kph % v_cruise_delta != 0:  # 사용자가 버튼을 길게 눌렀으며, 현재 속도 v_cruise_kph가 10의 배수가 아닐 경우
      v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / v_cruise_delta) * v_cruise_delta # 10의 배수로 반올림 또는 내림
    else:  # 속도가 10의 배수로 이미 맞춰져 있는 경우
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]
    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph

def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph 이상일 경우 설정 속도가 없었던 것으로 간주
    if b.type == car.CarState.ButtonEvent.Type.accelCruise and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_MIN, V_CRUISE_MAX)))

def limit_curvature(curvature, v_ego):
  """Clamp a desired curvature to one the car can hold at this speed.

  The bound is a lateral-acceleration ceiling, so it tightens as speed rises and
  effectively disappears at parking speeds, where MAX_CURVATURE takes over.
  """
  max_curvature = min(MAX_LATERAL_ACCEL / (v_ego ** 2), MAX_CURVATURE)
  return clip(curvature, -max_curvature, max_curvature)


_curvature_limited_frames = 0

def _log_curvature_limit(requested, limited, v_ego):
  # Log the first frame of a burst and then at most once a second, so a stuck
  # plan stays visible without flooding the log from this 100 Hz path.
  global _curvature_limited_frames
  if requested == limited:
    _curvature_limited_frames = 0
    return
  if _curvature_limited_frames % 100 == 0:
    cloudlog.warning(
      "desired curvature limited: %.5f -> %.5f 1/m (%.2f -> %.2f m/s^2 at %.1f m/s)"
      % (requested, limited, requested * v_ego ** 2, limited * v_ego ** 2, v_ego))
  _curvature_limited_frames += 1


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(v_ego, 0.1)

  # TODO 이 부분은 좀 더 고민이 필요함. 현재는 .2초의 추가 지연을 사용하여 다른 지연을 추정
  # Match the official v0.8.13 planner lookahead and torqued sample alignment.
  delay = compensated_steer_delay(CP.steerActuatorDelay)
  # MPC가 휠을 돌리고 지연 전의 조정을 계획할 수 있음.
  # Bound curvatures[0] before it is used, since it anchors the rate limit below:
  # an implausible value there would otherwise carry straight through.
  current_curvature_desired = limit_curvature(curvatures[0], v_ego)
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # 이것은 실제 목표 속도가 아닌 목표 속도의 "예상 속도"
  desired_curvature_rate = curvature_rates[0]
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2)
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                          -max_curvature_rate,
                                          max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                     current_curvature_desired - max_curvature_rate * DT_MDL,
                                     current_curvature_desired + max_curvature_rate * DT_MDL)

  # The rate limit above only constrains movement away from the anchor; the psi
  # extrapolation can still land outside what the car can hold, so bound the
  # result itself. This is the last thing between the plan and the steering.
  limited_curvature = limit_curvature(safe_desired_curvature, v_ego)
  _log_curvature_limit(safe_desired_curvature, limited_curvature, v_ego)

  return limited_curvature, safe_desired_curvature_rate
