import math
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from selfdrive.modeld.constants import T_IDXS
from selfdrive.ntune import ntune_common_get

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

def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(v_ego, 0.1)

  # TODO 이 부분은 좀 더 고민이 필요함. 현재는 .2초의 추가 지연을 사용하여 다른 지연을 추정
  delay = max(0.01, CP.steerActuatorDelay)
  # MPC가 휠을 돌리고 지연 전의 조정을 계획할 수 있음.
  current_curvature_desired = curvatures[0]
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

  return safe_desired_curvature, safe_desired_curvature_rate