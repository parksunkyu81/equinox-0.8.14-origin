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
CRUISE_SHORT_PRESS_DELTA_KPH = 5
CRUISE_LONG_PRESS_DELTA_KPH = 10
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

def _align_v_cruise_to_interval(v_cruise_kph, interval_kph, button_type):
  """버튼 방향에 맞춰 설정 속도를 지정 간격 경계로 정렬한다."""
  return CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / interval_kph) * interval_kph


def update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled, metric):
  # 크루즈 컨트롤이 활성화되지 않은 경우 속도 변경을 적용하지 않음
  if not enabled:
    return v_cruise_kph

  # 이 포크는 계기판 단위와 관계없이 내부 설정 속도를 km/h로 관리하며,
  # 짧게 5km/h, 길게 10km/h 변경하는 사용자 설정을 유지한다.
  _ = metric
  long_press = False
  button_type = None

  for b in buttonEvents:
    if b.type.raw in button_timers and not b.pressed:
      if button_timers[b.type.raw] > CRUISE_LONG_PRESS:
        return v_cruise_kph  # 길게 누른 뒤 버튼을 놓을 때 중복 변경 방지
      button_type = b.type.raw
      break
  else:
    for k in button_timers.keys():
      if button_timers[k] and button_timers[k] % CRUISE_LONG_PRESS == 0:
        button_type = k
        long_press = True
        break

  if button_type:
    v_cruise_delta = CRUISE_LONG_PRESS_DELTA_KPH if long_press else CRUISE_SHORT_PRESS_DELTA_KPH
    remainder = v_cruise_kph % v_cruise_delta

    # 63km/h처럼 경계에서 벗어난 값은 RES+ 65, SET- 60으로 먼저 정렬한다.
    # 이미 경계에 맞아 있으면 설정한 5/10km/h만큼 변경한다.
    if not math.isclose(remainder, 0.0, abs_tol=1e-3):
      v_cruise_kph = _align_v_cruise_to_interval(v_cruise_kph, v_cruise_delta, button_type)
    else:
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  current_speed_kph = clip(v_ego * CV.MS_TO_KPH, V_CRUISE_MIN, V_CRUISE_MAX)

  for b in buttonEvents:
    if b.type == ButtonType.accelCruise and v_cruise_last < 250:
      # RES+: 이전 목표속도를 복원하되 5km/h 상향 경계에 맞춘다.
      restored_speed = clip(v_cruise_last, V_CRUISE_MIN, V_CRUISE_MAX)
      return int(_align_v_cruise_to_interval(restored_speed,
                                             CRUISE_SHORT_PRESS_DELTA_KPH,
                                             ButtonType.accelCruise))

    if b.type == ButtonType.decelCruise:
      # SET-: 현재속도를 기준으로 5km/h 하향 경계에 맞춘다.
      return int(_align_v_cruise_to_interval(current_speed_kph,
                                             CRUISE_SHORT_PRESS_DELTA_KPH,
                                             ButtonType.decelCruise))

  # 버튼 종류를 확인하지 못한 예외 상황에서는 가장 가까운 5km/h로 설정한다.
  nearest_speed = math.floor((current_speed_kph + CRUISE_SHORT_PRESS_DELTA_KPH / 2.0) /
                             CRUISE_SHORT_PRESS_DELTA_KPH) * CRUISE_SHORT_PRESS_DELTA_KPH
  return int(clip(nearest_speed, V_CRUISE_MIN, V_CRUISE_MAX))


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