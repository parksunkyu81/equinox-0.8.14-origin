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
V_CRUISE_ENABLE_MIN = 1

LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_SHORT_PRESS_INTERVAL_KPH = 5
CRUISE_LONG_PRESS_INTERVAL_KPH = 10
CRUISE_STEP_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise)

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

def _next_v_cruise_interval(v_cruise_kph, interval_kph):
  """Return the next strictly higher km/h interval.

  Examples:
    63 -> 65 for interval 5
    63 -> 70 for interval 10
    65 -> 70 for interval 5
  """
  interval_kph = max(float(interval_kph), 1.0)
  return (math.floor(float(v_cruise_kph) / interval_kph) + 1.0) * interval_kph


def update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled):
  # This fork uses km/h only. Both RES+ and SET- advance the set speed to
  # the next 5 km/h boundary on a short press, or the next 10 km/h boundary
  # once on a long press.
  if not enabled:
    return v_cruise_kph

  button_type = None
  long_press = False

  # Resolve short/long press when the button is released.
  for b in buttonEvents:
    key = b.type.raw
    if key not in button_timers or key not in CRUISE_STEP_BUTTONS or b.pressed:
      continue

    held_frames = button_timers[key]
    if held_frames >= CRUISE_LONG_PRESS:
      if held_frames == CRUISE_LONG_PRESS:
        button_type = key
        long_press = True
      else:
        return v_cruise_kph  # long press was already handled while held
    elif held_frames > 0:
      button_type = key
    break

  # Apply the long-press step exactly once at the threshold.
  if button_type is None:
    for key, held_frames in button_timers.items():
      if key in CRUISE_STEP_BUTTONS and held_frames == CRUISE_LONG_PRESS:
        button_type = key
        long_press = True
        break

  if button_type is not None:
    interval = CRUISE_LONG_PRESS_INTERVAL_KPH if long_press else CRUISE_SHORT_PRESS_INTERVAL_KPH
    v_cruise_kph = _next_v_cruise_interval(v_cruise_kph, interval)
    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  current_speed_kph = clip(v_ego * CV.MS_TO_KPH, V_CRUISE_MIN, V_CRUISE_MAX)

  for b in buttonEvents:
    if b.type == ButtonType.accelCruise and v_cruise_last < 250:
      restored_speed = clip(v_cruise_last, V_CRUISE_MIN, V_CRUISE_MAX)
      return int(clip(_next_v_cruise_interval(restored_speed,
                                              CRUISE_SHORT_PRESS_INTERVAL_KPH),
                      V_CRUISE_MIN, V_CRUISE_MAX))

    if b.type == ButtonType.decelCruise:
      # Requested behavior: SET- also advances to the next 5 km/h boundary.
      return int(clip(_next_v_cruise_interval(current_speed_kph,
                                              CRUISE_SHORT_PRESS_INTERVAL_KPH),
                      V_CRUISE_MIN, V_CRUISE_MAX))

  nearest_speed = round(current_speed_kph / CRUISE_SHORT_PRESS_INTERVAL_KPH) * CRUISE_SHORT_PRESS_INTERVAL_KPH
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