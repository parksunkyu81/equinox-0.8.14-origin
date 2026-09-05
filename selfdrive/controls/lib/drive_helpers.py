import math
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL, DT_MDL
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
# Expressed as lateral acceleration so it scales with speed -- about a 14 m radius
# at 28 km/h and a 193 m one at 100 km/h. This is a guard against a broken plan,
# not a tuning knob for how the car corners, and the frame-to-frame jerk limit
# below is what actually bounds an excursion now: this only caps where one can
# settle, and only above anything real driving asks for.
#
# 3.0 -> 2.2 -> 4.0. The 2.2 pass classified any frame near a >5 m planned path
# as belonging to the lane-blend bug of 8f829051, which was wrong: a genuine
# tight turn at low speed legitimately reaches ~9 m at the 2.5 s horizon, so that
# rule swallowed exactly the cornering the ceiling had to clear. Re-classified on
# the signature that actually distinguishes the bug -- the PLAN diverging from the
# MODEL path (planned > 5 m while the model path stays under 3 m) -- over 56768
# engaged frames that are not bug-related:
#
#   demand  p50 0.08   p99 1.94   p99.9 2.84   p99.99 3.54   max 3.61 m/s^2
#
#   clipped by 2.2: 267 frames (13.4 s), worst cut 39%
#              2.5: 110            3.0: 37            3.5: 8            4.0: 0
#
# The worst clipped frame is 2026-09-05--06-16-06 at +1541.2 s: an 18 m radius
# turn at 23 km/h where the car was already pulling 2.34 m/s^2 by its own
# measured curvature and the plan asked for 3.61. A 2.2 ceiling there commands
# less than the car is already doing -- it straightens the wheel mid-corner and
# runs the car wide, which on a left turn is into oncoming traffic. 3.0 has the
# same fault on 37 frames, so the original value was already slightly too tight.
#
# 4.0 is the first value that clips none of the recorded cornering, with 11%
# headroom over the 3.61 m/s^2 maximum.
MAX_LATERAL_ACCEL = 4.0

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


# Frame-to-frame bound on the steering target that actually leaves this module.
#
# The clip named "rate limit" further down is anchored on curvatures[0] of the
# same frame, so it only stops the psi extrapolation from running away from that
# frame's own value -- it does not constrain how much the delivered target may
# change since the previous frame, and nothing else did either. That is how the
# 2026-09-05--09-24-53 +378.43 s plan corruption reached the torque controller as
# a demand climbing from 0.2 to 19.0 m/s^2 in half a second.
#
# Limited as lateral jerk, using the MAX_LATERAL_JERK already declared above, so
# it scales with speed and disappears at parking speeds. Measured cost over 56768
# engaged frames that are not bug-related (see MAX_LATERAL_ACCEL above for how
# those are identified):
#
#   |d(lateral accel)/dt|  p99 1.19   p99.9 2.27   max 7.23 m/s^3
#   frames above the limit:  3.0 -> 17    4.0 -> 5    5.0 -> 3    6.0 -> 1    8.0 -> 0
#
# So 5.0 touches 3 frames in 56768 (0.005%), each a single frame, against a p99.9
# of 2.27 -- genuine cornering ramps well under half the limit. It cost nothing
# measurable on the stock-model drive, whose ramp never exceeds 2.29 m/s^3.
#
# It never reduces steady-state authority in a curve, only the rate of approach,
# which is what makes it safe to apply unconditionally: unlike a confidence gate,
# it cannot under-steer a curve the model is tracking correctly. That property is
# why it, and not the magnitude ceiling, is the load-bearing guard here -- the
# ceiling had to be relaxed to 4.0 precisely because a tight ceiling does
# under-steer real corners.
_last_limited_curvature = None
_curvature_rate_limited_frames = 0


def _rate_limit_curvature(curvature, v_ego, lat_active):
  """Bound how fast the delivered steering target may change between frames."""
  global _last_limited_curvature, _curvature_rate_limited_frames

  # Re-seed whenever lateral control is not in charge, so the first engaged frame
  # starts from the plan rather than ramping out of a stale value.
  if not lat_active:
    _last_limited_curvature = None
    _curvature_rate_limited_frames = 0
    return curvature
  if _last_limited_curvature is None:
    _last_limited_curvature = curvature
    return curvature

  max_step = MAX_LATERAL_JERK / (v_ego ** 2) * DT_CTRL
  limited = clip(curvature,
                 _last_limited_curvature - max_step,
                 _last_limited_curvature + max_step)
  if limited != curvature:
    if _curvature_rate_limited_frames % 100 == 0:
      cloudlog.warning(
        "desired curvature rate limited: %.5f -> %.5f 1/m (%.2f -> %.2f m/s^2 at %.1f m/s)"
        % (curvature, limited, curvature * v_ego ** 2, limited * v_ego ** 2, v_ego))
    _curvature_rate_limited_frames += 1
  else:
    _curvature_rate_limited_frames = 0

  # This holds a curvature, not a lateral acceleration, so while it lags a
  # falling target through an acceleration the held value drifts back above the
  # ceiling -- simulated at 2.34 m/s^2 against a 2.20 ceiling on
  # 2026-09-05--06-16-06. Re-apply the ceiling before storing, so the state can
  # never carry a value the ceiling would reject. This only ever moves the value
  # toward zero, so it cannot breach the rate limit itself.
  limited = limit_curvature(limited, v_ego)
  _last_limited_curvature = limited
  return limited


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


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates, lat_active=True):
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

  # The clip above only constrains movement away from the anchor; the psi
  # extrapolation can still land outside what the car can hold, so bound the
  # result itself.
  limited_curvature = limit_curvature(safe_desired_curvature, v_ego)
  _log_curvature_limit(safe_desired_curvature, limited_curvature, v_ego)

  # How fast the bounded target may move, frame to frame. Applied last, so it
  # also bounds the approach to the ceiling above: this is the final thing
  # between the plan and the steering. It re-applies the ceiling internally.
  limited_curvature = _rate_limit_curvature(limited_curvature, v_ego, lat_active)

  return limited_curvature, safe_desired_curvature_rate
