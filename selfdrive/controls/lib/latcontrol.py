from abc import abstractmethod, ABC

from common.numpy_fast import clip
from common.realtime import DT_CTRL
from selfdrive.car.gm.steering_limits import GM_MIN_STEER_SPEED_MS


# ============================================================
# Equinox 2020 Diesel optimized LatControl base
# ------------------------------------------------------------
# 10km/h부터 lateral controller와 GM LKAS command path를 함께 활성화.
# 이쿼녹스 2020 디젤 토크 튜닝 기준:
# - CarControllerParams.MIN_STEER_SPEED = 10km/h
# - torqued.py 코너 학습 시작 = 10km/h
# - latcontrol_torque.py 유효 조향 시작 = 10km/h
#
# 따라서 공통 LatControl 기준도 10km/h로 맞추는 것이 일관성 있음.
# 10km/h = 약 2.778m/s
# ============================================================

MIN_STEER_SPEED = GM_MIN_STEER_SPEED_MS

# Saturation reporting floor. Was 10.0 m/s (36 km/h), which disabled the check
# across most of this car's real driving: lateral control runs from 10 km/h, so
# everything between 10 and 36 km/h could sit at full steering command without
# ever being reported.
#
# Measured on 2026-08-26--01-04-22 at +1848 s (a tight left turn at 12-19 km/h
# while braking): actuators.steer held +/-1.000 -- the full GM steering command
# -- continuously for about 6 s while the wheel stalled near 45 deg and the
# planner's target climbed past 100 deg, so lateral accel error grew to about
# 2 m/s^2. saturated stayed False the whole time purely because of this 36 km/h
# gate, so the driver got no "핸들을 잡아주세요" prompt while the controller was
# out of authority.
#
# Reporting only: this value gates the sat_count filter feeding pid_log.saturated
# and, through it, the steerSaturated WARNING alert. It is evaluated after the
# torque output is computed and never feeds back into it, and steerSaturated is
# ET.WARNING alone (no soft/immediate disable). controlsd additionally requires
# hands-off plus >0.20 m path deviation before raising it, and steerLimitTimer
# (0.4 s on GM) still filters brief saturation.
SATURATION_CHECK_SPEED = GM_MIN_STEER_SPEED_MS

# sat_count 안전 범위
SAT_COUNT_MIN = 0.0


class LatControl(ABC):
  def __init__(self, CP, CI):
    self.sat_count_rate = 1.0 * DT_CTRL

    try:
      self.sat_limit = float(CP.steerLimitTimer)
    except Exception:
      self.sat_limit = 0.8

    self.sat_count = 0.0

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active, CS, VM, params, last_actuators, steer_limited,
             desired_curvature, desired_curvature_rate, llk):
    pass

  def reset(self):
    self.sat_count = 0.0

  def _check_saturation(self, saturated, CS, steer_limited):
    """
    Saturation 판단 최적화:
    - 저속에서는 조향각/타이어/차량 움직임 특성 때문에 saturation 오검출 가능성이 큼
    - 운전자가 핸들을 잡고 있거나 steer_limited 상태면 saturation 누적하지 않음
    - sat_count는 항상 0 ~ sat_limit 범위로 제한
    """

    try:
      v_ego = float(CS.vEgo)
    except Exception:
      v_ego = 0.0

    try:
      steering_pressed = bool(CS.steeringPressed)
    except Exception:
      steering_pressed = False

    should_count_sat = (
      bool(saturated) and
      v_ego > SATURATION_CHECK_SPEED and
      not bool(steer_limited) and
      not steering_pressed
    )

    if should_count_sat:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, SAT_COUNT_MIN, self.sat_limit)

    return self.sat_count > max(0.0, self.sat_limit - 1e-3)
