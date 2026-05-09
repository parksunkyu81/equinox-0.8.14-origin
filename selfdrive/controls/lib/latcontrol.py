from abc import abstractmethod, ABC

from common.numpy_fast import clip
from common.realtime import DT_CTRL


# ============================================================
# Equinox 2020 Diesel optimized LatControl base
# ------------------------------------------------------------
# 기존 MIN_STEER_SPEED = 0.3m/s 는 약 1.08km/h라 너무 낮음.
# 이쿼녹스 2020 디젤 토크 튜닝 기준:
# - CarControllerParams.MIN_STEER_SPEED = 3.0m/s
# - torqued.py 코너 학습 시작 = 3.0m/s
# - latcontrol_torque.py 유효 조향 시작 = 3.0m/s
#
# 따라서 공통 LatControl 기준도 3.0m/s로 맞추는 것이 일관성 있음.
# 3.0m/s = 약 10.8km/h
# ============================================================

MIN_STEER_SPEED = 3.0

# Saturation 판단은 너무 저속에서 하면 오검출이 많음.
# 기존 10.0m/s = 36km/h 기준 유지.
SATURATION_CHECK_SPEED = 10.0

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