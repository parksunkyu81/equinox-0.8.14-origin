from dataclasses import dataclass


ACC_MULT_SPEED_KPH = (0.0, 20.0, 30.0, 60.0, 80.0, 100.0)
ACC_MULT_VALUES = (0.186, 0.178, 0.175, 0.170, 0.172, 0.184)


def clip(value, lower, upper):
  return max(lower, min(upper, value))


def interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]

  for i in range(1, len(xp)):
    if x <= xp[i]:
      ratio = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + ratio * (fp[i] - fp[i - 1])
  return fp[-1]


def pedal_accel_multiplier(speed_mps):
  return interp(speed_mps * 3.6, ACC_MULT_SPEED_KPH, ACC_MULT_VALUES)


@dataclass
class EquinoxVehicle:
  """Small deterministic vehicle plant driven by GM CarController outputs."""

  initial_speed_kph: float = 80.0
  speed_mps: float = 0.0
  accel_mps2: float = 0.0
  steering_angle_deg: float = 0.0
  steering_rate_deg_s: float = 0.0
  distance_m: float = 0.0

  def __post_init__(self):
    self.reset(self.initial_speed_kph)

  def reset(self, speed_kph=None):
    if speed_kph is None:
      speed_kph = self.initial_speed_kph
    self.speed_mps = max(0.0, float(speed_kph) / 3.6)
    self.accel_mps2 = 0.0
    self.steering_angle_deg = 0.0
    self.steering_rate_deg_s = 0.0
    self.distance_m = 0.0

  def step(self, dt, pedal_command, steer_command=0.0,
           brake_pressed=False, driver_gas_pressed=False):
    dt = max(1e-4, float(dt))
    pedal = clip(float(pedal_command), 0.0, 0.85)
    steer = clip(float(steer_command), -1.0, 1.0)

    if brake_pressed:
      desired_accel = -3.0
    elif driver_gas_pressed:
      desired_accel = 1.2
    elif pedal > 0.001:
      desired_accel = pedal / pedal_accel_multiplier(self.speed_mps)
    else:
      # Engine/driveline drag while the virtual pedal is released.
      desired_accel = -0.08 - 0.0025 * self.speed_mps

    accel_tau = 0.35
    self.accel_mps2 += (desired_accel - self.accel_mps2) * dt / (accel_tau + dt)
    self.speed_mps = max(0.0, self.speed_mps + self.accel_mps2 * dt)
    self.distance_m += self.speed_mps * dt

    desired_angle = steer * 18.0
    steer_tau = 0.25
    previous_angle = self.steering_angle_deg
    self.steering_angle_deg += (desired_angle - self.steering_angle_deg) * dt / (steer_tau + dt)
    self.steering_rate_deg_s = (self.steering_angle_deg - previous_angle) / dt

    return self

